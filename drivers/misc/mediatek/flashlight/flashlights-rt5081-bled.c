/*****************************************************************
Copyright (C), 1996-2017, TP-LINK TECHNOLOGIES CO., LTD.

File name   : flashlights-rt5081-bled.c

Description : Backlight for sub camera flash dirver.

Author      : Lin Yimin

Email       : linyimin@tp-link.com.cn

History     :
------------------------------------------------------------------
V0.1, 2017-08-02, linyimin       create file.
******************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#include "richtek/rt-flashlight.h"
#include "mtk_charger.h"

#include "flashlight.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef RT5081_BLED_DTNAME
#define RT5081_BLED_DTNAME "mediatek,flashlights_rt5081_bled"
#endif

#define RT5081_BLED_NAME "flashlights-rt5081-bled"


/* define channel, level */
#define RT5081_CHANNEL_NUM 3
#define RT5081_CHANNEL_BLED 2


#define RT5081_NONE (-1)
#define RT5081_DISABLE 0
#define RT5081_ENABLE 1
#define RT5081_ENABLE_TORCH 1
#define RT5081_ENABLE_FLASH 2

#define RT5081_LEVEL_NUM 32
#define RT5081_LEVEL_TORCH 32
#define RT5081_LEVEL_FLASH RT5081_LEVEL_NUM
#define RT5081_WDT_TIMEOUT 1248 /* ms */

/* define mutex, work queue and timer */
static DEFINE_MUTEX(rt5081_mutex);
static DEFINE_MUTEX(rt5081_enable_mutex);
static DEFINE_MUTEX(rt5081_disable_mutex);
static struct work_struct rt5081_work_bled;

static struct hrtimer rt5081_timer_bled;

static unsigned int rt5081_timeout_ms[RT5081_CHANNEL_NUM];

/* define usage count */
static int use_count;

/* define RTK flashlight device */
struct flashlight_device *backlight_dev;

#define RT_BLED_DEVICE  "rt5081_pmu_bled"

/* define charger consumer */
static struct charger_consumer *flashlight_charger_consumer;
#define CHARGER_SUPPLY_NAME "charger_port1"

/* is decrease voltage */
static int is_decrease_voltage;

/******************************************************************************
 * rt5081 operations
 *****************************************************************************/

/*
	0.2344mA per step
*/
static const unsigned char rt5081_torch_level[RT5081_LEVEL_TORCH] = {
	/*6.09mA, 7.74, 11.02, 14.06, 19.46, 24.38, 29.07, 33.05, 36.80, 40.08*/
	0x1A, 0x21, 0x2F, 0x3C, 0x53, 0x68, 0x7C, 0x8D, 0x9D, 0xAB,
	0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB,
	0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB,
	0xAB, 0xAB
};

static const unsigned char rt5081_strobe_level[RT5081_LEVEL_FLASH] = {
	0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB,
	0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB,
	0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB,
	0xAB, 0xAB
};


static int rt5081_en_bled;

static int rt5081_level_bled;


static int rt5081_is_charger_ready(void)
{
	if (flashlight_is_ready(backlight_dev))
		return FLASHLIGHT_CHARGER_READY;
	else
		return FLASHLIGHT_CHARGER_NOT_READY;
}

static int rt5081_is_torch(int level)
{
	/* Use torch mode always, for long time flash */
	return 0;

	if (level >= RT5081_LEVEL_TORCH)
		return -1;

	return 0;
}

static int rt5081_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= RT5081_LEVEL_NUM)
		level = RT5081_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int rt5081_enable(void)
{
	int ret = 0;
	flashlight_mode_t mode = FLASHLIGHT_MODE_TORCH;

	if (!backlight_dev) {
		fl_err("Failed to enable since no flashlight device.\n");
		return -1;
	}

	/* set flash mode if any channel is flash mode */
	if (rt5081_en_bled == RT5081_ENABLE_FLASH)
		mode = FLASHLIGHT_MODE_FLASH;

	if (rt5081_en_bled)
		ret |= flashlight_set_mode(
				backlight_dev, mode);
	else
		ret |= flashlight_set_mode(
				backlight_dev, FLASHLIGHT_MODE_OFF);

	if (ret < 0)
		fl_err("Failed to enable.\n");

	return ret;
}

/* flashlight disable function */
static int rt5081_disable(void)
{
	int ret = 0;

	if (!backlight_dev ) {
		fl_err("Failed to disable since no flashlight device.\n");
		return -1;
	}

	/* disable channel 1 */
	ret |= flashlight_set_mode(backlight_dev, FLASHLIGHT_MODE_OFF);

	if (ret < 0)
		fl_err("Failed to disable.\n");

	return ret;
}

/* set flashlight level */
static int rt5081_set_level_bled(int level)
{
	level = rt5081_verify_level(level);
	rt5081_level_bled = level;

	if (!backlight_dev) {
		fl_err("Failed to set ht level since no flashlight device.\n");
		return -1;
	}

	/* set brightness level */
	if (!rt5081_is_torch(level))
		flashlight_set_torch_brightness(
				backlight_dev, rt5081_torch_level[level]);
	flashlight_set_strobe_brightness(
			backlight_dev, rt5081_strobe_level[level]);

	return 0;
}

static int rt5081_set_level(int channel, int level)
{
	if (channel == RT5081_CHANNEL_BLED)
		rt5081_set_level_bled(level);
	else {
		fl_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int rt5081_set_scenario(int scenario)
{

	/* notify charger to increase or decrease voltage */
	if (!flashlight_charger_consumer) {
		fl_err("Failed with no charger consumer handler.\n");
		return -1;
	}

	mutex_lock(&rt5081_mutex);
	if (scenario & FLASHLIGHT_SCENARIO_CAMERA_MASK) {
		if (!is_decrease_voltage) {
			fl_info("Decrease voltage level.\n");
			charger_manager_enable_high_voltage_charging(flashlight_charger_consumer, false);
			is_decrease_voltage = 1;
		}
	} else {
		if (is_decrease_voltage) {
			fl_info("Increase voltage level.\n");
			charger_manager_enable_high_voltage_charging(flashlight_charger_consumer, true);
			is_decrease_voltage = 0;
		}
	}
	mutex_unlock(&rt5081_mutex);

	return 0;
}

/* flashlight init */
static int rt5081_init(void)
{
	/* clear flashlight state */
	rt5081_en_bled = RT5081_NONE;



	/* clear charger status */
	is_decrease_voltage = 0;

	return 0;
}

/* flashlight uninit */
static int rt5081_uninit(void)
{
	/* clear flashlight state */
	rt5081_en_bled = RT5081_NONE;


	/* clear charger status */
	is_decrease_voltage = 0;

	return rt5081_disable();
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static void rt5081_work_disable_bled(struct work_struct *data)
{
	fl_dbg("ht work queue callback\n");
	rt5081_disable();
}


static enum hrtimer_restart rt5081_timer_func_bled(struct hrtimer *timer)
{
	schedule_work(&rt5081_work_bled);
	return HRTIMER_NORESTART;
}


int rt5081_bled_timer_start(int channel, ktime_t ktime)
{
	if (channel == RT5081_CHANNEL_BLED)
		hrtimer_start(&rt5081_timer_bled, ktime, HRTIMER_MODE_REL);
	else {
		fl_err("Error channel\n");
		return -1;
	}

	return 0;
}

int rt5081_bled_timer_cancel(int channel)
{
	if (channel == RT5081_CHANNEL_BLED)
		hrtimer_cancel(&rt5081_timer_bled);
	else {
		fl_err("Error channel\n");
		return -1;
	}

	return 0;
}

/******************************************************************************
 * Flashlight operation wrapper function
 *****************************************************************************/
static int rt5081_operate(int channel, int enable)
{
	ktime_t ktime;

	/* setup enable/disable */
	if (channel == RT5081_CHANNEL_BLED) {
		rt5081_en_bled = enable;
		if (rt5081_en_bled)
			{
			if (rt5081_is_torch(rt5081_level_bled))
				rt5081_en_bled = RT5081_ENABLE_FLASH;
			}
	} else {
		fl_err("Error channel\n");
		return -1;
	}

	/* operate flashlight and setup timer */
	if ((rt5081_en_bled != RT5081_NONE) ) {
		if ((rt5081_en_bled == RT5081_DISABLE)) {
			rt5081_disable();
			rt5081_bled_timer_cancel(RT5081_CHANNEL_BLED);
		} else {
			if (rt5081_timeout_ms[RT5081_CHANNEL_BLED]) {
				ktime = ktime_set(
						rt5081_timeout_ms[RT5081_CHANNEL_BLED] / 1000,
						(rt5081_timeout_ms[RT5081_CHANNEL_BLED] % 1000) * 1000000);
				rt5081_bled_timer_start(RT5081_CHANNEL_BLED, ktime);
			}
			rt5081_enable();
		}

		/* clear flashlight state */
		rt5081_en_bled = RT5081_NONE;
	}

	return 0;
}

/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int rt5081_bled_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= RT5081_CHANNEL_NUM) {
		fl_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		fl_dbg("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d. Not use now!\n",
				channel, (int)fl_arg->arg);
		rt5081_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		fl_dbg("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		rt5081_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_SCENARIO:
		fl_dbg("FLASH_IOC_SET_SCENARIO(%d): %d\n",
				channel, (int)fl_arg->arg);
		rt5081_set_scenario(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		fl_dbg("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		rt5081_operate(channel, fl_arg->arg);
		break;

	case FLASH_IOC_IS_CHARGER_READY:
		fl_dbg("FLASH_IOC_IS_CHARGER_READY(%d)\n", channel);
		fl_arg->arg = rt5081_is_charger_ready();
		break;

	default:
		fl_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int rt5081_open(void *pArg)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int rt5081_release(void *pArg)
{
	/* uninit chip and clear usage count */
	mutex_lock(&rt5081_mutex);
	use_count--;
	if (!use_count)
		rt5081_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&rt5081_mutex);

	fl_dbg("Release: %d\n", use_count);

	return 0;
}

static int rt5081_set_driver(void)
{
	int ret = 0;

	/* init chip and set usage count */
	mutex_lock(&rt5081_mutex);
	if (!use_count)
		ret = rt5081_init();
	use_count++;
	mutex_unlock(&rt5081_mutex);

	fl_dbg("Set driver: %d\n", use_count);

	return ret;
}

static ssize_t rt5081_strobe_store(struct flashlight_arg arg)
{
	rt5081_set_driver();
	rt5081_set_scenario(
			FLASHLIGHT_SCENARIO_CAMERA | FLASHLIGHT_SCENARIO_COUPLE);
	rt5081_set_level(arg.channel, arg.level);

	if (arg.level < 0)
		rt5081_operate(arg.channel, RT5081_DISABLE);
	else
		rt5081_operate(arg.channel, RT5081_ENABLE);

	msleep(arg.dur);
	rt5081_set_scenario(
			FLASHLIGHT_SCENARIO_FLASHLIGHT | FLASHLIGHT_SCENARIO_COUPLE);
	rt5081_operate(arg.channel, RT5081_DISABLE);
	rt5081_release(NULL);

	return 0;
}

static struct flashlight_operations rt5081_ops = {
	rt5081_open,
	rt5081_release,
	rt5081_bled_ioctl,
	rt5081_strobe_store,
	rt5081_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int rt5081_probe(struct platform_device *pdev)
{
	fl_dbg("Probe start22.\n");

	/* init work queue */
	INIT_WORK(&rt5081_work_bled, rt5081_work_disable_bled);


	/* init timer */
	hrtimer_init(&rt5081_timer_bled, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rt5081_timer_bled.function = rt5081_timer_func_bled;

	rt5081_timeout_ms[RT5081_CHANNEL_BLED] = 600;


	/* register flashlight operations */
	if (flashlight_dev_register(RT5081_BLED_NAME, &rt5081_ops))
		return -EFAULT;

	/* clear attributes */
	use_count = 0;
	is_decrease_voltage = 0;

	/* get RTK flashlight handler */
	backlight_dev = find_flashlight_by_name(RT_BLED_DEVICE);
	if (!backlight_dev) {
		fl_err("Failed to get ht flashlight device22.\n");
		return -EFAULT;
	}

	/* setup strobe mode timeout */
	if (flashlight_set_strobe_timeout(backlight_dev, 400, 600) < 0)
		fl_err("Failed to set strobe timeout22.\n");

	/* get charger consumer manager */
	flashlight_charger_consumer = charger_manager_get_by_name(&backlight_dev->dev, CHARGER_SUPPLY_NAME);
	if (!flashlight_charger_consumer) {
		fl_err("Failed to get charger manager.\n");
		return -EFAULT;
	}

	fl_dbg("Probe done22.\n");

	return 0;
}

static int rt5081_remove(struct platform_device *pdev)
{
	fl_dbg("Remove start.\n");

	/* flush work queue */
	flush_work(&rt5081_work_bled);


	/* unregister flashlight operations */
	flashlight_dev_unregister(RT5081_BLED_NAME);

	/* clear RTK flashlight device */
	backlight_dev = NULL;


	fl_dbg("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rt5081_of_match[] = {
	{.compatible = RT5081_BLED_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, rt5081_of_match);
#else
static struct platform_device rt5081_platform_device[] = {
	{
		.name = RT5081_BLED_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, rt5081_platform_device);
#endif

static struct platform_driver rt5081_platform_driver = {
	.probe = rt5081_probe,
	.remove = rt5081_remove,
	.driver = {
		.name = RT5081_BLED_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rt5081_of_match,
#endif
	},
};

static int __init flashlight_rt5081_init(void)
{
	int ret;

	fl_dbg("Init start22.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&rt5081_platform_device);
	if (ret) {
		fl_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&rt5081_platform_driver);
	if (ret) {
		fl_err("Failed to register platform driver\n");
		return ret;
	}

	fl_dbg("Init done22.\n");

	return 0;
}

static void __exit flashlight_rt5081_exit(void)
{
	fl_dbg("Exit start22.\n");

	platform_driver_unregister(&rt5081_platform_driver);

	fl_dbg("Exit done22.\n");
}

/* replace module_init() since conflict in kernel init process */
late_initcall(flashlight_rt5081_init);
module_exit(flashlight_rt5081_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("linyimin@tp-link.com.cn>");
MODULE_DESCRIPTION("TP-LINK backlight flash driver");

