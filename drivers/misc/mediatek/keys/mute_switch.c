#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "mute_switch.h"

//#include <upmu_common.h>
#include <linux/timer.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/time.h>

#include <linux/string.h>

#include <linux/notifier.h>
#include <linux/fb.h>

#include <mach/gpio_const.h>
//#include <mt-plat/mt_gpio.h>
#include <mt-plat/mtk_gpio.h>
//#include <linux/irqchip/mt-eic.h>
#include <linux/irqchip/mtk-eic.h>

#define MUTE_SWITCH_INPUTDEV_NAME		"mute_switch_input_dev"
/* [liguanxiong] Use linux common gpio api */
#define	DEFAULT_MUTE_SWITCH_OUT_PIN			GPIO0

#define MUTE_SWITCH_TAG					"[MUTE_SWITCH] "
#define MUTE_SWITCH_FUN(f)              printk(KERN_INFO 	MUTE_SWITCH_TAG"%s\n", __FUNCTION__)
#define MUTE_SWITCH_ERR(fmt, args...)   printk(KERN_ERR  	MUTE_SWITCH_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MUTE_SWITCH_LOG(fmt, args...)   printk(KERN_NOTICE	MUTE_SWITCH_TAG fmt, ##args)
#define MUTE_SWITCH_DBG(fmt, args...)   printk(KERN_ERR 	MUTE_SWITCH_TAG fmt, ##args)


static unsigned long long mute_switch_int_top_time;

static 	int mute_switch_irq;
static	struct delayed_work eint_delay_work;
static struct input_dev *mute_switch_idev;
static int gpio_state = 0;
/* [liguanxiong start] */
static unsigned int mute_switch_gpio = DEFAULT_MUTE_SWITCH_OUT_PIN;
static int irq_trigger_type = IRQF_TRIGGER_HIGH;
/* [liguanxiong end] */

static void mute_switch_eint_delay_work(struct work_struct *work)
{
	/* [liguanxiong] Use linux common gpio api */
	gpio_state = 1 - gpio_get_value(mute_switch_gpio);
	input_report_rel(mute_switch_idev, REL_Z, gpio_state+1);
	input_report_rel(mute_switch_idev, REL_Y, 3);
	input_sync(mute_switch_idev);
	MUTE_SWITCH_ERR("mute_switch state change, now is %d\n\n", gpio_state);
}

#define SWITCH_DEBOUNCE_TIME		10    /* 10ms */
static void mute_switch_eint_func(void)
{
	mute_switch_int_top_time = sched_clock();
	/* [liguanxiong edit] add key debounce */
	cancel_delayed_work(&eint_delay_work);
	schedule_delayed_work(&eint_delay_work, msecs_to_jiffies(SWITCH_DEBOUNCE_TIME));
	/* [liguanxiong start] IRQ use level trigger type */
	if (irq_trigger_type == IRQF_TRIGGER_HIGH) {
		irq_trigger_type = IRQF_TRIGGER_LOW;
	} else {
		irq_trigger_type = IRQF_TRIGGER_HIGH;
	}
	irq_set_irq_type(mute_switch_irq, irq_trigger_type);
	/* [liguanxiong end] */
	enable_irq(mute_switch_irq);
}

static irqreturn_t mute_switch_eint_handler(int irq, void *desc)
{
	disable_irq_nosync(mute_switch_irq);
	mute_switch_eint_func();

	return IRQ_HANDLED;
}

static  int mute_switch_setup_eint(struct platform_device *pdev)
{
	int ret;
	u32 ints[2] = { 0, 0 };
	struct device_node *irq_node = NULL;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_cfg;

	/*configure to GPIO function, external interrupt */
	pinctrl= devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(&pdev->dev, "Cannot find mute_switch pinctrl!\n");
		return ret;
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		dev_err(&pdev->dev, "fwq Cannot find mute_switch pinctrl state!\n");
		return ret;
	}

	irq_node = of_find_compatible_node(NULL, NULL, "mediatek, mute-eint");
	if (irq_node) {
		of_property_read_u32_array(irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "mute_switch");
		/* [liguanxiong] Get mute_switch gpio form dts */
		mute_switch_gpio = ints[0];
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		mute_switch_irq = irq_of_parse_and_map(irq_node, 0);
		MUTE_SWITCH_LOG("mute_switch_irq = %d\n", mute_switch_irq);
		/* [liguanxiong] IRQ use level trigger type */
		ret = request_irq(mute_switch_irq, mute_switch_eint_handler,IRQF_TRIGGER_HIGH, "mute_switch-eint", NULL);
		if (ret != 0) {
			MUTE_SWITCH_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}

		enable_irq_wake(mute_switch_irq);
	}else {
		MUTE_SWITCH_ERR("Can't find compatible node\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************************/
static ssize_t mute_switch_state_show(struct device_driver *ddri, char *buf)
{
	/* [liguanxiong] Use linux common gpio api */
	gpio_state = 1 - gpio_get_value(mute_switch_gpio);
	input_report_rel(mute_switch_idev, REL_Z, gpio_state+1);
	input_report_rel(mute_switch_idev, REL_Y, 3);
	input_sync(mute_switch_idev);
	MUTE_SWITCH_ERR("report the current state: %d!\n", gpio_state);
	return snprintf(buf, PAGE_SIZE, "%d\n", gpio_state);
}

static ssize_t mute_switch_devnum_show(struct device_driver *ddri, char *buf)
{
	unsigned int devnum;
	const char *devname = NULL;
	int ret;

	/* [yanlin start] Fix devnum mismatch */
	struct input_handle *handle;

	//devname = dev_name(&mute_switch_idev->dev);
	list_for_each_entry(handle, &mute_switch_idev->h_list, d_node)
		if (strncmp(handle->name, "event", 5) == 0) {
			devname = handle->name;
			break;
		}
	/* [yanlin end] */

	ret = sscanf(devname+5, "%d", &devnum);
	return snprintf(buf, PAGE_SIZE, "%d\n", devnum);
}

static ssize_t mute_switch_setdelay_store(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!strncmp(buf, "1", 1)) {
		/* [liguanxiong] Use linux common gpio api */
		gpio_state = 1 - gpio_get_value(mute_switch_gpio);
		input_report_rel(mute_switch_idev, REL_Z, gpio_state+1);
		input_report_rel(mute_switch_idev, REL_Y, 3);
		input_sync(mute_switch_idev);
		MUTE_SWITCH_ERR("report the current state: %d!\n", gpio_state);
	}

	MUTE_SWITCH_ERR("mute_switch_setdelay_store done!\n");
	return count;
}

static DRIVER_ATTR(state, S_IWUSR | S_IRUGO, mute_switch_state_show, NULL);
static DRIVER_ATTR(devnum, S_IWUSR | S_IRUGO, mute_switch_devnum_show, NULL);
static DRIVER_ATTR(setdelay,S_IWUSR | S_IRUGO, NULL,  mute_switch_setdelay_store);

static struct driver_attribute *mute_switch_attr_list[] = {
    &driver_attr_state,
	&driver_attr_devnum,
	&driver_attr_setdelay,
};

static int mute_switch_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mute_switch_attr_list)/sizeof(mute_switch_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mute_switch_attr_list[idx])))
		{
			MUTE_SWITCH_ERR("driver_create_file (%s) = %d\n",mute_switch_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*******************************************************************************************/
static int mute_switch_probe(struct platform_device *pdev);
static int mute_switch_remove(struct platform_device *pdev);

#ifdef CONFIG_OF
static const struct of_device_id mute_switch_of_match[] = {
	{.compatible = "mediatek,mute-switch",},
	{},
};
#endif

static struct platform_driver mute_switch_driver = {
	.probe	  = mute_switch_probe,
	.remove	 = mute_switch_remove,
	.driver = {

		.name  = "mute-switch",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mute_switch_of_match,
#endif
	}
};

static int mute_switch_probe(struct platform_device *pdev)
{
	int ret = -1;

	//mutex_init(&mute_switch_mutex);

	INIT_DELAYED_WORK(&eint_delay_work, mute_switch_eint_delay_work);
	mute_switch_idev = input_allocate_device();
	if (NULL == mute_switch_idev) {
		return -ENOMEM;
	}
	mute_switch_idev->name = MUTE_SWITCH_INPUTDEV_NAME;

	set_bit(EV_REL, mute_switch_idev->evbit);
	set_bit(EV_SYN, mute_switch_idev->evbit);
	input_set_capability(mute_switch_idev, EV_REL, REL_Z);
	input_set_capability(mute_switch_idev, EV_REL, REL_Y);
	ret = input_register_device(mute_switch_idev);
	if (ret) {
		MUTE_SWITCH_ERR("register input dev fail!\n");
		return ret;
	}
	ret = mute_switch_setup_eint(pdev);
	if(ret){
		MUTE_SWITCH_ERR("lgx: mute_switch_setup_eint fail!\n");
		goto err_setup_irq;
	}
	/* [liguanxiong] Use linux common gpio api */
	gpio_state = 1 - gpio_get_value(mute_switch_gpio);
	ret = mute_switch_create_attr(&mute_switch_driver.driver);
	if (ret) {
		MUTE_SWITCH_ERR("mute_switch_create_attr fail!\n");
		goto err_setup_irq;
	}
	return ret;
err_setup_irq:
	input_unregister_device(mute_switch_idev);
	input_free_device(mute_switch_idev);
	return ret;

}

static int mute_switch_remove(struct platform_device *pdev)
{
	MUTE_SWITCH_LOG("mute_switch_remove\n");
	return 0;
}


static int __init mute_switch_init(void)
{
	int ret = -1;
	ret = platform_driver_register(&mute_switch_driver);
	if (ret)
	{
		MUTE_SWITCH_LOG("%s: failed to register MUTE_SWITCH driver\n", __func__);
	}
	else
	{
		MUTE_SWITCH_LOG("%s: register MUTE_SWITCH driver success\n", __func__);
	}
	return ret;
}

static void __exit mute_switch_exit(void)
{
	platform_driver_unregister(&mute_switch_driver);
}

module_init(mute_switch_init);
module_exit(mute_switch_exit);

MODULE_AUTHOR("Li Guanxiong");
MODULE_DESCRIPTION("MUTE_SWITCH DRIVER");
MODULE_LICENSE("GPL");
