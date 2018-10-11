/* BU52013HFV Hall Sensor driver
 *
 * author: liliwen (liliwen@tp-link.com.cn)
 *
 */

#include "hallsen_drv.h"

#include <linux/notifier.h>
//#include <linux/fb.h>

#include <mach/gpio_const.h>
/* [yanlin start] use new interface */
//#include <mt-plat/mt_gpio.h>
#include <mt-plat/mtk_gpio.h>
//#include <linux/irqchip/mt-eic.h>
#include <linux/irqchip/mtk-eic.h>
/* [yanlin end] */

#define		HALLSEN_DEV_NAME			"BU52013HFV"
#define		HALL_INPUTDEV_NAME			"hall_input_device"
#define		DRIVER_VERSION				"1.0.0"

#define		HALLSEN_OUT_PIN				(GPIO24 | 0x80000000)

#define HALLSEN_TAG					"[HALLSEN] "
#define HALLSEN_FUN(f)              printk(KERN_INFO 	HALLSEN_TAG"%s\n", __FUNCTION__)
#define HALLSEN_ERR(fmt, args...)   printk(KERN_ERR  	HALLSEN_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define HALLSEN_LOG(fmt, args...)   printk(KERN_NOTICE	HALLSEN_TAG fmt, ##args)
#define HALLSEN_DBG(fmt, args...)   printk(KERN_ERR 	HALLSEN_TAG fmt, ##args)

static int hallsen_probe(struct platform_device *dev);
static int hallsen_remove(struct platform_device *dev);
static int hallsen_suspend(struct device *device);
static int hallsen_resume(struct device *device);

struct pinctrl *hallsen_pinctrl;
struct pinctrl_state *hall_pins_eint_int;
struct work_struct	hall_eint_work;
static unsigned long long hall_int_top_time;
struct input_dev *hall_idev;
//struct notifier_block hall_notifier;

int hallsen_irq;
//static int unblank_state = 1;
unsigned int hallgpiopin, hallirqdebounce;
static int gpio_in;

static ssize_t show_board_info(struct device_driver *ddri, char *buf)
{
	return sprintf(buf,"type:\t%s\nvendor:\t%s\ndriver_version:\t%s\n",
		HALLSEN_DEV_NAME, "ROHM", DRIVER_VERSION);
}

static ssize_t show_gpio_state(struct device_driver *ddri, char *buf)
{
	/* [yanlin start] use new interface */
	//gpio_in = mt_get_gpio_in(HALLSEN_OUT_PIN);
	gpio_in = gpio_get_value(hallgpiopin);
	/* [yanlin end] */
	input_report_rel(hall_idev, REL_Z, gpio_in+1);
	input_report_rel(hall_idev, REL_Y, 3);
	input_sync(hall_idev);
	return sprintf(buf, "%d\n", gpio_in);
}

static ssize_t show_devnum(struct device_driver *ddri, char *buf)
{
	unsigned int devnum;
	const char *devname = NULL;
	int ret;

	/* [yanlin start] Fix devnum mismatch */
	struct input_handle *handle;

	//devname = dev_name(&hall_idev->dev);
	list_for_each_entry(handle, &hall_idev->h_list, d_node)
		if (strncmp(handle->name, "event", 5) == 0) {
			devname = handle->name;
			break;
		}
	/* [yanlin end] */

	ret = sscanf(devname+5, "%d", &devnum);
	return snprintf(buf, PAGE_SIZE, "%d\n", devnum);
}

/* [yanlin start] for hw info in /sys/kernel */
static ssize_t show_hall_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"type:\t%s\nvendor:\t%s\ndriver_version:\t%s\n",
		HALLSEN_DEV_NAME, "ROHM", DRIVER_VERSION);
}
static DEVICE_ATTR(hall_info, S_IWUSR | S_IRUGO, show_hall_info, NULL);
/* [yanlin end] */

static DRIVER_ATTR(board_info,     S_IWUSR | S_IRUGO, show_board_info,	NULL);
static DRIVER_ATTR(gpio_state,     S_IWUSR | S_IRUGO, show_gpio_state,	NULL);
static DRIVER_ATTR(devnum,         S_IWUSR | S_IRUGO, show_devnum,      NULL);

static struct driver_attribute *hallsen_attr_list[] = {
	&driver_attr_board_info,
	&driver_attr_gpio_state,
	&driver_attr_devnum,
};

static int hallsen_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(hallsen_attr_list)/sizeof(hallsen_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, hallsen_attr_list[idx])))
		{
			HALLSEN_ERR("driver_create_file (%s) = %d\n", hallsen_attr_list[idx]->attr.name, err);
			break;
		}
	}

	/* [yanlin add] for hw info in /sys/kernel */
	err = sysfs_create_file(kernel_kobj, &dev_attr_hall_info.attr);

	return err;
}

static int hallsen_delete_attr(struct device_driver *driver)
{
	int idx , err = 0;
	int num = (int)(sizeof(hallsen_attr_list)/sizeof(hallsen_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, hallsen_attr_list[idx]);
	}

	/* [yanlin add] for hw info in /sys/kernel */
	sysfs_remove_file(kernel_kobj, &dev_attr_hall_info.attr);

	return err;
}

/*static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			unblank_state = 1;
		} else if (*blank == FB_BLANK_POWERDOWN) {
			unblank_state = 0;
		}
	}
	return 0;
}*/

static void hallsen_eint_work(struct work_struct *work)
{
	/* [yanlin start] use new interface */
	//gpio_in = mt_get_gpio_in(HALLSEN_OUT_PIN);
	gpio_in = gpio_get_value(hallgpiopin);
	/* [yanlin end] */
	HALLSEN_LOG("Hall irq is ok, gpio_in state is %d\n", gpio_in);
	input_report_rel(hall_idev, REL_Z, gpio_in+1);
	input_report_rel(hall_idev, REL_Y, 3);
	input_sync(hall_idev);
	enable_irq(hallsen_irq);
}

static void hallsen_eint_func(void)
{
	hall_int_top_time = sched_clock();
	schedule_work(&hall_eint_work);
}

static irqreturn_t hallsen_eint_handler(int irq, void *desc)
{
	disable_irq_nosync(hallsen_irq);
	hallsen_eint_func();

	return IRQ_HANDLED;
}

static inline int hallsen_setup_eint(struct platform_device *hallsen_device)
{
	int ret;
	u32 ints[2] = { 0, 0 };
	struct device_node *node = NULL;

	/*configure to GPIO function, external interrupt */
	hallsen_pinctrl = devm_pinctrl_get(&hallsen_device->dev);
	if (IS_ERR(hallsen_pinctrl)) {
		ret = PTR_ERR(hallsen_pinctrl);
		dev_err(&hallsen_device->dev, "fwq Cannot find hall hallsen_pinctrl!\n");
		return ret;
	}

	hall_pins_eint_int = pinctrl_lookup_state(hallsen_pinctrl, "pin_cfg");
	if (IS_ERR(hall_pins_eint_int)) {
		ret = PTR_ERR(hall_pins_eint_int);
		dev_err(&hallsen_device->dev, "fwq Cannot find hall pinctrl state_eint_int!\n");
		return ret;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek, hall-eint");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "hall");
		hallgpiopin = ints[0];
		hallirqdebounce = ints[1];
		gpio_set_debounce(hallgpiopin, hallirqdebounce);
		pinctrl_select_state(hallsen_pinctrl, hall_pins_eint_int);
		hallsen_irq = irq_of_parse_and_map(node, 0);
		HALLSEN_LOG("hallsen_irq = %d\n", hallsen_irq);
		ret = request_irq(hallsen_irq, hallsen_eint_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "hall-eint", NULL);
		if (ret != 0) {
			HALLSEN_ERR("HALL EINT IRQ LINE NOT AVAILABLE\n");
			return -1;
		} else {
			HALLSEN_LOG("Hall set EINT finished, hall_irq=%d, headsetdebounce=%d\n",
				     hallsen_irq, hallirqdebounce);
			enable_irq_wake(hallsen_irq);
		}
	} else {
		HALLSEN_ERR("[HALL]%s can't find compatible node\n", __func__);
		return -1;
	}
	return 0;
}

static long hallsen_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	return err;
}

static int hallsen_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int hallsen_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations hallsen_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hallsen_unlocked_ioctl,
	.open = hallsen_open,
	.release = hallsen_release,
};

const struct file_operations *hallsen_get_fops(void)
{
	return &hallsen_fops;
}

static const struct of_device_id hallsen_of_match[] = {
	{.compatible = "mediatek,hallsen"},
	{},
};

static const struct dev_pm_ops hallsen_pm_ops = {
	.suspend = hallsen_suspend,
	.resume = hallsen_resume,
};

static struct platform_driver hallsen_driver = {
	.probe = hallsen_probe,
	.remove = hallsen_remove,
	.driver = {
			.name = "hall_sensor",
			.pm = &hallsen_pm_ops,
			.of_match_table = hallsen_of_match,
		   },
};

static int hallsen_probe(struct platform_device *pdev)
{
	int ret = 0;
	HALLSEN_LOG("hallsen_drv probe begin\n");
	ret = hallsen_setup_eint(pdev);
	if (ret) {
		HALLSEN_ERR("setup eint fail!\n");
		return ret;
	}
	device_init_wakeup(&pdev->dev, 1);
	INIT_WORK(&hall_eint_work, hallsen_eint_work);
	hall_idev = input_allocate_device();
	if (NULL == hall_idev) {
		return -ENOMEM;
	}
	hall_idev->name = HALL_INPUTDEV_NAME;
	set_bit(EV_KEY, hall_idev->evbit);
	set_bit(EV_SYN, hall_idev->evbit);
	input_set_capability(hall_idev, EV_REL, REL_Z);
	input_set_capability(hall_idev, EV_REL, REL_Y);
	ret = input_register_device(hall_idev);
	if (ret) {
		HALLSEN_ERR("register input dev fail!\n");
		goto hall_err_setup_irq;
	}
	/* [yanlin start] use new interface */
	//gpio_in = mt_get_gpio_in(HALLSEN_OUT_PIN);
	gpio_in = gpio_get_value(hallgpiopin);
	/* [yanlin end] */

	// hall_notifier.notifier_call = fb_notifier_callback;
	/* ret = fb_register_client(&hall_notifier);
	if (ret) {
		HALLSEN_ERR("register fbno fail!\n");
		return ret;
	} */
	ret = hallsen_create_attr(&hallsen_driver.driver);
	if (ret) {
		HALLSEN_ERR("create attribute err = %d\n", ret);
		goto hall_err_setup_irq;
	}
	HALLSEN_LOG("hallsen_drv probe ok\n");
	return ret;
hall_err_setup_irq:
	input_unregister_device(hall_idev);
	input_free_device(hall_idev);
	return ret;
}

static int hallsen_remove(struct platform_device *pdev)
{
	input_unregister_device(hall_idev);
	if (hallsen_delete_attr(&hallsen_driver.driver))
		HALLSEN_ERR("hallsen_delete_attr fail\n");
	return 0;
}

static int hallsen_suspend(struct device *device)
{				/* wake up */
	return 0;
}

static int hallsen_resume(struct device *device)
{				/* wake up */
	return 0;
}

static int __init hallsen_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&hallsen_driver);
	if (ret)
		HALLSEN_ERR("platform_driver_register error:(%d)\n", ret);
	else
		HALLSEN_LOG("platform_driver_register done!\n");

	HALLSEN_LOG("Hall driver init done!\n");
	return ret;

}

static void __exit hallsen_exit(void)
{
	platform_driver_unregister(&hallsen_driver);

	HALLSEN_LOG("Hall driver exit Done!\n");
}

module_init(hallsen_init);
module_exit(hallsen_exit);

MODULE_AUTHOR("LiLiwen");
MODULE_DESCRIPTION("HALL Sensor BU52013HFV driver");
MODULE_LICENSE("GPL");
