/* Copyright (C) 1996-2016, TP-LINK TECHNOLOGIES CO., LTD.
 *
 * File name: gpio-board-config.c
 *
 * Description: This drvier reads the value of GPIO-116. TP902/TP903 has two
 *              configuration: A and C. GPIO-116's output can be used to judge
 *              the configuration that TP902/903 uses.
 *
 * Author: Li Guanxiong
 *
 * Email: liguanxiong@tp-link.com.cn
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <mach/gpio_const.h>
#include <mt-plat/mtk_gpio.h>
#include <linux/sysfs.h>

#define BOARD_CONFIG_GPIO_DRIVER_NAME  "mediatek,board_config"

static struct kobject *board_config_kobj;
static int probe_success = 0;

/* [liguanxiong] define the default board config gpio here */
#define DEFAULT_BOARD_CONFIG_GPIO	115
static unsigned board_config_gpio = DEFAULT_BOARD_CONFIG_GPIO;

int get_board_config(void)
{
	if (probe_success == 0)
	{
		/* return error if gipo-board-config driver not  readly */
		return -1;
	}
	else
	{
		/* [liguanxiong] use common linux gpio api */
		return gpio_get_value(board_config_gpio);
	}
}
EXPORT_SYMBOL(get_board_config);

static ssize_t board_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* [liguanxiong] use common linux gpio api */
	return scnprintf(buf, PAGE_SIZE, "%d", gpio_get_value(board_config_gpio));
}

static DEVICE_ATTR(board_config, S_IRUGO, board_config_show, NULL);

static struct attribute *board_config_attrs[] = {
	&dev_attr_board_config.attr,
	NULL,
};

static struct attribute_group board_config_attr_group = {
	.attrs = board_config_attrs,
};

static struct of_device_id board_config_gpio_of_match[] = {
	{.compatible = BOARD_CONFIG_GPIO_DRIVER_NAME,},
	{},
};

int board_config_gpio_probe(struct platform_device *pdev)
{
	int ret;

	/* [liguanxiong] parse the gpio pin number from dts */
	ret = of_property_read_u32(pdev->dev.of_node, "gpio_config",&board_config_gpio);
	if (ret) {
		pr_err("%s: can not parse gpio from dts, use default\n", __func__);
		board_config_gpio = DEFAULT_BOARD_CONFIG_GPIO;
	}

#if 0	// [liguanxiong start] do not apply pinctrl here for special GPIO
	struct pinctrl *board_config_pinctrl;
	struct pinctrl_state *gpio_state_default;

	board_config_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(board_config_pinctrl)) {
		pr_err("%s: failed to get pinctrl\n", __func__);
		return PTR_ERR(board_config_pinctrl);
	}

	printk("LGX: board_config_gpio_probe\n");
	gpio_state_default = pinctrl_lookup_state(board_config_pinctrl,
			"board_config_default");
	if (IS_ERR(gpio_state_default)) {
		pr_err("%s: can not get default pinstate\n", __func__);
		return -EINVAL;
	}

	ret = pinctrl_select_state(board_config_pinctrl,
			gpio_state_default);
	if (ret) {
		pr_err("%s: set state failed!\n", __func__);
		return -EINVAL;
	}
#endif	// [liguanxiong end] do not apply pinctrl here for special GPIO

	/* [liguanxiong] requeset gpio before use it*/
	ret = gpio_request(board_config_gpio, "board_config_gpio");
	if (ret) {
		pr_err("%s: set state failed!\n", __func__);
		return -EINVAL;
	}
	board_config_kobj = kobject_create_and_add("board_config",
			NULL);
	if (!board_config_kobj) {
		pr_err("%s: Fail to create board_config_kobj\n",
				__func__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(board_config_kobj,
			&board_config_attr_group);
	if (ret) {
		kobject_put(board_config_kobj);
	}
	probe_success = 1;

	return ret;
}

int board_config_gpio_remove(struct platform_device *pdev)
{
	if (board_config_kobj) {
		kobject_put(board_config_kobj);
	}

	return 0;
}

static struct platform_driver board_config_gpio_driver = {
	.probe = board_config_gpio_probe,
	.remove = board_config_gpio_remove,
	.driver = {
		.name = BOARD_CONFIG_GPIO_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = board_config_gpio_of_match,
	}
};

static int __init board_config_gpio_init(void)
{
	return platform_driver_register(&board_config_gpio_driver);
}

static void __exit board_config_gpio_exit(void)
{
	return platform_driver_unregister(&board_config_gpio_driver);
}

module_init(board_config_gpio_init);
module_exit(board_config_gpio_exit);

MODULE_DESCRIPTION("TP908 BOARD CONFIG GPIO DRIVER");
MODULE_LICENSE("GPL v2");
