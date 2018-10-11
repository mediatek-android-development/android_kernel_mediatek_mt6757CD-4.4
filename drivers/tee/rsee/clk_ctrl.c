#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>

#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#endif				/* !defined(CONFIG_MTK_CLKMGR) */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#endif
/*#include <mach/irqs.h>*/
#include "clk_ctrl.h"
//#include "rsee_clk_hal.h"
/*#include <mach/mt_gpio.h>*/

#define rsee_dbg printk

#if defined(CONFIG_MTK_CLKMGR)
 /* mt_clkmgr.h will be removed after CCF porting is finished. */
#include <mach/mt_clkmgr.h>
#endif				/* defined(CONFIG_MTK_CLKMGR) */

#if (defined(CONFIG_MTK_FPGA))
#define  CONFIG_MT_SPI_FPGA_ENABLE
#endif
/*auto select transfer mode*/
/*#define SPI_AUTO_SELECT_MODE*/
#ifdef SPI_AUTO_SELECT_MODE
#define SPI_DATA_SIZE 16
#endif

/*open base log out*/
/*#define SPI_DEBUG*/
#define SPI_DEBUG
/*open verbose log out*/
/*#define SPI_VERBOSE*/
#define SPI_VERBOSE

#define IDLE 0
#define INPROGRESS 1
#define PAUSED 2

#define PACKET_SIZE 0x400

#define SPI_FIFO_SIZE 32
/*	#define FIFO_TX 0
	#define FIFO_RX 1 */
enum spi_fifo {
	FIFO_TX,
	FIFO_RX,
	FIFO_ALL
};

#define INVALID_DMA_ADDRESS 0xffffffff

struct rsee_clk_t {
	struct platform_device *pdev;

	char *name;
	int enabled;

	spinlock_t status_lock;
#if !defined(CONFIG_MTK_CLKMGR)
	struct clk *clk_main;	/* main clock for spi bus */
#endif				/* !defined(CONFIG_MTK_CLKMGR) */
};

// temply saveed in globle
struct rsee_clk_t *g_spi0_tmp;

static int enable_clk(struct rsee_clk_t *ms)
{
    int ret = -1;
#if (!defined(CONFIG_MT_SPI_FPGA_ENABLE))
#if defined(CONFIG_MTK_CLKMGR)
	enable_clock(MT_CG_PERI_SPI0, "spi");
#else

/*	clk_prepare_enable(ms->clk_main); */

	/*
	 * prepare the clock source
	 */
	if(!ms) return -1;
	ret = clk_prepare(ms->clk_main);
//	rsee_dbg(" %s %d, %x\n", __func__, __LINE__, ret);

	ret = clk_enable(ms->clk_main);

//	rsee_dbg(" %s %d, %x\n", __func__, __LINE__, ret);
#endif
#endif
    return ret;
}

static int rsee_clk_enable_clk(struct rsee_clk_t *ms)
{
	return enable_clk(ms);
}

static void disable_clk(struct rsee_clk_t *ms)
{

#if (!defined(CONFIG_MT_SPI_FPGA_ENABLE))
#if defined(CONFIG_MTK_CLKMGR)
	disable_clock(MT_CG_PERI_SPI0, "spi");
#else
/*	clk_disable_unprepare(ms->clk_main); */
	clk_disable(ms->clk_main);
#endif
#endif

}

static void rsee_clk_disable_clk(struct rsee_clk_t *ms)
{
	disable_clk(ms);
}

static ssize_t clk_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct rsee_clk_t *ms = platform_get_drvdata(pdev);
	int enable;
	int old_status = ms->enabled;

	if (!strncmp(buf, "1", 1))
	{
		enable = 1;
	}
	else if (!strncmp(buf, "0", 1))
	{
		enable = 0;
	}
	else
	{
		return 0;
	}

	if (old_status == enable)
	{
		return 0;
	}

	if (enable)
	{
		rsee_clk_enable_clk(ms);
	}
	else
	{
		rsee_clk_disable_clk(ms);
	}

	spin_lock(&ms->status_lock);
	ms->enabled = enable;
	spin_unlock(&ms->status_lock);

	return count;
}

static ssize_t clk_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct rsee_clk_t *ms = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ms->enabled);
}

static DEVICE_ATTR(clk_enable, 0600, clk_enable_show, clk_enable_store);

static struct device_attribute *spi_attribute[] = {
	&dev_attr_clk_enable,
};

static int rsee_clk_create_attribute(struct device *dev)
{
	int num, idx;
	int err = 0;

	num = (int)(sizeof(spi_attribute) / sizeof(spi_attribute[0]));
	for (idx = 0; idx < num; idx++) {
		err = device_create_file(dev, spi_attribute[idx]);
		if (err)
			break;
	}
	return err;
}

static void rsee_clk_remove_attribute(struct device *dev)
{
	int num, idx;

	num = (int)(sizeof(spi_attribute) / sizeof(spi_attribute[0]));
	for (idx = 0; idx < num; idx++)
		device_remove_file(dev, spi_attribute[idx]);
}



static int __init rsee_clk_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct rsee_clk_t *ms;

	rsee_dbg(" %s %d, \n", __func__, __LINE__);

	ms = (struct rsee_clk_t *)kzalloc(sizeof(*ms), GFP_KERNEL);
	if (!ms) {
		dev_err(&pdev->dev, "memory alloc failed.\n");
		return -ENOMEM;
	}

	rsee_dbg(" %s %d, \n", __func__, __LINE__);
	platform_set_drvdata(pdev, ms);
	ms->pdev = pdev;

#ifdef CONFIG_OF
	ret = of_property_read_string(pdev->dev.of_node, "clkname", &ms->name);
	if (ret) {
		dev_err(&pdev->dev, "of_property_read_string  failed, node %s\n", pdev->dev.of_node->name);
		ret = -ENODEV;
		goto out_free;
	}

	rsee_dbg(" %s %d, \n", __func__, __LINE__);

	if (pdev->dev.of_node) {
#if !defined(CONFIG_MTK_CLKMGR)
		ms->clk_main = devm_clk_get(&pdev->dev, ms->name);
		if (IS_ERR(ms->clk_main)) {
			dev_err(&pdev->dev, "cannot get %s clock or dma clock. main clk err : %ld .\n",
				PTR_ERR(ms->clk_main));
			ret =  PTR_ERR(ms->clk_main);
			goto out_free;
		}
#endif
	}
#endif

	rsee_dbg(" %s %d, \n", __func__, __LINE__);
	rsee_clk_create_attribute(&pdev->dev);
	spin_lock_init(&ms->status_lock);
	ms->enabled = 0;

	g_spi0_tmp = ms;
	goto out;

 out_free:
	kfree(ms);
 out:
	return ret;
}

static int __exit rsee_clk_remove(struct platform_device *pdev)
{
	struct rsee_clk_t *ms = platform_get_drvdata(pdev);

	rsee_clk_remove_attribute(&pdev->dev);
	kfree(ms);

	return 0;
}


static const struct of_device_id rsee_clk_of_match[] = {
	{.compatible = "rsee,peri-clk",},
	{},
};

MODULE_DEVICE_TABLE(of, rsee_clk_of_match);

struct platform_driver rsee_clk_driver = {
	.driver = {
		   .name = "rsee-clk",
		   .owner = THIS_MODULE,
		   .of_match_table = rsee_clk_of_match,
		   },
	.probe = rsee_clk_probe,
	.remove = __exit_p(rsee_clk_remove),
};

static int __init rsee_clk_init(void)
{
	int ret;
	struct device_node *node;
	rsee_dbg(" %s %d, \n", __func__, __LINE__);
	for_each_matching_node(node, rsee_clk_of_match)
	{
		rsee_dbg(" %s %d, create node %s,\n", __func__, __LINE__, node->name);
		of_platform_device_create(node, NULL, NULL);
	}

	ret = platform_driver_register(&rsee_clk_driver);
	rsee_dbg(" %s %d, \n", __func__, __LINE__);
	return ret;
}

static void __init rsee_clk_exit(void)
{
	platform_driver_unregister(&rsee_clk_driver);
}

int rsee_spi_clk_enable(void)
{
	rsee_clk_enable_clk(g_spi0_tmp);
}
int rsee_spi_clk_disable(void)
{
	rsee_clk_disable_clk(g_spi0_tmp);
    return 0;
}

subsys_initcall(rsee_clk_init);
module_exit(rsee_clk_exit);

MODULE_DESCRIPTION("rsee Clock Controller driver");
MODULE_AUTHOR("Hyne Zhu <zhuhy@rongcard.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("rsee: peri-clk");
