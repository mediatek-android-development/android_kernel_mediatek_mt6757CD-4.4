/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_NXP.c
 *
 * Project:
 * --------
 *    Audio smart pa Function
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include <linux/input.h>	/* BUS_I2C */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>

#define SY_TAG "[TFA] "
#define SY_ERR(fmt, args...)  printk(KERN_ERR SY_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args )
#define SY_FUN(f)    printk(KERN_ERR SY_TAG"%s   %d \n", __FUNCTION__, __LINE__)

#define SY_DEBUG
#if defined(SY_DEBUG)
#define SY_LOG(fmt,args...)  printk(KERN_ERR SY_TAG "%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#else
#define SY_LOG(arg...)
#endif

#define NXPExtSpk_AUDIO_DEVICE 	"sound_NXPExtSpk"
#define TFA98XX_I2C_ADDRESS_L		(0x34)
#define TFA98XX_I2C_ADDRESS_R		(0x36)

/* I2C variable */
static struct i2c_client *NXPExtSpk_i2c_client = NULL;
static struct i2c_client *g_tfaClientLeft = NULL;
static struct i2c_client *g_tfaClientRight = NULL;
static struct pinctrl *tfa_pinctrl;
static struct pinctrl_state *tfa_reset;
static struct pinctrl_state *tfa_enable;

int pre_init = 0;
int smart_init = 0;

//static struct i2c_board_info __initdata i2c_NXPExtSpk = { I2C_BOARD_INFO(NXPExtSpk_AUDIO_DEVICE, (TFA_I2C_ADDRESS_L))};
static struct miscdevice AudDrv_NXPExtSpk_audio_device;
static int  gn_NXPExtSpk_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int  gn_NXPExtSpk_i2c_remove(struct i2c_client *client);


static ssize_t i2cdev_read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
	char *tmp;
	int ret = 0;
    pr_warn("+%s\n", __func__);

	if (count > 8192){
        pr_warn("%s, count > 8192\n", __func__);
		count = 8192;
    }

	if(NXPExtSpk_i2c_client == NULL) {
		pr_err("i2c client is null! return.\n");
		return -1;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
        pr_warn("%s, tmp == NULL\n", __func__);
		return -ENOMEM;
    }

    ret = i2c_master_recv(NXPExtSpk_i2c_client, tmp, count);
	if(ret < 0) {
		pr_err("i2c recv error: %d.\n", ret);
	}
    pr_warn("0x%02x, count: %ld.\n", tmp[0], (unsigned long)count);

	if (ret >= 0)
		ret = copy_to_user(data, tmp, count) ? (-EFAULT) : ret;
	kfree(tmp);
    pr_warn("-%s, ret %d\n", __func__, ret);

	return ret;
}



static ssize_t i2cdev_write(struct file *fp, const char __user *data, size_t count, loff_t *offset)
{
	int ret;
	char *tmp;
    pr_warn("+%s\n", __func__);

	if(NXPExtSpk_i2c_client == NULL) {
		pr_err("i2c client is null! return.\n");
		return -1;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
        pr_warn("%s, tmp == NULL.\n", __func__);
		return -ENOMEM;
    }
	if (copy_from_user(tmp, data, count)) {
		kfree(tmp);
        pr_warn("%s, copy_from_user err.\n", __func__);
		return -EFAULT;
	}

    pr_warn("0x%02x, count: %ld.\n", tmp[0], (unsigned long)count);

	ret = i2c_master_send(NXPExtSpk_i2c_client, tmp, count);
	if(ret < 0) {
		pr_err("i2c send error: %d.\n", ret);
	}
    pr_warn("-%s, ret %d\n", __func__, ret);

	kfree(tmp);
	return ret;
}

static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    pr_warn("+%s cmd = 0x%d arg = 0x%ld.\n", __func__, cmd, arg);
    pr_warn("%s pro = %d, smt = %d.\n", __func__, pre_init, smart_init);

    switch (cmd) {
        case I2C_SLAVE:
        case I2C_SLAVE_FORCE:

            /* NOTE:  devices set up to work with "new style" drivers
             * can't use I2C_SLAVE, even when the device node is not
             * bound to a driver.  Only I2C_SLAVE_FORCE will work.
             *
             * Setting the PEC flag here won't affect kernel drivers,
             * which will be using the i2c_client node registered with
             * the driver model core.  Likewise, when that client has
             * the PEC flag already set, the i2c-dev driver won't see
             * (or use) this setting.
             */
			if(arg == TFA98XX_I2C_ADDRESS_L) {
				NXPExtSpk_i2c_client = g_tfaClientLeft;
			}
			else if(arg == TFA98XX_I2C_ADDRESS_R) {
				NXPExtSpk_i2c_client = g_tfaClientRight;
			}
			else {
				pr_err("Unsupport i2c address: 0x%ld.\n", arg);
				NXPExtSpk_i2c_client = NULL;
			}

            return 0;
        default:
            /* NOTE:  returning a fault code here could cause trouble
             * in buggy userspace code.  Some old kernel bugs returned
             * zero in this case, and userspace code might accidentally
             * have depended on that bug.
             */
            return -ENOTTY;
    }

    return 0;
}

static int i2cdev_open(struct inode *inode, struct file *file)
{
    SY_FUN();
    pr_warn("%s\n", __func__);

    return 0;
}

static int i2cdev_release(struct inode *inode, struct file *file)
{
    SY_FUN();
    pr_warn("%s\n", __func__);

    return 0;
}

static const struct i2c_device_id gn_NXPExtSpk_i2c_id[] = {
    {NXPExtSpk_AUDIO_DEVICE,0 },
    { }
};

static const struct of_device_id pa_of_match[] = {
	{.compatible = "mediatek,tfasmartpa"},
	{},
};

MODULE_DEVICE_TABLE(i2c, gn_NXPExtSpk_i2c_id);


static struct i2c_driver gn_NXPExtSpk_i2c_driver = {
    .probe		= gn_NXPExtSpk_i2c_probe,
    .remove		= gn_NXPExtSpk_i2c_remove,
    .id_table 	= gn_NXPExtSpk_i2c_id,
    .driver	= {
        .name	= NXPExtSpk_AUDIO_DEVICE,
	    .of_match_table = pa_of_match,
    },
};

static int  gn_NXPExtSpk_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    int ret=0;

    SY_FUN();
    pr_warn("%s\n", __func__);
    pr_warn("%s:... enable smartpa gpio ...",__func__);
    tfa_pinctrl = devm_pinctrl_get(&client->dev);
    if(IS_ERR(tfa_pinctrl)) {
                    pr_warn("Cannot find pinctrl....");
                    smart_init = 2;
    } else {
           tfa_reset= pinctrl_lookup_state(tfa_pinctrl,"smartpa_reset");
           tfa_enable= pinctrl_lookup_state(tfa_pinctrl,"smartpa_enable");
           if(IS_ERR(tfa_enable)) {
               pr_warn("%s:lookup error .......",__func__);
               smart_init = 4;
           } else {
               pr_warn("%s:enable smart pa ...",__func__);
               pinctrl_select_state(tfa_pinctrl,tfa_enable);
               msleep(10);
               smart_init = 1;
           }
    }

	if(client == NULL) {
		pr_err("client is null! return.\n");
		return -1;
	}
	pr_info("client->addr: 0x%x.\n", client->addr);

	if(client->addr == TFA98XX_I2C_ADDRESS_L) {	// left device
		g_tfaClientLeft = client;
        pre_init = 1;
	}
	else if(client->addr == TFA98XX_I2C_ADDRESS_R) {	// right device
		g_tfaClientRight = client;
        pre_init = 2;
	}
	else {
		pr_err("Unsupport i2c address: 0x%02x.\n", client->addr);
		return -1;
	}

    // register MISC device
    if(client->addr == TFA98XX_I2C_ADDRESS_L){
        if ((ret = misc_register(&AudDrv_NXPExtSpk_audio_device)))
       {
            printk("AudDrv_probe misc_register Fail:%d \n", ret);
            return ret;
       }
    }
    pr_warn("-%s\n", __func__);

    return 0;
}

static int  gn_NXPExtSpk_i2c_remove(struct i2c_client *client)
{
	i2c_del_driver(&gn_NXPExtSpk_i2c_driver);
    return 0;
}


static const struct file_operations i2cdev_fops = {
    .owner		= THIS_MODULE,
    .llseek		= no_llseek,
    .read		= i2cdev_read,
    .write		= i2cdev_write,
    .unlocked_ioctl	= i2cdev_ioctl,
    .open		= i2cdev_open,
    .release	= i2cdev_release,
};

static struct miscdevice AudDrv_NXPExtSpk_audio_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "i2c_smartpa",
    .fops = &i2cdev_fops,
};


static int gn_NXPExtSpk_probe(struct platform_device *pdev)
{
    SY_FUN();
    pr_warn("%s\n", __func__);


    if(i2c_add_driver(&gn_NXPExtSpk_i2c_driver)){
        SY_ERR("add i2c driver error\n");
		printk("%s:: NXP probe error - ----- \n",__func__);
        return -1;
    }
    pr_warn("-%s\n", __func__);

	return 0;
}

static int gn_NXPExtSpk_remove(struct platform_device *pdev)
{
    SY_ERR(" mtk sound driver remove! \n");

    return 0;
}

static struct platform_driver gn_NXPExtSpk_driver = {
    .probe      = gn_NXPExtSpk_probe,
    .remove     = gn_NXPExtSpk_remove,
    .driver     = {
        .name  ="sound_speaker",
    }
};


static struct platform_device gn_NXPExtSpk_device = {
    .name = "sound_speaker",
    .id = -1,
    .dev = {
    }
};


static __init int gn_NXPExtSpk_init(void)
{
    SY_FUN();
    pr_warn("%s\n", __func__);

    if(platform_driver_register(&gn_NXPExtSpk_driver)){
        SY_ERR("failed to register NXPExtSpk sound driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&gn_NXPExtSpk_device)){
        SY_ERR("failed to register NXPExtSpk sound device\n");
        return -ENODEV;
    }
    pr_warn("-%s\n", __func__);

    return 0;
}

static __exit void gn_NXPExtSpk_exit(void)
{
    SY_FUN();
    platform_driver_unregister(&gn_NXPExtSpk_driver);
    platform_device_unregister(&gn_NXPExtSpk_device);

}

module_init(gn_NXPExtSpk_init);
module_exit(gn_NXPExtSpk_exit);

MODULE_DESCRIPTION("AudDrv_NXP");
MODULE_AUTHOR("customer<customer@customer.com>");
MODULE_LICENSE("GPL");

