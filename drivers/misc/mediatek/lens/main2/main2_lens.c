/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*
 * MAIN2 AF voice coil motor driver
 *
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

/* Add by meizu BSP hudong@meizu.com */
#ifdef CONFIG_MEIZU_BSP
#include <linux/meizu-sys.h>
#endif
/*Add end */

#include "lens_info.h"
#include "lens_list.h"

#define AF_DRVNAME "MAIN2AF"

#if defined(CONFIG_MTK_LEGACY)
#define I2C_CONFIG_SETTING 1
#elif defined(CONFIG_OF)
#define I2C_CONFIG_SETTING 2 /* device tree */
#else

#define I2C_CONFIG_SETTING 1
#endif


#if I2C_CONFIG_SETTING == 1
#define LENS_I2C_BUSNUM 0
#define I2C_REGISTER_ID            0x28
#endif

#define PLATFORM_DRIVER_NAME "lens_actuator_main2_af"
#define AF_DRIVER_CLASS_NAME "actuatordrv_main2_af"


#if I2C_CONFIG_SETTING == 1
static struct i2c_board_info kd_lens_dev __initdata = {
	I2C_BOARD_INFO(AF_DRVNAME, I2C_REGISTER_ID)
};
#endif

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static stAF_DrvList g_stAF_DrvList[MAX_NUM_OF_LENS] = {
	#ifdef CONFIG_MTK_LENS_LC898212XDAF_SUPPORT
	{1, AFDRV_LC898212XDAF_F, LC898212XDAF_F_SetI2Cclient, LC898212XDAF_F_Ioctl, LC898212XDAF_F_Release},
	#endif
	#ifdef CONFIG_MTK_LENS_BU64748AF_SUPPORT
	{1, AFDRV_BU64748AF, bu64748af_main2_SetI2Cclient, bu64748af_main2_Ioctl, bu64748af_main2_Release},
	#endif
	#ifdef CONFIG_MTK_LENS_AK7371AF_SUPPORT
	{1, AFDRV_AK7371AF, AK7371AF_MAIN2_SetI2Cclient, AK7371AF_MAIN2_Ioctl, AK7371AF_MAIN2_Release},
	#endif
};

static stAF_DrvList *g_pstAF_CurDrv;

static spinlock_t g_AF_SpinLock;

static int g_s4AF_Opened;

static struct i2c_client *g_pstAF_I2Cclient;

static dev_t g_AF_devno;
static struct cdev *g_pAF_CharDrv;
static struct class *actuator_class;

/* Add by meizu BSP hudong@meizu.com */
#ifdef CONFIG_MEIZU_BSP
extern struct vcm_factory_fops ak7371_main2_fops;
extern struct vcm_factory_fops bu64748_main2_fops;
static struct vcm_factory_fops *af_fops = NULL;

extern u8 *imx386_primax_otp_buf;
extern u8 *imx386_sunny_otp_buf;
static int enable_status;
#endif

static long AF_SetMotorName(__user stAF_MotorName * pstMotorName)
{
	long i4RetValue = -1;
	int i;
	stAF_MotorName stMotorName;

	if (copy_from_user(&stMotorName , pstMotorName, sizeof(stAF_MotorName)))
		LOG_INF("copy to user failed when getting motor information\n");

	LOG_INF("Set Motor Name : %s\n", stMotorName.uMotorName);

	for (i = 0; i < MAX_NUM_OF_LENS; i++) {
		if (g_stAF_DrvList[i].uEnable != 1)
			break;

		LOG_INF("Search Motor Name : %s\n", g_stAF_DrvList[i].uDrvName);
		if (strcmp(stMotorName.uMotorName, g_stAF_DrvList[i].uDrvName) == 0) {
			g_pstAF_CurDrv = &g_stAF_DrvList[i];
			i4RetValue = g_pstAF_CurDrv->pAF_SetI2Cclient(g_pstAF_I2Cclient, &g_AF_SpinLock, &g_s4AF_Opened);
			break;
		}
	}
	return i4RetValue;
}

/* ////////////////////////////////////////////////////////////// */
static long AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_S_SETDRVNAME:
		i4RetValue = AF_SetMotorName((__user stAF_MotorName *)(a_u4Param));
		break;

	default:
		if (g_pstAF_CurDrv)
			i4RetValue = g_pstAF_CurDrv->pAF_Ioctl(a_pstFile, a_u4Command, a_u4Param);
		break;
	}

	return i4RetValue;
}

#ifdef CONFIG_COMPAT
static long AF_Ioctl_Compat(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	i4RetValue = AF_Ioctl(a_pstFile, a_u4Command, (unsigned long)compat_ptr(a_u4Param));

	return i4RetValue;
}
#endif

/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
/* 3.Update f_op pointer. */
/* 4.Fill data structures into private_data */
/* CAM_RESET */
static int AF_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (g_s4AF_Opened) {
		LOG_INF("The device is opened\n");
		return -EBUSY;
	}

	spin_lock(&g_AF_SpinLock);
	g_s4AF_Opened = 1;
	spin_unlock(&g_AF_SpinLock);

	LOG_INF("End\n");

	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (g_pstAF_CurDrv) {
		g_pstAF_CurDrv->pAF_Release(a_pstInode, a_pstFile);
		g_pstAF_CurDrv = NULL;
	} else {
		spin_lock(&g_AF_SpinLock);
		g_s4AF_Opened = 0;
		spin_unlock(&g_AF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

/* Add by meizu BSP hudong@meizu.com */
#ifdef CONFIG_MEIZU_BSP
static int set_pos;
static int max_diff_pos;
static int af_driver_id;
#define AK7371_AF_ID	0x7371
#define AK7374_AF_ID	0x7374
#define BU64748_AF_ID	0x64748
static ssize_t af_pos_show(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	char *p = buf;
	int pos;
	int ret = 0;
	if (enable_status < 0) {
		p += sprintf(p, "result=fail, enable_status:%d.\n", enable_status);
	}else if ((set_pos < 0) || (set_pos > 1023)) {
		p += sprintf(p, "result=fail, invalid af move position.\n");
	} else {
		if (af_fops != NULL) {
			ret = af_fops->vcm_pos_get(g_pstAF_I2Cclient, &pos);

			p += sprintf(p, "current_af_position=%d\n", pos);

			/* check whether af moveto is ok */
			switch (af_driver_id) {
				case AK7371_AF_ID:
				case AK7374_AF_ID:
					if ((ret>=0) && (pos >= (set_pos - max_diff_pos)) && (pos <= (set_pos + max_diff_pos)))
						p += sprintf(p, "result=pass\n");
					else
						p += sprintf(p, "result=fail\n");
					break;
				case BU64748_AF_ID:
				default:
					/* pos type is signed short */
					if ((ret>=0) && ((pos <= max_diff_pos) || (pos >= (0xFFFF - max_diff_pos))))
						p += sprintf(p, "result=pass\n");
					else
						p += sprintf(p, "result=fail\n");
					break;
			}
		} else {
			p += sprintf(p, "result=fail, invalid af pos interface, please check af enable.\n");
		}
	}

	return (p - buf);
}

static ssize_t af_pos_store(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{
	int pos, ret = 0;
	sscanf(buf, "%d\n", &pos);
	set_pos = pos;

	if ((pos < 0) || (pos > 1023)) {
		LOG_INF("%s: invalid af moveto position.\n", __func__);
		return -EINVAL;
	}

	if (af_fops != NULL) {
		ret = af_fops->vcm_pos_set(g_pstAF_I2Cclient, pos);
		if (ret < 0) {
			LOG_INF("%s: set af pos failed.\n", __func__);
			return ret;
		}
	} else {
		LOG_INF("%s: invalid af_fops.\n", __func__);
	}

	/* wait 33ms for af stable */
	msleep(33);

	return count;
}

static ssize_t af_enable_show(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	return 1;
}

static ssize_t af_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{
	int enable;
	u8 *otp_buf = NULL;
	sscanf(buf, "%d\n", &enable);

	if (imx386_primax_otp_buf != NULL){
		otp_buf = imx386_primax_otp_buf;
	} else if (imx386_sunny_otp_buf != NULL) {
		otp_buf = imx386_sunny_otp_buf;
	}else
		otp_buf = NULL;

	if (otp_buf == NULL) {
		LOG_INF("otp data is NULL!\n");
		return count;
	}

	switch (otp_buf[20]) {
		case 1:
		case 4:
			af_fops = &ak7371_main2_fops;
			max_diff_pos = 8;
			af_driver_id = AK7371_AF_ID;
			break;
		case 2:
			af_fops = &bu64748_main2_fops;
			max_diff_pos = 200;
			af_driver_id = BU64748_AF_ID;
			break;
		default:
			af_fops = &bu64748_main2_fops;
			max_diff_pos = 200;
			af_driver_id = BU64748_AF_ID;
			break;
	}

	if (af_fops != NULL) {
		if (enable) {
			enable_status = af_fops->vcm_enable(g_pstAF_I2Cclient, 1);
		} else {
			enable_status = af_fops->vcm_enable(g_pstAF_I2Cclient, 0);
		}
	}

	return count;
}

static struct device_attribute dev_attr_af_pos = {

	.attr = {.name = "af_pos", .mode = 0644},
	.show = af_pos_show,
	.store= af_pos_store,
};

static struct device_attribute dev_attr_af_enable = {

	.attr = {.name = "af_enable", .mode = 0644},
	.show = af_enable_show,
	.store= af_enable_store,
};
#endif
/* Add end */

static const struct file_operations g_stAF_fops = {
	.owner = THIS_MODULE,
	.open = AF_Open,
	.release = AF_Release,
	.unlocked_ioctl = AF_Ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = AF_Ioctl_Compat,
#endif
};

/* Add by meizu BSP hudong@meizu.com */
#ifdef CONFIG_MEIZU_BSP
static struct device *vcm_device = NULL;
#endif
/* Add end */

static inline int Register_AF_CharDrv(void)
{
	struct device *vcm_device = NULL;

	LOG_INF("Start\n");

	/* Allocate char driver no. */
	if (alloc_chrdev_region(&g_AF_devno, 0, 1, AF_DRVNAME)) {
		LOG_INF("Allocate device no failed\n");

		return -EAGAIN;
	}
	/* Allocate driver */
	g_pAF_CharDrv = cdev_alloc();

	if (NULL == g_pAF_CharDrv) {
		unregister_chrdev_region(g_AF_devno, 1);

		LOG_INF("Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pAF_CharDrv, &g_stAF_fops);

	g_pAF_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pAF_CharDrv, g_AF_devno, 1)) {
		LOG_INF("Attatch file operation failed\n");

		unregister_chrdev_region(g_AF_devno, 1);

		return -EAGAIN;
	}

	actuator_class = class_create(THIS_MODULE, AF_DRIVER_CLASS_NAME);
	if (IS_ERR(actuator_class)) {
		int ret = PTR_ERR(actuator_class);

		LOG_INF("Unable to create class, err = %d\n", ret);
		return ret;
	}

	vcm_device = device_create(actuator_class, NULL, g_AF_devno, NULL, AF_DRVNAME);

	if (NULL == vcm_device)
		return -EIO;

/* Add by meizu BSP hudong@meizu.com */
#ifdef CONFIG_MEIZU_BSP
	device_create_file(vcm_device, &dev_attr_af_enable);
	device_create_file(vcm_device, &dev_attr_af_pos);
	meizu_sysfslink_register_name(vcm_device, "cam2_af");
#endif
/* Add end */

	LOG_INF("End\n");
	return 0;
}

static inline void Unregister_AF_CharDrv(void)
{
	LOG_INF("Start\n");
/* Add by meizu BSP hudong@meizu.com */
#ifdef CONFIG_MEIZU_BSP
	device_remove_file(vcm_device, &dev_attr_af_enable);
	device_remove_file(vcm_device, &dev_attr_af_pos);
#endif
/* Add end */
	/* Release char driver */
	cdev_del(g_pAF_CharDrv);

	unregister_chrdev_region(g_AF_devno, 1);

	device_destroy(actuator_class, g_AF_devno);

	class_destroy(actuator_class);

	LOG_INF("End\n");
}

/* //////////////////////////////////////////////////////////////////// */

static int AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id AF_i2c_id[] = { {AF_DRVNAME, 0}, {} };

/* Compatible name must be the same with that defined in codegen.dws and cust_i2c.dtsi */
/* TOOL : kernel-3.10\tools\dct */
/* PATH : vendor\mediatek\proprietary\custom\#project#\kernel\dct\dct */
#if I2C_CONFIG_SETTING == 2
static const struct of_device_id MAINAF_of_match[] = {
	{.compatible = "mediatek,CAMERA_MAIN_TWO_AF"},
	{},
};
#endif

static struct i2c_driver AF_i2c_driver = {
	.probe = AF_i2c_probe,
	.remove = AF_i2c_remove,
	.driver.name = AF_DRVNAME,
#if I2C_CONFIG_SETTING == 2
	.driver.of_match_table = MAINAF_of_match,
#endif
	.id_table = AF_i2c_id,
};

static int AF_i2c_remove(struct i2c_client *client)
{
	/* Add by meizu BSP hudong@meizu.com */
#ifdef CONFIG_MEIZU_BSP
	Unregister_AF_CharDrv();
#endif
/* Add end */
	return 0;
}

/* Kirby: add new-style driver {*/
static int AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	LOG_INF("Start\n");

	/* Kirby: add new-style driver { */
	g_pstAF_I2Cclient = client;

	/* Register char driver */
	i4RetValue = Register_AF_CharDrv();

	if (i4RetValue) {

		LOG_INF(" register char device failed!\n");

		return i4RetValue;
	}

	spin_lock_init(&g_AF_SpinLock);

	LOG_INF("Attached!!\n");

	return 0;
}

static int AF_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&AF_i2c_driver);
}

static int AF_remove(struct platform_device *pdev)
{
	i2c_del_driver(&AF_i2c_driver);
	return 0;
}

static int AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int AF_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver g_stAF_Driver = {
	.probe = AF_probe,
	.remove = AF_remove,
	.suspend = AF_suspend,
	.resume = AF_resume,
	.driver = {
		   .name = PLATFORM_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};

static struct platform_device g_stAF_device = {
	.name = PLATFORM_DRIVER_NAME,
	.id = 0,
	.dev = {}
};

static int __init MAINAF_i2C_init(void)
{
	#if I2C_CONFIG_SETTING == 1
	i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
	#endif

	if (platform_device_register(&g_stAF_device)) {
		LOG_INF("failed to register AF driver\n");
		return -ENODEV;
	}

	if (platform_driver_register(&g_stAF_Driver)) {
		LOG_INF("Failed to register AF driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit MAINAF_i2C_exit(void)
{
	platform_driver_unregister(&g_stAF_Driver);
}
module_init(MAINAF_i2C_init);
module_exit(MAINAF_i2C_exit);

MODULE_DESCRIPTION("MAIN2AF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");
