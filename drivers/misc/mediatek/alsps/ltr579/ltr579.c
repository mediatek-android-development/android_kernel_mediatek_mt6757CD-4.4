/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "ltr579.h"
#include "alsps.h"

#define GN_MTK_BSP_PS_DYNAMIC_CALI
//#define DEMO_BOARD
//#define LTR579_DEBUG
//#define SENSOR_DEFAULT_ENABLED

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define LTR579_DEV_NAME			"ltr579"
#define LTR579_I2C_ADDR			0x53

/*----------------------------------------------------------------------------*/
#define APS_TAG					"[ALS/PS] "
#define APS_FUN(f)              printk(KERN_INFO 	APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)   printk(KERN_ERR  	APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)   printk(KERN_NOTICE	APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   printk(KERN_ERR 	APS_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr579_i2c_id[] = {{LTR579_DEV_NAME,0},{}};
static unsigned long long int_top_time;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;

/*----------------------------------------------------------------------------*/
static int ltr579_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ltr579_i2c_remove(struct i2c_client *client);
static int ltr579_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/* [yanlin start] Porting sensors */
//static int ltr579_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr579_i2c_suspend(struct device *dev);
//static int ltr579_i2c_resume(struct i2c_client *client);
static int ltr579_i2c_resume(struct device *dev);
/* [yanlin end] */

#ifdef LTR579_DEBUG
static int ltr579_dump_reg(void);
#endif

//static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

/* [yanlin start] Modified als adjust for 908: *1.8 under TL84 */
#if defined(CONFIG_TPLINK_PRODUCT_TP908)
static int winfac = 375;
#else
static int winfac = 600;
#endif
/* [yanlin end] */

static int is_cali_finish = 0;
static int labc_strategy = 1; // 0: linear,   1: grading
static struct wake_lock ps_wlock;

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr579_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;
	//struct delayed_work check_ps_work;

	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;

#ifdef CONFIG_OF
	struct device_node *irq_node;
	int		irq;
#endif

	/*data*/
	u16			als;
	u16 		ps;
	u8			_align;
	u16			als_level_num;
	u16			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;

	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable; 		/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
};

struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;

static struct ltr579_priv *ltr579_obj = NULL;
static struct i2c_client *ltr579_i2c_client = NULL;

static DEFINE_MUTEX(ltr579_mutex);
static DEFINE_MUTEX(ltr579_i2c_mutex);

static int ltr579_local_init(void);
static int ltr579_remove(void);
static int ltr579_init_flag =-1; // 0<==>OK -1 <==> fail

static int ps_enabled = 0;
static int als_enabled = 0;

/* [yanlin start] fix ps error */
static int als_enable_state = 0;
static int ltr579_als_enable(struct i2c_client *client, int enable);
/* [yanlin end] */

static struct alsps_init_info ltr579_init_info = {
		.name = "ltr579",
		.init = ltr579_local_init,
		.uninit = ltr579_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

/* [yanlin start] Porting sensors */
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltr579_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr579_i2c_suspend, ltr579_i2c_resume)
};
#endif

static struct i2c_driver ltr579_i2c_driver = {
	.probe      = ltr579_i2c_probe,
	.remove     = ltr579_i2c_remove,
	.detect     = ltr579_i2c_detect,
//	.suspend    = ltr579_i2c_suspend,
//	.resume     = ltr579_i2c_resume,
	.id_table   = ltr579_i2c_id,
	.driver = {
		.name           = LTR579_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
		.pm   = &ltr579_pm_ops,
#endif
/* [yanlin end] Porting sensors */
	},
};

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr579_dynamic_calibrate(void);
static int dynamic_calibrate = 0;
#endif
/*-----------------------------------------------------------------------------*/

/*
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ltr579_i2c_read_reg(u8 regnum)
{
    u8 buffer[1],reg_value[1];
	int res = 0;

	mutex_lock(&ltr579_i2c_mutex);
	buffer[0]= regnum;
	res = i2c_master_send(ltr579_obj->client, buffer, 0x1);
	if(res <= 0)
	{
	   APS_ERR("read reg send res = %d\n",res);
	   /* [liguanxinog edit] release mute_lock before return */
	   mutex_unlock(&ltr579_i2c_mutex);
		return res;
	}
	res = i2c_master_recv(ltr579_obj->client, reg_value, 0x1);
	if(res <= 0)
	{
		APS_ERR("read reg recv res = %d\n",res);
		/* [liguanxinog edit] release mute_lock before return */
		mutex_unlock(&ltr579_i2c_mutex);
		return res;
	}
	mutex_unlock(&ltr579_i2c_mutex);

	return reg_value[0];
}

// I2C Write
static int ltr579_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];
	int res = 0;

	mutex_lock(&ltr579_i2c_mutex);
	databuf[0] = regnum;
	databuf[1] = value;
	res = i2c_master_send(ltr579_obj->client, databuf, 0x2);
	mutex_unlock(&ltr579_i2c_mutex);

	if (res < 0)
	{
		APS_ERR("wirte reg send res = %d\n",res);
		return res;
	}
	else
		return 0;
}

/*----------------------------------------------------------------------------*/
static void ltr579_power(struct alsps_hw *hw, unsigned int on)
{
#ifdef DEMO_BOARD
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "ltr579"))
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "ltr579"))
			{
				APS_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
#endif
}
/********************************************************************/
/*
 * ###############
 * ## PS CONFIG ##
 * ###############

 */
static int ltr579_ps_set_thres(void)
{

	int res;
	u8 databuf[2];

	struct i2c_client *client = ltr579_obj->client;
	struct ltr579_priv *obj = ltr579_obj;
	//APS_FUN();
	//APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);

	if(1 == ps_cali.valid)
	{
		/* [yanlin start] add log */
		APS_ERR("dynamic_calibrate=%d, far_away=%d, close=%d\n",
			dynamic_calibrate, ps_cali.far_away, ps_cali.close);
		/* [yanlin end] */

		databuf[0] = LTR579_PS_THRES_LOW_0;
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR579_PS_THRES_LOW_1;
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR579_PS_THRES_UP_0;
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR579_PS_THRES_UP_1;
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		ps_cali.valid = 0;
	}
	else
	{
		/* [yanlin start] add log */
		APS_ERR("dynamic_calibrate=%d, low=%d, high=%d\n",
			dynamic_calibrate, atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));
		/* [yanlin end] */

		databuf[0] = LTR579_PS_THRES_LOW_0;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR579_PS_THRES_LOW_1;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);

		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR579_PS_THRES_UP_0;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		databuf[0] = LTR579_PS_THRES_UP_1;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	}

	res = 0;
	return res;

EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	res = LTR579_ERR_I2C;
	return res;
}

/* [liguanxiong start] add esd check */
static int ltr579_esd_check(void)
{
	int ret = -1;
	u8 regdata;
	u8 databuf[2];
	struct i2c_client *client = ltr579_obj->client;
	struct ltr579_priv *obj = ltr579_obj;

	//APS_LOG("start ESD check! \n");
	regdata = ltr579_i2c_read_reg(LTR579_PS_PULSES);
	if (32 != regdata) {
		/* sometimes esd cause ps register reset to default, we need to reconfigure it */
		APS_LOG("ESD: reconfigure ps register\n");
		ret = ltr579_i2c_write_reg(LTR579_PS_PULSES, 32); //32pulses
		ret = ltr579_i2c_write_reg(LTR579_PS_LED, 0x36); // 60khz & 100mA
		ret = ltr579_i2c_write_reg(LTR579_PS_MEAS_RATE, 0x5C); // 11bits & 50ms time

		/*for interrup work mode support */
		if (0 == obj->hw->polling_mode_ps)
		{
			databuf[0] = LTR579_INT_CFG;
			databuf[1] = 0x01;
			ret = i2c_master_send(client, databuf, 0x2);

			databuf[0] = LTR579_INT_PST;
			databuf[1] = 0x02;
			ret = i2c_master_send(client, databuf, 0x2);
			if (ret <= 0)
			{
				APS_ERR("ESD CHECK: Failed to reset ps interrupt mode\n");
				return -1;
			}
		}
	}
	return 0;
}
/* [liguanxiong end] */

static int ltr579_ps_enable(struct i2c_client *client, int enable)
{
	u8 regdata;
	int err;

	APS_LOG("ltr579_ps_enable() ...start!\n");

	if (enable) {
		ltr579_esd_check();
	}

	if (enable != 0 && ps_enabled == 1)
	{
		APS_LOG("PS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && ps_enabled == 0)
	{
		APS_LOG("PS: Already disabled \n");
		return 0;
	}

	regdata = ltr579_i2c_read_reg(LTR579_MAIN_CTRL);
	/* [yanlin start] fix ps error */
	regdata &= 0x03;
	/* [yanlin end] */
	if (enable != 0) {
		APS_LOG("PS: enable ps only \n");
		regdata |= 0x01;
	}
	else {
		APS_LOG("PS: disable ps only \n");
		regdata &= 0xFE;
	}

	is_cali_finish = 0;
	err = ltr579_i2c_write_reg(LTR579_MAIN_CTRL, regdata);
	if (err < 0)
	{
		APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
		return err;
	}
	mdelay(WAKEUP_DELAY);
	ltr579_i2c_read_reg(LTR579_MAIN_CTRL);

	if (0 == ltr579_obj->hw->polling_mode_ps && enable != 0)
	{
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
		err = ltr579_dynamic_calibrate();
		if (err < 0)
		{
			APS_LOG("ltr579_dynamic_calibrate() failed\n");
		}
#endif
		is_cali_finish = 1;
		ltr579_ps_set_thres();
	}
	else if (0 == ltr579_obj->hw->polling_mode_ps && enable == 0)
	{
		//cancel_work_sync(&ltr579_obj->eint_work);
	}

	if (enable != 0)
		ps_enabled = 1;
	else
		ps_enabled = 0;

	/* [yanlin start] fix ps error */
	if (ps_enabled == 0 && als_enable_state == 0 && als_enabled == 1)
	{
		APS_LOG("PS(0): time to disable als!\n");
		ltr579_als_enable(client, 0);
	}
	/* [yanlin end] */

	return err;
}

/********************************************************************/
static int ltr579_ps_read(struct i2c_client *client, u16 *data)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr579_i2c_read_reg(LTR579_PS_DATA_0);
	//APS_DBG("ps_rawdata_psval_lo = %d\n", psval_lo);
	if (psval_lo < 0){
	    APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}

	psval_hi = ltr579_i2c_read_reg(LTR579_PS_DATA_1);
    //APS_DBG("ps_rawdata_psval_hi = %d\n", psval_hi);
	if (psval_hi < 0){
	    APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}

	psdata = ((psval_hi & 7)* 256) + psval_lo;
	*data = psdata;
    //APS_DBG("ltr579_ps_read: ps_rawdata = %d\n", psdata);

out:
	final_prox_val = psdata;
	return psdata;
}

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
#define PS_DYNAMIC_CALI_DELAY_TIME		10  /* default 60ms*/
static int ltr579_dynamic_calibrate(void)
{
	int i = 0;
	int data;
	int data_total = 0;
	int noise = 0;
	int count = 5;
	int ps_thd_val_low, ps_thd_val_high;
	struct ltr579_priv *obj = ltr579_obj;

	if (!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return -1;
	}

	ltr579_i2c_write_reg(LTR579_PS_MEAS_RATE, 0x59); /* 11bits & 6.25ms time */
	for (i = 0; i < count; i++) {
		// wait for ps value be stable
		/* [liguanxiong edit] change delay time 60ms -> 10ms */
		msleep(PS_DYNAMIC_CALI_DELAY_TIME);

		data = ltr579_ps_read(ltr579_obj->client, &ltr579_obj->ps);
		if (data < 0) {
			i--;
			continue;
		}

		if (data & 0x0800) {
			break;
		}

		data_total += data;
	}

	ltr579_i2c_write_reg(LTR579_PS_MEAS_RATE, 0x5C); /* 11bits & 50ms time */
	noise = data_total / count;
	dynamic_calibrate = noise;

	/* [yanlin start] Dynamic calibration for 908 */
	if (noise < 100) {
		ps_thd_val_high = noise + 88;
		ps_thd_val_low  = noise + 52;
	}
	else if (noise < 200) {
		ps_thd_val_high = noise + 80;
		ps_thd_val_low  = noise + 50;
	}
	else if (noise < 1260) {
		ps_thd_val_high = noise + 75;
		ps_thd_val_low  = noise + 45;
	}
	else {
		ps_thd_val_high = 1400;
		ps_thd_val_low  = 1300;
	}
	/* [yanlin end] */

	atomic_set(&obj->ps_thd_val_high, ps_thd_val_high);
	atomic_set(&obj->ps_thd_val_low, ps_thd_val_low);
    ps_cali.valid = 1;
    ps_cali.far_away = ps_thd_val_low;
    ps_cali.close = ps_thd_val_high;
	/* [liguanxiong edit] report faraway status after calibrate, if p-sensor is not covered! */
	if (noise < ps_thd_val_low) {
		ps_report_interrupt_data(1);
		APS_LOG("%s:p-sensor is not covered, report faraway status after calibrate \n", __func__);
	}

	//APS_LOG("%s:noise = %d\n", __func__, noise);
	//APS_LOG("%s:obj->ps_thd_val_high = %d\n", __func__, ps_thd_val_high);
	//APS_LOG("%s:obj->ps_thd_val_low = %d\n", __func__, ps_thd_val_low);

	return 0;
}
#endif
/********************************************************************/
/*
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr579_als_enable(struct i2c_client *client, int enable)
{
	int err = 0;
	u8 regdata = 0;

	/* [yanlin start] fix ps error */
	als_enable_state = enable;
	if (ps_enabled == 1 && enable == 0)
	{
		APS_LOG("ALS(0): PS is enabled, dont disable als!\n");
		return 0;
	}
	/* [yanlin end] */

	if (enable != 0 && als_enabled == 1)
	{
		APS_LOG("ALS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && als_enabled == 0)
	{
		APS_LOG("ALS: Already disabled \n");
		return 0;
	}

	regdata = ltr579_i2c_read_reg(LTR579_MAIN_CTRL);
	/* [yanlin start] fix ps error */
	regdata &= 0x03;
	/* [yanlin end] */
	if (enable != 0) {
		/* [yanlin] modified log */
		APS_LOG("ALS(1): enable als only:0x%x \n", regdata);
		regdata |= 0x02;
	}
	else {
		/* [yanlin] modified log */
		APS_LOG("ALS(0): disable als only:0x%x \n", regdata);
		regdata &= 0xFD;
	}

	err = ltr579_i2c_write_reg(LTR579_MAIN_CTRL, regdata);
	/* [liguanxiong edit] reconfig ps thres, if ps enabled and object near when switching als state */
	if (intr_flag_value == 1 && ps_enabled == 1) {
		ltr579_i2c_write_reg(LTR579_PS_THRES_LOW_0, 0x00);
		ltr579_i2c_write_reg(LTR579_PS_THRES_LOW_1, 0x00);
		ltr579_i2c_write_reg(LTR579_PS_THRES_UP_0, 0x00);
		ltr579_i2c_write_reg(LTR579_PS_THRES_UP_1, 0x00);
		APS_LOG("ltr579_ps_set_thres when disable als \n");
	}
	if (err < 0)
	{
		APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
		return err;
	}

	mdelay(WAKEUP_DELAY);

	if (enable != 0)
		als_enabled = 1;
	else
		als_enabled = 0;

	return 0;
}

static int ltr579_als_read(struct i2c_client *client, u16* data)
{
	int alsval_0, alsval_1, alsval_2, alsval;
	int luxdata_int;

	alsval_0 = ltr579_i2c_read_reg(LTR579_ALS_DATA_0);
	alsval_1 = ltr579_i2c_read_reg(LTR579_ALS_DATA_1);
	alsval_2 = ltr579_i2c_read_reg(LTR579_ALS_DATA_2);
	alsval = (alsval_2 * 256 * 256) + (alsval_1 * 256) + alsval_0;
	//APS_DBG("alsval_0 = %d,alsval_1=%d,alsval_2=%d,alsval=%d\n", alsval_0, alsval_1, alsval_2, alsval);

	/* [yanlin start] check als raw data */
	switch (((u8)(ALS_RESO_MEAS & 0x0070)) >> 4) {
	case 0x00: // 20bit
		if (alsval > 0x0fffff)
			return -1;
		break;
	case 0x01: // 19bit
		if (alsval > 0x07ffff)
			return -1;
		break;
	case 0x02: // 18bit
		if (alsval > 0x03ffff)
			return -1;
		break;
	case 0x03: // 17bit
		if (alsval > 0x01ffff)
			return -1;
		break;
	case 0x04: // 16bit
		if (alsval > 0x00ffff)
			return -1;
		break;
	default:
		return -1;
	}
	/* [yanlin end] */

	if (alsval == 0)
	{
		luxdata_int = 0;
		goto out;
	}
#if 0
	luxdata_int = alsval * 8 / als_gainrange / 10;//formula: ALS counts * 0.8/gain/int , int=1
#else
	/* [yanlin start] modified formula */
	luxdata_int = alsval * 8 * winfac * ALS_RANGE_3 / als_gainrange / 1000;
	/* [yanlin end] */
#endif
	//APS_DBG("ltr579_als_read: als_value_lux = %d\n", luxdata_int);
out:

	/* [yanlin start] check als data after calculated */
	if (luxdata_int > 0x00ffff)
		luxdata_int = 0x00ffff;
	/* [yanlin end] */

	*data = luxdata_int;
	final_lux_val = luxdata_int;
	return luxdata_int;
}
/********************************************************************/
static int ltr579_get_ps_value(struct ltr579_priv *obj, u16 ps)
{
	int val = 1;
	int buffer = 0;
	int ps_flag;

	buffer = ltr579_i2c_read_reg(LTR579_MAIN_STATUS);
	if (buffer < 0) {
		return -1;
	}

	ps_flag = buffer & 0x04;
	ps_flag = ps_flag >> 2;
	if (ps_flag == 1) //Near
	{
		intr_flag_value = 1;
		val = 0;
	}
	else if (ps_flag == 0) //Far
	{
		intr_flag_value = 0;
		val = 1;
	}

	return val;
}
/********************************************************************/
static int ltr579_get_als_value(struct ltr579_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	//APS_DBG("als  = %d\n",als);
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}

	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}

		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		return -1;
	}
}
/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ltr579_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
		atomic_read(&ltr579_obj->i2c_retry), atomic_read(&ltr579_obj->als_debounce),
		atomic_read(&ltr579_obj->ps_mask), atomic_read(&ltr579_obj->ps_thd_val), atomic_read(&ltr579_obj->ps_debounce));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}

	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{
		atomic_set(&ltr579_obj->i2c_retry, retry);
		atomic_set(&ltr579_obj->als_debounce, als_deb);
		atomic_set(&ltr579_obj->ps_mask, mask);
		atomic_set(&ltr579_obj->ps_thd_val, thres);
		atomic_set(&ltr579_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr579_obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&ltr579_obj->trace, trace);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_als(struct device_driver *ddri, char *buf)
{
	int res;

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}
	res = ltr579_als_read(ltr579_obj->client, &ltr579_obj->als);
	return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);

}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}
	res = ltr579_ps_read(ltr579_obj->client, &ltr579_obj->ps);
    return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_reg(struct device_driver *ddri, char *buf)
{
	int i,len=0;
	int reg[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0d,0x0e,0x0f,
			   0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26};
	for(i=0;i<27;i++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%04X value: 0x%04X\n", reg[i],ltr579_i2c_read_reg(reg[i]));
	}
	return len;
}

#ifdef LTR579_DEBUG
static int ltr579_dump_reg(void)
{
	int i=0;
	int reg[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0d,0x0e,0x0f,
		       0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26};
	for(i=0;i<27;i++)
	{
		APS_DBG("reg:0x%04X value: 0x%04X\n", reg[i], ltr579_i2c_read_reg(reg[i]));
	}
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}

	if(ltr579_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n",
			ltr579_obj->hw->i2c_num, ltr579_obj->hw->power_id, ltr579_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr579_obj->als_suspend), atomic_read(&ltr579_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr579_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++;
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}

	for(idx = 0; idx < ltr579_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr579_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr579_obj->als_level, ltr579_obj->hw->als_level, sizeof(ltr579_obj->als_level));
	}
	else if(ltr579_obj->als_level_num != read_int_from_buf(ltr579_obj, buf, count,
			ltr579_obj->hw->als_level, ltr579_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}

	for(idx = 0; idx < ltr579_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr579_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr579_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr579_obj->als_value, ltr579_obj->hw->als_value, sizeof(ltr579_obj->als_value));
	}
	else if(ltr579_obj->als_value_num != read_int_from_buf(ltr579_obj, buf, count,
			ltr579_obj->hw->als_value, ltr579_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

#define DRIVER_VERSION "1.0.0"
static ssize_t ltr579_show_hw_info(struct device_driver *ddri, char *buf)
{
	return sprintf(buf,"type:\t%s\nvendor:\t%s\ndriver_version:\t%s\n",
			"LTR-578ALS-028", "LITE-ON", DRIVER_VERSION);
}
/*---------------------------------------------------------------------------------------*/
static ssize_t ltr579_show_winfac(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", winfac);
}
static ssize_t ltr579_store_winfac(struct device_driver *ddri, const char *buf, size_t count)
{
	int value = 0;
	int ret = 0;
	ret = kstrtoint(buf, 10, &value);
	if (ret == 0) {
		winfac = value;
		APS_DBG("set als winfac = %d\n", winfac);
	}
	return count;
}
/*---------------------------------------------------------------------------------------*/
static ssize_t ltr579_show_labc_strategy(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", labc_strategy);
}

/* [yanlin start] for hw info in /sys/kernel */
#define DRIVER_VERSION "1.0.0"
static ssize_t show_alsps_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"type:\t%s\nvendor:\t%s\ndriver_version:\t%s\n",
			"LTR-578ALS-028", "LITE-ON", DRIVER_VERSION);
}
static DEVICE_ATTR(alsps_info, S_IWUSR | S_IRUGO, show_alsps_info, NULL);
/* [yanlin end] */

/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, ltr579_show_als,		NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, ltr579_show_ps,		NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, ltr579_show_config,	ltr579_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, ltr579_show_alslv,	ltr579_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, ltr579_show_alsval,	ltr579_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, ltr579_show_trace,	ltr579_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, ltr579_show_status,	NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, ltr579_show_send,	ltr579_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, ltr579_show_recv,	ltr579_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, ltr579_show_reg,		NULL);
static DRIVER_ATTR(hw_info,     S_IWUSR | S_IRUGO, ltr579_show_hw_info,	NULL);
static DRIVER_ATTR(winfac,     S_IWUSR | S_IRUGO, ltr579_show_winfac,	ltr579_store_winfac);
static DRIVER_ATTR(labc_strategy,     S_IWUSR | S_IRUGO, ltr579_show_labc_strategy, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr579_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
    &driver_attr_hw_info,
    &driver_attr_winfac,
    &driver_attr_labc_strategy,
};

/*----------------------------------------------------------------------------*/
static int ltr579_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr579_attr_list)/sizeof(ltr579_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ltr579_attr_list[idx])))
		{
			APS_ERR("driver_create_file (%s) = %d\n", ltr579_attr_list[idx]->attr.name, err);
			break;
		}
	}

	/* [yanlin add] for hw info in /sys/kernel */
	err = sysfs_create_file(kernel_kobj, &dev_attr_alsps_info.attr);

	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr579_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr579_attr_list)/sizeof(ltr579_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, ltr579_attr_list[idx]);
	}

	/* [yanlin add] for hw info in /sys/kernel */
	sysfs_remove_file(kernel_kobj, &dev_attr_alsps_info.attr);

	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/
//#ifndef CUSTOM_KERNEL_SENSORHUB
#if 0
static int ltr579_check_and_clear_intr(struct i2c_client *client)
{
	APS_FUN();

	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
	//	  return 0;

	buffer[0] = LTR579_MAIN_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;
	}

	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[1] = buffer[0] & 0xFD;
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[1] = buffer[0] & 0xEF;
		}
		else
		{
			buffer[1] = buffer[0] & 0xED;
		}
		buffer[0] = LTR579_MAIN_STATUS;
		res = i2c_master_send(client, buffer, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}
	else
		return 0;

EXIT_ERR:
	APS_ERR("ltr579_check_and_clear_intr fail\n");
	return 1;
}

static int ltr579_check_intr(struct i2c_client *client)
{
	APS_FUN();

	int res = 0;
	int intp = 0;
	int intl = 0;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
	//    return 0;

	buffer[0] = LTR579_MAIN_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	res = 0;
	intp = 0;
	intl = 0;
	if (0 != (buffer[0] & 0x02))
	{
		res = 1;	//PS int
		intp = 1;
	}
	if (0 != (buffer[0] & 0x10))
	{
		res = 2;	//ALS int
		intl = 1;
	}
	if ((intp == 1) && (intl == 1))
	{
		res = 4;	//ALS & PS int
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr579_check_intr fail\n");
	return 0;
}

static int ltr579_clear_intr(struct i2c_client *client)
{
	int res;
	u8 buffer[2];

	APS_FUN();

	buffer[0] = LTR579_MAIN_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	APS_DBG("buffer[0] = %d \n",buffer[0]);
	buffer[1] = buffer[0] & 0xED;
	buffer[0] = LTR579_MAIN_STATUS;

	res = i2c_master_send(client, buffer, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr579_clear_intr fail\n");
	return 1;
}
#endif //#ifndef CUSTOM_KERNEL_SENSORHUB
/*----------------------------------------------------------------------------*/
#if 0
static void ltr579_check_ps_work(struct work_struct *work)
{
	struct ltr579_priv *obj = (struct ltr579_priv *)container_of(work, struct ltr579_priv, check_ps_work);
	int err;
	hwm_sensor_data sensor_data;
	APS_FUN();

	if (test_bit(CMC_BIT_PS, &obj->enable)) {
		ltr579_ps_read(obj->client, &obj->ps);
		APS_LOG("ltr579_check_ps_work rawdata ps=%d high=%d low=%d\n", obj->ps, atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));

		sensor_data.values[0] = ltr579_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

		if (ps_report_interrupt_data(sensor_data.values[0]))
		{
			APS_ERR("call ps_report_interrupt_data fail \n");
		}
	}

	return;
}
#endif
/*----------------------------------------------------------------------------*/
static void ltr579_eint_work(struct work_struct *work)
{
	struct ltr579_priv *obj = (struct ltr579_priv *)container_of(work, struct ltr579_priv, eint_work);
	u8 databuf[2];
	int res = 0;
	int err;
	int value = 1;

#if 0
	err = ltr579_check_intr(obj->client);
	if (err < 0) {
		goto EXIT_INTR;
	}
	else
#endif
	{
		//get raw data
		obj->ps = ltr579_ps_read(obj->client, &obj->ps);
		if (obj->ps < 0)
		{
			err = -1;
			goto EXIT_INTR;
		}

		//APS_DBG("ltr579_eint_work: rawdata ps=%d!\n",obj->ps);
		//printk("ltr579 rawdata ps = %d!\n",obj->ps);
		value = ltr579_get_ps_value(obj, obj->ps);
		//APS_DBG("intr_flag_value=%d\n",intr_flag_value);
		/* [liguanxiong edit] sometimes interrupt will trigger before calibration is finished,
			we should ignore it */
		if (is_cali_finish == 0) {
			goto EXIT_INTR;
		}
		if(intr_flag_value){
			/* [yanlin start] add log */
			APS_ERR("--- Near state: ps=%d, low=%d, high=%d ---\n",
				obj->ps, atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));
			/* [yanlin end] */
#if 1
			databuf[0] = LTR579_PS_THRES_LOW_0;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
			databuf[0] = LTR579_PS_THRES_LOW_1;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
			databuf[0] = LTR579_PS_THRES_UP_0;
			/* [yanlin start] fix ps error */
			//databuf[1] = (u8)(0x00FF);
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			/* [yanlin end] */
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
			databuf[0] = LTR579_PS_THRES_UP_1;
			/* [yanlin start] fix ps error */
			//databuf[1] = (u8)((0xFF00) >> 8);
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);
			/* [yanlin end] */
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
#endif
		}
		else{
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
			if(obj->ps > 20 && obj->ps < (dynamic_calibrate - 50)){
				/* [yanlin start] Dynamic calibration for 908 */
				if(obj->ps < 100){
					atomic_set(&obj->ps_thd_val_high, obj->ps + 88);
					atomic_set(&obj->ps_thd_val_low, obj->ps + 52);
				}else if(obj->ps < 200){
					atomic_set(&obj->ps_thd_val_high, obj->ps + 80);
					atomic_set(&obj->ps_thd_val_low, obj->ps + 50);
				}else if(obj->ps < 1260){
					atomic_set(&obj->ps_thd_val_high, obj->ps + 75);
					atomic_set(&obj->ps_thd_val_low, obj->ps + 45);
				}else{
					atomic_set(&obj->ps_thd_val_high, 1400);
					atomic_set(&obj->ps_thd_val_low, 1300);
				}
				/* [yanlin end] */

				dynamic_calibrate = obj->ps;
			}

			/* [yanlin start] add log */
			APS_ERR("--- Far state: ps=%d, set low=%d, high=%d ---\n",
				obj->ps, atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));
			/* [yanlin end] */

			databuf[0] = LTR579_PS_THRES_LOW_0;
			/* [yanlin start] fix ps error */
			//databuf[1] = (u8)(0 & 0x00FF);
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			/* [yanlin end] */
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
			databuf[0] = LTR579_PS_THRES_LOW_1;
			/* [yanlin start] fix ps error */
			//databuf[1] = (u8)((0 & 0xFF00) >> 8);
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			/* [yanlin end] */
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
			databuf[0] = LTR579_PS_THRES_UP_0;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
			databuf[0] = LTR579_PS_THRES_UP_1;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_INTR;
			}
#endif
		}
		wake_lock_timeout(&ps_wlock, msecs_to_jiffies(1000));
		//let up layer to know
		res = ps_report_interrupt_data(value);
		APS_LOG("%s:ps_report_interrupt_data = %d\n", __func__, value);
	}

EXIT_INTR:
	//ltr579_clear_intr(obj->client);
#ifdef CONFIG_OF
	enable_irq(obj->irq);
#endif
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void ltr579_eint_func(void)
{
	struct ltr579_priv *obj = ltr579_obj;
	if(!obj)
	{
		return;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}
#ifdef CONFIG_OF
static irqreturn_t ltr579_eint_handler(int irq, void *desc)
{
	disable_irq_nosync(ltr579_obj->irq);
	ltr579_eint_func();

	return IRQ_HANDLED;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int ltr579_setup_eint(struct i2c_client *client)
{

	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = { 0, 0 };

	//APS_FUN();
	alspsPltFmDev = get_alsps_platformdev();
	ltr579_obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");

	/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	}

	/* eint request */
	if (ltr579_obj->irq_node) {
		of_property_read_u32_array(ltr579_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		ltr579_obj->irq = irq_of_parse_and_map(ltr579_obj->irq_node, 0);
		APS_LOG("ltr579_obj->irq = %d\n", ltr579_obj->irq);
		if (!ltr579_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		APS_ERR("irq to gpio = %d \n", irq_to_gpio(ltr579_obj->irq));
		if (request_irq(ltr579_obj->irq, ltr579_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		/* [yanlin start] Enable ALS-eint wake up */
		ret = enable_irq_wake(ltr579_obj->irq);
		if (ret)
		{
			APS_ERR("%s: enable_irq_wake(%d) failed, ret=(%d)\n", __func__, ltr579_obj->irq, ret);
		}
		/* [yanlin end] */
	}
	else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return 0;
}
/**********************************************************************************************/

/*-------------------------------MISC device related------------------------------------------*/
static int ltr579_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr579_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int ltr579_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long ltr579_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr579_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_cali;
	int threshold[2];
	APS_DBG("cmd= %d\n", cmd);
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			err = ltr579_ps_enable(obj->client, enable);
			if (err < 0)
			{
				APS_ERR("enable ps fail: %d en: %d\n", err, enable);
				goto err_out;
			}
			if (enable)
				set_bit(CMC_BIT_PS, &obj->enable);
			else
				clear_bit(CMC_BIT_PS, &obj->enable);
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			APS_DBG("ALSPS_GET_PS_DATA\n");
			obj->ps = ltr579_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}

			dat = ltr579_get_ps_value(obj, obj->ps);
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			obj->ps = ltr579_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			err = ltr579_als_enable(obj->client, enable);
			if (err < 0)
			{
				APS_ERR("enable als fail: %d en: %d\n", err, enable);
				goto err_out;
			}
			if (enable)
				set_bit(CMC_BIT_ALS, &obj->enable);
			else
				clear_bit(CMC_BIT_ALS, &obj->enable);
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA:
			obj->als = ltr579_als_read(obj->client, &obj->als);
			if (obj->als < 0)
			{
				goto err_out;
			}

			dat = ltr579_get_als_value(obj, obj->als);
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_RAW_DATA:
			obj->als = ltr579_als_read(obj->client, &obj->als);
			if (obj->als < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			obj->ps = ltr579_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}
			if(obj->ps > atomic_read(&obj->ps_thd_val_low))
				dat = 1;
			else
				dat = 0;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_CLR_CALI:
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
			break;

		case ALSPS_IOCTL_GET_CALI:
			ps_cali = obj->ps_cali ;
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_SET_CALI:
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
			obj->ps_cali = ps_cali;
			break;

		//case ALSPS_CALIBRATE_PS:    //ps
			//ps_calibration();
			//break;

		//case ALSPS_WRITE_CALIBRATE:     //
			//get_ calibration_data();
			//set_ps_pulse();
			//set_ps_thres();
			//break;

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

			ltr579_ps_set_thres();
			break;

		case ALSPS_GET_PS_THRESHOLD_HIGH:
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
/*------------------------------------------------------------------------------------------*/

		default:
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;
}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations ltr579_fops = {
	.owner = THIS_MODULE,
	.open = ltr579_open,
	.release = ltr579_release,
	.unlocked_ioctl = ltr579_unlocked_ioctl,
};

static struct miscdevice ltr579_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr579_fops,
};

/*--------------------------------------------------------------------------------*/
static int ltr579_init_client(void)
{
	int res;
	int init_als_gain;
	u8 databuf[2];

	struct i2c_client *client = ltr579_obj->client;

	struct ltr579_priv *obj = ltr579_obj;

	mdelay(PON_DELAY);

	/* ===============
	* ** IMPORTANT **
	* ===============
	* Other settings like timing and threshold to be set here, if required.
	* Not set and kept as device default for now.
	*/
	res = ltr579_i2c_write_reg(LTR579_PS_PULSES, 32); //32pulses
	if (res<0)
	{
		APS_LOG("ltr579_init_client() PS Pulses error...\n");
		goto EXIT_ERR;
	}
	res = ltr579_i2c_write_reg(LTR579_PS_LED, 0x36); // 60khz & 100mA
	if (res<0)
	{
		APS_LOG("ltr579_init_client() PS LED error...\n");
		goto EXIT_ERR;
	}
	res = ltr579_i2c_write_reg(LTR579_PS_MEAS_RATE, 0x5C); // 11bits & 50ms time
	if (res<0)
	{
		APS_LOG("ltr579_init_client() PS time error...\n");
		goto EXIT_ERR;
	}

	/*for interrup work mode support */
	if (0 == obj->hw->polling_mode_ps)
	{
		ltr579_ps_set_thres();

		databuf[0] = LTR579_INT_CFG;
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0)
		{
			goto EXIT_ERR;
		}

		databuf[0] = LTR579_INT_PST;
		databuf[1] = 0x02;
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0)
		{
			goto EXIT_ERR;
		}
	}
#ifdef SENSOR_DEFAULT_ENABLED
	res = ltr579_ps_enable(client, 1);
	if (res < 0)
	{
		APS_ERR("enable ps fail: %d\n", res);
		goto EXIT_ERR;
	}
#endif
	// Enable ALS to Full Range at startup

	/* [yanlin start] changed gain from 3 to 18 */
	init_als_gain = ALS_RANGE_18;
	/* [yanlin end] */

	als_gainrange = init_als_gain;//Set global variable
	//APS_ERR("ALS sensor gainrange %d!\n", init_als_gain);

	switch (als_gainrange)
	{
	case ALS_RANGE_1:
		res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range1);
		break;

	case ALS_RANGE_3:
		res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range3);
		break;

	case ALS_RANGE_6:
		res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range6);
		break;

	case ALS_RANGE_9:
		res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range9);
		break;

	case ALS_RANGE_18:
		res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range18);
		break;

	default:
		res = ltr579_i2c_write_reg(LTR579_ALS_GAIN, MODE_ALS_Range3);
		break;
	}

	res = ltr579_i2c_write_reg(LTR579_ALS_MEAS_RATE, ALS_RESO_MEAS);// 18 bit & 100ms measurement rate
	//APS_ERR("ALS sensor resolution & measurement rate: %d!\n", ALS_RESO_MEAS);
#ifdef SENSOR_DEFAULT_ENABLED
	res = ltr579_als_enable(client, 1);
	if (res < 0)
	{
		APS_ERR("enable als fail: %d\n", res);
		goto EXIT_ERR;
	}
#endif
	if ((res = ltr579_setup_eint(client)) != 0)
	{
		APS_ERR("setup eint: %d\n", res);
		goto EXIT_ERR;
	}
#if 0
	if ((res = ltr579_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
		goto EXIT_ERR;
	}
#endif
	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return 1;
}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("ltr579_obj als enable value = %d\n", en);

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return -1;
	}

	res = ltr579_als_enable(ltr579_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr579_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr579_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr579_obj->enable);
	mutex_unlock(&ltr579_mutex);

	//cancel_delayed_work(&ltr579_obj->check_ps_work);
	//schedule_delayed_work(&ltr579_obj->check_ps_work, msecs_to_jiffies(300));

	return 0;
}

static int als_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int als_get_data(int* value, int* status)
{
	int err = 0;

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return -1;
	}

	ltr579_obj->als = ltr579_als_read(ltr579_obj->client, &ltr579_obj->als);
	if (ltr579_obj->als < 0)
		err = -1;
	else {
		//*value = ltr579_get_als_value(ltr579_obj, ltr579_obj->als);
		*value = ltr579_obj->als;
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("ltr579_obj ps enable value = %d\n", en);

	if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return -1;
	}

	res = ltr579_ps_enable(ltr579_obj->client, en);
	if (res < 0) {
		APS_ERR("ps_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr579_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr579_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &ltr579_obj->enable);
	mutex_unlock(&ltr579_mutex);

	return 0;
}

static int ps_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;

    if(!ltr579_obj)
	{
		APS_ERR("ltr579_obj is null!!\n");
		return -1;
	}

	ltr579_obj->ps = ltr579_ps_read(ltr579_obj->client, &ltr579_obj->ps);
	if (ltr579_obj->ps < 0)
		err = -1;
	else {
		*value = ltr579_get_ps_value(ltr579_obj, ltr579_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}
/*-----------------------------------------------------------------------------------*/

/*-----------------------------------i2c operations----------------------------------*/
static int ltr579_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr579_priv *obj = NULL;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	int err = 0;

	APS_FUN();

	//client->timing = 400;
	client->addr = LTR579_I2C_ADDR;
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr579_obj = obj;

	obj->hw = hw;
	INIT_WORK(&obj->eint_work, ltr579_eint_work);
	//INIT_DELAYED_WORK(&obj->check_ps_work, ltr579_check_ps_work);

	wake_lock_init(&ps_wlock, WAKE_LOCK_SUSPEND, "PROXIMITY");
	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100
	/*-----------------------------value need to be confirmed-----------------------------------------*/

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);

#ifdef SENSOR_DEFAULT_ENABLED
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
#else
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
#endif

	APS_LOG("ltr579_init_client() start...!\n");
	ltr579_i2c_client = client;
	err = ltr579_init_client();
	if(err)
	{
		goto exit_init_failed;
	}
	APS_LOG("ltr579_init_client() OK!\n");

	err = misc_register(&ltr579_device);
	if(err)
	{
		APS_ERR("ltr579_device register failed\n");
		goto exit_misc_device_register_failed;
	}

    als_ctl.is_use_common_factory =false;
	ps_ctl.is_use_common_factory = false;

	/*------------------------ltr579 attribute file for debug--------------------------------------*/
	//err = ltr579_create_attr(&(ltr579_init_info.platform_diver_addr->driver));
	err = ltr579_create_attr(&(ltr579_i2c_driver.driver));
	if(err)
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------ltr579 attribute file for debug--------------------------------------*/

	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 1, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
	}

	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 1, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
	}

	ltr579_init_flag =0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
	misc_deregister(&ltr579_device);
exit_init_failed:
	wake_lock_destroy(&ps_wlock);
	kfree(obj);
exit:
	ltr579_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr579_init_flag =-1;
	return err;
}

static int ltr579_i2c_remove(struct i2c_client *client)
{
	int err;

	//err = ltr579_delete_attr(&(ltr579_init_info.platform_diver_addr->driver));
	err = ltr579_delete_attr(&(ltr579_i2c_driver.driver));
	if(err)
	{
		APS_ERR("ltr579_delete_attr fail: %d\n", err);
	}

	misc_deregister(&ltr579_device);
	if(err)
	{
		APS_ERR("misc_deregister fail: %d\n", err);
	}

	wake_lock_destroy(&ps_wlock);
	ltr579_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int ltr579_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, LTR579_DEV_NAME);
	return 0;
}

/* [yanlin start] Porting sensors */
//static int ltr579_i2c_suspend(struct i2c_client *client, pm_message_t msg)
static int ltr579_i2c_suspend(struct device *dev)
{
#if 1
	struct i2c_client *client = to_i2c_client(dev);
#endif
	struct ltr579_priv *obj = i2c_get_clientdata(client);

	int err;
	APS_FUN();

//	if(msg.event == PM_EVENT_SUSPEND)
//	{
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}

		atomic_set(&obj->als_suspend, 1);
		err = ltr579_als_enable(obj->client, 0);
		if(err < 0)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		/* [liguanxiong removed] no need to disable p-sensor in suspend */
#if 0
		atomic_set(&obj->ps_suspend, 1);
		err = ltr579_ps_enable(obj->client, 0);
		if(err < 0)
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		ltr579_power(obj->hw, 0);
#endif
//	}
/* [yanlin end] */
	return 0;
}

/* [yanlin start] Porting sensors */
//static int ltr579_i2c_resume(struct i2c_client *client)
static int ltr579_i2c_resume(struct device *dev)
{
#if 1
	struct i2c_client *client = to_i2c_client(dev);
#endif
/* [yanlin end] */
	struct ltr579_priv *obj = i2c_get_clientdata(client);

	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	ltr579_power(obj->hw, 1);

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		err = ltr579_als_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	/* [liguanxiong removed] no need to disable p-sensor in suspend */
#if 0
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = ltr579_ps_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);
		}
	}
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int ltr579_remove(void)
{
	APS_FUN();

	ltr579_power(hw, 0);
	i2c_del_driver(&ltr579_i2c_driver);
	ltr579_init_flag = -1;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int  ltr579_local_init(void)
{
	APS_FUN();

	ltr579_power(hw, 1);

	if(i2c_add_driver(&ltr579_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(-1 == ltr579_init_flag)
	{
	   return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init ltr579_init(void)
{
    const char *name = "mediatek,ltr579";
	APS_FUN();
    hw = get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("get dts info fail\n");

	alsps_driver_add(&ltr579_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr579_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(ltr579_init);
module_exit(ltr579_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liteon");
MODULE_DESCRIPTION("LTR-579ALSPS Driver");
MODULE_LICENSE("GPL");

