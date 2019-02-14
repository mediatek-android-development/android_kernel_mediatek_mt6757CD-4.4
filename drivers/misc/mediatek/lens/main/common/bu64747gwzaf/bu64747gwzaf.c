/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

/*
 * BU64747GWZAF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"

#define AF_DRVNAME "BU64747GWZAF_DRV"
#define AF_I2C_SLAVE_ADDR        0xEC
//#define EEPROM_I2C_SLAVE_ADDR    0xa0

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;
static unsigned int g_u4CheckDrvStatus = 0;


//#include "bu64747_coef_claf4007d.h"
//#include "bu64747_claf.h"
#include "bu64747_coef_CF.h"
#include "bu64747_FW.h"

/* #define BU64747_DEBUG */
#ifdef BU64747_DEBUG
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#endif



/***************************************************
// Global Define
// ***************************************************/
#define	TRUE		        (0)
#define	FALSE		        (-1)
#define	_SLV_FBAF_	     (0x76)


/***************************************************
// I2C Command Packet Definition
// ***************************************************/
	/* Packet Command OP code */
#define	_OP_FIRM_DWNLD	(0x80)
#define	_OP_Periphe_RW      (0x82)
#define	_OP_Memory__RW   (0x84)
#define	_OP_AD_TRNSFER	(0x86)
#define	_OP_COEF_DWNLD	(0x88)
#define	_OP_PrgMem__RD	(0x8A)
#define	_OP_SpecialCMD	    (0x8C)
#define	_cmd_8C_EI		        (0x0001)
#define	_cmd_8C_DI		        (0x0002)
#define	_cmd_8C_STRB	        (0x0004)
#define	_cmd_8C_TRI_SHT	    (0x0008)
#define	_cmd_8C_TRI_con	    (0x0010)
#define	_M_CD_CEFTYP        ( 0xCD)
#define	_M_F6_FIRMVER      (0xF6)
#define	_M_F7_FBAF_STS     (0xF7)



int s4AF_ReadReg_BU64747GWZAF(unsigned short int i2c_id, unsigned char *a_pSendData, unsigned short int a_sizeSendData, unsigned char *a_pRecvData, unsigned short int a_sizeRecvData)
{
	int i4RetValue;
	struct i2c_msg msg[2];
	LOG_INF("s4AF_ReadReg_BU64747GWZAF,  g_u4CheckDrvStatus(0x%x)\n", g_u4CheckDrvStatus);

	if (g_u4CheckDrvStatus > 1)
		return -1;

	g_pstAF_I2Cclient->addr = i2c_id >> 1;

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(g_pstAF_I2Cclient->adapter, msg, sizeof(msg)/sizeof(msg[0]));
       printk("i2c_transfer  end!\n");
	   
	if (i4RetValue != 2) {
		g_u4CheckDrvStatus++;
		LOG_INF("I2C Read failed!!\n");
		return -1;
	}
	return 0;
}

void RD_I2C(unsigned char slvadr, unsigned char size, unsigned char *dat, unsigned char *a_pRecvData)
{

	if (size == 1) {
		dat[1] = 0;
		s4AF_ReadReg_BU64747GWZAF(slvadr << 1, dat, 2, a_pRecvData, 2);
	} else if (size == 2) {
		s4AF_ReadReg_BU64747GWZAF(slvadr << 1, dat, 2, a_pRecvData, 2);  /**a_pRecvData  Big-endian*/
	}
	
}


int s4AF_WriteReg_BU64747GWZAF(unsigned short int i2c_id, unsigned char *a_pSendData, unsigned short int a_sizeSendData)
{
	int i4RetValue = 0;
	
	if (g_u4CheckDrvStatus > 1)
		return -1;

	g_pstAF_I2Cclient->addr = i2c_id >> 1;
	LOG_INF("s4AF_WriteReg_BU64747GWZAF,  g_pstAF_I2Cclient->addr(0x%x)\n", g_pstAF_I2Cclient->addr);

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, a_pSendData, a_sizeSendData);

	if (i4RetValue != a_sizeSendData) {
		g_u4CheckDrvStatus++;
		LOG_INF("I2C send failed!!, Addr = 0x%x, Data = 0x%x\n", a_pSendData[0], a_pSendData[1]);
		return -1;
	}
       printk("i2c_master_send  end!\n");

	return 0;
}


void WR_I2C(unsigned char slvadr, unsigned short int size, unsigned char *dat)
{
      printk("BU64747GWZAF_DRV WR_I2C slvadr(0x%x), size(%d),g_u4CheckDrvStatus(%d) \n",  slvadr, size, g_u4CheckDrvStatus);

	s4AF_WriteReg_BU64747GWZAF(slvadr << 1, dat, size);
}

 
/** bu64747_interface   */
/*- ///////////////////////////////////////////////////
// Peripheral Block Read Write*/
void I2C_func_PER_WRITE(unsigned char u08_adr , unsigned short int u16_dat )
{
	unsigned char out[4];

	out[0] = _OP_Periphe_RW;
	out[1] = u08_adr;
	out[2] = (u16_dat) & 0xFF;
	out[3] = (u16_dat >> 8) & 0xFF;

	WR_I2C(_SLV_FBAF_, 4, out);
}

void I2C_func_PER__READ( unsigned char u08_adr,  unsigned short int  *a_pu2Result)
{
	unsigned char u08_dat[2];
	unsigned char a_puBuff[2] = { 0, 0 };

	u08_dat[0] = _OP_Periphe_RW;	/* Op-code */
	u08_dat[1] = u08_adr;	/* target address */
	 
	 RD_I2C(_SLV_FBAF_, 2, u08_dat, a_puBuff);
	*a_pu2Result = ((a_puBuff[0] << 8) | (a_puBuff[1] ));

}
void  I2C_func_CEF_WRITE(unsigned char u08_adr, unsigned short u16_dat )
{		
	unsigned char out[4];	
	out[0] = _OP_Memory__RW;	
	out[1] = u08_adr;	
	out[2] = (u16_dat) & 0xFF;	
	out[3] = (u16_dat >> 8) & 0xFF;	
	WR_I2C(_SLV_FBAF_, 4, out);
}


/* Coefficient Block Read Write*/

void I2C_func_CEF__READ( unsigned char u08_adr,  unsigned short int  *a_pu2Result)
{
	unsigned char u08_dat[2];
	unsigned char a_puBuff[2] = { 0, 0 };

	u08_dat[0] = _OP_Memory__RW;	/* Op-code */
	u08_dat[1] = u08_adr;	/* target address */
	 
	 RD_I2C(_SLV_FBAF_, 2, u08_dat, a_puBuff);
	*a_pu2Result = ((a_puBuff[0] << 8) | (a_puBuff[1] ));
	
}


/* Main Primitive Function*/
void  I2C_write_FBAF(unsigned char u08_adr, unsigned short u16_dat )
{
	I2C_func_PER_WRITE( u08_adr, u16_dat );
}

void I2C_read__FBAF( unsigned char u08_adr,  unsigned short int  *a_pu2Result)
{
	I2C_func_PER__READ(u08_adr, a_pu2Result); 	
}


/* Driver Control Function*/

void  I2C_func_PON______(void)
{
	I2C_write_FBAF( 0xEF, 0x0080 );
}

void	 I2C_func_POFF_____(void)
{
	I2C_write_FBAF( 0xEF, 0x0000 );
}



void  I2C_func_DSP_START(void)
{
	unsigned char out[2];

	out[0] = _OP_SpecialCMD;
	out[1] = _cmd_8C_EI;	
	WR_I2C(_SLV_FBAF_, 2, out);
}


void	 func_CHK_VERSION(void)
{
	unsigned short	s16_dat;
	 I2C_read__FBAF( 0x5F, & s16_dat);
	printk("   IC_Version:%4X\n",s16_dat);
}

/*****************************************************
 **** Target CONTROL
 *****************************************************/
void  afTarget(unsigned short target )
{
	unsigned char out[3];
	out[0] = 0xF2;
	out[1] = (target>> 8) & 0xFF;
	out[2] = (target) & 0xFF;	

	printk(" BU64747GWZAF_DRV afTarget(0x%x)\n", target );


	WR_I2C(_SLV_FBAF_, 3, out);

}

void  setVCMPos(unsigned short target )
{
         I2C_func_CEF_WRITE( 0x30, 0x000D );

	  afTarget( target );
}





/* ***************************************************** */
/* **** Download the data */
/* ***************************************************** */
void  download(unsigned short int u16_type, unsigned short int u16_coef_type)
{
	/* Data Transfer Size per one I2C access */
	#define		DWNLD_TRNS_SIZE		(32)

	unsigned char temp[DWNLD_TRNS_SIZE + 1];
	unsigned short int block_cnt;
	unsigned short int total_cnt;
	unsigned short int lp;
	unsigned short int n;
	unsigned short int u16_i;

	if (u16_type == 1)   //Firmware Download
		n = DOWNLOAD_BIN_LEN;
	else
		n = DOWNLOAD_COEF_LEN;	/* Coef Download */

	block_cnt = n / DWNLD_TRNS_SIZE + 1;
	total_cnt = block_cnt;

	printk(" BU64747GWZAF_DRV u16_type(%d) block_cnt(%d)\n", u16_type,block_cnt );
	
	while (1) {
		/* Residual Number Check */
		if (block_cnt == 1)
		{
			lp = n % DWNLD_TRNS_SIZE;
			if	( lp == 0 ) 							
				 lp = DWNLD_TRNS_SIZE;									
		}	
		else
			lp = DWNLD_TRNS_SIZE;
		
     	 printk("BU64747GWZAF_DRV lp(%d)\n",lp );

		/* Transfer Data set */
		if (lp != 0) {
			if (u16_type == 1) {
				temp[0] = _OP_FIRM_DWNLD;                                                                                //_OP_FIRM_DWNLD
				for (u16_i = 1; u16_i <= lp; u16_i ++)
					temp[u16_i] = DOWNLOAD_BIN[(total_cnt - block_cnt) *	DWNLD_TRNS_SIZE + u16_i - 1];
			} else {
				temp[0] =  _OP_COEF_DWNLD;                                                                                         //_OP_COEF_DWNLD;
				for (u16_i = 1; u16_i <= lp; u16_i += 1)
					temp[u16_i] = DOWNLOAD_COEF[(total_cnt - block_cnt) * DWNLD_TRNS_SIZE + u16_i - 1];
			}

			/* Data Transfer */
			WR_I2C(_SLV_FBAF_, lp + 1, temp);
		}

		/* Block Counter Decrement */
		block_cnt = block_cnt - 1;

		if (block_cnt == 0)
			break;
	}
}




/*  *****************************************************
 **** Program Download Function
 *****************************************************/
void  func_PROGRAM_DOWNLOAD(unsigned char type )
{
	unsigned short sts;
	unsigned short u16_dat;

	download( type, 0 );
	
	I2C_func_CEF__READ( _M_F7_FBAF_STS, & sts);

	if ( ( sts&0x0004 ) == 0x0004 ) 
	{
		I2C_func_CEF__READ( _M_F6_FIRMVER , &u16_dat);
		printk("@@@ FirmwareDownload _OK_ @@@ \n" );
		printk("@@@ Firm Ver :       %.4d @@@ \n", u16_dat );
	}
	else
	{
		printk("XXX DOWNLOAD  NG NG NG NG XXX \n" );
	}
}

/*****************************************************
  **** COEF Download
*****************************************************/
void  func_COEF_DOWNLOAD(unsigned char coef_type )
{
	unsigned short u16_dat;																	
	download( 0, coef_type );

	I2C_func_CEF__READ( _M_CD_CEFTYP,  &u16_dat);
	printk("@@@ CoefDownload   _DONE_ @@@ \n" );		 
	printk("@@@ Coef Ver :     %d @@@ \n", u16_dat );	 
}


void bu64747gwzafInit(void)
{
     	 printk(" Enter bu64747gwzafInit \n" );

	I2C_func_POFF_____();					// reset the IC
	I2C_func_PON______()	;				// pon
	func_CHK_VERSION();						// ver check
       msleep(100);

	
	func_PROGRAM_DOWNLOAD( 1 );				// 
	func_COEF_DOWNLOAD( 0 );
	I2C_func_DSP_START();
}

void bu64747gwzafStandby(void)
{
	I2C_func_POFF_____()	;				// reset the IC
}



static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		bu64747gwzafInit();
		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

//	LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); 
	setVCMPos((unsigned short)g_u4TargetPosition);
	
	spin_lock(g_pAF_SpinLock);
	g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
	spin_unlock(g_pAF_SpinLock);
	
	

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFPara(__user stAF_MotorCmd * pstMotorCmd)
{
	stAF_MotorCmd stMotorCmd;

	if (copy_from_user(&stMotorCmd , pstMotorCmd, sizeof(stMotorCmd)))
		LOG_INF("copy to user failed when getting motor command\n");

	LOG_INF("Motor CmdID : %x\n", stMotorCmd.u4CmdID);

	LOG_INF("Motor Param : %x\n", stMotorCmd.u4Param);

	switch (stMotorCmd.u4CmdID) {
	case 1:
		//setOISMode((int)stMotorCmd.u4Param); /* 1 : disable */
		break;
	}

	return 0;
}

/* ////////////////////////////////////////////////////////////// */
int BU64747GWZAF_getHall(void)
{
	unsigned short NRMHAL = 0;

	if (g_pAF_Opened!= NULL && *g_pAF_Opened == 2) {
		I2C_func_CEF__READ( 0x2C, &NRMHAL);
	}

	return NRMHAL;
}

int BU64747GWZAF_getCurDet(void)
{
	unsigned short CURDET = 0;

	if (g_pAF_Opened!= NULL && *g_pAF_Opened == 2) {
		I2C_func_CEF__READ( 0x3B, &CURDET);
	}

	return CURDET;
}

int BU64747GWZAF_getCurPos(void)
{
	int CURPOS= 0;

	if (g_pAF_Opened!= NULL && *g_pAF_Opened == 2) {
		CURPOS = (int)g_u4CurrPosition;
	}

	return CURPOS;
}

long BU64747GWZAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	case AFIOC_S_SETPARA:
		i4RetValue = setAFPara((__user stAF_MotorCmd *) (a_u4Param));
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int BU64747GWZAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		bu64747gwzafStandby();
		msleep(20);
	}

	g_u4CheckDrvStatus = 0;

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

void BU64747GWZAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	LOG_INF("SetI2Cclient\n");
}
