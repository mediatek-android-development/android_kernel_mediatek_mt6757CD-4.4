/*
 * 
 * BU24228GWLAF voice coil motor driver
 * 
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"

#include "bu24228gwlaf_data1.h"
#include "bu24228gwlaf_data2_7B.h"
#include "bu24228gwlaf_data2_5B.h"

#define AF_DRVNAME "BU24228GWLAF_DRV"
#define AF_I2C_SLAVE_ADDR        0x7C

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#define OIS_7B 1
#define OIS_5B 2

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static bool OIS_status = 0;

static unsigned long g_u4AF_INF;
//	==> [BY86] 9-bit AF
static unsigned long g_u4AF_MACRO = 511;
//	<== [BY86] 9-bit AF
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int OIS_DATA_FLAG=0;
static int isOpened = 0;
static bool initState = false;

static int OIS_VERSION_EEPROM(u16 eeprom_reg)
{
	int i;
	u8 u8data = 0;
	u8 eeprom_data[44];
	u8 puSendCmd2[10];

	g_pstAF_I2Cclient->addr = 0x51;
	for(i=0; i<2; i++)
	{
		puSendCmd2[0] = (u8)((eeprom_reg>>8)&0xFF);
		puSendCmd2[1] = (u8)(eeprom_reg&0xFF);

		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2) < 0) {
			LOG_INF("read I2C send failed!!\n");
			return -1;
		}

		if (i2c_master_recv(g_pstAF_I2Cclient, &u8data, 1) < 0) {
			LOG_INF("Read I2C OIS EEPROM failed!!\n");
			return -1;
		}
		eeprom_data[i]=u8data;
		LOG_INF("Get OIS EEPROM 0x%x%x=%d\n", puSendCmd2[0], puSendCmd2[1], eeprom_data[i]);

		eeprom_reg=eeprom_reg+1;
	}

	if(eeprom_data[0]>=16 && eeprom_data[1]>=4) {
		OIS_DATA_FLAG = OIS_7B;
		LOG_INF("The version is 7B\n");
	}
	else {
		OIS_DATA_FLAG = OIS_5B;
		LOG_INF("The version is 5B\n");
	}

	return 0;
}
static int Download_OIS_EEPROM(u16 eeprom_reg)
{
	int i, j;
	u8 u8data = 0;
	u8 eeprom_data[44];
	u8 puSendCmd2[10];

	g_pstAF_I2Cclient->addr = 0x51;

	for(i=0; i<=43; i++)
	{
		puSendCmd2[0] = (u8)((eeprom_reg>>8)&0xFF);
		puSendCmd2[1] = (u8)(eeprom_reg&0xFF);

		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2) < 0) {
			LOG_INF("read I2C send failed!!\n");
			return -1;
		}

		if (i2c_master_recv(g_pstAF_I2Cclient, &u8data, 1) < 0) {
			LOG_INF("Read I2C OIS EEPROM failed!!\n");
			return -1;
		}
		eeprom_data[i]=u8data;
		LOG_INF("Get OIS EEPROM 0x%X%X=%x\n", puSendCmd2[0], puSendCmd2[1], eeprom_data[i]);
		eeprom_reg=eeprom_reg+1;
	}

	eeprom_reg=0x1DC0;
	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	for( i=0; i<11; i++)
	{
		puSendCmd2[0]= (u8)((eeprom_reg>>8)&0xFF) ;
		puSendCmd2[1]= (u8)( eeprom_reg&0xFF) ;
		puSendCmd2[2] = eeprom_data[i*4];
		puSendCmd2[3] = eeprom_data[i*4+1];
		puSendCmd2[4] = eeprom_data[i*4+2];
		puSendCmd2[5] = eeprom_data[i*4+3];

		for(j=0; j<=5; j++)
		{
			LOG_INF("Write OIS Register; puSendCmd[%d]=0x%x\n", j, puSendCmd2[j]);
		}

		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 6) < 0) {
			LOG_INF("OIS_WriteI2C Download 51 failed!!\n");
			return -1;
		}
		eeprom_reg=eeprom_reg+4;
	}

	return 0;
}

static int OIS_DownloadData(u8 data_no)
{
	int i;
	u8 puSendCmd[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	u16 data1_start_addr=0x0000;

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	LOG_INF("OIS_Download Data START_%d!!\n", (int)data_no);

	if (data_no == 1)
	{
		for( i=0; i<238; i++)
		{
			puSendCmd[0]= (u8)((data1_start_addr>>8)&0xFF) ;
			puSendCmd[1]= (u8)(data1_start_addr&0xFF) ;
			puSendCmd[2] = bu24228gwlaf_data1[i*4];
			puSendCmd[3] = bu24228gwlaf_data1[i*4+1];
			puSendCmd[4] = bu24228gwlaf_data1[i*4+2];
			puSendCmd[5] = bu24228gwlaf_data1[i*4+3];

			if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 6) < 0) {
				LOG_INF("OIS_Write I2C Download 51 failed!!\n");
				return -1;
			}
			data1_start_addr=data1_start_addr+4;
		}
	}
	else
	{
		data1_start_addr = 0x1C00;

		for( i=0; i<112; i++)
		{
			puSendCmd[0]= (u8)((data1_start_addr>>8)&0xFF) ;
			puSendCmd[1]= (u8)(data1_start_addr&0xFF) ;
			if(OIS_DATA_FLAG == OIS_7B) {
				puSendCmd[2] = bu24228gwlaf_data2_7B[i*4];
				puSendCmd[3] = bu24228gwlaf_data2_7B[i*4+1];
				puSendCmd[4] = bu24228gwlaf_data2_7B[i*4+2];
				puSendCmd[5] = bu24228gwlaf_data2_7B[i*4+3];
			}
			else if(OIS_DATA_FLAG == OIS_5B) {
				puSendCmd[2] = bu24228gwlaf_data2_5B[i*4];
				puSendCmd[3] = bu24228gwlaf_data2_5B[i*4+1];
				puSendCmd[4] = bu24228gwlaf_data2_5B[i*4+2];
				puSendCmd[5] = bu24228gwlaf_data2_5B[i*4+3];	
			}
			else {
				puSendCmd[2] = bu24228gwlaf_data2_7B[i*4];
				puSendCmd[3] = bu24228gwlaf_data2_7B[i*4+1];
				puSendCmd[4] = bu24228gwlaf_data2_7B[i*4+2];
				puSendCmd[5] = bu24228gwlaf_data2_7B[i*4+3];	
			}

			if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 6) < 0) {
				LOG_INF("OIS_Write I2C Download 51 failed!!\n");
				return -1;
			}

			data1_start_addr=data1_start_addr+4;
		}
	}
	LOG_INF("OIS_Download Data END!!\n");

	return 0;
}

static int OIS_ReadI2C_32bit(u16 ois_reg, u32 *data)
{
	int i4RetValue = 0;
	unsigned char u08_reg[2];
	unsigned char u08_data[4];
	struct i2c_msg msg[2];

	u08_reg[0] = ((ois_reg >> 8) & 0xFF);
	u08_reg[1] = (ois_reg & 0xFF);

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = u08_reg;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 4;
	msg[1].buf = u08_data;

	i4RetValue = i2c_transfer(g_pstAF_I2Cclient->adapter, msg, sizeof(msg)/sizeof(msg[0]));
	if (i4RetValue) {
		LOG_INF("i2c_transfer  end!\n");
	}
	else if (i4RetValue != 2) {
		LOG_INF("I2C Read failed!!\n");
		return -1;
	}

	*data = (((msg[1].buf[0] << 24) & 0xFF000000) + ((msg[1].buf[1] << 16) & 0xFF0000) + ((msg[1].buf[2] << 8) & 0xFF00) + (msg[1].buf[3] & 0xFF));

	LOG_INF("OIS_ReadI2C_32bit; pBuff 0~4=0x%x %x %x %x\n", msg[1].buf[0], msg[1].buf[1], msg[1].buf[2], msg[1].buf[3]);
	LOG_INF("OIS_ReadI2C_32bit; 0x%x, 0x%x\n", ois_reg, *data);

	return 0;
}

static int OIS_ReadI2C_16bit(u16 ois_reg, u16 *data)
{
	int i4RetValue = 0;
	unsigned char u08_reg[2];
	unsigned char u08_data[2];
	struct i2c_msg msg[2];

	u08_reg[0] = ((ois_reg >> 8) & 0xFF);
	u08_reg[1] = (ois_reg & 0xFF);

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = u08_reg;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = u08_data;

	i4RetValue = i2c_transfer(g_pstAF_I2Cclient->adapter, msg, sizeof(msg)/sizeof(msg[0]));
	if (i4RetValue) {
		LOG_INF("i2c_transfer  end!\n");
	}
	else if (i4RetValue != 2) {
		LOG_INF("I2C Read failed!!\n");
		return -1;
	}

	if(ois_reg == 0x60F2)
		*data = ((((msg[1].buf[0] << 8 ) & 0xFF00) + (msg[1].buf[1] & 0xFF)) & 0x1FF);
	else
		*data = (((msg[1].buf[0] << 8 ) & 0xFF00) + (msg[1].buf[1] & 0xFF));

	LOG_INF("OIS_ReadI2C_16bit 0x%x, 0x%x\n", ois_reg, *data);

	return 0;
}

static int OIS_ReadI2C_8bit(u16 ois_reg, u8 *data)
{
	int i4RetValue;
	unsigned char u08_reg[2];
	struct i2c_msg msg[2];

	u08_reg[0] = ((ois_reg >> 8) & 0xFF);
	u08_reg[1] = (ois_reg & 0xFF);

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = u08_reg;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data;

	i4RetValue = i2c_transfer(g_pstAF_I2Cclient->adapter, msg, sizeof(msg)/sizeof(msg[0]));
	if (i4RetValue) {
		LOG_INF("i2c_transfer  end!\n");
	}
	else if (i4RetValue != 2) {
		LOG_INF("I2C Read failed!!\n");
		return -1;
	}

	*data = msg[1].buf[0];
	LOG_INF("OIS_ReadI2C_8bit 0x%x, 0x%x\n", ois_reg, *data);

	return 0;
}

int Check_OIS_Ready(void)
{
	u8 OIS_Ready=0;
//	==> [BY86] for ESTA 3A startPreview timeout
	int retry=100;
//	<== [BY86] for ESTA 3A startPreview timeout
	while(retry>0)
	{
		OIS_ReadI2C_8bit(0x6024, &OIS_Ready);
		LOG_INF("Check_OIS_Ready, 0x6024=%d retry = %d\n", (int)OIS_Ready,retry);

		if(OIS_Ready==1)
			break;
		mdelay(5);
		retry--;
	}
	return OIS_Ready;
}

int OIS_WriteI2C(u8 length, u16 ois_reg, u32 data)
{
	u8 puSendCmd[3] = { (u8)((ois_reg>>8)&0xFF), (u8)(ois_reg&0xFF), (u8)(data&0xFF) };
	u8 puSendCmd2[6] = { (u8)((ois_reg>>8)&0xFF), (u8)(ois_reg&0xFF), (u8)((data>>24)&0xFF), (u8)((data>>16)&0xFF), (u8)((data>>8)&0xFF), (u8)(data&0x0FF) };
	LOG_INF("WriteI2C 0x%x, 0x%x, 0x%x\n", length, ois_reg, data);

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	if (length == 0)
	{
		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3) < 0) {
			LOG_INF("OIS_WriteI2C 1Byte failed!!\n");
			return -1;
		}
	}
	else
	{
		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 6) < 0)
		{
			LOG_INF("OIS_WriteI2C 4Byte failed!!\n");
			return -1;
		}
	}

	return 0;
}

static int AF_Position_WriteI2C(u16 ois_reg, u16 data)
{
	int i;
	u8 puSendCmd2[4] = { (u8)((ois_reg>>8)&0xFF), (u8)(ois_reg&0xFF), (u8)((data>>8)&0xFF), (u8)(data&0x0FF) };
	LOG_INF("AF_Position_WriteI2C 0x%x=0x%x\n", ois_reg, data);

	for(i=0; i<=3; i++)
	{
		LOG_INF("AF_Position_WriteI2C puSendCmd2[%d]=0x%x\n", i, puSendCmd2[i]);
	}

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 4) < 0)
	{
		LOG_INF("OIS_WriteI2C failed!!\n");
		return -1;
	}

	return 0;
}

static inline int getAFInfo(__user stAF_MotorInfo * pstMotorInfo)
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
	int ret = 0;
	int OIS_Ready=3;
	u32 OIS_chksum=0x15B74;

	LOG_INF("BU24228GWLAF moveAF !!! \n");

	isOpened = 1;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) 
	{
		unsigned short InitPos;

		if(initState == true)
		{
			LOG_INF("Skip OIS Init\n");
		}
		else //Init OIS
		{
			LOG_INF("Init OIS Only One time~\n");
			LOG_INF("++++++++\n");
			OIS_VERSION_EEPROM(0x0011);
			LOG_INF("--------\n");

			OIS_WriteI2C(5, 0xF020, 0x01011810);
			OIS_WriteI2C(5, 0xF024, 0x00010002);
			OIS_WriteI2C(5, 0xF028, 0x000000E1);
			OIS_WriteI2C(5, 0xF02C, 0x0B130000);
			mdelay(1);
			OIS_WriteI2C(5, 0xF02C, 0x0C800000);

			OIS_Ready=Check_OIS_Ready();
			LOG_INF("Gyro Activation Done, OIS_Ready=%d \n", OIS_Ready);

			OIS_WriteI2C(0, 0xF010, 0);
			mdelay(1);

			OIS_DownloadData(1);
			OIS_DownloadData(2);

			OIS_ReadI2C_32bit(0xF008, &OIS_chksum);

			Download_OIS_EEPROM(0x0780);

			OIS_WriteI2C(0, 0xF006, 0);

			OIS_Ready=Check_OIS_Ready();
			LOG_INF("Download Complete, OIS_Ready=%d \n", OIS_Ready);

			OIS_WriteI2C(0, 0x6020, 1);
			mdelay(100);

			//OIS_Ready=Check_OIS_Ready();
			//LOG_INF("Servo On, OIS_Ready=%d \n", OIS_Ready);

			//	AF Init
			OIS_WriteI2C(0, 0x60F1, 6);
			//	==> [BY86] Workaround: AF range 0~254
			AF_Position_WriteI2C(0x60F2, 50);
			//	<== [BY86] Workaround: AF range 0~254

			OIS_WriteI2C(0, 0x6023, 0);

//	==> [BY86] Unmark this after tuning OK
//			OIS_WriteI2C(0, 0x6021, 0x03);//zero shutter on
//			OIS_WriteI2C(0, 0x6020, 0x02);//OIS on
//	<== [BY86] Unmark this after tuning OK
            initState = true;
		}
		//	<== [BY86] Fix capture time 30s

		ret = OIS_ReadI2C_16bit(0x60F2, &InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		}
		else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	LOG_INF("move [curr] %ld [target] %ld\n", g_u4CurrPosition, g_u4TargetPosition);

//	==> [BY86] workaround for CTS verifier Switch resolution too slow 
	if (g_u4TargetPosition == 0)
		g_u4TargetPosition = 1;	
//	<== [BY86] workaround for CTS verifier Switch resolution too slow

	if (AF_Position_WriteI2C(0x60F2, (u16)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	}
	else {
		LOG_INF("set I2C failed when moving the motor\n");
		LOG_INF("set I2C failed when moving the OIS AF motor\n");
	}

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
//	==> [BY86] OIS control flow
static inline int setOIS(stAF_MotorCmd *pstMotorCmd)
{
	u8 servoState = 0;
	u8 oisMode = 0;

	LOG_INF("*****setOIS*******\n");
	LOG_INF("pstMotorCmd->u4Param = %u", pstMotorCmd->u4Param);
	if(pstMotorCmd->u4Param == 0) {
		OIS_ReadI2C_8bit(0x6020, &servoState);
		OIS_ReadI2C_8bit(0x6021, &oisMode);
		if(servoState == 1 && oisMode != 0x3) {
			OIS_WriteI2C(0, 0x6020, 0x01);// Servo on/ OIS off
			LOG_INF("Servo On!!");
			mdelay(100);
			OIS_WriteI2C(0, 0x6021, 0x03);// zero shutter on
			LOG_INF("OIS mode = 0x03!!");
			OIS_WriteI2C(0, 0x6020, 0x02);// OIS on
			OIS_status=1;
			LOG_INF("OIS On!!");
		}
	}
	else if(pstMotorCmd->u4Param == 1) {
		OIS_ReadI2C_8bit(0x6020, &servoState);
		OIS_ReadI2C_8bit(0x6021, &oisMode);
		if(servoState == 2 && oisMode == 0x3) {
			OIS_WriteI2C(0, 0x6020, 0x01);// Servo on/ OIS off
			mdelay(100);
			OIS_WriteI2C(0, 0x6021, 0x0);// zero shutter off
			OIS_status=0;
			LOG_INF("OIS Off!!");
		}
	}
	else
		LOG_INF("setOIS didn't set anything...\n");

	return 0;
}
//	<== [BY86] OIS control flow

/* ////////////////////////////////////////////////////////////// */
long BU24228GWLAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
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
//	==> [BY86] OIS control flow
	case AFIOC_S_SETPARA:
		i4RetValue = setOIS((stAF_MotorCmd *) (a_u4Param));
//	<== [BY86] OIS control flow
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
int BU24228GWLAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		moveAF(100);
		LOG_INF("Wait\n");
		msleep(5);

		moveAF(50);
		LOG_INF("Wait\n");
		msleep(5);

		moveAF(25);
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	isOpened = 0;
	initState = false;

	LOG_INF("End\n");

	return 0;
}

void BU24228GWLAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}

ssize_t OIS_Switch_show (struct device *dev, struct device_attribute *attr, char *buf)
{		
	ssize_t ret = 0;
	sprintf( buf, "OIS_Switch=%d\n", OIS_status );
	ret = strlen( buf) + 1;

	return ret;
}

ssize_t  OIS_Switch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	//int OIS_Ready=3;
	bool Sitch;

	LOG_INF("OIS_Switch_store enter......\n");
	Sitch=buf[0]-'0';

	if(isOpened == 1)
	{
		if( Sitch == 1)
		{
			OIS_status=1;

			OIS_WriteI2C(0, 0x6020, 0x01);// Servo on/ OIS off
			OIS_WriteI2C(0, 0x6021, 0x03);// zero shutter on
			mdelay(100);
			OIS_WriteI2C(0, 0x6020, 0x02);// OIS on
			//OIS_Ready=Check_OIS_Ready();
			//LOG_INF("OIS_Switch_store Sitch=%d, OIS_Ready=%d\n", OIS_status, OIS_Ready);
		}
		else if(Sitch == 0)
		{
			OIS_status=0;

			OIS_WriteI2C(0, 0x6020, 0x01);// Servo on/ OIS off
			mdelay(100);
			//OIS_Ready=Check_OIS_Ready();
			//LOG_INF("OIS_Switch_store Sitch=%d, OIS_Ready=%d\n", OIS_status, OIS_Ready);
		}
		else
			LOG_INF("you send neither 1 nor 0 buf[0]=%d\n",buf[0]);
	}
	else
		LOG_INF("Camera didn't work now...\n");

	return count;
}

#if 0
ssize_t OIS_ZeroShutter_store (struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t   ret = 0;
	LOG_INF("OIS_ZeroShutter_store\n");

	OIS_WriteI2C(0, 0x6020, 0x01);// Servo on/ OIS off
	mdelay(100);
	OIS_WriteI2C(0, 0x6021, 0x03);// zero shutter on
	mdelay(1);
	OIS_WriteI2C(0, 0x6020, 0x02);// OIS on

	OIS_status =1;

	return ret;
}

ssize_t OIS_Movie_store (struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t   ret = 0;
	LOG_INF("OIS_Movie_store\n");

	OIS_WriteI2C(0, 0x6020, 0x01);// Servo on/ OIS off
	mdelay(100);
	OIS_WriteI2C(0, 0x6021, 0x61);// movie mode on
	mdelay(1);
	OIS_WriteI2C(0, 0x6020, 0x02);// OIS on

	OIS_status =1;

	return ret;
}

ssize_t FrontAF_store (struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int step=0;

	sscanf(buf,"%d",&step);
	LOG_INF("------- step = %d\n",step);

	AF_Position_WriteI2C(0x60F2, step);

	return count;
}
#endif
