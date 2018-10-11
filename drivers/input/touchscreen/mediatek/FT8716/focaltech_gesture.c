/****************************************************
*
* Filename      : focaltech_gesture.c
*
* Author        : JiangTingyu - Jiangty08@gmail.com
* Description   : ---
* Create        : 2017-10-19 17:38:22
*
****************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include<linux/wakelock.h>
#define  TPD_PACKET_LENGTH					128
#define PROC_READ_DRAWDATA					10
extern struct i2c_client *fts_i2c_client;
#include "focaltech_drv_printlog.h"

/* [lichenggang start] add for gesture_id judge*/
#ifdef FTS_GESTRUE_EN
//#ifdef SPEC_GESTURE_COORD_COMPRESS_ENABLED
#define UINT16 unsigned short
#define UINT8 unsigned char
static UINT16 usPointXMax;
static UINT16 usPointYMax;
static UINT16 usPointXMin;
static UINT16 usPointYMin;
static UINT16 usPointXMaxatY;
static UINT16 usPointYMinatX;
static UINT16 usPointXMinatY;
static UINT16 usPointYMaxatX;
#endif

static unsigned short coordinate_report[14] = {0};
static uint32_t gesture;
static uint32_t gesture_upload;
static unsigned short coordinate_x[256]    = {0};
static unsigned short coordinate_y[256]    = {0};
static unsigned short coordinate_doblelion_1_x[256] = {0};
static unsigned short coordinate_doblelion_2_x[256] = {0};
static unsigned short coordinate_doblelion_1_y[256] = {0};
static unsigned short coordinate_doblelion_2_y[256] = {0};
static int global_gesture_id=0;
static int gesture_direction=0;

/* [jiangtingyu start]optimize gesture*/
#define RAW_DATA_LENGTH (FTS_GESTRUE_POINTS * 5)
static unsigned char raw_data_buf[RAW_DATA_LENGTH] = { 0 };
/* [jiangtingyu end] */

/* [yanlin start] optimize gesture */
#define TP_MAX_X_VALUE		1080
#define TP_MAX_Y_VALUE		1920
static int TP_DATA_ERROR = 0;
#define PIXEL_DISTANCE_WIDTH   63
#define PIXEL_DISTANCE_HEIGHT  63
#define GESTURE_LIMIT_WIDTH    17000
#define GESTURE_LIMIT_HEIGHT   17000
/* [yanlin end] */

/* [lichenggang start] add for support more gesture */
#ifdef FTS_GESTRUE_EN
static void _get_coordinate(unsigned char *buf, int pointnum)
{
	int i;
	for(i = 0;i < pointnum;i++){
		coordinate_x[i] =  (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[4 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[5 + (4 * i)]) & 0xFF);
		TPD_ERR("%s:pointx[%d] = %d,pointy[%d] = %d\n",__func__,i,coordinate_x[i],i,coordinate_y[i]);
	}
	if((global_gesture_id==0x31) || (global_gesture_id==0x32)||(global_gesture_id==0x54)){
		if(coordinate_x[0]<coordinate_x[pointnum-1])
			gesture_direction=0;
		else
			gesture_direction=1;
	}else if((global_gesture_id==0x51)||(global_gesture_id==0x52)){
		if(coordinate_y[0]<coordinate_y[pointnum-1])
			gesture_direction=0;
		else
			gesture_direction=1;
	}
}
#endif
/* [lichenggang end] */


#ifdef FTS_GESTRUE_EN
static void SpecialGestureTraceMaxAndMin(UINT16 usCurPointX, UINT16 usCurPointY)
{
	if (usCurPointX < usPointXMin)
	{
		usPointXMin = usCurPointX;
		usPointXMinatY= usCurPointY;
	}

	if (usCurPointX > usPointXMax)
	{
		usPointXMax = usCurPointX;
		usPointXMaxatY= usCurPointY;
	}

	if (usCurPointY < usPointYMin)
	{
		usPointYMin = usCurPointY;
		usPointYMinatX= usCurPointX;
	}

	if (usCurPointY > usPointYMax)
	{
		usPointYMax = usCurPointY;
		usPointYMaxatX= usCurPointX;
	}
}
#endif

#ifdef FTS_GESTRUE_EN
static void SpecGestureTraceGet(UINT8 uiPointSum)
{
	UINT16 usCurPointX;
	UINT16 usCurPointY;
	UINT8 i = 0;

	usPointXMax = coordinate_x[0];
	usPointXMin = coordinate_x[0];
	usPointYMax = coordinate_y[0];
	usPointYMin = coordinate_y[0];

	for( i = 0; i< uiPointSum; i++)
	{
		usCurPointX = coordinate_x[i];
		usCurPointY = coordinate_y[i];
		SpecialGestureTraceMaxAndMin(usCurPointX,usCurPointY);
	}
	printk("%s:usPointXMax =%d,usPointXMin =%d,usPointYMax=%d,usPointYMin =%d\n",
						__func__,usPointXMax,usPointXMin,usPointYMax,usPointYMin);

	/* [yanlin start] optimize gesture data */
	if ((usPointXMax > TP_MAX_X_VALUE) || (usPointXMin == 0) || (usPointYMax > TP_MAX_Y_VALUE) || (usPointYMin == 0)) {
		printk("%s: gesture data is error\n", __func__);
		TP_DATA_ERROR = 1;
	}
	/* [yanlin end] */
}
#endif

#ifdef FTS_GESTRUE_EN
static int _compress_coordinate(unsigned char *buf, int pointnum)
{
	int gestrue_id = buf[0];
	int retPointNum=pointnum;
	UINT16 uiTempPointSum = pointnum; // buf[1];

	usPointXMin = 0xFFFF;
	usPointYMin = 0xFFFF;
	usPointXMax = 0;
	usPointYMax = 0;

	SpecGestureTraceGet(pointnum);
	pr_err("%s_1:usPointYMax=%d\n",__func__,usPointYMax);

	/* [yanlin start] Gesture limit */
	if (gestrue_id >= GESTURE_O) {
		if ((((usPointXMax - usPointXMin) * PIXEL_DISTANCE_WIDTH) < GESTURE_LIMIT_WIDTH)
				|| (((usPointYMax - usPointYMin) * PIXEL_DISTANCE_HEIGHT) < GESTURE_LIMIT_HEIGHT)) {
			printk("%s: gesture limit error\n", __func__);
			TP_DATA_ERROR = 1;
		}
	}
	/* [yanlin end] */

	if((gestrue_id <= GESTURE_DOWN) && (gestrue_id >= GESTURE_LEFT) && pointnum>2)
	{
	// buf[1] = 2;
	retPointNum =2;
	coordinate_x[1] = coordinate_x[uiTempPointSum-1];
	coordinate_y[1] = coordinate_y[uiTempPointSum-1];
	}
	else if((gestrue_id==GESTURE_O)||(gestrue_id==GESTURE_CW_O))// if char is 'o',make up the PointNum of 6
	{
		// buf[1]=6;
		retPointNum =6;
		coordinate_x[1] = usPointYMinatX;
		coordinate_y[1] = usPointYMin ;
		// Xmin
		coordinate_x[2] = usPointXMin;
		coordinate_y[2] = usPointXMinatY;
		// Ymax
		coordinate_x[3] = usPointYMaxatX;
		coordinate_y[3] = usPointYMax ;
		// xmax
		coordinate_x[4] = usPointXMax;
		coordinate_y[4] = usPointXMaxatY;
		// end point
		coordinate_x[5] = coordinate_x[uiTempPointSum-1];
		coordinate_y[5] = coordinate_y[uiTempPointSum-1];
	}else{
		if((gestrue_id>=GESTURE_LEFT_V)&&(gestrue_id<=GESTURE_V) && pointnum!=3)
		{
			if(gestrue_id==GESTURE_LEFT_V) // '<'
			{
				coordinate_x[1] = usPointXMin;
				coordinate_y[1] = usPointXMinatY;
			}
			else if(gestrue_id==GESTURE_RIGHT_V)// '>'
			{
				coordinate_x[1] = usPointXMax;
				coordinate_y[1] = usPointXMaxatY;
			}
			/* else if(gestrue_id==GESTURE_DOWN_V) // '^'
			{
				coordinate_x[1] = usPointYMinatX;
				coordinate_y[1] = usPointYMin;
			} */
			else if(gestrue_id==GESTURE_V) // 'v'
			{
				coordinate_x[1] = usPointYMaxatX;
				coordinate_y[1] = usPointYMax;
			}
			coordinate_x[2] = coordinate_x[uiTempPointSum-1]; // g_stSpecGesStatus
			coordinate_y[2] = coordinate_y[uiTempPointSum-1]; // g_stSpecGesStatus
			//buf[1] = 3;
			retPointNum =3;
		}else if(((gestrue_id==GESTURE_W)||(gestrue_id==GESTURE_M))&&(pointnum!=5)){
			//0x31: 'W'  ,0x32:'M'
			UINT16  usinflectionPointNum =0;
			UINT16 stepX;
			UINT16 usStartPointX = usPointXMin;
			UINT16 usEndPointX =  usPointXMax;
			usinflectionPointNum=1;
			stepX = abs(usEndPointX-usStartPointX)/4;
			if(gestrue_id==GESTURE_W){//  'W'
				coordinate_x[0] = usStartPointX;
				coordinate_y[0] = usPointYMin;
				coordinate_x[1] = usStartPointX+stepX;
				coordinate_y[1] = usPointYMax;
				coordinate_x[2] = usStartPointX+2*stepX;
				coordinate_y[2] = usPointYMin;
				coordinate_x[3] = usStartPointX+3*stepX;
				coordinate_y[3] = usPointYMax;
				coordinate_x[4] = usEndPointX;
				coordinate_y[4] = usPointYMin;
			}else{// 'M'
				coordinate_x[0] = usStartPointX;
				coordinate_y[0] = usPointYMax;
				coordinate_x[1] = usStartPointX+stepX;
				coordinate_y[1] = usPointYMin;
				coordinate_x[2] = usStartPointX+2*stepX;
				coordinate_y[2] = usPointYMax;
				coordinate_x[3] = usStartPointX+3*stepX;
				coordinate_y[3] = usPointYMin;
				coordinate_x[4] = usEndPointX;
				coordinate_y[4] = usPointYMax ;
			}
			//buf[1] = 5;
			retPointNum =5;
		}
	}
	return retPointNum;
}
#endif

#ifdef FTS_GESTRUE_EN
static void _get_coordinate_report(unsigned char *buf, int pointnum)
{
	int clk;
	int gestrue_id = buf[0];
	pr_err("pointnum=%d\n",pointnum);
	_get_coordinate(buf, pointnum);
#ifdef FTS_GESTRUE_EN
	pointnum=_compress_coordinate(buf, pointnum);
#endif

	if((gestrue_id != GESTURE_O)&&(gestrue_id!=GESTURE_CW_O)){
		if(!gesture_direction){
			coordinate_report[1] = coordinate_x[0];
			coordinate_report[2] = coordinate_y[0];
			coordinate_report[3] = coordinate_x[pointnum-1];
			coordinate_report[4] = coordinate_y[pointnum-1];
			coordinate_report[5] = coordinate_x[1];
			coordinate_report[6] = coordinate_y[1];
			coordinate_report[7] = coordinate_x[2];
			coordinate_report[8] = coordinate_y[2];
			coordinate_report[9] = coordinate_x[3];
			coordinate_report[10] = coordinate_y[3];
			coordinate_report[11] = coordinate_x[4];
			coordinate_report[12] = coordinate_y[4];
		}else{
			if(gestrue_id==GESTURE_M || gestrue_id==GESTURE_W)
			{
				coordinate_report[1] = coordinate_x[pointnum-1];
				coordinate_report[2] = coordinate_y[pointnum-1];
				coordinate_report[3] = coordinate_x[0];
				coordinate_report[4] = coordinate_y[0];
				coordinate_report[5] = coordinate_x[3];
				coordinate_report[6] = coordinate_y[3];
				coordinate_report[7] = coordinate_x[2];
				coordinate_report[8] = coordinate_y[2];
				coordinate_report[9] = coordinate_x[1];
				coordinate_report[10] = coordinate_y[1];
				coordinate_report[11] = coordinate_x[0];
				coordinate_report[12] = coordinate_y[0];
			}else if(gestrue_id==GESTURE_V || gestrue_id==GESTURE_LEFT_V || gestrue_id==GESTURE_RIGHT_V){
				coordinate_report[1] = coordinate_x[0];
				coordinate_report[2] = coordinate_y[0];
				coordinate_report[3] = coordinate_x[pointnum-1];
				coordinate_report[4] = coordinate_y[pointnum-1];
				coordinate_report[5] = coordinate_x[1];
				coordinate_report[6] = coordinate_y[1];
				coordinate_report[7] = coordinate_x[2];
				coordinate_report[8] = coordinate_y[2];
				coordinate_report[9] = coordinate_x[3];
				coordinate_report[10] = coordinate_y[3];
				coordinate_report[11] = coordinate_x[4];
				coordinate_report[12] = coordinate_y[4];
			}else {
				coordinate_report[1] = coordinate_x[pointnum-1];
				coordinate_report[2] = coordinate_y[pointnum-1];
				coordinate_report[3] = coordinate_x[0];
				coordinate_report[4] = coordinate_y[0];
				coordinate_report[5] = coordinate_x[3];
				coordinate_report[6] = coordinate_y[3];
				coordinate_report[7] = coordinate_x[2];
				coordinate_report[8] = coordinate_y[2];
				coordinate_report[9] = coordinate_x[1];
				coordinate_report[10] = coordinate_y[1];
				coordinate_report[11] = coordinate_x[0];
				coordinate_report[12] = coordinate_y[0];
			}
		}
	}else{
		coordinate_report[1] = coordinate_x[0];
		coordinate_report[2] = coordinate_y[0];
		coordinate_report[3] = coordinate_x[pointnum-2];
		coordinate_report[4] = coordinate_y[pointnum-2];
		coordinate_report[5] = coordinate_x[1];
		coordinate_report[6] = coordinate_y[1];
		coordinate_report[7] = coordinate_x[2];
		coordinate_report[8] = coordinate_y[2];
		coordinate_report[9] = coordinate_x[3];
		coordinate_report[10] = coordinate_y[3];
		coordinate_report[11] = coordinate_x[4];
		coordinate_report[12] = coordinate_y[4];
		clk = coordinate_x[pointnum-1];
	}
	coordinate_report[13] = gesture_direction;
	if (gestrue_id==GESTURE_O){
		coordinate_report[13]=0;
	}
	else if (gestrue_id==GESTURE_CW_O){
		coordinate_report[13]=1;
	}
}
#endif
/* [lichenggang end] */


/* [lichenggang start] add for gesture_id judge*/
#ifdef FTS_GESTRUE_EN
void fts_read_Gestruedata(void)
{
	int ret = -1,i = 0;
	int gesture_id = 0;

	u8 reg_addr = 0xd3;
	//u8 read_data;
	short pointnum = 0;

	raw_data_buf[0] = 0xd3;
	TPD_ERR( "%s\n", __func__);
	ret = fts_i2c_read(fts_i2c_client, raw_data_buf, 1, raw_data_buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0)
	{
		TPD_ERR( "%s read Gestruedata failed.\n", __func__);
		return ;
	}
	memset(coordinate_report, 0, sizeof(coordinate_report));
	{
		gesture_id = raw_data_buf[0];
		global_gesture_id=gesture_id;
		printk("read gesture data:gestrue_id =0x%x\n",gesture_id);
		pointnum = (short)(raw_data_buf[1]) & 0xff;
		TPD_ERR( "pointnum=%d\n", pointnum);
		raw_data_buf[0] = 0xd3;
		if((gesture_id != GESTURE_DOUBLELINE) && (gesture_id != GESTURE_CW_DOUBLELINE)){
			if((pointnum * 4 + 2)<255){
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 1, raw_data_buf, (pointnum * 4 + 2));
			}else{
				/* [jiangtingyu start]optimize gesture*/
				if((pointnum * 4 + 2) > RAW_DATA_LENGTH)
				{
					pr_err("error::pointnum data will greater than raw_data_buf\n");
					memset(raw_data_buf, 0, sizeof(raw_data_buf));
					return;
				}
				/* [jiangtingyu end] */
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 1, raw_data_buf, 255);
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 0, raw_data_buf+255, (pointnum * 4 + 2) -255);
			}
			if (ret < 0){
				pr_err( "[focaltech]:%s read touchdata failed.\n", __func__);
				return;
			}
			_get_coordinate_report(raw_data_buf, pointnum);
			/* [yanlin start] optimize gesture data */
			if (TP_DATA_ERROR == 1) {
				TP_DATA_ERROR = 0;
				memset(raw_data_buf, 0, sizeof(raw_data_buf));
				return;
			}
			/* [yanlin end] */
		}else{ // gesture DOUBLELINE
			if((pointnum * 4 + 4)<255){
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 1, raw_data_buf, (pointnum * 4 + 4));
			}else{
				/* [jiangtingyu start]optimize gesture*/
				if((pointnum * 4 + 4) > RAW_DATA_LENGTH)
				{
					pr_err("error::DOUBLELINE pointnum data will greater than raw_data_buf\n");
					memset(raw_data_buf, 0, sizeof(raw_data_buf));
					return;
				}
				/* [jiangtingyu end] */
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 1, raw_data_buf, 255);
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 0, raw_data_buf+255, (pointnum * 4 + 4) -255);
			}

			for(i = 0;i < pointnum;i++){
				coordinate_doblelion_1_x[i] = (((s16) raw_data_buf[2 + (4 * i)]) & 0x0F) <<
					8 | (((s16) raw_data_buf[3 + (4 * i)])& 0xFF);
				coordinate_doblelion_1_y[i] = (((s16) raw_data_buf[4 + (4 * i)]) & 0x0F) <<
					8 | (((s16) raw_data_buf[5 + (4 * i)]) & 0xFF);
				printk("pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_doblelion_1_x[i],
					i,coordinate_doblelion_1_y[i]);
				/* [yanlin start] Optimize double line gesture data */
				if ((coordinate_doblelion_1_x[i] == 0)
						|| (coordinate_doblelion_1_x[i] > TP_MAX_X_VALUE)
						|| (coordinate_doblelion_1_y[i] == 0)
						|| (coordinate_doblelion_1_y[i] > TP_MAX_Y_VALUE)) {
					printk("DOUBLELINE gesture data is error\n");
					memset(raw_data_buf, 0, sizeof(raw_data_buf));
					return;
				}
				/* [yanlin end] */
			}
			coordinate_report[5] = coordinate_doblelion_1_x[0];
			coordinate_report[6] = coordinate_doblelion_1_y[0];
			coordinate_report[7] = coordinate_doblelion_1_x[pointnum-1];
			coordinate_report[8] = coordinate_doblelion_1_y[pointnum-1];
			pointnum = raw_data_buf[pointnum * 4 + 2]<<8 |raw_data_buf[pointnum * 4 + 3];
			 //ret = focaltech_i2c_Read(client, raw_data_buf, 0, raw_data_buf, (pointnum * 4));
			if((pointnum * 4 )<255){
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 0, raw_data_buf, (pointnum * 4));
			}else{
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 0, raw_data_buf, 255);
				ret = fts_i2c_read(fts_i2c_client, &reg_addr, 0, raw_data_buf+255, (pointnum * 4) -255);
			}
			for(i = 0;i < pointnum;i++){
				coordinate_doblelion_2_x[i] = (((s16) raw_data_buf[0 + (4 * i)]) & 0x0F) <<
					8 | (((s16) raw_data_buf[1 + (4 * i)])& 0xFF);
				coordinate_doblelion_2_y[i] = (((s16) raw_data_buf[2 + (4 * i)]) & 0x0F) <<
					8 | (((s16) raw_data_buf[3 + (4 * i)]) & 0xFF);
				printk("DOUBLELINE::pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_doblelion_2_x[i],i,coordinate_doblelion_2_y[i]);
			}
			if (ret < 0){
				pr_err( "[focaltech]:%s read touchdata failed.\n", __func__);
				return;
			}
			coordinate_report[1] = coordinate_doblelion_2_x[0];
			coordinate_report[2] = coordinate_doblelion_2_y[0];
			coordinate_report[3] = coordinate_doblelion_2_x[pointnum-1];
			coordinate_report[4] = coordinate_doblelion_2_y[pointnum-1];

			if(gesture_id==GESTURE_DOUBLELINE)
				coordinate_report[13] = 0;
			else
				coordinate_report[13] = 1;
		}
		gesture = (gesture_id == GESTURE_LEFT)			? Right2LeftSwip :
				 (gesture_id == GESTURE_RIGHT)			? Left2RightSwip :
				 (gesture_id == GESTURE_UP)				? Down2UpSwip :
				 (gesture_id == GESTURE_DOWN)			? Up2DownSwip :
				 (gesture_id == GESTURE_DOUBLECLICK)	? DouTap:
				 (gesture_id == GESTURE_DOUBLELINE)		? DouSwip:
				 (gesture_id == GESTURE_CW_DOUBLELINE)	? DouSwip:
				 (gesture_id == GESTURE_LEFT_V)			? RightVee:
				 (gesture_id == GESTURE_RIGHT_V)		? LeftVee:
				 (gesture_id == GESTURE_V)			    ? UpVee:
				 (gesture_id == GESTURE_O)				? Circle:
				 (gesture_id == GESTURE_CW_O)			? Circle:
				 (gesture_id == GESTURE_W)				? Wgestrue:
				 (gesture_id == GESTURE_M)				? Mgestrue:
				 UnkownGestrue;

		if(gesture != UnkownGestrue ){
			TPD_ERR("report_gesture::gesture_id=0x%x,gesture=%d\n",gesture_id,gesture);
			global_gesture_id = UNKOWN_GESTURE_ID;
			memset(raw_data_buf, 0, sizeof(raw_data_buf));/* [jiangtingyu start]optimize gesture*/
            gesture_direction = 0;
			gesture_upload = gesture;
			input_report_key(tpd->dev, KEY_F4, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_F4, 0);
			input_sync(tpd->dev);
		}else{
			TPD_ERR("......report_gesture::can't judge gesture.....\n");
			global_gesture_id=UNKOWN_GESTURE_ID;
			//fts_read_reg(fts_i2c_client, 0xa5, &state);
			fts_write_reg(fts_i2c_client, 0xa5, 0x00);
			fts_write_reg(fts_i2c_client, 0xd0, 0x01);
			fts_write_reg(fts_i2c_client, 0xd1, 0x3f);
			fts_write_reg(fts_i2c_client, 0xd2, 0x07);
			fts_write_reg(fts_i2c_client, 0xd6, 0xff);
		}
		//coordinate_report[0] = gesture;
		//coordinate_report[13] = (gesture == Circle)?0:2;
	}
return;
}
#endif
/* [lichenggang end] */

ssize_t gesture_coordinate_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[512];
	ret = sprintf(page, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", gesture_upload,
					coordinate_report[1], coordinate_report[2], coordinate_report[3], coordinate_report[4],
					coordinate_report[5], coordinate_report[6], coordinate_report[7], coordinate_report[8],
					coordinate_report[9], coordinate_report[10], coordinate_report[11], coordinate_report[12],
					coordinate_report[13]);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}



