#include <linux/fs.h>
#include <linux/delay.h>
#include "bu64748_function.h"
#include "OIS_coef.h"
#include "OIS_prog.h"

extern	int SOutEx(u8 slaveAddress, u8 *dat, int size);
extern	int SInEx(u8 slaveAddress, u8 *dat, int size, u8 *ret, int ret_size);

int BU64748_Initial(void)
{
	int str = ADJ_OK;

	I2C_func_POFF_____();		// power Off.
	I2C_func_PON______();		// power On.
	func_CHK_VERSION();			// version check.

	str = func_PROGRAM_DOWNLOAD();	//program download.

	if(str != ADJ_OK)
		return str;
	
	str = func_COEF_DOWNLOAD();		//Coefficient download.

	if(str != ADJ_OK)
		return str;

	I2C_func_DSP_START();		//DSP Start.

	AF_TARGET(0x200);
	Set_Close_Mode();			//Set Close mode.

	return str;
}

void Set_Close_Mode(void)
{
	I2C_func_MEM_WRITE( _M_30_EQCTL , 0x000D);
}

void Set_Open_Mode(void)
{
	I2C_func_MEM_WRITE( _M_30_EQCTL , 0x0004);
}

/////////////////////////////////////////////////////
//			Target CONTROL
//
//	POSDAC[9:0] = 0x000(Infinite) ¡V 0x3FF(Macro)
//	POS_H = POSDAC[9:8] ( 2bit )
//	POS_L = POSDAC[7:0] ( 8bit )
//	SLAVE_ADR + 0xF2 + POS_H + POS_L
/////////////////////////////////////////////////////
void AF_TARGET( u16 target )
{
	u8 out[3] = {0};

	out[0] = 0xF2;
	out[1] = (target>>8)&0xFF;
	out[2] = target & 0xFF;

	SOutEx( _SLV_FBAF_, out, 3 );
}

void I2C_func_PON______(void)
{
	I2C_func_PER_WRITE( 0xEF, 0x0080 );
}

void I2C_func_POFF_____(void)
{
	I2C_func_PER_WRITE( 0xEF, 0x0000 );
}
	
void func_CHK_VERSION(void)
{
	u16 u16_dat = 0;
	u16_dat = I2C_func_PER_READ( 0x5F );
	pr_debug("[bu64748af]IC Version : 0x%x.\n",u16_dat);
}

int func_PROGRAM_DOWNLOAD(void)
{
	int sts=ADJ_OK;		
	int ver_check = 0;
	u16 u16_dat;

	download(0);

	ver_check = I2C_func_MEM_READ( _M_F7_FBAF_STS );	// Check Status
	pr_debug("[bu64748af]ver_check : 0x%x\n",ver_check);

	if ( ( ver_check & 0x0004 ) == 0x0004 )
	{
		u16_dat = I2C_func_MEM_READ(_M_FIRMVER);

		pr_debug("[bu64748af]FW Ver : %d\n",u16_dat);
		pr_debug("[bu64748af]FW Download OK.\n");
	}
	else
	{
		pr_debug("[bu64748af]FW Download NG.\n");
		return PROG_DL_ERR;
	}
	return sts;
}

int func_COEF_DOWNLOAD(void)
{
	int sts = ADJ_OK;
	u16 u16_dat;

	download( 1 );
	
	u16_dat = I2C_func_MEM_READ( _M_CD_CEFTYP );

	pr_debug("[bu64748af]COEF Ver : %d\n",u16_dat);
	pr_debug("[bu64748af]COEF Download OK.\n");
	return sts;
}

//////////////////////////////////////////////////////
//		Download the data from file
//////////////////////////////////////////////////////
void download(int type)
{
	/* Data Transfer Size per one I2C access */
	#define		DWNLD_TRNS_SIZE		(32)

	unsigned char temp[DWNLD_TRNS_SIZE + 1];
	int block_cnt;
	int total_cnt;
	int lp;
	int n;
	int u16_i;

	if (type == 0)
		n = DOWNLOAD_BIN_LEN;
	else
		n = DOWNLOAD_COEF_LEN;	/* RHM_HT 2013/07/10    Modified */

	block_cnt = n / DWNLD_TRNS_SIZE + 1;
	total_cnt = block_cnt;

	while (1) {
		/* Residual Number Check */
		if (block_cnt == 1)
			lp = n % DWNLD_TRNS_SIZE;
		else
			lp = DWNLD_TRNS_SIZE;

		/* Transfer Data set */
		if (lp != 0) {
			if (type == 0) {
				temp[0] = _OP_FIRM_DWNLD;
				for (u16_i = 1; u16_i <= lp; u16_i += 1)
					temp[u16_i] = DOWNLOAD_BIN[(total_cnt - block_cnt) *
					DWNLD_TRNS_SIZE + u16_i - 1];
			} else {
				temp[0] = _OP_COEF_DWNLD;
				for (u16_i = 1; u16_i <= lp; u16_i += 1)
					temp[u16_i] = DOWNLOAD_COEF[(total_cnt - block_cnt) *
					DWNLD_TRNS_SIZE + u16_i - 1];
			}

			/* Data Transfer */
			//WR_I2C(_SLV_OIS_, lp + 1, temp);
			SOutEx( _SLV_FBAF_, temp, lp + 1 );
		}

		/* Block Counter Decrement */
		block_cnt = block_cnt - 1;

		if (block_cnt == 0)
			break;
	}
}


void I2C_func_DSP_START(void)
{
	u8 out[2] = {0};

	out[0] = _OP_SpecialCMD;
	out[1] = _cmd_8C_EI;

	SOutEx( _SLV_FBAF_, out, 2);
	return;
}


///////////////////////////////////////////////////
// Peripheral Block Read Write
///////////////////////////////////////////////////
void I2C_func_PER_WRITE(u8 u08_adr, u16 u16_dat)
{
	u8 out[4]={0};

	out[0] = _OP_Periphe_RW;
	out[1] = u08_adr;
	out[2] = u16_dat & 0xFF;
	out[3] = (u16_dat >> 8) & 0xFF;

	SOutEx( _SLV_FBAF_, out, 4 );
}

u16 I2C_func_PER_READ( u8 u08_adr )
{
	u8 in[2] = {0};
	u8 read[2] = {0};
	u16 u16_dat;

	in[0] = _OP_Periphe_RW;
	in[1] = u08_adr;

	SInEx( _SLV_FBAF_, in, 2, read, 2);

	u16_dat = ( read[0] * 256 ) + read[1];
	return u16_dat;
}

///////////////////////////////////////////////////
// Memory Block Read Write
///////////////////////////////////////////////////
void I2C_func_MEM_WRITE( u8 u08_adr, u16 u16_dat )
{
	u8 out[4];

	out[0] = _OP_Memory__RW;
	out[1] = u08_adr;
	out[2] = u16_dat & 0xFF;
	out[3] = (u16_dat >> 8 & 0xFF);

	SOutEx( _SLV_FBAF_, out, 4 );	
}

u16 I2C_func_MEM_READ( u8 u08_adr )
{
	u8 in[2] = {0};
	u8 read[2] = {0};
	u16 u16_dat;

	in[0] = _OP_Memory__RW;
	in[1] = u08_adr;

	SInEx( _SLV_FBAF_, in, 2, read, 2);

	u16_dat = ( read[0] * 256 ) + read[1];
	return u16_dat;
}

int bu64748_main_af_cur_pos(int* pos)
{
	int ret = 0;
	u8 in[2] = {0};
	u8 read[2] = {0};


	in[0] = 0x84;
	in[1] = 0x23;

	ret = SInEx( _SLV_FBAF_, in, 2, read, 2);

	*pos = ( read[0] * 256 ) + read[1];
	return ret;
}


