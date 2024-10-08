/*
 *drivers/input/touchscreen/ft5x06_ex_fun.c
 *
 *FocalTech IC driver expand function for debug.
 *
 *Copyright (c) 2010  Focal tech Ltd.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *Note:the error code of EIO is the general error in this file.
 */

//#include "tpd.h"
//#include "tpd_custom_fts.h"
#include <linux/netdevice.h>
#include <linux/mount.h>
//#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/i2c/focaltech.h>
#include <linux/i2c/focaltech_ex_fun.h>
#include <linux/i2c/focaltech_ctl.h>

u8 fts_ctpm_get_i_file_ver(void);
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
static struct mutex g_device_mutex;
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);

u8 *I2CDMABuf_va = NULL;
volatile u32 I2CDMABuf_pa = NULL;

extern struct Upgrade_Info fts_updateinfo_curr;

int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth);

#define SPRD_AUTO_UPGRADE


#ifdef SPRD_AUTO_UPGRADE


//#ifdef CONFIG_PROJS_V2804
#if defined(CONFIG_PROJS_V2804) || defined(CONFIG_PROJS_V2808)
static unsigned char CTPM_FW[] = {
#include "DIJING_FT6336U_2800_Ver0x19_20160105_app.i"
};
#elif defined(CONFIG_PROJS_V2806)
static unsigned char CTPM_FW[] = {
#include "DIJING_FT6336U_2800_Ver0x1b_20160330_app.i"
};
#elif defined(CONFIG_PROJS_V2520)
static const unsigned char CTPM_FW_hlt[]= {
#include "Tinno_2520_HLT_Ver0x15_20161110_app.i"
};
static const unsigned char CTPM_FW_yijian[]= {
#include "Tinno_2520_GZYJ_Ver0x14_20161221_app.i"
};

static unsigned char CTPM_FW[] ={
#include "Tinno_2520_HLT_Ver0x15_20161110_app.i"
	};

#elif defined(CONFIG_PROJS_V2510)
static const unsigned char CTPM_FW_hlt[]= {
#include "Tinno_2510_HLT_Ver0x12_20161110_app.i"

};
static const unsigned char CTPM_FW_yijian[]= {
#include "Tinno_2510_GZYJ_Ver0x12_20161110_app.i"
};

static unsigned char CTPM_FW[] = {
#include "Tinno_2510_HLT_Ver0x12_20161110_app.i"
};
#else


static unsigned char CTPM_FW[] = {
#include "v250x/Tinno_2500_GZYJ_Ver0x17_20160119_app.i"  
}; 

static unsigned char CTPM_FW_HLT[] = {
#include "v250x/Tinno_V2500_FT6336G_VA4_20161126_app.i"  
}; 

#endif // CONFIG_PROJS_V2804
#endif // SPRD_AUTO_UPGRADE




static DEFINE_MUTEX(g_device_mutex);

#ifdef CONFIG_MACH_V250X
void tp_firmware_update(int prod_version){

	printk("[FTS] tp_firmware_update prod_version = %d\n",prod_version);

 	if(prod_version == 130){


	  memcpy(CTPM_FW,CTPM_FW_HLT,sizeof(CTPM_FW));

	}else{

	  printk("[FTS]  yijian tp \n");
	}

      }
#endif


/*
*fts_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int fts_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;
	
#if 1//for normal I2c transfer
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
#else// for DMA I2c transfer
	if(writelen!=0)
	{
		//DMA Write
		if(0)//if(writelen < 8  )
		{
			
			//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
			ret= i2c_master_send(client, writebuf, writelen);
		}
		else
		{
			for(i = 0 ; i < writelen; i++)
			{
				I2CDMABuf_va[i] = writebuf[i];
			}

			client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		
			if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
				dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
			//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
			//return ret;
			client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

		}
	}
	//DMA Read 
	if(readlen!=0)
	{
		if(0)//if (readlen <8) {
		{
			ret = i2c_master_recv(client, (unsigned char *)readbuf, readlen);
		}
		else
		{

			client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
			ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, readlen);

			for(i = 0; i < readlen; i++)
	        {
	            readbuf[i] = I2CDMABuf_va[i];
	        }
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

		}
	}
	#endif
	return ret;
}
/*write data by i2c*/

int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	//int i = 0;


	//printk("fts_i2c_Write  %d  %x  %x %x %x",writelen,writebuf,I2CDMABuf_va,I2CDMABuf_pa,client);
	
  // client->addr = client->addr & I2C_MASK_FLAG;
  // client->ext_flag |= I2C_DIRECTION_FLAG; 
  // client->timing = 100;
    #if 1
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	#else
	
	if(0)//if(writelen < 8)
	{
		
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		ret = i2c_master_send(client, writebuf, writelen);
	}
	else
	{
		for(i = 0 ; i < writelen; i++)
		{
			I2CDMABuf_va[i] = writebuf[i];
		}

		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

		if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
			dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

	} 
	#endif
	return ret;

}

int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_Write(client, buf, sizeof(buf));
}


int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

int fts_ctpm_auto_clb(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	fts_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	fts_write_reg(client, 2, 0x4);
	msleep(300);
	if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)) //5x36,5x36i
	{
		for(i=0;i<100;i++)
		{
			fts_read_reg(client, 0x02, &uc_temp);
			if (0x02 == uc_temp ||
				0xFF == uc_temp)
			{
				/*if 0x02, then auto clb ok, else 0xff, auto clb failure*/
			    break;
			}
			msleep(20);	    
		}
	} else {
		for(i=0;i<100;i++)
		{
				fts_read_reg(client, 0, &uc_temp);
			if (0x0 == ((uc_temp&0x70)>>4))  /*return to normal mode, calibration finish*/
			{
			    break;
			}
			msleep(20);	    
		}
	}
	/*calibration OK*/
	fts_write_reg(client, 0, 0x40);  /*goto factory mode for store*/
	msleep(200);   /*make sure already enter factory mode*/
	fts_write_reg(client, 2, 0x5);  /*store CLB result*/
	msleep(300);
	fts_write_reg(client, 0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}


int hid_to_i2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;

	fts_i2c_Write(client, auc_i2c_write_buf, 3);

	msleep(10);

	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;

	fts_i2c_Read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

	if(0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
	{
		bRet = 1;		
	}
	else bRet = 0;

	return bRet;
	
}

int  fts_5x46_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	i_ret = hid_to_i2c(client);

	if(i_ret == 0)
	{
		printk("[FTS] hid change to i2c fail ! \n");
	}

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		fts_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		//write 0x55 to register 0xfc 
		fts_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hid_to_i2c(client);

		if(i_ret == 0)
		{
			printk("[FTS] hid change to i2c fail ! \n");
			continue;
		}
		msleep(10);
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		i_ret = fts_i2c_Write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			printk("[FTS] failed writing  0x55 and 0xaa ! \n");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		
		reg_val[0] = reg_val[1] = 0x00;
		
		fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			printk("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;
	printk("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);	//erase app area 
	msleep(1350);
	for(i = 0;i < 15;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);
	}

	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_Write(client, auc_i2c_write_buf, 4);
	bt_ecc = 0;
	printk("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);
		}
	}
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}	
		fts_i2c_Write(client, packet_buf, temp + 6);
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);
		}
	}
	msleep(50);
		/*call the upgrade function */
	printk("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_Write(client, auc_i2c_write_buf, 1); 
	msleep(300);
	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_Write(client, auc_i2c_write_buf, 6); 
	msleep(dw_lenth/256);
	for(i = 0;i < 100;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			break;
		}
		msleep(1);
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}
	printk("checksum %X %X \n",reg_val[0],bt_ecc);	
	printk("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(130);	//make sure CTP startup normally 
	return 0;
}
int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	if(pbt_buf[0] != 0x02)
	{
		printk("[FTS] FW first byte is not 0x02. so it is invalid \n");
		return -1;
	}
	printk("[geroge] FW dw_lenth = 0x %x \n",dw_lenth);
	if(dw_lenth > 0x11f)
	{
		fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
		printk("[geroge] FW dw_lenth = 0x %x \n",dw_lenth);
		if(dw_lenth < fw_length)
		{
			printk("[FTS] Fw length is invalid \n");
			return -1;
		}
	}
	else
	{
		printk("[FTS] Fw length is invalid \n");
		return -1;
	}

	printk("[geroge]  Fw length is ok  \n");
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		fts_write_reg(client, 0xbc, FT_UPGRADE_AA);
		//msleep(fts_updateinfo_curr.delay_aa);
		msleep(15);
		fts_write_reg(client, 0xbc, FT_UPGRADE_55);
		if(i<=15)
		{
		msleep(fts_updateinfo_curr.delay_55+i*3);
		}
		else
		{
		msleep(fts_updateinfo_curr.delay_55-(i-15)*2);
		}
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		fts_i2c_Write(client, auc_i2c_write_buf, 1);

		auc_i2c_write_buf[0] = FT_UPGRADE_AA;
		fts_i2c_Write(client, auc_i2c_write_buf, 1);
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			printk("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
	} else {
			dev_err(&client->dev, "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	fts_i2c_Write(client, auc_i2c_write_buf, 5);
	printk("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(fts_updateinfo_curr.delay_earse_flash);
	for(i = 0;i < 200;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(0xb0 == reg_val[0] && 0x02 == reg_val[1])
		{
			printk("[FTS] erase app finished \n");
			break;
		}
		msleep(50);
	}

	bt_ecc = 0;
	printk("Step 5:write firmware(FW) to ctpm flash\n");
	dw_lenth = fw_length;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
}
		fts_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				printk("[FTS] write a block data finished \n");
				break;
			}
			msleep(1);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_Write(client, packet_buf, temp + 6);
		for(i = 0;i < 30;i++)
{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				printk("[FTS] write a block data finished \n");
				break;
			}
			msleep(1);
		}
	}

	//ui_sz = sizeof(CTPM_FW);
	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	printk("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	printk("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	printk("Step 8: reset the new FW OK \n");
	return 0;
}

/*update project setting
*only update these settings for COB project, or for some special case
*/
int fts_ctpm_update_project_setting(struct i2c_client *client)
{
	u8 uc_i2c_addr;	/*I2C slave address (7 bit address)*/
	u8 uc_io_voltage;	/*IO Voltage 0---3.3v;	1----1.8v*/
	u8 uc_panel_factory_id;	/*TP panel factory ID*/
	u8 buf[FTS_SETTING_BUF_LEN];
	u8 reg_val[2] = {0};
	u8 auc_i2c_write_buf[10] = {0};
	u8 packet_buf[FTS_SETTING_BUF_LEN + 6];
	u32 i = 0;
	int i_ret;

	uc_i2c_addr = client->addr;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;


	/*Step 1:Reset  CTPM
	*write 0xaa to register 0xfc
	*/
	fts_write_reg(client, 0xfc, 0xaa);
	msleep(50);

	/*write 0x55 to register 0xfc */
	fts_write_reg(client, 0xfc, 0x55);
	msleep(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = fts_i2c_Write(client, auc_i2c_write_buf, 2);
		msleep(5);
	} while (i_ret <= 0 && i < 5);


	/*********Step 3:check READ-ID***********************/
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;

	fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
		dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			 reg_val[0], reg_val[1]);
	else
		return -EIO;

	auc_i2c_write_buf[0] = 0xcd;
	fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	dev_dbg(&client->dev, "bootloader version = 0x%x\n", reg_val[0]);

	/*--------- read current project setting  ---------- */
	/*set read start address */
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;

	fts_i2c_Read(client, buf, 4, buf, FTS_SETTING_BUF_LEN);
	dev_dbg(&client->dev, "[FTS] old setting: uc_i2c_addr = 0x%x,\
			uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
			buf[0], buf[2], buf[4]);

	 /*--------- Step 4:erase project setting --------------*/
	auc_i2c_write_buf[0] = 0x63;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*----------  Set new settings ---------------*/
	buf[0] = uc_i2c_addr;
	buf[1] = ~uc_i2c_addr;
	buf[2] = uc_io_voltage;
	buf[3] = ~uc_io_voltage;
	buf[4] = uc_panel_factory_id;
	buf[5] = ~uc_panel_factory_id;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	packet_buf[4] = 0;
	packet_buf[5] = FTS_SETTING_BUF_LEN;

	for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
		packet_buf[6 + i] = buf[i];

	fts_i2c_Write(client, packet_buf, FTS_SETTING_BUF_LEN + 6);
	msleep(100);

	/********* reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);

	msleep(200);
	return 0;
}

int pfw_vendor;

#ifdef SPRD_AUTO_UPGRADE
static int fts_switch_to_update(struct i2c_client *client)
{
	int ret = 0, i=0;
	uint8_t arrCommand[] = {0x55, 0xaa};
	uint8_t arrCommand1[] = {0xBC, 0xaa};	
	uint8_t arrCommand2[] = {0xBC, 0x55};	
	
	//ret = fts_write_reg(0xBC, 0xAA); // zhangxiaofei add for ft6x06 touch panel driver
	ret = fts_i2c_Write(client, arrCommand1,  2);
	if (ret < 0) 
	{
		printk("[FTS] fts_switch_to_update 0xAA \n");
		goto err;
	}
	//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
	
	printk("[FTS] fts_switch_to_update 0xAA ret = %d \n",ret);
	//{
	msleep(50);
	//}

	/*write 0x55 to register 0xfc*/
	//ret = fts_write_reg(0xFC, 0x55);
	//ret = fts_write_reg(0xBC, 0x55); // zhangxiaofei add for ft6x06 touch panel driver
	ret = fts_i2c_Write(client, arrCommand2,  2);
	if (ret < 0) {
		printk("[FTS] fts_switch_to_update 0x55 \n");
		goto err;
	}
	
	printk("[FTS] fts_switch_to_update 0x55 ret = %d \n",ret);
		//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
	
		msleep(50); //40
	//}

//#else
	//}
	
//#endif 

	do{
		//mutex_lock(&ts->mutex);
		//ret = i2c_master_send(ts->client, (const char*)arrCommand, sizeof(arrCommand));
		//mutex_unlock(&ts->mutex);
		ret = fts_i2c_Write(client, arrCommand,  2);
		
	
	msleep(50); //40
	
		++i;
	}while(ret < 0 && i < 5);

	ret = 0;
err:
      printk("[FTS] fts_switch_to_update ret = %d i=%d  \n",ret,i);
	return ret;
}


static int ft6x06_get_tp_id(struct i2c_client *client,int *ptp_id)
{
	int rc;
	char tp_id[2];	
	
	char arrCommand[4] = {0x90, 0x00, 0x00, 0x00};

	rc = fts_i2c_Write(client, arrCommand,  4);
	
	if (rc < 0) {
		//CTP_DBG("i2c_master_send failed(%d)", rc);
		return -EIO;
	} 
	msleep(50); //40

	//rc = fts_read_reg(client, 0, &tp_id);
	rc = fts_i2c_Read(client,NULL,0,&tp_id,2);
	
	printk("[FTS] ft6x06_get_tp_id tp_id0 = 0x%x; tp_id1=0x%x\n",tp_id[0],tp_id[1]);		 
	if (rc < 0) {
		//CTP_DBG("i2c_master_recv failed(%d)", rc);
		return -EIO;
	} 
	
	*ptp_id = (( int )tp_id[0] << 8) | (( int )tp_id[1]);
			
	return 0;
}

static int ft6x06_get_vendor_from_bootloader(struct i2c_client *client, uint8_t *vendor, uint8_t *version)
{
	int rc = 0, tp_id;
	uint8_t version_id = 0, buf[5];
	int resettimes = 0;
	char Command[4] = {0xcd, 0x00, 0x00, 0x00};	
	char Command1[4] = {0x03, 0x00, 0x07, 0xB0};	
	
	char Command2[4] = {0x07,0x00,0x00,0x00};	
	
		do{
			fts_switch_to_update(client);
			ft6x06_get_tp_id(client,&tp_id);
	printk("[FTS] ft6x06_get_vendor_from_bootloader tp_id = 0x%x\n",tp_id);		 
			if(0x791C == tp_id){
				break;

				
		}
		}while(0==tp_id&&(resettimes++)<20);


	//rc = fts_cmd_write(0xcd, 0x00, 0x00, 0x00, 1);
	rc = fts_i2c_Write(client, Command,  1);
	if (rc < 1) {
		//CTP_DBG("i2c_master_send failed(%d)", rc);
		goto err;
	} 
	//rc = i2c_master_recv(ts->client, &version_id, 1);
	rc = fts_read_reg(client, 0, &version_id);
	if (rc < 1) {
		//CTP_DBG("i2c_master_recv failed(%d)", rc);
		goto err;
	} 
	
	msleep(50); //40
	//*pfw_version = version_id;
	*version = -1;//Force to update.
	//CTP_DBG("bootloader version = 0x%x\n", version_id);

	/* --------- read current project setting  ---------- */
	//set read start address
	//rc = fts_cmd_write(0x03, 0x00, 0x78, 0x00, 4);  // FT6x06
	//rc = fts_cmd_write(0x03, 0x00, 0x07, 0xB0, 4);  // FT6X36
	rc = fts_i2c_Write(client, Command1,  4);
	if (rc < 0) {
		//CTP_DBG("i2c_master_send failed(%d)", rc);
		goto err;
	} 
	
	msleep(50); //40
	rc = fts_i2c_Read(client,0,1,&buf,sizeof(buf));
	if (rc < 0){
		//CTP_DBG("i2c_master_recv failed(%d)", rc);
		goto err;
	}

	//CTP_DBG("vendor_id = 0x%x\n", buf[4]);
	
	msleep(50); //40
	*vendor = buf[4];
	
//	printk("[FTS]1 ft6x06_get_vendor_from_bootloader vendor = 0x%x\n",buf[4]);
	//CTP_DBG("Try to reset TP!");
	//rc = fts_cmd_write(0x07,0x00,0x00,0x00,1);
	rc = fts_i2c_Write(client, Command2,  4);
	if (rc < 0) {
		//CTP_DBG("reset failed");
		goto err;
	}
	msleep(200);
	
	return 0;
err:
	return rc;


}
int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	u8 uc_host_fm_ver = FT_REG_FW_VER;
	u8 uc_tp_fm_ver;
    int i_ret;
	int pfw_version;
	
	unsigned char uc_reg_value;

    int i=0;

	fts_read_reg(client, FT_REG_FW_VER, &uc_tp_fm_ver);
	
	fts_read_reg(client,FT_REG_VENDOR_ID, &pfw_vendor);
	printk("[FTS] fts_ctpm_auto_upgrade pfw_vendorr = 0x%x\n",pfw_vendor);

	
	if((pfw_vendor == 0)||(pfw_vendor == 0xa8))
	{
		ft6x06_get_vendor_from_bootloader(client,&pfw_vendor,&pfw_version);
	}

	#ifdef CONFIG_MACH_V250X
	   tp_firmware_update(pfw_vendor);
	#endif

	#if defined(CONFIG_PROJS_V2520)||defined(CONFIG_PROJS_V2510)
	
		printk("[FTS]1 fts_ctpm_auto_upgrade pfw_vendor = 0x%x\n",pfw_vendor);
		
		switch(pfw_vendor)
			{
			case 0x82:
				//if(CTPM_FW == NULL)
					//CTPM_FW = MALLOC(osh, sizeof(CTPM_FW_hlt));
				//CTPM_FW = CTPM_FW_hlt;
				break;
			case 0x86:
				//if(CTPM_FW == NULL)
					//CTPM_FW = MALLOC(osh, sizeof(CTPM_FW_yijian));
				//CTPM_FW = CTPM_FW_yijian;
				memcpy(CTPM_FW,CTPM_FW_yijian,sizeof(CTPM_FW));
				break;				
			default:
				//CTPM_FW = NULL;
				return -EIO;
				break;
				
			}
	#endif
	
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();	
	printk("[FTS] geroge uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",uc_tp_fm_ver, uc_host_fm_ver);
/*
*avoid the old module about 500pcs to upgrade the firmware,
*the firmware version of old module is from 0x01 to 0x03,
*but the new module firmware version is more than 0x10.
*bug385939
*/

	if (/*the firmware in touch panel maybe corrupted */
		uc_tp_fm_ver == FT_REG_FW_VER ||
		/*the firmware in host flash is new, need upgrade */
	   uc_tp_fm_ver < uc_host_fm_ver
	    ) {

		msleep(100);
		dev_dbg(&client->dev, "[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
				uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
		
   printk("geroge   i_ret  = %d\n",i_ret);

		
		if (i_ret == 0)	{
			msleep(300);
			//uc_host_fm_ver = fts_ctpm_get_i_file_ver();
			//dev_dbg(&client->dev, "[FTS] upgrade to new version 0x%x\n",
			//		uc_host_fm_ver);

			fts_read_reg(client, FT_REG_FW_VER, &uc_tp_fm_ver);
			printk("geroge  uc_tp_fm_ver upgrade tp to new version 0x%x\n",uc_tp_fm_ver);
			
		} else {
			pr_err("[FTS] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}
	}

	return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fw_len = sizeof(CTPM_FW);
	if ((fts_updateinfo_curr.CHIP_ID==0x54))
	{
		if (fw_len < 8 || fw_len > 54 * 1024) 
		{
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		pbt_buf = CTPM_FW;
		i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
		return i_ret;
	}
	 if ((fts_updateinfo_curr.CHIP_ID==0x36 || fts_updateinfo_curr.CHIP_ID==0x64))
	{
		pbt_buf = CTPM_FW;
		i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
		return i_ret;
	}
	if (fw_len < 8 || fw_len > 32 * 1024) {
		dev_err(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}
	if (1){
		pbt_buf = CTPM_FW;
		i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
		else if(fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)
			fts_ctpm_auto_clb(client);	/*start auto CLB */
	} else {
		dev_err(&client->dev, "%s:FW format error\n", __func__);
		return -EBADFD;
	}
	return i_ret;
}
u8 fts_ctpm_get_i_file_ver(void)
{
	u16 ui_sz;
	ui_sz = sizeof(CTPM_FW);

printk("geroge   ui_sz  =  %d\n",ui_sz);
	printk("geroge	 fts_updateinfo_curr.CHIP_ID	=  0x %x\n",fts_updateinfo_curr.CHIP_ID);
	
		

	printk("geroge   CTPM_FW_ui_sz_2   =  %0x\n",CTPM_FW[ui_sz - 2]);
	printk("geroge   CTPM_FW[0x10a]   =  %0x\n",CTPM_FW[0x10a]);
	if (ui_sz > 2)
	{
	    if(fts_updateinfo_curr.CHIP_ID==0x36 || fts_updateinfo_curr.CHIP_ID==0x64)
                return CTPM_FW[0x10a];
	    else
		return CTPM_FW[ui_sz - 2];
	}
	return 0x00;	/*default value */
}
u8 fts_ctpm_get_i_file_vendor(void)
{
	u16 ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 1)
	{
	    if(fts_updateinfo_curr.CHIP_ID==0x36 || fts_updateinfo_curr.CHIP_ID==0x64)
                return CTPM_FW[0x108];
	    else
		return CTPM_FW[ui_sz - 1];
	}
	return 0x00;	/*default value */
}
#endif
void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			 udelay(1);
		}
	}
}

extern void focaltech_get_upgrade_array(struct i2c_client *client);

int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u8 is_5336_fwsize_30 = 0;
	u32 packet_number;
	u32 j=0;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
      // struct Upgrade_Info upgradeinfo;
	   
	//fts_get_upgrade_info(&upgradeinfo);
	
	focaltech_get_upgrade_array(client);
	
	if(*(pbt_buf+dw_lenth-12) == 30)
	{
		is_5336_fwsize_30 = 1;
	}
	else 
	{
		is_5336_fwsize_30 = 0;
	}
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
	        msleep(100);
		printk("[FTS] Step 1:Reset  CTPM\n");
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		//if (DEVICE_IC_TYPE == IC_FT6208 || DEVICE_IC_TYPE == IC_FT6x06)
		if(fts_updateinfo_curr.CHIP_ID==0x36 || fts_updateinfo_curr.CHIP_ID==0x06 ||fts_updateinfo_curr.CHIP_ID==0x64 ) 
			fts_write_reg(client, 0xbc, FT_UPGRADE_AA);
		else
			fts_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);


		/*write 0x55 to register 0xfc */
		//if(DEVICE_IC_TYPE == IC_FT6208 || DEVICE_IC_TYPE == IC_FT6x06)
		if(fts_updateinfo_curr.CHIP_ID==0x36 || fts_updateinfo_curr.CHIP_ID==0x06 || fts_updateinfo_curr.CHIP_ID==0x64)
			fts_write_reg(client, 0xbc, FT_UPGRADE_55);
		else
			fts_write_reg(client, 0xfc, FT_UPGRADE_55);
		if(i<=15)
		{
		msleep(fts_updateinfo_curr.delay_55+i*3);
		}
		else
		{
		msleep(fts_updateinfo_curr.delay_55-(i-15)*2);
		}

   
		/*********Step 2:Enter upgrade mode *****/
		printk("[FTS] Step 2:Enter upgrade mode \n");
		#if 0
			auc_i2c_write_buf[0] = FT_UPGRADE_55;
			auc_i2c_write_buf[1] = FT_UPGRADE_AA;
			do {
				j++;
				i_ret = fts_i2c_Write(client, auc_i2c_write_buf, 2);
				msleep(5);
			} while (i_ret <= 0 && j < 5);
		#else
			auc_i2c_write_buf[0] = FT_UPGRADE_55;
			fts_i2c_Write(client, auc_i2c_write_buf, 1);
			msleep(5);
			auc_i2c_write_buf[0] = FT_UPGRADE_AA;
			fts_i2c_Write(client, auc_i2c_write_buf, 1);
		#endif

#if 1
		/*********Step 3:check READ-ID***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
#endif	
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	
	auc_i2c_write_buf[0] = 0xcd;
	fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	/*********0705 mshl ********************/
	/*if (reg_val[0] > 4)
		is_5336_new_bootloader = 1;*/

	if (reg_val[0] <= 4)
	{
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	}
	else if(reg_val[0] == 7)
	{
		is_5336_new_bootloader = BL_VERSION_Z7 ;
	}
	else if(reg_val[0] >= 0x0f && ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)))
	{
		is_5336_new_bootloader = BL_VERSION_GZF ;
	}
	else
	{
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	}


	printk("[FTS] Step 4:erase app and panel paramenter area\n");
	/*Step 4:erase app and panel paramenter area*/
	printk("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(fts_updateinfo_curr.delay_earse_flash);
	/*erase panel parameter area */
	if(is_5336_fwsize_30)
	{
	    auc_i2c_write_buf[0] = 0x63;
	    fts_i2c_Write(client, auc_i2c_write_buf, 1);
	}
	msleep(100);

	printk("[FTS] Step 5:write firmware(FW) to ctpm flash\n");
	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("Step 5:write firmware(FW) to ctpm flash\n");

	//dw_lenth = dw_lenth - 8;
	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
	{
		dw_lenth = dw_lenth - 8;
	}
	else if(is_5336_new_bootloader == BL_VERSION_GZF) 
	{
	      dw_lenth = dw_lenth - 14;
	}
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		fts_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		if((((j+1) * FTS_PACKET_LENGTH)%1024)==0)
		printk("write bytes:0x%04x\n", (j+1) * FTS_PACKET_LENGTH);
		//delay_qt_ms(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_Write(client, packet_buf, temp + 6);
		msleep(20);
	}
#if 0
	/*send the last six byte*/
	for (i = 0; i<6; i++)
	{
		if (is_5336_new_bootloader && ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)))//5x36,5x36i 
			temp = 0x7bfa + i;
		else
			temp = 0x6ffa + i;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		temp =1;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];
		fts_i2c_Write(client, packet_buf, 7);
		msleep(20);
	}
#else
	/*send the last six byte*/
	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
	{
		for (i = 0; i<6; i++)
		{
			if (is_5336_new_bootloader  == BL_VERSION_Z7 ) 
			{
				temp = 0x7bfa + i;
			}
			else if(is_5336_new_bootloader == BL_VERSION_LZ4)
			{
				temp = 0x6ffa + i;
			}
			packet_buf[2] = (u8)(temp>>8);
			packet_buf[3] = (u8)temp;
			temp =1;
			packet_buf[4] = (u8)(temp>>8);
			packet_buf[5] = (u8)temp;
			packet_buf[6] = pbt_buf[ dw_lenth + i]; 
			bt_ecc ^= packet_buf[6];
  
			fts_i2c_Write(client, packet_buf, 7);
			msleep(20);
		}
	}
	else if(is_5336_new_bootloader == BL_VERSION_GZF)
	{
	    
		for (i = 0; i<12; i++)
		{
			if (is_5336_fwsize_30) 
			{
				temp = 0x7ff4 + i;
			}
			else 
			{
				temp = 0x7bf4 + i;
			}
			packet_buf[2] = (u8)(temp>>8);
			packet_buf[3] = (u8)temp;
			temp =1;
			packet_buf[4] = (u8)(temp>>8);
			packet_buf[5] = (u8)temp;
			packet_buf[6] = pbt_buf[ dw_lenth + i]; 
			bt_ecc ^= packet_buf[6];
  
			fts_i2c_Write(client, packet_buf, 7);
			msleep(20);

		}
	}

#endif
	printk("[FTS] Step 6: read out checksum\n");
	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	printk("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	fts_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	printk("[FTS] Step 7: reset the new FW\n");
	/*********Step 7: reset the new FW***********************/
	printk("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;
}
/*sysfs debug*/

/*
*get firmware size

@firmware_name:firmware name
*note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int fts_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}



/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int fts_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}



/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret=0;
	int fwsize = fts_GetFirmwareSize(firmware_name);

	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",
					__func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 54 * 1024) {
		dev_dbg(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}
	
    
	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);

	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(pbt_buf);
		pbt_buf = NULL;
		//return -EIO;
		i_ret = -EIO;
		goto err_ret;
	}
	
	/*call the upgrade function */
	if ((fts_updateinfo_curr.CHIP_ID==0x54))
	{
		i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x36 ||fts_updateinfo_curr.CHIP_ID==0x64))
	{
		i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else
	{
	i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	if (i_ret != 0)
		dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
	else if(fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)
		fts_ctpm_auto_clb(client);

err_ret:
	
	
	if(pbt_buf != NULL)
	kfree(pbt_buf);
    
	return i_ret;
}

static ssize_t fts_tpfwver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&g_device_mutex);
	fts_read_reg(client, FT_REG_FW_VER, &fwver);
	if (fts_read_reg(client, FT_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE,
					"get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&g_device_mutex);

	return num_read_chars;
}

static ssize_t fts_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t fts_vendor_name_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	

	 #ifdef CONFIG_MACH_V250X
		
		if(pfw_vendor == 130)	return sprintf(buf, "%s\n", "HLT");

		else return sprintf(buf, "%s\n", "YIJIAN");

	 #elif defined(CONFIG_PROJS_V2520)||defined(CONFIG_PROJS_V2510)
	 switch(pfw_vendor)
	 	{
	 	case 0x82:
			return sprintf(buf, "%s\n", "HLT");
			break;
			
		case 0x86:
			return sprintf(buf, "%s\n", "YIJIAN");
			break;
		default:
			return sprintf(buf, "%s\n", "UNKNOW");
			break;
			
	 	}
	#else
	    return sprintf(buf, "%s\n", "DIJING");
	#endif
}


static ssize_t fts_chip_info_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	 

	return sprintf(buf, "%s\n", "ft6336u");
}


static ssize_t fts_vendor_name_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t fts_chip_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}




static ssize_t fts_tprwreg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t fts_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 regaddr = 0xff, regvalue = 0xff;
	u8 valbuf[5] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = wmreg;
		if (fts_read_reg(client, regaddr, &regvalue) < 0)
			dev_err(&client->dev, "Could not read the register(0x%02x)\n",
						regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n",
					regaddr, regvalue);
	} else {
		regaddr = wmreg >> 8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue) < 0)
			dev_err(&client->dev, "Could not write the register(0x%02x)\n",
							regaddr);
		else
			dev_err(&client->dev, "Write 0x%02x into register(0x%02x) successful\n",
							regvalue, regaddr);
	}

error_return:
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t fts_fwupdate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/*upgrade from *.i*/
static ssize_t fts_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
#if 0
	//struct fts_ts_data *data = NULL;
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	//data = (struct fts_ts_data *)i2c_get_clientdata(client);

	mutex_lock(&g_device_mutex);

	disable_irq(client->irq);
	//mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0) {
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		pr_info("%s [FTS] upgrade to new version 0x%x\n", __func__,
					 uc_host_fm_ver);
	} else
		dev_err(&client->dev, "%s ERROR:[FTS] upgrade failed.\n",
					__func__);

	enable_irq(client->irq);
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mutex_unlock(&g_device_mutex);

	return count;
	#endif
}

static ssize_t fts_fwupgradeapp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t fts_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	ssize_t num_read_chars = 0;
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	mutex_lock(&g_device_mutex);
	disable_irq(client->irq);
       //mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	if(0==fts_ctpm_fw_upgrade_with_app_file(client, fwname))
	{
		num_read_chars = snprintf(buf, PAGE_SIZE,
					"FTP firmware upgrade success!\n");
	}
	else
	{
		num_read_chars = snprintf(buf, PAGE_SIZE,
					"FTP firmware upgrade fail!\n");
	}

	enable_irq(client->irq);
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mutex_unlock(&g_device_mutex);

	return num_read_chars;
}


/*sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO | S_IWUSR, fts_tpfwver_show,
			fts_tpfwver_store);

/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO | S_IWUSR, fts_fwupdate_show,
			fts_fwupdate_store);

/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO | S_IWUSR, fts_tprwreg_show,
			fts_tprwreg_store);


/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO | S_IWUSR, fts_fwupgradeapp_show,
			fts_fwupgradeapp_store);

static DEVICE_ATTR(vendor_name, S_IRUGO | S_IWUSR, fts_vendor_name_show,
			fts_vendor_name_store);

static DEVICE_ATTR(chip_info, S_IRUGO | S_IWUSR, fts_chip_info_show,
			fts_chip_info_store);


/*add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_vendor_name.attr,
	&dev_attr_chip_info.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/*create sysfs for debug*/
int fts_create_sysfs(struct i2c_client *client)
{
	int err;
	
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err) {
		dev_err(&client->dev,
					 "%s() - ERROR: sysfs_create_group() failed.\n",
					 __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_device_mutex);
		pr_info("ft6x06:%s() - sysfs_create_group() succeeded.\n",
				__func__);
	}
	return err;
}

void fts_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	mutex_destroy(&g_device_mutex);
	
	
}
/*create apk debug channel*/

#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_RAWDATA			3
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA			7

#define PROC_NAME	"ft5x0x-debug"
static unsigned char proc_operate_mode = PROC_RAWDATA;
static struct proc_dir_entry *ft5x0x_proc_entry;
/*interface of write proc*/
static ssize_t ft5x0x_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *ops)
{
	struct i2c_client *client = (struct i2c_client *)PDE_DATA(file_inode(filp));
	unsigned char writebuf[FTS_PACKET_LENGTH];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			printk("%s\n", upgrade_file_path);
			disable_irq(client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(client, upgrade_file_path);

			enable_irq(client->irq);
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		printk("%s:register addr=0x%02x\n", __func__, writebuf[1]);
		ret = fts_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_RAWDATA:
		break;
	case PROC_AUTOCLB:
		fts_ctpm_auto_clb(client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = fts_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
static ssize_t ft5x0x_debug_read(struct file *filp, char __user *page, size_t len, loff_t *pos)
{
	struct i2c_client *client = (struct i2c_client *)PDE_DATA(file_inode(filp));
	int ret = 0;

	//u8 tx = 0, rx = 0;
	//int i, j;

	unsigned char buf[PAGE_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		/*after calling ft5x0x_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = fts_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} else
			printk("%s:value=0x%02x\n", __func__, buf[0]);
		num_read_chars = 1;
		break;
	case PROC_RAWDATA:
		break;
	case PROC_READ_DATA:
		readlen = len;
		ret = fts_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);

	return num_read_chars;
}
static struct file_operations proc_ops = {
	.read = ft5x0x_debug_read,
	.write = ft5x0x_debug_write,
};
int ft5x0x_create_apk_debug_channel(struct i2c_client * client)
{
	ft5x0x_proc_entry = proc_create_data(PROC_NAME, 0777 | S_IFREG, NULL, &proc_ops, (void *)client);
	if (NULL == ft5x0x_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	}
	return 0;
}

void ft5x0x_release_apk_debug_channel(void)
{
#if 0
	if (ft5x0x_proc_entry)
		remove_proc_entry(PROC_NAME, NULL);
#endif
}

