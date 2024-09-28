/* drivers/video/sprdfb/lcd_nt35512_ctc_mipi.c
 *
 * Support for nt35512_ctc mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include "../sprdfb_panel.h"
//#define printk printf

#define  LCD_DEBUG
//#define THREE_LANE_SUPPORT

#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

#define MAX_DATA   60

typedef struct LCM_Init_Code_tag {
	unsigned int tag;
	unsigned char data[MAX_DATA];
}LCM_Init_Code;

typedef struct LCM_force_cmd_code_tag{
	unsigned int datatype;
	LCM_Init_Code real_cmd_code;
}LCM_Force_Cmd_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

static LCM_Force_Cmd_Code rd_prep_code[]={
	{0x39, {LCM_SEND(8), {0x6, 0, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}}},
	{0x37, {LCM_SEND(2), {0x3, 0}}},
};

static LCM_Force_Cmd_Code rd_prep_code_1[]={
	{0x37, {LCM_SEND(2), {0x1, 0}}},
};

static LCM_Init_Code sleep_in[] =  {
{LCM_SEND(1), {0x28}},
{LCM_SLEEP(10)},
{LCM_SEND(1), {0x10}},
{LCM_SLEEP(120)},
};

static LCM_Init_Code sleep_out[] =  {
{LCM_SEND(1), {0x11}},
{LCM_SLEEP(120)},
{LCM_SEND(1), {0x29}},
{LCM_SLEEP(20)},
};

static LCM_Init_Code init_data[] = {
	{LCM_SEND(7),{5,0,0xFF,0xAA,0x55,0xA5,0x80}},
	{LCM_SEND(2),{0x6F,0X0E}},
	{LCM_SEND(2),{0xF4,0X3A}},
	{LCM_SEND(7),{5,0,0xFF,0xAA,0x55,0xA5,0x00}},
	{LCM_SEND(8),{6,0,0xF0,0x55,0xAA,0x52,0x08,0x01}},
	{LCM_SEND(6),{4,0,0xB0,0x0A,0x0A,0x0A}},
	{LCM_SEND(6),{4,0,0xB6,0x44,0x44,0x44}},
	{LCM_SEND(6),{4,0,0xB1,0x0A,0x0A,0x0A}},
	{LCM_SEND(6),{4,0,0xB7,0x34,0x34,0x34}},
	{LCM_SEND(6),{4,0,0xB2,0x00,0x00,0x00}},
	{LCM_SEND(6),{4,0,0xB8,0x24,0x24,0x24}},
	{LCM_SEND(2),{0xBF,0X01}},
	{LCM_SEND(6),{4,0,0xB3,0x08,0x08,0x08}},
	{LCM_SEND(6),{4,0,0xB9,0x34,0x34,0x34}},
	{LCM_SEND(6),{4,0,0xB5,0x06,0x06,0x06}},
	{LCM_SEND(2),{0xC3,0X05}},
	{LCM_SEND(6),{4,0,0xBA,0x14,0x14,0x14}},
	{LCM_SEND(6),{4,0,0xBC,0x00,0x90,0x00}},
	{LCM_SEND(6),{4,0,0xBD,0x00,0x90,0x00}},
	{LCM_SEND(5),{3,0,0xBE,0x00,0x7F}},
	{LCM_SEND(2),{0xC2,0X03}},
	{LCM_SEND(55),{53,0,0xD1,0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{LCM_SEND(55),{53,0,0xD2,0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{LCM_SEND(55),{53,0,0xD3,0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{LCM_SEND(55),{53,0,0xD4,0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{LCM_SEND(55),{53,0,0xD5,0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{LCM_SEND(55),{53,0,0xD6,0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{LCM_SEND(8),{6,0,0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{LCM_SEND(5),{3,0,0xB1,0xFC,0x00}},
	{LCM_SEND(2),{0xB6,0X05}},
	{LCM_SEND(5),{3,0,0xB7,0x70,0x70}},
	{LCM_SEND(7),{5,0,0xB8,0x01,0x05,0x05,0x05}},
	{LCM_SEND(6),{4,0,0xBC,0x02,0x02,0x02}},
	{LCM_SEND(8),{6,0,0xBD,0x01,0x90,0x1C,0x1C,0x00}},
	{LCM_SEND(13),{11,0,0xCB,0x02,0x0B,0xDC,0x01,0x12,0x33,0x33,0x11,0x11,0x0C}},
	{LCM_SEND(2),{0x35,0X00}},
	{LCM_SEND(2),{0x11,0X00}},
	{LCM_SLEEP(120)},
	{LCM_SEND(2),{0x29,0X00}},
	{LCM_SLEEP(20)},
};

static int32_t nt35512_ctc_mipi_init(struct panel_spec *self)
{
	int32_t i;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode =
		self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	LCD_PRINT("nt35512_ctc_mipi_init\n");


	mipi_set_cmd_mode();
	mipi_eotp_set(1,0);

	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		tag = (init->tag >> 24);
		if (tag & LCM_TAG_SEND) {
			mipi_gen_write(init->data, (init->tag & LCM_TAG_MASK));
			udelay(20);
		} else if (tag & LCM_TAG_SLEEP) {
			mdelay((init->tag & LCM_TAG_MASK));
		}
		init++;
	}
	mipi_eotp_set(1,1);
	return 0;
}

static uint32_t nt35512_ctc_readid(struct panel_spec *self)
{
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code;
	uint8_t read_data[5] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;


	mipi_set_cmd_mode_t mipi_set_cmd_mode =
		self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	LCD_PRINT("lcd_nt35512_ctc_mipi kernel read id!\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(0,1);

	for (j = 0; j < 4; j++) {
		rd_prepare = rd_prep_code;
		for (i = 0; i < ARRAY_SIZE(rd_prep_code); i++) {
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if (tag & LCM_TAG_SEND)
				mipi_force_write(rd_prepare->datatype,
					rd_prepare->real_cmd_code.data,
					(rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			else if (tag & LCM_TAG_SLEEP)
				mdelay((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			rd_prepare++;
		}

		read_rtn = mipi_force_read(0xc5, 3, (uint8_t *)read_data);
		LCD_PRINT("lcd_nt35512_ctc_mipi read id 0xa1 value is 0x%x, 0x%x, 0x%x !\n",
			read_data[0], read_data[1], read_data[2]);

		mipi_eotp_set(1,1);
		
		if((0x55 == read_data[0])&&(0x12 == read_data[1])){
			LCD_PRINT("lcd_nt35512_ctc_mipi read id success!\n");
			return 0x5512;
		}
	}
	
	printk(KERN_ERR "lcd_nt35512_ctc_mipi identify fail!\n");
	return 0;
}

static uint32_t nt35512_ctc_readpowermode(struct panel_spec *self)
{
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code_1;
	uint8_t read_data[1] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;

	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	LCD_PRINT("lcd_nt35512_ctc_mipi read power mode!\n");
	mipi_eotp_set(0, 1);
	for (j = 0; j < 4; j++) {
		rd_prepare = rd_prep_code_1;
		for (i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++) {
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if (tag & LCM_TAG_SEND)
				mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data,
					(rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			else if (tag & LCM_TAG_SLEEP)
				msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			rd_prepare++;
		}
		read_rtn = mipi_force_read(0x0A, 1, (uint8_t *)read_data);
		/*printk("lcd_otm80193a_mipi read power mode 0x0A value is 0x%x!,
			read result(%d)\n", read_data[0], read_rtn);*/
		if ((0x9c == read_data[0])  && (0 == read_rtn)) {
			LCD_PRINT("lcd_nt35512_ctc_mipi read power mode success!\n");
			mipi_eotp_set(1, 1);
			return 0x9c;
		}
	}

	LCD_PRINT(KERN_ERR "lcd_nt35512_ctc_mipi read power mode fail!0x0A value is 0x%x!, read result(%d)\n",
		read_data[0], read_rtn);
	mipi_eotp_set(1, 1);
	return 0x0;
}

static int32_t nt35512_ctc_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

#ifndef FB_CHECK_ESD_IN_VFP
	mipi_set_lp_mode_t mipi_set_data_lp_mode =
		self->info.mipi->ops->mipi_set_data_lp_mode;
	mipi_set_hs_mode_t mipi_set_data_hs_mode =
		self->info.mipi->ops->mipi_set_data_hs_mode;
	mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
	mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
	uint16_t work_mode = self->info.mipi->work_mode;
#endif

	LCD_PRINT("nt35512_ctc_check_esd!\n");
#ifndef FB_CHECK_ESD_IN_VFP
	if (SPRDFB_MIPI_MODE_CMD == work_mode)
		mipi_set_lp_mode();
	else
		mipi_set_data_lp_mode();
#endif
	power_mode = nt35512_ctc_readpowermode(self);
	//power_mode = 0x0;
#ifndef FB_CHECK_ESD_IN_VFP
	if (SPRDFB_MIPI_MODE_CMD == work_mode)
		mipi_set_hs_mode();
	else
		mipi_set_data_hs_mode();
#endif
	if (power_mode == 0x9c) {
		LCD_PRINT("nt35512_ctc_check_esd OK!\n");
		return 1;
	} else {
		LCD_PRINT(KERN_ERR "nt35512_ctc_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}

static int32_t nt35512_ctc_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk(KERN_DEBUG "nt35512_ctc_enter_sleep, is_sleep = %d\n", is_sleep);

	if (is_sleep) {
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	} else {
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}
	mipi_eotp_set(1, 0);

	for (i = 0; i < size; i++) {
		tag = (sleep_in_out->tag >> 24);
		if (tag & LCM_TAG_SEND)
			mipi_gen_write(sleep_in_out->data, (sleep_in_out->tag & LCM_TAG_MASK));
		else if (tag & LCM_TAG_SLEEP)
			msleep((sleep_in_out->tag & LCM_TAG_MASK));
		sleep_in_out++;
	}
	mipi_eotp_set(1, 1);

	return 0;
}

static struct panel_operations lcd_nt35512_ctc_mipi_operations = {
	.panel_init = nt35512_ctc_mipi_init,
	.panel_readid = nt35512_ctc_readid,
	.panel_esd_check = nt35512_ctc_check_esd,
	.panel_enter_sleep = nt35512_ctc_enter_sleep,
};

static struct timing_rgb lcd_nt35512_ctc_mipi_timing = {
	.hfp = 48, /*46*/
	.hbp = 48, /*44*/
	.hsync = 8,
	.vfp = 16, /*14*/
	.vbp = 4,
	.vsync = 8,/* 2*/
};

static struct info_mipi lcd_nt35512_ctc_mipi_info = {
	.work_mode				= SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width		= 24, /*18,16*/
	.lan_number			= 2,
	//.phy_feq				= 500*1000,
	.phy_feq				= 481*1000,
	.h_sync_pol				= SPRDFB_POLARITY_POS,
	.v_sync_pol				= SPRDFB_POLARITY_POS,
	.de_pol					= SPRDFB_POLARITY_POS,
	.te_pol					= SPRDFB_POLARITY_POS,
	.color_mode_pol			= SPRDFB_POLARITY_NEG,
	.shut_down_pol			= SPRDFB_POLARITY_NEG,
	.timing					= &lcd_nt35512_ctc_mipi_timing,
	.ops					= NULL,
};

struct panel_spec lcd_nt35512_ctc_mipi_spec = {
	.width					= 480,
	.height					= 800,
	.fps					= 60,
	.reset_timing = {10, 50, 100},
	.type					= LCD_MODE_DSI,
	.direction				= LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	
	/*.suspend_mode = SEND_SLEEP_CMD,*/
	.info = {
		.mipi				= &lcd_nt35512_ctc_mipi_info
	},
	.ops					= &lcd_nt35512_ctc_mipi_operations,
};

struct panel_cfg lcd_nt35512_ctc_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x5512,
	.lcd_name = "lcd_nt35512_ctc_mipi",
	.panel = &lcd_nt35512_ctc_mipi_spec,
};

static int __init lcd_nt35512_ctc_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_nt35512_ctc_mipi);
}

subsys_initcall(lcd_nt35512_ctc_mipi_init);

