/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifndef CONFIG_64BIT
#include <soc/sprd/hardware.h>
#include <soc/sprd/board.h>
#endif
#include <soc/sprd/adi.h>
#include "../common/parse_hwinfo.h"

#define SPRD_PWM0_CTRL_OFST                       0xEC
#define SPRD_PWM0_PATTERN_HIGHT_OFST              0x10
#define SPRD_PWM0_PATTERT_LOW_OFST                0xC
#define SPRD_PWM0_TONE_OFST                       0x8
#define SPRD_PWM0_RATION_OFST                     0x4
#define SPRD_WHTLED_CTRL_OFST                     0xF0

#define SPRD_BLTC_BASE	SPRD_ADI_BASE+0x8440
#define BLTC_CTRL				0x0000
#define BLTC_R_PRESCL			0x0004
#define BLTC_G_PRESCL			0x0014
#define BLTC_B_PRESCL			0x0024
#define BLTC_PRESCL_OFFSET		0x0004
#define BLTC_DUTY_OFFSET		0x0004
#define PWM_MOD_COUNTER 0xFE
extern int sci_efuse_ib_trim_get(unsigned int *p_cal_data);

unsigned int ib_trim_cal_data = 0;
static int init_flag = 1;
unsigned long pwm_duty;
int sprd_flash_on(void)
{
	#if 0
	if (init_flag && sci_efuse_ib_trim_get(&ib_trim_cal_data)) {
	/*
		1. set ib_trim_cal_data to WHTLED_CTRL的IB_TRIM（0x400388F0 [15:9]）
		2. set IB_TRIM_EM_SEL of RGB_CTRL to 0（0x400388EC [11]）
	*/
		sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x7F << 9);
		sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, (ib_trim_cal_data & 0x7F) << 9);
		sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, (0x1 << 11));
		init_flag = 0;
		printk("parallel sprd_flash_on trim = %d\n", ib_trim_cal_data);
	}

	printk("parallel sprd_flash_on \n");
	/*ENABLE THE PWM0 CONTROLLTER: RTC_PWM0_EN=1 & PWM0_EN=1*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xFFFF);
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xC805);

	/*SET PWM0 PATTERN HIGH*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_PATTERN_HIGHT_OFST, 0xFFFF);

	/*SET PWM0 PATTERN LOW*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_PATTERT_LOW_OFST, 0xFFFF);

	/*TONE DIV USE DEFAULT VALUE*/
	sci_adi_clr(ANA_PWM_BASE + SPRD_PWM0_TONE_OFST, 0x0100);

	/*SET PWM0 DUTY RATIO = 100%: MOD=FF & DUTY=FF*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_RATION_OFST, 0x0100);

	/*ENABLE PWM0 OUTPUT: PWM0_EN=1*/
	sci_adi_set(ANA_PWM_BASE, 0x100);

	/*SET LOW LIGHT */
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0xFFFF);
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1e);//0x1e

	/*ENABLE WHTLED*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1);
	#else
	printk("parallel sprd_flash_on \n");
	if (init_flag){
	sci_adi_clr(ANA_CTL_GLB_BASE + 0x0c,(0x1<<13));
	init_flag = 0;
	}
	sci_adi_set(ANA_CTL_GLB_BASE, (0x1<<10));//ARM_MODULE_EN-enable pclk
    sci_adi_set(ANA_CTL_GLB_BASE + 0x08, (0x1<<8));//RTC_CLK_EN-enable rtc
    sci_adi_clr(ANA_CTL_GLB_BASE + 0xec, (0x1<<0));//SW POWERDOWN DISABLE
//    sci_adi_clr(ANA_CTL_GLB_BASE + 0xec, (0x1f<<4));//CURRENT CONTROL DEFAULT
	sci_adi_clr(ANA_CTL_GLB_BASE + 0xec, (0x16<<4));//CURRENT CONTROL DEFAULT
	sci_adi_set(ANA_CTL_GLB_BASE + 0xf0, (0x1<<8));//WHTLED_SERIES_EN=1
	sci_adi_set(ANA_CTL_GLB_BASE + 0xf0, (0x1<<0));//WHTLED POWERDOWN ENABLE
	sci_adi_clr(ANA_CTL_GLB_BASE + 0xec, (0x1<<0));//RGB_CTRL SW ENABLE
//	sci_adi_set(ANA_CTL_GLB_BASE + 0xec, (0x1f<<4));//RGB_CTRL CURRENT VALUE:BIT[8:4]
	sci_adi_set(ANA_CTL_GLB_BASE + 0xec, (0x16<<4));//RGB_CTRL CURRENT VALUE:BIT[8:4]
	pwm_duty = 255;
	sci_adi_write(SPRD_BLTC_BASE + BLTC_R_PRESCL + BLTC_DUTY_OFFSET, ((pwm_duty&0xff)<<8)|PWM_MOD_COUNTER,0xffff);
	sci_adi_set(SPRD_BLTC_BASE + BLTC_CTRL, (0x1<<0)|(0x1<<1));
	#endif
	return 0;
}

int sprd_flash_high_light(void)
{
	printk("parallel sprd_flash_high_light \n");
	#if 0
	/*SET HIGH LIGHT*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0xFFFF);
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x40);//0x40
	/*ENABLE WHTLED*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1);
	#else


    pwm_duty = 255;
    sci_adi_write(SPRD_BLTC_BASE + BLTC_R_PRESCL + BLTC_DUTY_OFFSET, ((pwm_duty&0xff)<<8)|    PWM_MOD_COUNTER,0xffff); //设置R路占空比
    sci_adi_write(SPRD_BLTC_BASE + BLTC_G_PRESCL + BLTC_DUTY_OFFSET, ((pwm_duty&0xff)<<8)|PWM_MOD_COUNTER,0xffff); //设置G路占空比
    sci_adi_write(SPRD_BLTC_BASE + BLTC_B_PRESCL + BLTC_DUTY_OFFSET, ((pwm_duty&0xff)<<8)|PWM_MOD_COUNTER,0xffff); //设置B路占空比

	sci_adi_set(SPRD_BLTC_BASE + BLTC_CTRL, (0x1<<0)|(0x1<<1));
	sci_adi_set(SPRD_BLTC_BASE + BLTC_CTRL, (0x1<<4)|(0x1<<5));
	sci_adi_set(SPRD_BLTC_BASE + BLTC_CTRL, (0x1<<8)|(0x1<<9));
	#endif
	return 0;
}

int sprd_flash_close(void)
{
	printk("parallel sprd_flash_close \n");
#if 0
	/*DISABLE WHTLED*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0xFFFF);
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1);

	/*DISABLE THE PWM0 CONTROLLTER: RTC_PWM0_EN=0 & PWM0_EN=0*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xC000);

	/*ENABLE PWM0 OUTPUT: PWM0_EN=0*/
	sci_adi_clr(ANA_PWM_BASE, 0x100);
#else
    sci_adi_clr(SPRD_BLTC_BASE + BLTC_CTRL, (0x1<<0));
	sci_adi_clr(SPRD_BLTC_BASE + BLTC_CTRL, (0x1<<4));
	sci_adi_clr(SPRD_BLTC_BASE + BLTC_CTRL, (0x1<<8));
#endif
	return 0;
}

int sprd_flash_cfg(struct sprd_flash_cfg_param *param, void *arg)
{
	return 0;
}
