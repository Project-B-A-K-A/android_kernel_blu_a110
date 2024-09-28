
/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kxtj9.h>
#include <linux/input-polldev.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#ifdef    CONFIG_HAS_EARLYSUSPEND			//added by Zorro
#include <linux/earlysuspend.h>				//added by Zorro
#endif /* CONFIG_HAS_EARLYSUSPEND */	//added by Zorro

static int kxtj9_suspend(struct i2c_client *client, pm_message_t mesg);
static int kxtj9_resume(struct i2c_client *client);


#define NAME			"kxtj2_1009"  //pty 20150211 "kxtj9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* DATA CONTROL REGISTER BITS */
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F		3
#define ODR200F		4
#define ODR400F		5
#define ODR800F		6
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL		(1 << 3)
#define KXTJ9_IEA		(1 << 4)
#define KXTJ9_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

#define CAL_COUNT	5	//校准平均数据组数    //add by xie
#define	CAL_PATH	"/productinfo/KIONIX_ACC_CAL.bin"	//校准数据存储位置  //add by xie
#if 0
#define KXTJ_INFO(fmt,arg...)           printk("<<-STP-INFO->> "fmt"\n",##arg)
#else
#define KXTJ_INFO(fmt,arg...)           do {} while (0)
#endif
/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtj9_odr_table[] = {
	{ 3,	ODR800F },
	{ 5,	ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};

struct kxtj9_data {
	struct i2c_client *client;
	struct kxtj9_platform_data pdata;
	struct input_dev *input_dev;
	int accel_cali[3];   //add by xie
	int accel_data[3];   //add by xie
#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
	struct input_polled_dev *poll_dev;
#endif
	unsigned int last_poll_interval;
	u8 enable;		//added by Zorro
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
#ifdef    CONFIG_HAS_EARLYSUSPEND				//added by Zorro
	struct early_suspend early_suspend;		//added by Zorro
#endif /* CONFIG_HAS_EARLYSUSPEND */		//added by Zorro
};
static int kxtj9_update_odr(struct kxtj9_data *tj9, unsigned int poll_interval);
static int kxtj9_enable(struct kxtj9_data *tj9);

static int kxtj9_i2c_read(struct kxtj9_data *tj9, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;
	
	//err = i2c_smbus_read_byte_data(tj9->client, CTRL_REG1);		//added by Zorro
	//dev_err(&tj9->client->dev, "kxtj9_report_acceleration_data,CTRL_REG1=%x\n", err);	//added by Zorro
	
	err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0)
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");

	x = le16_to_cpu(acc_data[tj9->pdata.axis_map_x]);
	y = le16_to_cpu(acc_data[tj9->pdata.axis_map_y]);
	z = le16_to_cpu(acc_data[tj9->pdata.axis_map_z]);

	x >>= tj9->shift;
	y >>= tj9->shift;
	z >>= tj9->shift;
	KXTJ_INFO("geroge x=%d, y=%d,z=%d\n", x, y, z);  //add by xie

	tj9->accel_data[tj9->pdata.axis_map_x] = (tj9->pdata.negate_x ? -x : x) + tj9->accel_cali[tj9->pdata.axis_map_x];	//做数据补偿   //add by xie
	tj9->accel_data[tj9->pdata.axis_map_y] = (tj9->pdata.negate_y ? -y : y) + tj9->accel_cali[tj9->pdata.axis_map_y];	//做数据补偿   //add by xie
	tj9->accel_data[tj9->pdata.axis_map_z] = (tj9->pdata.negate_z ? -z : z) + tj9->accel_cali[tj9->pdata.axis_map_z];	//做数据补偿   //add by  xie

	//tj9->accel_data[tj9->pdata.axis_map_x] = x + tj9->accel_cali[tj9->pdata.axis_map_x];	//做数据补偿   //add by xie
	//tj9->accel_data[tj9->pdata.axis_map_y] = y + tj9->accel_cali[tj9->pdata.axis_map_y];	//做数据补偿   //add by xie
	//tj9->accel_data[tj9->pdata.axis_map_z] = z + tj9->accel_cali[tj9->pdata.axis_map_z];	//做数据补偿   //add by  xie

	KXTJ_INFO("geroge accel_data x=%d, y=%d, z=%d\n", tj9->accel_data[tj9->pdata.axis_map_x], tj9->accel_data[tj9->pdata.axis_map_y], tj9->accel_data[tj9->pdata.axis_map_z]);

        input_report_abs(tj9->input_dev, ABS_X, tj9->accel_data[tj9->pdata.axis_map_x]);
	input_report_abs(tj9->input_dev, ABS_Y, tj9->accel_data[tj9->pdata.axis_map_y]);
	input_report_abs(tj9->input_dev, ABS_Z, tj9->accel_data[tj9->pdata.axis_map_z]);

	//input_report_abs(tj9->input_dev, ABS_X, tj9->pdata.negate_x ? -x : x);
	//input_report_abs(tj9->input_dev, ABS_Y, tj9->pdata.negate_y ? -y : y);
	//input_report_abs(tj9->input_dev, ABS_Z, tj9->pdata.negate_z ? -z : z);
	input_sync(tj9->input_dev);
}

static irqreturn_t kxtj9_isr(int irq, void *dev)
{
	struct kxtj9_data *tj9 = dev;
	int err;
	/* data ready is the only possible interrupt type */
	kxtj9_report_acceleration_data(tj9);
	//dev_err(&tj9->client->dev, "KXTJ9 ISR entered\n");	//added by Zorro

	err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
	if (err < 0)
		dev_err(&tj9->client->dev,
			"error clearing interrupt status: %d\n", err);

	return IRQ_HANDLED;
}

/* Returns the calibration value of the device */     //this function add by xie
static ssize_t kionix_accel_get_cali(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	int calibration[3];
//	int checksum;
	int   sum_x, sum_y, sum_z;
	int    i;
	struct file *fp=NULL;
	mm_segment_t old_fs;
	loff_t pos; 
	int	cal_buf[4];

	int err;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);
	
	err = kxtj9_update_odr(tj9, 20);	//set ODR to 50Hz/20ms
	if (err < 0) 
	{
		dev_err(&tj9->client->dev,
			"set ODR error, err = %d. Abort.\n", err);
		return err;
	}

	err = kxtj9_enable(tj9);     	//enable sensor

	if (err < 0) 
	{
		dev_err(&tj9->client->dev,
			"sensor enable error, err = %d. Abort.\n", err);
		return err;
	}

	msleep(25);     							//throw away the first sensor data
 
	tj9->accel_cali[0] = tj9->accel_cali[1] = tj9->accel_cali[2] = 0;
	sum_x = sum_y = sum_z = 0;
	for(i = 0; i<CAL_COUNT; i++)		//Get multi-group data for average
	{
		kxtj9_report_acceleration_data(tj9);
		sum_x  += tj9->accel_data[tj9->pdata.axis_map_x];
		sum_y  += tj9->accel_data[tj9->pdata.axis_map_y];
		sum_z  += tj9->accel_data[tj9->pdata.axis_map_z];
//		printk("tj9->accel_data[tj9->axis_map_x]=%d, tj9->accel_data[tj9->axis_map_y]=%d, tj9->accel_data[tj9->axis_map_z]=%d\n",
//				tj9->accel_data[tj9->axis_map_x], tj9->accel_data[tj9->axis_map_y], tj9->accel_data[tj9->axis_map_z]);
		msleep(25);
	 }
	       
	sum_x = sum_x/CAL_COUNT;
	sum_y = sum_y/CAL_COUNT;
	sum_z = sum_z/CAL_COUNT;
      KXTJ_INFO("sum_x=%d, sum_y=%d, sum_z=%d\n",
				sum_x, sum_y, sum_z);
	//X, Y, Z offset

#if defined(CONFIG_PROJS_V2520)||defined(CONFIG_PROJS_V2510)	
	tj9->accel_cali[tj9->pdata.axis_map_x] = -sum_x;
	tj9->accel_cali[tj9->pdata.axis_map_y] = -sum_y;
	tj9->accel_cali[tj9->pdata.axis_map_z] =  1024 -sum_z ;
#else
	tj9->accel_cali[tj9->pdata.axis_map_x] = -sum_x;
	tj9->accel_cali[tj9->pdata.axis_map_y] = -sum_y;
	tj9->accel_cali[tj9->pdata.axis_map_z] = -sum_z -1024;
#endif

//    printk("tj9->accel_cali[tj9->axis_map_x] =%d, tj9->accel_cali[tj9->axis_map_y]=%d, tj9->accel_cali[tj9->axis_map_z]=%d\n",
//				tj9->accel_cali[tj9->axis_map_x] , tj9->accel_cali[tj9->axis_map_y], tj9->accel_cali[tj9->axis_map_z]);	
	
	cal_buf[0] = calibration[0] = tj9->accel_cali[tj9->pdata.axis_map_x];
	cal_buf[1] = calibration[1] = tj9->accel_cali[tj9->pdata.axis_map_y];
	cal_buf[2] = calibration[2] = tj9->accel_cali[tj9->pdata.axis_map_z];
	cal_buf[3] = calibration[0] + calibration[1] + calibration[2];		//for checksum
//	sprintf(cal_buf,"%8x%8x%8x%8x",calibration[0], calibration[1], calibration[2], checksum);
	KXTJ_INFO("cal_buf[0] is %d",calibration[0]);
	KXTJ_INFO("cal_buf[1]  is %d",calibration[1]);
	KXTJ_INFO("cal_buf[2]  is %d",calibration[2]);
	KXTJ_INFO("cal_buf[3]  is %d",calibration[3]);

	mutex_unlock(&input_dev->mutex);
	
	fp = filp_open(CAL_PATH, O_CREAT|O_RDWR|O_SYNC, S_IRWXU|S_IRWXG|S_IRWXO);		//Open or create file
	if(IS_ERR(fp))	
	{
		dev_err(&tj9->client->dev,
			"Create calibration file fail.\n");
	}else{
	
	old_fs=get_fs();
	set_fs(KERNEL_DS);
	pos = 0;		//Write position start from 0
	vfs_write(fp, (u8*)cal_buf, 16, &pos);
    
	filp_close(fp, NULL);
	set_fs(old_fs);
	}

	return sprintf(buf, "%d %d %d\n", calibration[0], calibration[1], calibration[2]);
}

static int kionix_strtok(const char *buf, size_t count, char **token, const int token_nr)
{
	char *buf2 = (char *)kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
	char **token2 = token;
	unsigned int num_ptr = 0, num_nr = 0, num_neg = 0;
	int i = 0, start = 0, end = (int)count;

	strcpy(buf2, buf);

	/* We need to breakup the string into separate chunks in order for kstrtoint
	 * or strict_strtol to parse them without returning an error. Stop when the end of
	 * the string is reached or when enough value is read from the string */
	while((start < end) && (i < token_nr)) {
		/* We found a negative sign */
		if(*(buf2 + start) == '-') {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				/* If there is a pending negative sign, we adjust the variables to account for it */
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
				/* Reset */
				num_ptr = num_nr = 0;
			}
			/* This indicates that there is a pending negative sign in the string */
			num_neg = 1;
		}
		/* We found a numeric */
		else if((*(buf2 + start) >= '0') && (*(buf2 + start) <= '9')) {
			/* If the previous char(s) are not numeric, set num_ptr to current char */
			if(num_nr < 1)
				num_ptr = start;
			num_nr++;
		}
		/* We found an unwanted character */
		else {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
			}
			/* Reset all the variables to start afresh */
			num_ptr = num_nr = num_neg = 0;
		}
		start++;
	}

	kfree(buf2);

	return (i == token_nr) ? token_nr : -1;
}

/* Allow users to change the calibration value of the device */   //this function add by xie
static ssize_t kionix_accel_set_cali(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	
	const int cali_count = 3; /* How many calibration that we expect to get from the string */
	char **buf2;
	long calibration[cali_count];
	int err = 0, i = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);
	
	buf2 = (char **)kzalloc(cali_count * sizeof(char *), GFP_KERNEL);
	
	if(kionix_strtok(buf, count, buf2, cali_count) < 0) {
		dev_err(&tj9->client->dev,
			"Not enough calibration data being read.\n");
	}
	else {
		/* Convert string to integers  */
		for(i = 0 ; i < cali_count ; i++) {
			err = kstrtoint((const char *)*(buf2 + i), 10, (int *)&calibration[i]);
			if (err < 0) {
							dev_err(&tj9->client->dev,
								"No calibration data will be updated.\n");
				goto exit;
			}
		}

		tj9->accel_cali[tj9->pdata.axis_map_x] = (int)calibration[0];
		tj9->accel_cali[tj9->pdata.axis_map_y] = (int)calibration[1];
		tj9->accel_cali[tj9->pdata.axis_map_z] = (int)calibration[2];
		KXTJ_INFO("map_x is %d",calibration[0]);
		KXTJ_INFO("map_y is %d",calibration[1]);
		KXTJ_INFO("map_z is %d",calibration[2]);
	}
	mutex_unlock(&input_dev->mutex);
	
exit:
	for(i = 0 ; i < cali_count ; i++)
		kfree(*(buf2+i));

	kfree(buf2);

	return (err < 0) ? err : count;
}

static DEVICE_ATTR(cali, 0777, kionix_accel_get_cali, kionix_accel_set_cali); //add by xie

static int kxtj9_update_g_range(struct kxtj9_data *tj9, u8 new_g_range)
{
	switch (new_g_range) {
	case KXTJ9_G_2G:
		tj9->shift = 4;
		break;
	case KXTJ9_G_4G:
		tj9->shift = 3;
		break;
	case KXTJ9_G_8G:
		tj9->shift = 2;
		break;
	default:
		return -EINVAL;
	}

	tj9->ctrl_reg1 &= 0xe7;
	tj9->ctrl_reg1 |= new_g_range;

	return 0;
}

static int kxtj9_update_odr(struct kxtj9_data *tj9, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtj9_odr_table); i++) {
		tj9->data_ctrl = kxtj9_odr_table[i].mask;
		if (poll_interval < kxtj9_odr_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

static int kxtj9_device_power_on(struct kxtj9_data *tj9)
{
	if (tj9->pdata.power_on)
		return tj9->pdata.power_on();

	return 0;
}

static void kxtj9_device_power_off(struct kxtj9_data *tj9)
{
	int err;

	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		dev_err(&tj9->client->dev, "soft power off failed\n");

	if (tj9->pdata.power_off)
		tj9->pdata.power_off();

}

static int kxtj9_enable(struct kxtj9_data *tj9)
{
	int err;

	err = kxtj9_device_power_on(tj9);
	if (err < 0)
		return err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (tj9->client->irq) {
		err = i2c_smbus_write_byte_data(tj9->client,
						INT_CTRL1, tj9->int_ctrl);
		if (err < 0)
			return err;
	}

	err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);
	if (err < 0)
		return err;

	/* turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	err = kxtj9_update_odr(tj9, tj9->last_poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (tj9->client->irq) {
		err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
		if (err < 0) {
			dev_err(&tj9->client->dev,
				"error clearing interrupt: %d\n", err);
			goto fail;
		}
	}

	return 0;

fail:
	kxtj9_device_power_off(tj9);
	return err;
}

static void kxtj9_disable(struct kxtj9_data *tj9)
{
	kxtj9_device_power_off(tj9);
//	int err;
//	err = i2c_smbus_read_byte_data(tj9->client, CTRL_REG1);		//added by Zorro
//	dev_err(&tj9->client->dev, "kxtj9_disable entered! CTRL_REG1=%x\n", err);	//added by Zorro	
}

static int kxtj9_input_open(struct input_dev *input)
{
	struct kxtj9_data *tj9 = input_get_drvdata(input);

	return kxtj9_enable(tj9);
}

static void kxtj9_input_close(struct input_dev *dev)
{
	struct kxtj9_data *tj9 = input_get_drvdata(dev);

	kxtj9_disable(tj9);
}

static void kxtj9_init_input_device(struct kxtj9_data *tj9,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = "kxtj9_acc";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tj9->client->dev;
}

static int kxtj9_setup_input_device(struct kxtj9_data *tj9)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tj9->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tj9->input_dev = input_dev;

	input_dev->open = kxtj9_input_open;
	input_dev->close = kxtj9_input_close;
	input_set_drvdata(input_dev, tj9);

	kxtj9_init_input_device(tj9, input_dev);

	err = input_register_device(tj9->input_dev);
	if (err) {
		dev_err(&tj9->client->dev,
			"unable to register input polled device %s: %d\n",
			tj9->input_dev->name, err);
		input_free_device(tj9->input_dev);
		return err;
	}

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj9_get_poll(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tj9->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj9_set_poll(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	disable_irq(client->irq);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	tj9->last_poll_interval = max(interval, tj9->pdata.min_interval);

	kxtj9_update_odr(tj9, tj9->last_poll_interval);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}


static ssize_t kxtj9_get_chip_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", "kxtj2_1009");
}


/* Returns currently enable status */
static ssize_t kxtj9_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tj9->enable);
}

/* Allow users to set power status */
static ssize_t kxtj9_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	unsigned int enable_temp;
	int err;

	err = kstrtouint(buf, 10, &enable_temp);
	if (err < 0)
		return err;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);
	tj9->enable = enable_temp;
	
	if(tj9->enable)
			kxtj9_enable(tj9);
	else
			kxtj9_disable(tj9);

	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll, 0777, kxtj9_get_poll, kxtj9_set_poll);
static DEVICE_ATTR(enable, 0777, kxtj9_get_enable, kxtj9_set_enable);		//added by Zorro
static DEVICE_ATTR(chipinfo, 0777, kxtj9_get_chip_info, NULL);

static struct attribute *kxtj9_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,		//added by Zorro
	&dev_attr_cali.attr,		//added by Zorro
	&dev_attr_chipinfo.attr,		
	NULL
};

static struct attribute_group kxtj9_attribute_group = {
	.attrs = kxtj9_attributes
};


#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
static void kxtj9_poll(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;
	unsigned int poll_interval = dev->poll_interval;

	kxtj9_report_acceleration_data(tj9);

	if (poll_interval != tj9->last_poll_interval) {
		kxtj9_update_odr(tj9, poll_interval);
		tj9->last_poll_interval = poll_interval;
	}
}

static void kxtj9_polled_input_open(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;

	kxtj9_enable(tj9);
}

static void kxtj9_polled_input_close(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;

	kxtj9_disable(tj9);
}

static int kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
	int err;
	struct input_polled_dev *poll_dev;
	poll_dev = input_allocate_polled_device();

	if (!poll_dev) {
		dev_err(&tj9->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	tj9->poll_dev = poll_dev;
	tj9->input_dev = poll_dev->input;

	poll_dev->private = tj9;
	poll_dev->poll = kxtj9_poll;
	poll_dev->open = kxtj9_polled_input_open;
	poll_dev->close = kxtj9_polled_input_close;

	kxtj9_init_input_device(tj9, poll_dev->input);

	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&tj9->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}

	return 0;
}

static void kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
	input_unregister_polled_device(tj9->poll_dev);
	input_free_polled_device(tj9->poll_dev);
}

#else

static inline int kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
	return -ENOSYS;
}

static inline void kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
}

#endif

static int kxtj9_verify(struct kxtj9_data *tj9)
{
	int retval;	
	retval = kxtj9_device_power_on(tj9);
	if (retval < 0)
		return retval;
	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&tj9->client->dev, "read err int source\n");
		goto out;
	}
	retval = (retval != 0x09 && retval != 0x08) ? -EIO : 0;	
out:
	kxtj9_device_power_off(tj9);
	return retval;
}
#ifdef CONFIG_OF
static struct kxtj9_platform_data *kxtj9_acc_parse_dt(struct device *dev)
{
	struct kxtj9_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct kxtj9_platform_data");
		return NULL;
	}
	//ret = of_property_read_u32(np, "init_interval", &pdata->init_interval);
	ret = of_property_read_u32(np, "poll_interval", &pdata->init_interval);
	if(ret){
		dev_err(dev, "fail to get poll_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "min_interval", &pdata->min_interval);
	if(ret){
		dev_err(dev, "fail to get min_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "g_range", &pdata->g_range);
	if(ret){
		dev_err(dev, "fail to get g_range\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_x", &pdata->axis_map_x);
	if(ret){
		dev_err(dev, "fail to get axis_map_x\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_y", &pdata->axis_map_y);
	if(ret){
		dev_err(dev, "fail to get axis_map_y\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_z", &pdata->axis_map_z);
	if(ret){
		dev_err(dev, "fail to get axis_map_z\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_x", &pdata->negate_x);
	if(ret){
		dev_err(dev, "fail to get negate_x\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_y", &pdata->negate_y);
	if(ret){
		dev_err(dev, "fail to get negate_y\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_z", &pdata->negate_z);
	if(ret){
		dev_err(dev, "fail to get negate_z\n");
		goto fail;
	}
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif
static int kxtj9_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	const struct kxtj9_platform_data *pdata = client->dev.platform_data;
	struct kxtj9_data *tj9;
	struct kxtj9_acc_driver *acceld = i2c_get_clientdata(client);
	int err;	
	
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	if (np && !pdata){
		pdata = kxtj9_acc_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		if(!pdata){
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif
	
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}	
	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}	
	tj9 = kzalloc(sizeof(*tj9), GFP_KERNEL);
	if (!tj9) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}	
	client->irq = gpio_to_irq(238);
	tj9->client = client;
	tj9->pdata = *pdata;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_free_mem;
	}
	err = kxtj9_verify(tj9);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}
	i2c_set_clientdata(client, tj9);
	tj9->ctrl_reg1 = tj9->pdata.res_12bit | tj9->pdata.g_range | RES_12BIT;		//Changed by Zorro
	tj9->last_poll_interval = tj9->pdata.init_interval;
	
	if (client->irq) {
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tj9->int_ctrl |= KXTJ9_IEN | KXTJ9_IEA | KXTJ9_IEL;
		tj9->ctrl_reg1 |= DRDYE;	
		err = kxtj9_setup_input_device(tj9);
		if (err)
			goto err_pdata_exit;		
		err = request_threaded_irq(client->irq, NULL, kxtj9_isr,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "kxtj9-irq", tj9);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err_destroy_input;
		}		
		err = sysfs_create_group(&client->dev.kobj, &kxtj9_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_free_irq;
		}	
	} else {	
		err = kxtj9_setup_polled_device(tj9);
		if (err)
			goto err_pdata_exit;
	}	
	return 0;

#ifdef    CONFIG_HAS_EARLYSUSPEND			//added by Zorro
	/* The higher the level, the earlier it resume, and the later it suspend */
	tj9->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 50;
	tj9->early_suspend.suspend = kxtj9_suspend;
	tj9->early_suspend.resume = kxtj9_resume;
	register_early_suspend(&tj9->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */	//added by Zorro

err_free_irq:
	free_irq(client->irq, tj9);
err_destroy_input:
	input_unregister_device(tj9->input_dev);
err_pdata_exit:
	if (tj9->pdata.exit)
		tj9->pdata.exit();
err_free_mem:
	kfree(tj9);
exit_alloc_platform_data_failed:
	return err;
}

static int kxtj9_remove(struct i2c_client *client)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	
#ifdef    CONFIG_HAS_EARLYSUSPEND			//added by Zorro
	unregister_early_suspend(&tj9->early_suspend);		//added by Zorro
#endif /* CONFIG_HAS_EARLYSUSPEND */				//added by Zorro
	
	if (client->irq) {
		sysfs_remove_group(&client->dev.kobj, &kxtj9_attribute_group);
		free_irq(client->irq, tj9);
		input_unregister_device(tj9->input_dev);
	} else {
		kxtj9_teardown_polled_device(tj9);
	}
	
	if (tj9->pdata.exit)
		tj9->pdata.exit();

	kfree(tj9);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxtj9_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;	
	mutex_lock(&input_dev->mutex);
	if (input_dev->users)
		kxtj9_disable(tj9);

	mutex_unlock(&input_dev->mutex);
	
	//dev_err(&tj9->client->dev, "kxtj9_suspend entered!\n");	//added by Zorro
	return 0;
}

static int kxtj9_resume(struct i2c_client *client)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;	
	mutex_lock(&input_dev->mutex);
	if (input_dev->users)
		kxtj9_enable(tj9);

	mutex_unlock(&input_dev->mutex);
	
	//dev_err(&tj9->client->dev, "kxtj9_resume entered!\n");	//added by Zorro	
	return 0;
}
#endif

//static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);

static const struct i2c_device_id kxtj9_acc_id[]
= { {NAME, 0}, {}, };

MODULE_DEVICE_TABLE(i2c, kxtj9_acc_id);

static const struct of_device_id kxtj9_acc_of_match[] = {
       { .compatible = "kionix,kxtj2_1009", },
       { }
};
MODULE_DEVICE_TABLE(of, kxtj9_acc_of_match);
static struct i2c_driver kxtj9_acc_driver = {
	.driver = {
		   .name = NAME,
		   .of_match_table = kxtj9_acc_of_match,
		   },
	.probe = kxtj9_probe,
	.remove = kxtj9_remove,
	.resume = kxtj9_resume,
	.suspend = kxtj9_suspend,
	.id_table = kxtj9_acc_id,
};

static int __init kxtj9_acc_init(void)
{
	return i2c_add_driver(&kxtj9_acc_driver);
}

static void __exit kxtj9_acc_exit(void)
{
	i2c_del_driver(&kxtj9_acc_driver);
	return;
}

module_init(kxtj9_acc_init);
module_exit(kxtj9_acc_exit);
MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
