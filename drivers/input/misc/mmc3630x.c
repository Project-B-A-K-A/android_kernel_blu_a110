/*
 * Copyright (C) 2012 memsic.
 *
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
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>                                                                                                           
#include <linux/sysctl.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <asm/uaccess.h>
#include <linux/device.h>


#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
//#include <linux/sched.h>
#define     POLL_INTERVAL_DEFAULT 20


#include "mmc3630x.h"

#define MMC3630X_MIN_DELAY 20
#define MMC3630X_MAX_DELAY 100
#define MMC3630X_PRODUCT_ID 0xA

#define MMC3630X_SINGLE_POWER 0

#define MMC3630X_DELAY_TM_MS    10
#define MMC3630X_DELAY_SET  75
#define MMC3630X_DELAY_RESET 75
#define MMC3630X_RETRY_COUNT    10
#define MMC3630X_DEFAULT_INTERVAL_MS    50
#define MMC3630X_TIMEOUT_SET_MS 15000

struct mmc3630x_vec {
	int x;
	int y;
	int z;
};

struct mmc3630x_data {
	struct i2c_client *i2c;
	struct input_dev *idev;
	struct mmc3630x_platform_data *pdata;
	struct hrtimer work_timer;
	struct completion report_complete;
	struct task_struct *thread;
	bool hrtimer_running;
	struct mmc3630x_vec last;
	int dir;       	
	int poll_interval;
	int rep_cnt;
	struct mutex lock;
	atomic_t enabled;
};

/*------------sensors data----------------------------------------------------*/
typedef struct {
	/* sensor values */
	int	values[3];
	uint32_t value_divide;
	/* sensor accuracy */
	int8_t status;
} hwm_sensor_data;

struct i2c_client *this_client = NULL;
static atomic_t m_flag =ATOMIC_INIT(0);;
static atomic_t o_flag = ATOMIC_INIT(0);;
static atomic_t m_delay = ATOMIC_INIT(20);

enum {
     OBVERSE_X_AXIS_FORWARD = 0,
     OBVERSE_X_AXIS_RIGHTWARD,
     OBVERSE_X_AXIS_BACKWARD,
     OBVERSE_X_AXIS_LEFTWARD,
     REVERSE_X_AXIS_FORWARD,
     REVERSE_X_AXIS_RIGHTWARD,
     REVERSE_X_AXIS_BACKWARD,
     REVERSE_X_AXIS_LEFTWARD,
     MMC3630X_DIR_COUNT,
};

struct mmc3630x_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 *      *      * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 *      *      * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 *      *      * src is 2, sign_* is (-)1 */
	int src_x;
	int src_y;
	int src_z;

	int sign_x;
	int sign_y;
	int sign_z;
};

static const struct mmc3630x_sensor_axis_remap
mmc3630x_axis_remap_tab[MMC3630X_DIR_COUNT] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

static int mmc3630x_i2c_rxdata(struct i2c_client *i2c, unsigned char *rxData, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		}, };
	unsigned char addr = rxData[0];

	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return -EIO;
	}

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
			length, addr, rxData[0]);
	return 0;
}

static int mmc3630x_i2c_txdata(struct i2c_client *i2c, unsigned char *txData, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		}, };

	if (i2c_transfer(i2c->adapter, msg, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return -EIO;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
			length, txData[0], txData[1]);
	return 0;
}

static int mmc3630x_remap_xyz(int *xyz, int layout)
{
	int temp[3] = {0};
	struct mmc3630x_sensor_axis_remap *map_tab;

	map_tab = &mmc3630x_axis_remap_tab[layout];

	temp[0] = xyz[0];
	temp[1] = xyz[1];
	temp[2] = xyz[2];
	xyz[0] = temp[map_tab->src_x] * map_tab->sign_x;
	xyz[1] = temp[map_tab->src_y] * map_tab->sign_y;
	xyz[2] = temp[map_tab->src_z] * map_tab->sign_z;

	return 0;
}

static int mmc3630x_read_xyz(struct mmc3630x_data *memsic, int *vec)
{
	unsigned char data[6];
	unsigned char buffer[2];
    int tmp[3] = {0};
	int rc = 0;
	//printk("%s xiexie\n",__FUNCTION__);

	/* read xyz raw data */
	data[0] = MMC3630X_REG_DATA;
	rc = mmc3630x_i2c_rxdata(memsic->i2c, data, 6);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
				MMC3630X_REG_DS, __LINE__, rc);
		goto exit;
	}
	tmp[0] = data[1] << 8 | data[0];
	tmp[1] = data[3] << 8 | data[2];
	tmp[2] = data[5] << 8 | data[4];

  //`  mmc3630x_remap_xyz(tmp, memsic->dir);

	vec[0] = tmp[0];
	vec[1] = tmp[1];
	vec[2] = tmp[2];

exit:
	/* send TM cmd before read */
	buffer[0] = MMC3630X_REG_CTRL;
	buffer[1] = MMC3630X_CTRL_TM; 
	if (mmc3630x_i2c_txdata(memsic->i2c, buffer, 2)) {
		dev_warn(&memsic->i2c->dev, "write reg %d failed at %d.(%d)\n",	MMC3630X_REG_CTRL, __LINE__, rc);
	}

	return rc;
}

static void mmc3630x_report_values(struct mmc3630x_data *memsic, int *xyz)
{
	//printk("mmc3630x report mmm sensor data\n");
	input_report_abs(memsic->idev, ABS_RX, xyz[0]);
	input_report_abs(memsic->idev, ABS_RY, xyz[1]);
	input_report_abs(memsic->idev, ABS_RZ, xyz[2]);
	input_sync(memsic->idev);
	//printk("ctsmemsic x = %d y  = %d z= %d\n", xyz[0],xyz[1],mxc400x_z);
}

static void mmc3630x_input_work_func(struct mmc3630x_data *memsic)
{
	int xyz[3] = { 0 };
	int err;

	err = mmc3630x_read_xyz(memsic, xyz);
	
	if (err < 0)
	{
		printk("mmc3630x_read_xyz failed\n");
	}
	else
	{
		mmc3630x_report_values(memsic, xyz);
	}
}

static enum hrtimer_restart mmc3630x_work(struct hrtimer *timer)
{
	struct mmc3630x_data *mag;
	ktime_t poll_delay;
	mag = container_of((struct hrtimer *)timer, struct mmc3630x_data, work_timer);

	complete(&mag->report_complete);
	if (mag->poll_interval > 0)
	{
		poll_delay = ktime_set(0, mag->poll_interval * NSEC_PER_MSEC);
	}
	else
	{
		poll_delay = ktime_set(0, POLL_INTERVAL_DEFAULT * NSEC_PER_MSEC);
	}
	mag->hrtimer_running = true;
	hrtimer_forward_now(&mag->work_timer, poll_delay);

	return HRTIMER_RESTART;
}

static int report_event(void *data)
{
	struct mmc3630x_data *mag = data;

	while(1)
	{
		/* wait for report event */
		wait_for_completion(&mag->report_complete);
		mutex_lock(&mag->lock);
		if (atomic_read(&mag->enabled) <= 0)
		{
			mutex_unlock(&mag->lock);
			continue;
		}
		mmc3630x_input_work_func(mag);
		mutex_unlock(&mag->lock);
	}
	return 0;
}

static struct input_dev *mmc3630x_init_input(struct mmc3630x_data *memsic)
{
	int status;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
	struct input_dev *input = NULL;


	printk("%s xiexie\n",__FUNCTION__);
	hrtimer_init(&memsic->work_timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	memsic->work_timer.function = mmc3630x_work;
	memsic->hrtimer_running = false;
	init_completion(&memsic->report_complete);
	memsic->thread = kthread_run(report_event, memsic, "sensor_report_event");
	if (IS_ERR(memsic->thread))
	{
		printk("unable to create report_event thread\n");
		return NULL;
	}
	sched_setscheduler_nocheck(memsic->thread, SCHED_FIFO, &param);

	input = input_allocate_device();
	if (!input)
		return NULL;

	input->name = "compass";
	input->phys = "mmc3630x/input0";
	input->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input->evbit);
	input_set_events_per_packet(input, 100);
	input_set_abs_params(input, ABS_RX, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_RY, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_RZ, -32768, 32767, 0, 0);
	//input_set_abs_params(input, ABS_RUDDER, 0, 3, 0, 0);
	
	/* Report the dummy value */
	input_set_abs_params(input, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

	input_set_capability(input, EV_REL, REL_X);
	input_set_capability(input, EV_REL, REL_Y);
	input_set_capability(input, EV_REL, REL_Z);

	status = input_register_device(input);
	if (status) {
		dev_err(&memsic->i2c->dev,
			"error registering input device\n");
		return NULL;
	}

	return input;
}

static int ECS_GetRawData(struct mmc3630x_data *memsic, int *vec)
{
	int err = 0;
	err = mmc3630x_read_xyz(memsic, vec);
	printk("%s xiexie\n",__FUNCTION__);	
	if(err !=0 )
	{
		printk(KERN_ERR "MMC3630X_IOC_TM failed\n");
		return -1;
	}

	// sensitivity 2048 count = 1 Guass = 100uT
	vec[0] = (vec[0] - MMC3630X_OFFSET_X) * 100 / MMC3630X_SENSITIVITY_X;
	vec[1] = (vec[1] - MMC3630X_OFFSET_X) * 100 / MMC3630X_SENSITIVITY_X;
	vec[2] = (vec[2] - MMC3630X_OFFSET_X) * 100 / MMC3630X_SENSITIVITY_X;

	return err;
}

static int mmc3630x_check_device(struct mmc3630x_data *memsic)
{
	int rc;
	unsigned char rd_buffer[2];
	printk("%s xiexie\n",__FUNCTION__);	
	rd_buffer[0] = MMC3630X_REG_PRODUCTID_1;
	rc = mmc3630x_i2c_rxdata(memsic->i2c, rd_buffer, 1);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed.(%d)\n",
				MMC3630X_REG_DS, rc);
		return rc;
	}
	printk("memsic current device is 0x%x\n", rd_buffer[0]);

	if (rd_buffer[0] != MMC3630X_PRODUCT_ID)
		return -ENODEV;

	return 0;
}

#ifdef CONFIG_OF
static int mmc3630x_parse_dt(struct i2c_client *client,struct mmc3630x_data *memsic)
{
	struct device_node *np = client->dev.of_node;
	int tmp = 0;
	int rc;
	printk("%s xiexie\n",__FUNCTION__);

	rc = of_property_read_u32(np, "memsic,dir", &tmp);
	/* does not have a value or the string is not null-terminated */
	if (rc && (rc != -EINVAL)) {
		dev_err(&client->dev, "Unable to read memsic,dir\n");
		return rc;
	} else {
        memsic->dir = 1;
    }
	memsic->dir = tmp;
	printk("memsic add in mmc3630,memsic->direction =%d.\n",memsic->dir);

	return 0;
}
#endif

static int mmc3630x_set_poll_delay(struct mmc3630x_data *memsic, unsigned int delay_msec)
{
  ktime_t poll_delay;
  
  if (atomic_read(&memsic->enabled))
	{
		if (memsic->poll_interval > 0)
		{
			poll_delay = ktime_set(0, memsic->poll_interval * NSEC_PER_MSEC);
		}
		else
		{
			poll_delay = ktime_set(0, POLL_INTERVAL_DEFAULT * NSEC_PER_MSEC);
		}
		hrtimer_start(&memsic->work_timer, poll_delay, HRTIMER_MODE_REL);
	}

	return 0;
}

static int mmc3630x_set_enable(struct mmc3630x_data *memsic, unsigned int enable)
{

	ktime_t poll_delay;
	if (!memsic)
	{
		printk("memsic pointer is NULL\n");
		return -1;
	}

	if (enable)
	{
		memsic->hrtimer_running = true;
		if (memsic->poll_interval > 0)
		{
			poll_delay = ktime_set(0, memsic->poll_interval * NSEC_PER_MSEC);
		}
		else
		{
			poll_delay = ktime_set(0, POLL_INTERVAL_DEFAULT * NSEC_PER_MSEC);
		}
		hrtimer_start(&memsic->work_timer, poll_delay, HRTIMER_MODE_REL);
		atomic_set(&memsic->enabled, 1);
		//printk("ctsmemsic set enable\n");
	} else {
		if (memsic->hrtimer_running)
		{
			memsic->hrtimer_running = false;
			hrtimer_cancel(&memsic->work_timer);
		}
		atomic_set(&memsic->enabled, 0);
	}

	return 0;
}
static ssize_t mmc3630x_chip_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", "mmc3630 chip");     
}

static ssize_t mmc3630x_layout_show(struct device *dev, struct device_attribute *attr, char *buf)
{    
	struct mmc3630x_data *memsic = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", memsic->dir);
}

static ssize_t mmc3630x_layout_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	  unsigned long data;
	  struct mmc3630x_data *memsic = dev_get_drvdata(dev);
	  int error;
 
	  error = kstrtoul(buf, 10, &data);
	  if (error)
		  return error;
	  memsic->dir = data;
	  return count;
}

static ssize_t mmc3630x_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int vec[3];
	struct mmc3630x_data *memsic = dev_get_drvdata(dev);

	ret = mmc3630x_read_xyz(memsic, vec);
	if (ret) {
		dev_warn(&memsic->i2c->dev, "read xyz failed\n");
	}
	
	return sprintf(buf, "%d %d %d\n", vec[0],vec[1],vec[2]);
}

static ssize_t mmc3630x_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("memsic enable show : %d\n", atomic_read(&m_delay));
	return sprintf(buf, "%d\n", atomic_read(&m_delay));
}

static DEVICE_ATTR(chipinfo, S_IRUSR|S_IRGRP, mmc3630x_chip_info, NULL);
static DEVICE_ATTR(layout, S_IRUSR|S_IRGRP|S_IWUSR, mmc3630x_layout_show, mmc3630x_layout_store);
static DEVICE_ATTR(value, S_IRUSR|S_IRGRP, mmc3630x_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUSR|S_IRGRP, mmc3630x_delay_show, NULL);

static struct attribute *mmc3630x_attributes[] = {
	&dev_attr_chipinfo.attr,
	&dev_attr_layout.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	NULL,
};

static const struct attribute_group mmc3630x_attr_group = {
		.attrs = mmc3630x_attributes,
};
/*----------------------------------------------------------------------------*/
static int mmc3630x_open(struct inode *inode, struct file *file)
{
	int ret = -1;
        printk("%s xiexie\n",__FUNCTION__);
	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int mmc3630x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mmc3630x_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	char buff[MMC3630X_BUFSIZE];				/* for chip information */

	int delay;				/* for GET_DELAY */
	int set_delay;
	int status; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	unsigned char data[16] = {0};
	int vec[3] = {0};
	struct mmc3630x_data *clientdata = i2c_get_clientdata(this_client);
	uint32_t enable;
	unsigned char reg_addr;
	unsigned char reg_value;

	switch (cmd)
	{
		case MMC3630X_IOC_SET_DELAY:
			if (copy_from_user(&set_delay, argp, sizeof(set_delay))) {
				printk("memsic add set delay failed\n");
				return -EFAULT;
			}

			if (set_delay < MMC3630X_MIN_DELAY) {
				set_delay = MMC3630X_MIN_DELAY;
			}
			if (set_delay > MMC3630X_MAX_DELAY) {
				set_delay = MMC3630X_MAX_DELAY;
			}

			clientdata->poll_interval = set_delay;
			mmc3630x_set_poll_delay(clientdata, clientdata->poll_interval);
			break;
		case MMC31XX_IOC_TM:
			data[0] = MMC3630X_REG_CTRL;
			data[1] = MMC3630X_CTRL_TM;
			if (mmc3630x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_TM failed\n");
				return -EFAULT;
			}
			/* wait TM done for coming data read */
			msleep(MMC3630X_DELAY_TM_MS);
			break;

		case MMC31XX_IOC_SET:
		case MMC31XX_IOC_RM:
#if MMC3630X_SINGLE_POWER
			data[0] = MMC3630X_REG_CTRL;
			data[1] = MMC3630X_CTRL_REFILL;
			if(mmc3630x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3630X_DELAY_SET);
#endif
			data[0] = MMC3630X_REG_CTRL;
			data[1] = MMC3630X_CTRL_SET;
			if(mmc3630x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3630X_REG_CTRL;
			data[1] = 0;
			if(mmc3630x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_RESET:
		case MMC31XX_IOC_RRM:
#if MMC3630X_SINGLE_POWER
			data[0] = MMC3630X_REG_CTRL;
			data[1] = MMC3630X_CTRL_REFILL;
			if(mmc3630x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3630X_DELAY_RESET);
#endif
			data[0] = MMC3630X_REG_CTRL;
			data[1] = MMC3630X_CTRL_RESET;
			if(mmc3630x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3630X_REG_CTRL;
			data[1] = 0;
			if(mmc3630x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_READ:
			data[0] = MMC3630X_REG_DATA;
			if(mmc3630x_i2c_rxdata(clientdata->i2c, data, 6) < 0)
			{
				printk(KERN_ERR "MMC3630x_IOC_READ failed\n");
				return -EFAULT;
			}

			vec[0] = data[1] << 8 | data[0];
			vec[1] = data[3] << 8 | data[2];
			vec[2] = data[5] << 8 | data[4];

			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3630x_IOC_READ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MMC31XX_IOC_READXYZ:
			mmc3630x_read_xyz(clientdata, vec);
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3630x_IOC_READXYZ: copy to user failed\n");
				return -EFAULT;
			}
			break;
		case ECOMPASS_IOC_GET_DELAY:
			delay = clientdata->poll_interval;
			if(copy_to_user(argp, &delay, sizeof(delay)))
			{
				printk(KERN_ERR "copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECOMPASS_IOC_GET_OPEN_STATUS:
			status = atomic_read(&m_flag);
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				printk("memsiccopy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = atomic_read(&m_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				printk("memsiccopy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECOMPASS_IOC_GET_OFLAG:
			sensor_status = atomic_read(&o_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				printk("memsiccopy_to_user failed.");
				return -EFAULT;
			}
			break;
		case MSENSOR_IOCTL_READ_SENSORDATA:
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			ECS_GetRawData(clientdata, vec);
			sprintf(buff, "%x %x %x", vec[0], vec[1], vec[2]);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_MSENSOR_ENABLE:
			if (copy_from_user(&enable, argp, sizeof(enable))) {
				return -EFAULT;
			}
			if (1 == enable){
				atomic_set(&m_flag, 1);
			} else {
				atomic_set(&m_flag, 0);
			}
			printk("MSENSOR_ENABLE  %s enable = %d xiexie \n",__FUNCTION__,enable);	
			mmc3630x_set_enable(clientdata, enable);
			break;
		case ECOMPASS_IOC_GET_LAYOUT:
			if(copy_to_user(argp, &(clientdata->dir), sizeof(clientdata->dir)))
			{
				printk("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case MSENSOR_IOCTL_SENSOR_ENABLE:             
			if (copy_from_user(&enable, argp, sizeof(enable))) {
				return -EFAULT;
			}
			if (enable < 0 || enable > 1) {
				return -EINVAL;
			}
			printk("%s enable = %d xiexie \n",__FUNCTION__,enable);	
			break;

		case MMC3630X_IOC_READ_REG:
			if (copy_from_user(&reg_addr, argp, sizeof(reg_addr)))
				return -EFAULT;
			data[0] = reg_addr;
			if (mmc3630x_i2c_rxdata(clientdata->i2c, data, 1) < 0) {
				return -EFAULT;
			}
			reg_value = data[0];
			if (copy_to_user(argp, &reg_value, sizeof(reg_value))) {
				return -EFAULT;
			}		
			break;
		case MMC3630X_IOC_WRITE_REG:
			if (copy_from_user(&data, argp, sizeof(data)))
				return -EFAULT;
			if (mmc3630x_i2c_txdata(clientdata->i2c, data, 2)< 0) {
				return -EFAULT;
			}

			break; 
		case MMC3630X_IOC_READ_REGS:
			if (copy_from_user(&data, argp, sizeof(data)))
				return -EFAULT;
			if (mmc3630x_i2c_rxdata(clientdata->i2c, data, 6 ) < 0) {
				return -EFAULT;
			}
			if (copy_to_user(argp, data, sizeof(data))) {
				return -EFAULT;
			}		
			break; 
		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			break;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static struct file_operations mmc3630x_fops = {
	.owner = THIS_MODULE,
	.open = mmc3630x_open,
	.release = mmc3630x_release,
	.unlocked_ioctl = mmc3630x_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mmc3630x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "mmc3630x",
    .fops = &mmc3630x_fops,
};
/*----------------------------------------------------------------------------*/

static int mmc3630x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	struct mmc3630x_data *memsic;
	int err = 0;

	printk("memsic probing mmc3630x xiexie\n"); //xiexie

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("mmc3630x i2c functionality check failed.\n");
		res = -ENODEV;
		goto out;
	}

	memsic = devm_kzalloc(&client->dev, sizeof(struct mmc3630x_data),
			GFP_KERNEL);
	if (!memsic) {
		dev_err(&client->dev, "memory allocation failed.\n");
		res = -ENOMEM;
		goto out;
	}
	
	if (client->dev.of_node) {
		res = mmc3630x_parse_dt(client, memsic);
		if (res) {
			dev_err(&client->dev,
				"Unable to parse platform data.(%d)", res);
			goto free_memsic;
		}
	} else {
		memsic->dir = 0;
	}

	memsic->poll_interval = MMC3630X_DEFAULT_INTERVAL_MS;//xiexie
	atomic_set(&memsic->enabled, 0);
	
	this_client = client;
	memsic->i2c = client;
	dev_set_drvdata(&client->dev, memsic);
	i2c_set_clientdata(memsic->i2c, memsic);
	this_client = client;

	mutex_init(&memsic->lock);
	
	res = mmc3630x_check_device(memsic);
	if (res) {
		dev_err(&client->dev, "Check device failed\n");
		goto free_memsic;
	}

	memsic->idev = mmc3630x_init_input(memsic);
	if (!memsic->idev) {
		dev_err(&client->dev, "init input device failed\n");
		res = -ENODEV;
		goto  free_memsic;
	}

	err = misc_register(&mmc3630x_device);
	if(err)
	{
		printk(KERN_ERR "mmc3630x_device register failed\n");
		goto exit_misc_device_register_failed;	
	}
	
	/* create sysfs group */
	res = sysfs_create_group(&client->dev.kobj, &mmc3630x_attr_group);
	if (res){
		res = -EROFS;
		dev_err(&client->dev,"Unable to creat sysfs group\n");
	}

	printk("memsic probing mmc3630x end xiexie\n");
	dev_info(&client->dev, "mmc3630x successfully probed\n");

	return 0;

exit_misc_device_register_failed:
	input_unregister_device(memsic->idev);
free_memsic:
	kfree(memsic);
out:
	return res;
}

static int mmc3630x_remove(struct i2c_client *client)
{
	struct mmc3630x_data *memsic = dev_get_drvdata(&client->dev);

	if (memsic->idev)
		input_unregister_device(memsic->idev);

	hrtimer_cancel(&memsic->work_timer);
	kthread_stop(memsic->thread);
	sysfs_remove_group(&client->dev.kobj, &mmc3630x_attr_group);	
	misc_deregister(&mmc3630x_device);

	return 0;
}

static int mmc3630x_suspend(struct device *dev)
{
	int res = 0;
	struct mmc3630x_data *memsic = dev_get_drvdata(dev);
	dev_dbg(dev, "suspended\n");

	if (atomic_read(&memsic->enabled))
	{
		return mmc3630x_set_enable(memsic, false);
	}

	return res;
}

static int mmc3630x_resume(struct device *dev)
{
	int res = 0;
	struct mmc3630x_data *memsic = dev_get_drvdata(dev);

	dev_dbg(dev, "resumed\n");

	if (!atomic_read(&memsic->enabled))
	{
		return mmc3630x_set_enable(memsic, true);
	}

	return res;
}

static const struct i2c_device_id mmc3630x_id[] = {
	{ MMC3630x_I2C_NAME, 0 },
	{ }
};

static struct of_device_id mmc3630x_match_table[] = {
	{ .compatible = "memsic,mmc3630", },
	{ },
};

static const struct dev_pm_ops mmc3630x_pm_ops = {
	.suspend = mmc3630x_suspend,
	.resume = mmc3630x_resume,
};

static struct i2c_driver mmc3630x_driver = {
	.probe 		= mmc3630x_probe,
	.remove 	= mmc3630x_remove,
	.id_table	= mmc3630x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= MMC3630x_I2C_NAME,
		.of_match_table = mmc3630x_match_table,
		.pm = &mmc3630x_pm_ops,
	},
};

static int __init mmc3630x_init(void)
{
	return i2c_add_driver(&mmc3630x_driver);
}

static void __exit mmc3630x_exit(void)
{
	i2c_del_driver(&mmc3630x_driver);
}


MODULE_DESCRIPTION("MEMSIC MMC3630X Magnetic Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

late_initcall(mmc3630x_init);
module_exit(mmc3630x_exit);

