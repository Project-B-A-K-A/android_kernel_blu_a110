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
#include <linux/i2c/sensors_io.h>

#include "mmc3524x.h"

#define MMC3524X_MIN_DELAY 20
#define MMC3524X_MAX_DELAY 100
#define MMC3524X_PRODUCT_ID 0x08

#define MMC3524X_PRODUCT_ID2 0x09



#define MMC3524X_DELAY_TM_MS    10
#define MMC3524X_DELAY_SET  75
#define MMC3524X_DELAY_RESET 75
#define MMC3524X_RETRY_COUNT    10
#define MMC3524X_DEFAULT_INTERVAL_MS    100
#define MMC3524X_TIMEOUT_SET_MS 15000

struct mmc3524x_vec {
	int x;
	int y;
	int z;
};

struct mmc3524x_data {
        struct mutex ecompass_lock;
        struct mutex ops_lock;
        struct i2c_client *i2c;
        struct input_dev *idev;
        struct mmc3524x_platform_data *pdata;
        struct delayed_work dwork;
        struct workqueue_struct *data_wq;
        struct mmc3524x_vec last;
        int auto_report; 
        int dir;       	
        int poll_interval;
        int enable;
        int power_enabled;
	int last_x;
	int last_y;
	int last_z;
	int rep_cnt;
	struct hrtimer		mag_timer;
	bool			use_hrtimer;
	struct task_struct	*mag_task;
};

/*------------sensors data----------------------------------------------------*/
typedef struct {

	/* sensor values */
	int	values[3];
	uint32_t value_divide;
	/* sensor accuracy */
	int8_t status;
} hwm_sensor_data;


static atomic_t m_flag =ATOMIC_INIT(0);;
static atomic_t o_flag = ATOMIC_INIT(0);;
static atomic_t m_delay = ATOMIC_INIT(20);

struct i2c_client *this_client = NULL;


enum {
     OBVERSE_X_AXIS_FORWARD = 0,
     OBVERSE_X_AXIS_RIGHTWARD,
     OBVERSE_X_AXIS_BACKWARD,
     OBVERSE_X_AXIS_LEFTWARD,
     REVERSE_X_AXIS_FORWARD,
     REVERSE_X_AXIS_RIGHTWARD,
     REVERSE_X_AXIS_BACKWARD,
     REVERSE_X_AXIS_LEFTWARD,
     MMC3524X_DIR_COUNT,
};

static char *mmc3524x_dir[MMC3524X_DIR_COUNT] = {
	[OBVERSE_X_AXIS_FORWARD] = "obverse-x-axis-forward",
	[OBVERSE_X_AXIS_RIGHTWARD] = "obverse-x-axis-rightward",
	[OBVERSE_X_AXIS_BACKWARD] = "obverse-x-axis-backward",
	[OBVERSE_X_AXIS_LEFTWARD] = "obverse-x-axis-leftward",
	[REVERSE_X_AXIS_FORWARD] = "reverse-x-axis-forward",
	[REVERSE_X_AXIS_RIGHTWARD] = "reverse-x-axis-rightward",
	[REVERSE_X_AXIS_BACKWARD] = "reverse-x-axis-backward",
	[REVERSE_X_AXIS_LEFTWARD] = "reverse-x-axis-leftward",
};

static s8 mmc3524x_rotation_matrix[MMC3524X_DIR_COUNT][9] = {
	[OBVERSE_X_AXIS_FORWARD] = {0, -1, 0, 1, 0, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_RIGHTWARD] = {1, 0, 0, 0, 1, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_BACKWARD] = {0, 1, 0, -1, 0, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_LEFTWARD] = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
	[REVERSE_X_AXIS_FORWARD] = {0, 1, 0, 1, 0, 0, 0, 0, -1},
	[REVERSE_X_AXIS_RIGHTWARD] = {1, 0, 0, 0, -1, 0, 0, 0, -1},
	[REVERSE_X_AXIS_BACKWARD] = {0, -1, 0, -1, 0, 0, 0, 0, -1},
	[REVERSE_X_AXIS_LEFTWARD] = {-1, 0, 0, 0, 1, 0, 0, 0, -1},
};

static struct mutex sensor_data_mutex;
static DEFINE_MUTEX(ecompass_ioctl_lock);
static struct mmc3524x_data *mmc3524x_data_struct;
static int sensor_data[CALIBRATION_DATA_SIZE];



static int mmc3524x_i2c_rxdata(struct i2c_client *i2c, unsigned char *rxData, int length)
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

static int mmc3524x_i2c_txdata(struct i2c_client *i2c, unsigned char *txData, int length)
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



static int mmc3524x_read_xyz(struct mmc3524x_data *memsic, int *vec)
{
	int count = 0;
	unsigned char data[6];
	unsigned char buffer[2];
	unsigned char rd_buffer[2];
	struct mmc3524x_vec tmp;
	struct mmc3524x_vec report;
	int rc = 0;
        unsigned char *direction;

	mutex_lock(&memsic->ecompass_lock);
	/* Read MD */
	rd_buffer[0] = MMC3524X_REG_DS;
	rc = mmc3524x_i2c_rxdata(memsic->i2c, rd_buffer, 1);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
				MMC3524X_REG_DS, __LINE__, rc);
		goto exit;

	}
	while ((!(rd_buffer[0] & 0x01)) && (count < MMC3524X_RETRY_COUNT)) {
		/* Read MD again*/
		rd_buffer[0] = MMC3524X_REG_DS;
		rc = mmc3524x_i2c_rxdata(memsic->i2c, rd_buffer, 1);
		if (rc) {
			dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
					MMC3524X_REG_DS, __LINE__, rc);
			goto exit;

		}

		/* Wait more time to get valid data */
		usleep_range(1000, 1500);
		count++;
	}

	if (count >= MMC3524X_RETRY_COUNT) {
		dev_err(&memsic->i2c->dev, "TM not work!!");
		rc = -EFAULT;
		goto exit;
	}

	/* read xyz raw data */
	data[0] = MMC3524X_REG_DATA;
	rc = mmc3524x_i2c_rxdata(memsic->i2c, data, 6);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
				MMC3524X_REG_DS, __LINE__, rc);
		goto exit;
	}
	
	
  //vec[0] = data[1] << 8 | data[0];
	//vec[1] = data[3] << 8 | data[2];
	//vec[2] = data[5] << 8 | data[4];
	//vec[2] = 65536 - vec[2] ;


	tmp.x = ((u8)data[1]) << 8 | (u8)data[0];                                                    //3524
	tmp.y = ((u8)data[3]) << 8 | (u8)data[2] ;
	tmp.z = ((u8)data[5]) << 8 | (u8)data[4];
	tmp.z = 65536 - tmp.z;

	dev_dbg(&memsic->i2c->dev, "raw data:%d %d %d %d %d %d",
			data[0], data[1], data[2], data[3], data[4], data[5]);
	dev_dbg(&memsic->i2c->dev, "raw x:%d y:%d z:%d\n", tmp.x, tmp.y, tmp.z);



        direction = &mmc3524x_rotation_matrix[memsic->dir][0];
        report.x = direction[0] * tmp.x + direction[1] * tmp.y + direction[2] * tmp.z;
        report.y = direction[3] * tmp.x + direction[4] * tmp.y + direction[5] * tmp.z;
        report.z = direction[6] * tmp.x + direction[7] * tmp.y + direction[8] * tmp.z;
	

	vec[0] = report.x;
	vec[1] = report.y;
	vec[2] = report.z;
	
	//vec[0] = tmp.x;
	//vec[1] = tmp.y;
	//vec[2] = tmp.z;

exit:
	/* send TM cmd before read */
	buffer[0] = MMC3524X_REG_CTRL;
	buffer[1] = MMC3524X_CTRL_TM; 
	if (mmc3524x_i2c_txdata(memsic->i2c, buffer, 2)) {
		dev_warn(&memsic->i2c->dev, "write reg %d failed at %d.(%d)\n",	MMC3524X_REG_CTRL, __LINE__, rc);
	}

	mutex_unlock(&memsic->ecompass_lock);
	return rc;
}
static int last_sensor_value[6];
static void mmc3524x_poll(struct work_struct *work)
{
	struct mmc3524x_data *memsic = container_of((struct delayed_work *)work,
			struct mmc3524x_data, dwork);

	printk("mmc3524x o_flag = %d m_flag = %d   memsic->poll_interval = %d\n", atomic_read(&o_flag), atomic_read(&m_flag),memsic->poll_interval);
	if (atomic_read(&o_flag)){
		//printk("mmc3524x report o sensor data\n");
                          if((last_sensor_value[0] == sensor_data[8])&&(last_sensor_value[1] == sensor_data[9])&&(last_sensor_value[2] == sensor_data[10]))
                          {
                                sensor_data[10] +=1; 
                          }
		input_report_abs(memsic->idev, ABS_HAT0X, sensor_data[8] * CONVERT_O);
		input_report_abs(memsic->idev, ABS_HAT0Y, sensor_data[9] * CONVERT_O);
		input_report_abs(memsic->idev, ABS_HAT1X, sensor_data[10] * CONVERT_O);
		input_report_abs(memsic->idev, ABS_HAT1Y, sensor_data[11]);
                           //input_sync(memsic->idev);
                           last_sensor_value[0] = sensor_data[8];
                           last_sensor_value[1] = sensor_data[9];
                           last_sensor_value[2] = sensor_data[10];
	}
	if (atomic_read(&m_flag)){
		//printk("mmc3524x report mmm sensor data\n");
                          if((last_sensor_value[3] == sensor_data[4])&&(last_sensor_value[4] == sensor_data[5])&&(last_sensor_value[5] == sensor_data[6]))
                          {
                                sensor_data[6] +=1; 
                          }
		input_report_abs(memsic->idev, ABS_RX, sensor_data[4] * CONVERT_M);
		input_report_abs(memsic->idev, ABS_RY, sensor_data[5] * CONVERT_M);
		input_report_abs(memsic->idev, ABS_RZ, sensor_data[6] * CONVERT_M);
		input_report_abs(memsic->idev, ABS_RUDDER, sensor_data[7]);
		//msensor_data->value_divide = CONVERT_M_DIV;
                           //input_sync(memsic->idev);
                           last_sensor_value[3] = sensor_data[4];
                           last_sensor_value[4] = sensor_data[5];
                           last_sensor_value[5] = sensor_data[6];
	}
	input_sync(memsic->idev);
	if (!memsic->use_hrtimer)
		schedule_delayed_work(&memsic->dwork, msecs_to_jiffies(memsic->poll_interval));
}

static enum hrtimer_restart mag_timer_handle(struct hrtimer *hrtimer)
{
	struct mmc3524x_data *memsic;
	ktime_t ktime;

	memsic = container_of(hrtimer, struct mmc3524x_data, mag_timer);
	queue_work(memsic->data_wq, &memsic->dwork.work);
	
	ktime = ktime_set(0, memsic->poll_interval * NSEC_PER_MSEC);
	hrtimer_forward_now(&memsic->mag_timer, ktime);

	return HRTIMER_RESTART;
}


static struct input_dev *mmc3524x_init_input(struct i2c_client *client)
{
	int status;
	struct input_dev *input = NULL;

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return NULL;

	input->name = "compass";
	input->phys = "mmc3524x/input0";
	input->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input->evbit);
	input_set_events_per_packet(input, 100);
	input_set_abs_params(input, ABS_RX, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_RY, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_RZ, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_RUDDER, 0, 3, 0, 0);
	
	/* Orientation  */
	input_set_abs_params(input, ABS_HAT0X,	0, 23040, 0, 0);
	input_set_abs_params(input, ABS_HAT0Y, -11520, 11520, 0, 0);
	input_set_abs_params(input, ABS_HAT1X, -5760, 5760, 0, 0);
	input_set_abs_params(input, ABS_HAT1Y,	0, 3, 0, 0);

	/* Report the dummy value */
	input_set_abs_params(input, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

	input_set_capability(input, EV_REL, REL_X);
	input_set_capability(input, EV_REL, REL_Y);
	input_set_capability(input, EV_REL, REL_Z);

	status = input_register_device(input);
	if (status) {
		dev_err(&client->dev,
			"error registering input device\n");
		return NULL;
	}

	return input;
}


static int ECS_GetRawData(struct mmc3524x_data *memsic, int *vec)
{
	int err = 0;
	err = mmc3524x_read_xyz(memsic, vec);
	
	if(err !=0 )
	{
		printk(KERN_ERR "MMC3524X_IOC_TM failed\n");
		return -1;
	}

	// sensitivity 2048 count = 1 Guass = 100uT
	vec[0] = (vec[0] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	vec[1] = (vec[1] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	vec[2] = (vec[2] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;

	return err;
}


// Daemon application save the data
static int ECS_SaveData(int buf[CALIBRATION_DATA_SIZE])
{

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);

#if 0
	{
		printk("memsicGet daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
	}
#endif

	return 0;
}

static int mmc3524x_check_device(struct mmc3524x_data *memsic)
{
	int rc;
	unsigned char rd_buffer[2];
	
	rd_buffer[0] = MMC3524X_REG_PRODUCTID_1;
	rc = mmc3524x_i2c_rxdata(memsic->i2c, rd_buffer, 1);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed.(%d)\n",
				MMC3524X_REG_DS, rc);
		return rc;
	}
        printk("memsic current device is 0x%x\n", rd_buffer[0]);

	
	if ((rd_buffer[0] & 0x0f )!= MMC3524X_PRODUCT_ID2)
		return -ENODEV;

	return 0;
}



#ifdef CONFIG_OF
static int mmc3524x_parse_dt(struct i2c_client *client,struct mmc3524x_data *memsic)
{
	struct device_node *np = client->dev.of_node;
	const char *tmp;
	int rc;
	int i;
        
       if (of_property_read_bool(np, "memsic,hrtimer"))
           memsic->use_hrtimer = 1;
       else
           memsic->use_hrtimer = 0;
     
   	rc = of_property_read_string(np, "memsic,dir", &tmp);

	/* does not have a value or the string is not null-terminated */
	if (rc && (rc != -EINVAL)) {
		dev_err(&client->dev, "Unable to read memsic,dir\n");
		memsic->dir = 0;
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(mmc3524x_dir); i++) {
		if (strcmp(mmc3524x_dir[i], tmp) == 0)
			break;
	}

	if (i >= ARRAY_SIZE(mmc3524x_dir)) {
		dev_err(&client->dev, "Invalid memsic,dir property");
		memsic->dir = 0;
		return -EINVAL;
	}

	memsic->dir = i;
	printk("memsic add in mmc3524,memsic->direction =%d.\n",memsic->dir);

	if (of_property_read_bool(np, "memsic,auto-report"))
		memsic->auto_report = 1;
	else
		memsic->auto_report = 0;

	return 0;
}
#endif



static int mmc3524x_set_poll_delay(struct mmc3524x_data *memsic, unsigned int delay_msec)
{
	ktime_t ktime;
	mutex_lock(&memsic->ops_lock);
	if (memsic->poll_interval != delay_msec) {
	    if (delay_msec < MMC3524X_MIN_DELAY) {
			delay_msec = MMC3524X_MIN_DELAY;
	    }
	    if (delay_msec > MMC3524X_MAX_DELAY) {
		    delay_msec = MMC3524X_MAX_DELAY;
	    }
	    memsic->poll_interval = delay_msec;
    }

	if (memsic->auto_report && !memsic->use_hrtimer)
	    mod_delayed_work(system_wq, &memsic->dwork, msecs_to_jiffies(delay_msec));
	else if (memsic->enable && memsic->use_hrtimer) {
	    printk("OSENSOR_ENABLE memsic set new delay and restart hrtime, timer_delay = %d\n", memsic->poll_interval);
	    ktime = ktime_set(0, memsic->poll_interval * NSEC_PER_MSEC);
	    hrtimer_start(&memsic->mag_timer, ktime, HRTIMER_MODE_REL);
    }
	mutex_unlock(&memsic->ops_lock);

	return 0;
}

static int mmc3524x_set_enable(struct mmc3524x_data *memsic, unsigned int enable)
{
	int rc = 0;
    int delay_ms;
	unsigned char buffer[2];
	ktime_t ktime;

	mutex_lock(&memsic->ops_lock);

	printk("geroge ------memsic->enable------> %d \n",memsic->enable);
	
	printk("geroge ------enable------> %d	\n",enable);

	if (enable && (!memsic->enable)) {
		printk("geroge ------------>1\n");
		buffer[0] = MMC3524X_REG_CTRL;
		buffer[1] = MMC3524X_CTRL_TM;
		rc = mmc3524x_i2c_txdata(memsic->i2c, buffer, 2);
		printk("geroge ------------>2\n");
		if (rc) {
			dev_err(&memsic->i2c->dev, "write reg %d failed.(%d)\n",
					MMC3524X_REG_CTRL, rc);
			goto exit;
	}

		if (memsic->auto_report && !memsic->use_hrtimer) {
			printk("geroge ------------>77777777777\n");
			delay_ms = memsic->poll_interval;
			schedule_delayed_work(&memsic->dwork,	msecs_to_jiffies(memsic->poll_interval));
		} else {
		printk("geroge ------------>66666666666\n");
		printk("geroge --------memsic->poll_interval  %d\n",memsic->poll_interval);
			ktime = ktime_set(0, memsic->poll_interval * NSEC_PER_MSEC);
			
			hrtimer_start(&memsic->mag_timer, ktime, HRTIMER_MODE_REL);
		} 
	} else if ((!enable) && memsic->enable) {


	printk("geroge ------------>888888888888\n");
		if (memsic->auto_report && !memsic->use_hrtimer)
			cancel_delayed_work_sync(&memsic->dwork);
		else
			hrtimer_cancel(&memsic->mag_timer);
		//printk("ctsmemsic set enable\n");
	} else {
	printk("geroge ------------>1212121212121\n");
		dev_warn(&memsic->i2c->dev,
				"ignore enable state change from %d to %ld\n",
				memsic->enable, enable);
		}
		printk("geroge ------------>3\n");
	memsic->enable = enable;
	printk("geroge ------------>4\n");

exit:
	printk("geroge ------------>5\n");
	mutex_unlock(&memsic->ops_lock);
	return rc;
}


static ssize_t mmc3524x_otp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	
	char data[10] = {0};
	unsigned char reg_addr;
	unsigned char reg_num;
	int rc = -1;
	
	data[0] = 0x1b;
	reg_num = 4;
	rc = mmc3524x_i2c_rxdata(mmc3524x_data_struct->i2c, data, reg_num);
	if (rc) {
			dev_err(&mmc3524x_data_struct->i2c->dev, "read reg %d failed at %d.(%d)\n",
					reg_addr, __LINE__, rc);
	}
	printk("otp data: %x %x %x %x\n", data[0], data[1], data[2], data[3]);	

	return sprintf(buf, "%x,%x,%x,%x\n", data[0],data[1],data[2],data[3]);
}


static ssize_t mmc3524x_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	int rc = -1;
	unsigned char buffer[2];

	buffer[0] = MMC3524X_REG_CTRL;
	buffer[1] = MMC3524X_CTRL_REFILL;
	rc = mmc3524x_i2c_txdata(mmc3524x_data_struct->i2c, buffer, 2);
	if (rc) {
		printk("write reg %d failed at %d.(%d)\n", MMC3524X_REG_CTRL, __LINE__, rc);
		return -EFAULT;
	}

	/* Time from refill cap to SET */
	msleep(MMC3524X_DELAY_SET);
	buffer[0] = MMC3524X_REG_CTRL;
	buffer[1] = MMC3524X_CTRL_SET;
	rc = mmc3524x_i2c_txdata(mmc3524x_data_struct->i2c, buffer, 2);
	if (rc) {
		printk("write reg %d failed at %d.(%d)\n", MMC3524X_REG_CTRL, __LINE__, rc);
		return -EFAULT;
	}
	dev_dbg(&mmc3524x_data_struct->i2c->dev, "mmc3524x reset is done\n");
	printk("mmc3524x set is done\n");

	return sprintf(buf, "%d\n", rc);
}



static ssize_t mmc3524x_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	//s8 *tmp;
	int vec[3];
	//struct mmc3524x_vec report;
	struct mmc3524x_data *memsic = dev_get_drvdata(dev);


	ret = mmc3524x_read_xyz(memsic, vec);
	if (ret) {
		dev_warn(&memsic->i2c->dev, "read xyz failed\n");
	}
	
	return sprintf(buf, "%d %d %d\n", vec[0],vec[1],vec[2]);
}


static ssize_t mmc3524x_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("memsic enable show : %d\n", atomic_read(&m_flag));
	return sprintf(buf, "%d\n", atomic_read(&m_flag));
}

static ssize_t mmc3524x_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("memsic enable show : %d\n", atomic_read(&m_delay));
	return sprintf(buf, "%d\n", atomic_read(&m_delay));
}
static ssize_t mmc3524x_chipinfo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", "mmc3524");
}
// add to create sysfs file node 
static DEVICE_ATTR(otp, 0444, mmc3524x_otp_show, NULL);
static DEVICE_ATTR(set, 0444, mmc3524x_set_show, NULL);
static DEVICE_ATTR(value, 0444, mmc3524x_value_show, NULL);
static DEVICE_ATTR(enable, 0664, mmc3524x_enable_show, NULL);
static DEVICE_ATTR(delay, 0664, mmc3524x_delay_show, NULL);
static DEVICE_ATTR(chipinfo, 0664, mmc3524x_chipinfo_show, NULL);

static struct attribute *mmc3524x_attributes[] = {
		&dev_attr_otp.attr,
		&dev_attr_set.attr,
		&dev_attr_value.attr,
		&dev_attr_enable.attr,
		&dev_attr_delay.attr,
		&dev_attr_chipinfo.attr,
		NULL,
};

static const struct attribute_group mmc3524x_attr_group = {
		.attrs = mmc3524x_attributes,
};


/*----------------------------------------------------------------------------*/
static int mmc3524x_open(struct inode *inode, struct file *file)
{
	int ret = -1;

	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mmc3524x_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char buff[MMC3524X_BUFSIZE];				/* for chip information */

	int value[12];			/* for SET_YPR */
	int delay;				/* for GET_DELAY */
	int set_delay;
	int status; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	unsigned char data[16] = {0};
	int vec[3] = {0};
        static int acc[3] = {0};
	struct i2c_client *client = this_client;
	struct mmc3524x_data *clientdata = i2c_get_clientdata(client);
	
	hwm_sensor_data* osensor_data;
	int enable;
	unsigned char reg_addr;
	unsigned char reg_value;
	//read_reg_str reg_str;

	switch (cmd)
	{
		case MMC3524X_IOC_SET_DELAY:
			if (copy_from_user(&set_delay, argp, sizeof(set_delay))) {
				 printk("memsic add set delay failed\n");
                                 return -EFAULT;
                         }
       			
			mmc3524x_set_poll_delay(clientdata, set_delay);
			break;
		case MMC31XX_IOC_TM:
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_TM;
			if (mmc3524x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_TM failed\n");
				return -EFAULT;
			}
			/* wait TM done for coming data read */
			//msleep(MMC3524X_DELAY_TM);
			break;

		case MMC31XX_IOC_SET:
		case MMC31XX_IOC_RM:
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_REFILL;
			if(mmc3524x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3524X_DELAY_SET);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_SET;
			if(mmc3524x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = 0;
			if(mmc3524x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_RESET:
		case MMC31XX_IOC_RRM:
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_REFILL;
			if(mmc3524x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3524X_DELAY_RESET);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_RESET;
			if(mmc3524x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = 0;
			if(mmc3524x_i2c_txdata(clientdata->i2c, data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_READ:
			data[0] = MMC3524X_REG_DATA;
			if(mmc3524x_i2c_rxdata(clientdata->i2c, data, 6) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_READ failed\n");
				return -EFAULT;
			}

			vec[0] = data[1] << 8 | data[0];
			vec[1] = data[3] << 8 | data[2];
			vec[2] = data[5] << 8 | data[4];

			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3524x_IOC_READ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MMC31XX_IOC_READXYZ:
//			ECS_ReadXYZData(vec, 3);
			mmc3524x_read_xyz(clientdata, vec);
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3524x_IOC_READXYZ: copy to user failed\n");
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

		case ECOMPASS_IOC_SET_YPR:
			if(argp == NULL)
			{
				printk("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(value, argp, sizeof(value)))
			{
				printk("memsiccopy_from_user failed.");
				return -EFAULT;
			}
			ECS_SaveData(value);
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

		case ECOMPASS_IOC_GET_LAYOUT:
			
			status = mmc3524x_data_struct->dir;
			//status = 5;  //memsic->dir
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				return -EFAULT;
			}
			break;
		case MSENSOR_IOCTL_MSENSOR_ENABLE:

			printk("geroge  MSENSOR_ENABLE  %s enable = %d xiexie \n",__FUNCTION__,enable);
			printk("geroge  %s enable = %d \n",__FUNCTION__,sizeof(enable));

			if(argp == NULL)
			{
				printk(KERN_ERR "geroge 11 IO parameter pointer is NULL!\r\n");
				break;
			}
			enable = 1;
			/*if (copy_from_user(&enable, argp, sizeof(enable))) {
				return -EFAULT;
			}*/
			if(copy_from_user(value, argp, sizeof(value)))
			{
				printk("memsiccopy_from_user failed.");
				return -EFAULT;
			}
			printk("geroge MSENSOR_ENABLE  %s value[0] = %d\n",__FUNCTION__,value[0]);			
                        #if 1
			if (1 == value[0]){
				atomic_set(&m_flag, 1);
			              mmc3524x_set_enable(clientdata, value[0]);
			} else {
				atomic_set(&m_flag, 0);
                                                      if(atomic_read(&o_flag) == 0){	
                                                      mmc3524x_set_enable(clientdata, value[0]);}
			}
                        #else			
			if (1 == value[0]){
				atomic_set(&m_flag, 1);
			} else {
				atomic_set(&m_flag, 0);
			}

			printk("MSENSOR_ENABLE  %s enable = %d xiexie \n",__FUNCTION__,value[0]);	
			mmc3524x_set_enable(clientdata, value[0]);
                        #endif
			break;

		case MSENSOR_IOCTL_OSENSOR_ENABLE:

			printk("geroge OSENSOR_ENABLE  %s enable = %d xiexie \n",__FUNCTION__,enable);	

			if(argp == NULL)
			{
				printk(KERN_ERR "geroge 22 IO parameter pointer is NULL!\r\n");
				break;
			}
			enable = 1;
			/*if (copy_from_user(&enable, argp, sizeof(enable))) {
				return -EFAULT;
			}*/
			if(copy_from_user(value, argp, sizeof(value)))
			{
				printk("memsiccopy_from_user failed.");
				return -EFAULT;
			}
			printk("geroge OSENSOR_ENABLE  %s value[0] = %d\n",__FUNCTION__,value[0]);
                        #if 1
                                        if (1 == value[0]){
				atomic_set(&o_flag, 1);
                                                      mmc3524x_set_enable(clientdata, value[0]);
			} else {
				atomic_set(&o_flag, 0);
                                                      if(atomic_read(&m_flag) == 0){
                                                      mmc3524x_set_enable(clientdata, value[0]);}
			}
                        #else
			if (1 == value[0]){
				atomic_set(&o_flag, 1);
			} else {
				atomic_set(&o_flag, 0);
			}
			printk("OSENSOR_ENABLE  %s value[0] = %d xiexie \n",__FUNCTION__,value[0]);	
			mmc3524x_set_enable(clientdata, value[0]);
                        #endif
			break; 

		case MSENSOR_IOCTL_SENSOR_ENABLE:
			
			if (copy_from_user(&enable, argp, sizeof(enable))) {
                              return -EFAULT;
                        }
			if (enable < 0 || enable > 1) {
				return -EINVAL;

			}

			if(1 == enable)
			{
				atomic_set(&o_flag, 1);
				atomic_set(&m_flag, 1);
			}
			else
			{
				atomic_set(&o_flag, 0);
                                atomic_set(&m_flag, 0);
			}
			
			mmc3524x_set_enable(clientdata, enable);
			break;

		case MMC3524X_IOC_READ_REG:
			if (copy_from_user(&reg_addr, argp, sizeof(reg_addr)))
				return -EFAULT;
			data[0] = reg_addr;
			if (mmc3524x_i2c_rxdata(clientdata->i2c, data, 1) < 0) {
				return -EFAULT;
			}
			reg_value = data[0];
			if (copy_to_user(argp, &reg_value, sizeof(reg_value))) {
				return -EFAULT;
			}		
			break;

		case MMC3524X_IOC_WRITE_REG:
			if (copy_from_user(&data, argp, sizeof(data)))
                                return -EFAULT;
			if (mmc3524x_i2c_txdata(clientdata->i2c, data, 2)< 0) {
			        return -EFAULT;
			}
			
		    break; 

		case MMC3524X_IOC_READ_REGS:
			if (copy_from_user(&data, argp, sizeof(data)))
				return -EFAULT;
			if (mmc3524x_i2c_rxdata(clientdata->i2c, data, 6 ) < 0) {
				return -EFAULT;
			}
			if (copy_to_user(argp, data, sizeof(data))) {
				return -EFAULT;
			}		
			break; 

		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			osensor_data = (hwm_sensor_data *)buff;
		        mutex_lock(&sensor_data_mutex);

			osensor_data->values[0] = sensor_data[8] * CONVERT_O;
			osensor_data->values[1] = sensor_data[9] * CONVERT_O;
			osensor_data->values[2] = sensor_data[10] * CONVERT_O;
			osensor_data->status = sensor_data[11];
			osensor_data->value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);

			sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
				osensor_data->values[2],osensor_data->status,osensor_data->value_divide);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}

			break;

                case ECOMPASS_IOC_SET_ACC_DATA: 
                        if (copy_from_user(acc, argp, sizeof(acc)))
                        {
                            printk("memsic %s ECOMPASS_IOC_SET_ACC_DATA failed\n", __func__);
                            return -EFAULT;
                        }
                        //printk("mmc3524x get data from hal x: %d y: %d z: %d\n", acc[0], acc[1], acc[2]);
                        break;
                case ECOMPASS_IOC_GET_ACC_DATA: 
                        if (copy_to_user(argp, acc, sizeof(acc)))
                        {
                            printk("memsic %s ECOMPASS_IOC_GET_ACC_DATA failed\n", __func__);
                            return -EFAULT;
                        }
                        printk("mmc3524x  data from hal x: %d y: %d z: %d\n", acc[0], acc[1], acc[2]);
                        break;
		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			break;
		}

	return 0;
}


/*----------------------------------------------------------------------------*/
static struct file_operations mmc3524x_fops = {
	.owner = THIS_MODULE,
	.open = mmc3524x_open,
	.release = mmc3524x_release,
	.unlocked_ioctl = mmc3524x_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mmc3524x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "mmc3524x",
    .fops = &mmc3524x_fops,
};
/*----------------------------------------------------------------------------*/


static int mmc3524x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	struct mmc3524x_data *memsic;
	int err = 0;

	printk("memsic probing mmc3524x\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("mmc3524x i2c functionality check failed.\n");
		res = -ENODEV;
		goto out;
	}

	memsic = devm_kzalloc(&client->dev, sizeof(struct mmc3524x_data),
			GFP_KERNEL);
	if (!memsic) {
		dev_err(&client->dev, "memory allocation failed.\n");
		res = -ENOMEM;
		goto out;
	}

	mmc3524x_data_struct = memsic;
	
	if (client->dev.of_node) {
		res = mmc3524x_parse_dt(client, memsic);
		if (res) {
			dev_err(&client->dev,
				"Unable to parse platform data.(%d)", res);
			goto out;
		}
	} else {
		memsic->dir = 0;
		memsic->auto_report = 1;
	}

	memsic->i2c = client;
	dev_set_drvdata(&client->dev, memsic);
        i2c_set_clientdata(memsic->i2c, memsic);
        this_client = client;
     
	mutex_init(&memsic->ecompass_lock);
	mutex_init(&memsic->ops_lock);
        mutex_init(&sensor_data_mutex);

	res = mmc3524x_check_device(memsic);
	if (res) {
		dev_err(&client->dev, "Check device failed\n");
		goto out_check_device;
	}

	memsic->idev = mmc3524x_init_input(client);
	if (!memsic->idev) {
		dev_err(&client->dev, "init input device failed\n");
		res = -ENODEV;
		goto  out_power_set;
	}


	if (memsic->auto_report && !memsic->use_hrtimer) {
		dev_info(&client->dev, "auto report is enabled\n");
		INIT_DELAYED_WORK(&memsic->dwork, mmc3524x_poll);
		memsic->data_wq =	create_freezable_workqueue("mmc3524x_data_work");
		if (!memsic->data_wq) {
			dev_err(&client->dev, "Cannot create workqueue.\n");
			goto out_register_classdev;
		}

	} else {
		hrtimer_init(&memsic->mag_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		memsic->mag_timer.function = mag_timer_handle;
		memsic->data_wq = alloc_workqueue("mmc3524x_data_work",WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
		INIT_WORK(&memsic->dwork.work, mmc3524x_poll);  
	}


	memsic->poll_interval = MMC3524X_DEFAULT_INTERVAL_MS;
	
	if(err = misc_register(&mmc3524x_device))
	{
		printk(KERN_ERR "mmc3524x_device register failed\n");
		goto exit_misc_device_register_failed;	
	}
	
	/* create sysfs group */
	res = sysfs_create_group(&client->dev.kobj, &mmc3524x_attr_group);
	if (res){
		res = -EROFS;
		dev_err(&client->dev,"Unable to creat sysfs group\n");
	}

	printk("memsic probing mmc3524x end\n");
	dev_info(&client->dev, "mmc3524x successfully probed\n");

	return 0;

out_power_set:
out_register_classdev:
exit_misc_device_register_failed:
	destroy_workqueue(memsic->data_wq);
	input_unregister_device(memsic->idev);
out_check_device:
out:
	return res;
}

static int mmc3524x_remove(struct i2c_client *client)
{
	struct mmc3524x_data *memsic = dev_get_drvdata(&client->dev);
	
	if (memsic->use_hrtimer) {
	  hrtimer_cancel(&memsic->mag_timer);
		cancel_work_sync(&memsic->dwork.work);
	} else {
		cancel_delayed_work_sync(&memsic->dwork);
	}
  
	if (memsic->idev)
		input_unregister_device(memsic->idev);

	sysfs_remove_group(&client->dev.kobj, &mmc3524x_attr_group);	
	misc_deregister(&mmc3524x_device);

	return 0;
}


static int mmc3524x_suspend(struct device *dev)
{
	int res = 0;
	struct mmc3524x_data *memsic = dev_get_drvdata(dev);
	dev_dbg(dev, "suspended\n");

	if (memsic->enable) {
		printk("OSENSOR_ENABLE ------------>2222\n");
		if (memsic->auto_report)
			cancel_delayed_work_sync(&memsic->dwork);

		if (!memsic->use_hrtimer)
			cancel_delayed_work_sync(&memsic->dwork);
		else if (memsic->use_hrtimer)
			hrtimer_cancel(&memsic->mag_timer);
	}
	return res;
}

static int mmc3524x_resume(struct device *dev)
{
	int res = 0;
	struct mmc3524x_data *memsic = dev_get_drvdata(dev);
	ktime_t ktime;
	
	dev_dbg(dev, "resumed\n");

	if (memsic->enable) {
		printk("OSENSOR_ENABLE ------------>111\n");
            if (!memsic->use_hrtimer) {
			queue_delayed_work(memsic->data_wq, &memsic->dwork, msecs_to_jiffies(memsic->poll_interval));
	    } else if (memsic->use_hrtimer) {
			ktime = ktime_set(0, memsic->poll_interval * NSEC_PER_MSEC);
			hrtimer_start(&memsic->mag_timer, ktime, HRTIMER_MODE_REL);
	    }
	}

	return res;
}


static const struct i2c_device_id mmc3524x_id[] = {
	{ MMC3524x_I2C_NAME, 0 },
	{ }
};

static struct of_device_id mmc3524x_match_table[] = {
	{ .compatible = "memsic,mmc3524", },
	{ },
};

static const struct dev_pm_ops mmc3524x_pm_ops = {
	.suspend = mmc3524x_suspend,
	.resume = mmc3524x_resume,
};

static struct i2c_driver mmc3524x_driver = {
	.probe 		= mmc3524x_probe,
	.remove 	= mmc3524x_remove,
	.id_table	= mmc3524x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= MMC3524x_I2C_NAME,
		.of_match_table = mmc3524x_match_table,
		.pm = &mmc3524x_pm_ops,
	},
};

static int __init mmc3524x_init(void)
{
	return i2c_add_driver(&mmc3524x_driver);
}

static void __exit mmc3524x_exit(void)
{
	i2c_del_driver(&mmc3524x_driver);
}

//late_initcall(mmc3524x_init);
//module_exit(mmc3524x_exit);

MODULE_DESCRIPTION("MEMSIC MMC3524X Magnetic Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

late_initcall(mmc3524x_init);
module_exit(mmc3524x_exit);

