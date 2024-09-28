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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/delay.h>
//  HEAD
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <soc/sprd/hardware.h>
#include <soc/sprd/sci_glb_regs.h>
#include <soc/sprd/adi.h>
#include <soc/sprd/adc.h>
#include <soc/sprd/arch_misc.h>
//=======
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/leds_sprd_pwm.h>

//#define SPRD_PWM_BL_DBG
#ifdef SPRD_PWM_BL_DBG
#define ENTER printk(KERN_INFO "[SPRD_PWM_LED_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[SPRD_PWM_LED_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_PWM_LED_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[SPRD_PWM_LED_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_PWM_BL_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_PWM_LED_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[SPRD_PWM_LED_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_PWM_LED_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

#ifndef BIT
#define BIT(x) (1<<x)
#endif

/* register definitions */
#define        PWM_PRESCALE    (0x0000)
#define        PWM_CNT         (0x0004)
#define        PWM_TONE_DIV    (0x0008)
#define        PWM_PAT_LOW     (0x000C)
#define        PWM_PAT_HIG     (0x0010)

#define        PWM_ENABLE      (1 << 8)
#define        PWM_DUTY            (1 << 8)
#define        PWM_MOD             0
#define        PWM_DIV             0x190
#define        PWM_LOW             0xffff
#define        PWM_HIG             0xffff
#define        PWM_SCALE       0xff
#define        PWM2_SCALE           0x0
#define        PWM_REG_MSK     0xffff
#if defined(CONFIG_PROJS_V2510)
#define        PWM_MOD_MAX     0xdf
#else
#define        PWM_MOD_MAX     0xff
#endif
#define        PWM_DUTY_MAX     0x7f

enum sprd_led_type
{
	SPRD_LED_TYPE_R = 0,
	SPRD_LED_TYPE_R_BL,	
	SPRD_LED_TYPE_G,
	SPRD_LED_TYPE_G_BL,
	SPRD_LED_TYPE_B,
	SPRD_LED_TYPE_B_BL,
	SPRD_LED_TYPE_TOTAL
};
static char *sprd_leds_rgb_name[SPRD_LED_TYPE_TOTAL] = {
	"red",
	"red_bl",	
	"green",
	"green_bl",
	"blue",
	"blue_bl",
};

struct sprd_leds_pwm_rgb {
        struct platform_device *dev;
        struct mutex mutex;
        spinlock_t value_lock;
        enum led_brightness value;
        struct led_classdev cdev;
		struct clk *clk;
		int pwm_index;
		int brightness_max;
        int brightness_min;
        int gpio_ctrl_pin;
        int gpio_active_level;        
        u16 leds_flag;
        u16 enable;
        unsigned long sprd_pwm_base_addr;
        u16 on_off;
        int suspend;
};

#define to_sprd_pwmled(led_cdev) \
	container_of(led_cdev, struct sprd_leds_pwm_rgb, cdev)

static void pwm_clk_en(struct clk *clk,int enable)
{
	unsigned long spin_lock_flags;
	static int current_state = 0;
	if (1 == enable) {
		if (0 == current_state) {
			clk_prepare_enable(clk);
			current_state = 1;
		}
	} else {
		if (1 == current_state) {
			clk_disable_unprepare(clk);
			current_state = 0;
		}
	}
	return;
}

static inline uint32_t pwm_read(uint32_t addr, uint32_t reg)
{
        return __raw_readl(addr + reg);
}

static void pwm_write(uint32_t addr, uint32_t value, uint32_t reg)
{
        __raw_writel(value, addr + reg);
}
static void sprd_leds_pwm_rgb_enable(struct sprd_leds_pwm_rgb *brgb)
{
	u32 led_level;
	led_level = brgb->value& PWM_MOD_MAX;

	

	led_level = led_level * brgb->brightness_max / 255;

	

	if (led_level <= brgb->brightness_min)
		led_level = brgb->brightness_min;
	PRINT_INFO("sprd_leds_pwm_rgb_enable led_level = %1d name = %s\n",led_level,brgb->cdev.name);
	pwm_clk_en(brgb->clk,1);
	pwm_write(brgb->sprd_pwm_base_addr, (led_level << 8) | PWM_MOD_MAX, PWM_CNT);
	pwm_write(brgb->sprd_pwm_base_addr, PWM_REG_MSK, PWM_PAT_LOW);
	pwm_write(brgb->sprd_pwm_base_addr, PWM_REG_MSK, PWM_PAT_HIG);
	if(strcmp(brgb->cdev.name,sprd_leds_rgb_name[SPRD_LED_TYPE_R_BL]) == 0 || \
            strcmp(brgb->cdev.name,sprd_leds_rgb_name[SPRD_LED_TYPE_G_BL]) == 0 || \
            strcmp(brgb->cdev.name,sprd_leds_rgb_name[SPRD_LED_TYPE_B_BL]) == 0) {
	pwm_write(brgb->sprd_pwm_base_addr, PWM_SCALE|PWM_ENABLE, PWM_PRESCALE);
	}else{
	pwm_write(brgb->sprd_pwm_base_addr, PWM2_SCALE|PWM_ENABLE, PWM_PRESCALE);
	}

	brgb->enable = 1;
}

static void sprd_leds_pwm_rgb_disable(struct sprd_leds_pwm_rgb *brgb)
{
	pwm_write(brgb->sprd_pwm_base_addr, 0, PWM_PRESCALE);
    pwm_clk_en(brgb->clk,0);
    PRINT_INFO("sprd_leds_pwm_rgb_disable\n");
}
static void sprd_leds_rgb_work(struct sprd_leds_pwm_rgb *brgb)
{
        unsigned long flags;

        mutex_lock(&brgb->mutex);
        spin_lock_irqsave(&brgb->value_lock, flags);
        if (brgb->value == LED_OFF) {
                spin_unlock_irqrestore(&brgb->value_lock, flags);
                sprd_leds_pwm_rgb_disable(brgb);
                goto out;
        }
        spin_unlock_irqrestore(&brgb->value_lock, flags);
        sprd_leds_pwm_rgb_enable(brgb);
        PRINT_INFO("sprd_leds_rgb_work!\n");

out:
        mutex_unlock(&brgb->mutex);
}

static void sprd_leds_pwm_rgb_set(struct led_classdev *pwm_rgb_cdev,enum led_brightness value)
{
	struct sprd_leds_pwm_rgb *brgb;
	unsigned long flags;

	brgb = to_sprd_pwmled(pwm_rgb_cdev);
	spin_lock_irqsave(&brgb->value_lock, flags);
	brgb->leds_flag = pwm_rgb_cdev->flags;
	brgb->value = value;
	spin_unlock_irqrestore(&brgb->value_lock, flags);

	if(1 == brgb->suspend) {
		PRINT_WARN("Do NOT change brightness in suspend mode\n");
		return;
	}
	PRINT_INFO("sprd_leds_pwm_rgb_set!\n");

	sprd_leds_rgb_work(brgb);
}

void set_gpio_ctrll_pin_state_bl(struct led_classdev *pwm_rgb_cdev,int on)
{
	struct sprd_leds_pwm_rgb *brgb;
	brgb = to_sprd_pwmled(pwm_rgb_cdev);
    on = on ? brgb->gpio_active_level : !brgb->gpio_active_level;
    gpio_direction_output(brgb->gpio_ctrl_pin,on);
    PRINT_INFO("set_gpio_ctrll_pin_state_bl=%d\n", on);
}

#ifdef CONFIG_OF
static struct sprd_pwm_led_platform_data *sprd_pwm_led_parse_dt(struct platform_device *pdev)
{
	int ret = -1;
	struct resource res;
	struct device_node *np = pdev->dev.of_node;
	struct sprd_pwm_led_platform_data* pdata = NULL;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		PRINT_ERR("failed to allocate pdata!\n");
			return NULL;
	}

	ret = of_address_to_resource(np,0,&res);
	if(ret){
		PRINT_ERR("get dts addr-data failed!\n");
		return -ENODEV;
	}
	pdata->sprd_pwm_base_addr=(unsigned long)ioremap_nocache(res.start,resource_size(&res));
	
	ret = of_property_read_u32(np, "led_hw", &pdata->led_hw);
	if(ret) {
		PRINT_ERR("failed to get led_hw\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "pwm_index", pdata->pwm_index, 3);
	if(ret) {
		PRINT_ERR("failed to get pwm_index\n");
		goto fail;
	}

	ret = of_property_read_u32(np, "brightness_max", &pdata->brightness_max);
	if(ret) {
		pdata->brightness_max = 255;
		PRINT_WARN("failed to get brightness_max\n");
	}

	ret = of_property_read_u32(np, "brightness_min", &pdata->brightness_min);
	if(ret) {
		pdata->brightness_min = 0;
		PRINT_WARN("failed to get brightness_min\n");
	}

	ret = of_property_read_u32(np, "gpio_ctrl_pin", &pdata->gpio_ctrl_pin);
	if(ret) {
		pdata->gpio_ctrl_pin = -1;
		PRINT_WARN("failed to get gpio_ctrl_pin\n");
	}

	ret = of_property_read_u32(np, "gpio_active_level", &pdata->gpio_active_level);
	if(ret) {
		pdata->gpio_active_level = 0;
		PRINT_WARN("failed to get gpio_active_level\n");
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static int sprd_pwm_led_probe(struct platform_device *pdev)
{
	struct sprd_leds_pwm_rgb *brgb;
	struct device *dev = &pdev->dev;
    struct clk* ext_clk = NULL;
   	 char pwm_clk_name[32];
	struct sprd_pwm_led_platform_data* pdata = NULL;
    	int ret = -1;
	int i;
		
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	if(np) {
		pdata = sprd_pwm_led_parse_dt(pdev);
		if (pdata == NULL) {
			PRINT_ERR("get dts data failed!\n");
			return -ENODEV;
			}
	} else {
			PRINT_ERR("dev.of_node is NULL!\n");
			return -ENODEV;
	}	
#else
	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		PRINT_ERR("No platform data!\n");
		return -ENODEV;
	}
#endif			      
	for (i = 0; i < SPRD_LED_TYPE_TOTAL; i++) {
		if(!(pdata->led_hw&BIT(i/2)))
			continue;
		brgb = kzalloc(sizeof(struct sprd_leds_pwm_rgb), GFP_KERNEL);		
		if (brgb == NULL) {
			dev_err(dev, "No memory for bltc_device\n");
			ret = -ENOMEM;
			goto err;
		}
		
		if(pdata->gpio_ctrl_pin > 0) {
			brgb->gpio_ctrl_pin = pdata->gpio_ctrl_pin;
			brgb->gpio_active_level= pdata->gpio_active_level;
			ret = gpio_request(brgb->gpio_ctrl_pin, "sprd_pwm_led_gpio_ctrl_pin");
			if (ret) {
				PRINT_ERR("request gpio_ctrl_pin error!\n");
				return -EIO;
			}
    		gpio_direction_output(brgb->gpio_ctrl_pin,brgb->gpio_active_level);
		} else {
			brgb->gpio_ctrl_pin = -1;
		}
		PRINT_INFO("gpio_ctrl_pin=%d\n", brgb->gpio_ctrl_pin);

		brgb->pwm_index = pdata->pwm_index[i/2];
		PRINT_INFO("PWM%d is used \n", brgb->pwm_index);

		//fixme, the pwm's clk name must like this:clk_pwmx
		sprintf(pwm_clk_name, "%s%d", "clk_pwm", brgb->pwm_index);
		brgb->clk = clk_get(&pdev->dev, pwm_clk_name);
		if (IS_ERR(brgb->clk)) {
			PRINT_ERR("get pwm's clk failed!\n");
			return -ENODEV;
		}
		
		ext_clk = clk_get(NULL, "ext_32k");    //ext_26m
		if (IS_ERR(ext_clk)) {
			PRINT_ERR("get pwm's ext_clk failed!\n");
			return -ENODEV;
		}
		
		clk_set_parent(brgb->clk,ext_clk);

		brgb->sprd_pwm_base_addr = pdata->sprd_pwm_base_addr+brgb->pwm_index * 0x20;
					
		brgb->brightness_max= pdata->brightness_max;
		brgb->brightness_min= pdata->brightness_min;
						
		brgb->cdev.brightness_set = sprd_leds_pwm_rgb_set;
		brgb->cdev.name = sprd_leds_rgb_name[i];
		brgb->cdev.brightness_get = NULL;
		brgb->enable = 0;
		spin_lock_init(&brgb->value_lock);
		mutex_init(&brgb->mutex);
		brgb->value = LED_OFF;
		platform_set_drvdata(pdev, brgb);
		ret = led_classdev_register(dev, &brgb->cdev);
		if (ret < 0) {
			goto err;
		}
		brgb->suspend = 0;
		++brgb;
	}

	PRINT_INFO("probe success\n");
	return 0;
err:
	
	if (i) {
		for (i = i-1; i >=0; i--) {
		if (!brgb)
			continue;
		led_classdev_unregister(&brgb->cdev);
		kfree(brgb);
		brgb = NULL;
		--brgb;
		}
	}	
	return ret;
}

static int sprd_pwm_led_remove(struct platform_device *dev)
{
	struct sprd_leds_pwm_rgb *brgb = platform_get_drvdata(dev);
	
	led_classdev_unregister(&brgb->cdev);
	brgb->value = LED_OFF;
	brgb->enable = 1;
	sprd_leds_pwm_rgb_disable(brgb);
	kfree(brgb);
	
	return 0;

}

static void sprd_pwm_led_shutdown(struct platform_device *dev)
{
	struct sprd_leds_pwm_rgb *brgb = platform_get_drvdata(dev);
	
	mutex_lock(&brgb->mutex);
	sprd_leds_pwm_rgb_disable(brgb);
	mutex_unlock(&brgb->mutex);

}

static const struct of_device_id pwmled_of_match[] = {
        { .compatible = "sprd,sprd_pwm_led", },
        { }
};
static struct platform_driver sprd_pwm_led_driver = {
        .probe = sprd_pwm_led_probe,
        .remove = sprd_pwm_led_remove,
        .suspend = NULL,
        .resume = NULL,
        .shutdown = sprd_pwm_led_shutdown,
        .driver = {
                .name = "sprd_pwm_led",
                .owner = THIS_MODULE,
                .of_match_table = pwmled_of_match,
        },
};

static int __init sprd_pwm_led_init(void)
{
	return platform_driver_register(&sprd_pwm_led_driver);
}

static void __exit sprd_pwm_led_exit(void)
{
        platform_driver_unregister(&sprd_pwm_led_driver);
}

module_init(sprd_pwm_led_init);
module_exit(sprd_pwm_led_exit);

MODULE_DESCRIPTION("Spreadtrum pwm led Driver");
