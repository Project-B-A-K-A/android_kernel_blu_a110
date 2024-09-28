#ifndef __LEDS_SPRD_PWM_H
#define __LEDS_SPRD_PWM_H

struct sprd_pwm_led_platform_data {
        int brightness_max;
        int brightness_min;
		int led_hw;
        int pwm_index[3];
        int gpio_ctrl_pin;
        int gpio_active_level;
		unsigned long sprd_pwm_base_addr;
};

#endif

