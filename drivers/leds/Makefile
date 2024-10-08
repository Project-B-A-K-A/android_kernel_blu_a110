
PROJECT_NAME := $(strip $(subst ro.project =,,$(shell cat $(PWD)/../build.ini |grep ^ro.project)))
ifeq ($(strip $(PROJECT_NAME)),)
  $(warning --PROJECT_NAME -is null--please--do--lunch--$(TARGET_PRODUCT)-$(TARGET_BUILD_VARIANT)---)
endif
NEW_MAKE_FILE := $(PWD)/../vendor/tinno/$(TARGET_PRODUCT)/$(PROJECT_NAME)/cust_config/Makefile
ifneq ($(wildcard $(NEW_MAKE_FILE)),)
  include $(NEW_MAKE_FILE)
endif

# LED Core
obj-$(CONFIG_NEW_LEDS)			+= led-core.o
obj-$(CONFIG_LEDS_CLASS)		+= led-class.o
obj-$(CONFIG_LEDS_TRIGGERS)		+= led-triggers.o

# LED Platform Drivers
obj-$(CONFIG_LEDS_88PM860X)		+= leds-88pm860x.o
obj-$(CONFIG_LEDS_ATMEL_PWM)		+= leds-atmel-pwm.o
obj-$(CONFIG_LEDS_BD2802)		+= leds-bd2802.o
obj-$(CONFIG_LEDS_LOCOMO)		+= leds-locomo.o
obj-$(CONFIG_LEDS_LM3530)		+= leds-lm3530.o
obj-$(CONFIG_LEDS_LM3533)		+= leds-lm3533.o
obj-$(CONFIG_LEDS_LM3642)		+= leds-lm3642.o
obj-$(CONFIG_LEDS_LM3648)		+= leds-lm3648.o
obj-$(CONFIG_LEDS_MIKROTIK_RB532)	+= leds-rb532.o
obj-$(CONFIG_LEDS_S3C24XX)		+= leds-s3c24xx.o
obj-$(CONFIG_LEDS_NET48XX)		+= leds-net48xx.o
obj-$(CONFIG_LEDS_WRAP)			+= leds-wrap.o
obj-$(CONFIG_LEDS_COBALT_QUBE)		+= leds-cobalt-qube.o
obj-$(CONFIG_LEDS_COBALT_RAQ)		+= leds-cobalt-raq.o
obj-$(CONFIG_LEDS_SUNFIRE)		+= leds-sunfire.o
obj-$(CONFIG_LEDS_PCA9532)		+= leds-pca9532.o
obj-$(CONFIG_LEDS_GPIO_REGISTER)	+= leds-gpio-register.o
obj-$(CONFIG_LEDS_GPIO)			+= leds-gpio.o
obj-$(CONFIG_LEDS_LP3944)		+= leds-lp3944.o
obj-$(CONFIG_LEDS_LP55XX_COMMON)	+= leds-lp55xx-common.o
obj-$(CONFIG_LEDS_LP5521)		+= leds-lp5521.o
obj-$(CONFIG_LEDS_LP5523)		+= leds-lp5523.o
obj-$(CONFIG_LEDS_LP5562)		+= leds-lp5562.o
obj-$(CONFIG_LEDS_LP8788)		+= leds-lp8788.o
obj-$(CONFIG_LEDS_TCA6507)		+= leds-tca6507.o
obj-$(CONFIG_LEDS_CLEVO_MAIL)		+= leds-clevo-mail.o
obj-$(CONFIG_LEDS_HP6XX)		+= leds-hp6xx.o
obj-$(CONFIG_LEDS_OT200)		+= leds-ot200.o
obj-$(CONFIG_LEDS_FSG)			+= leds-fsg.o
obj-$(CONFIG_LEDS_PCA955X)		+= leds-pca955x.o
obj-$(CONFIG_LEDS_PCA9633)		+= leds-pca9633.o
obj-$(CONFIG_LEDS_DA903X)		+= leds-da903x.o
obj-$(CONFIG_LEDS_DA9052)		+= leds-da9052.o
obj-$(CONFIG_LEDS_WM831X_STATUS)	+= leds-wm831x-status.o
obj-$(CONFIG_LEDS_WM8350)		+= leds-wm8350.o
obj-$(CONFIG_LEDS_PWM)			+= leds-pwm.o
obj-$(CONFIG_LEDS_REGULATOR)		+= leds-regulator.o
obj-$(CONFIG_LEDS_INTEL_SS4200)		+= leds-ss4200.o
obj-$(CONFIG_LEDS_LT3593)		+= leds-lt3593.o
obj-$(CONFIG_LEDS_ADP5520)		+= leds-adp5520.o
obj-$(CONFIG_LEDS_DELL_NETBOOKS)	+= dell-led.o
obj-$(CONFIG_LEDS_MC13783)		+= leds-mc13783.o
obj-$(CONFIG_LEDS_NS2)			+= leds-ns2.o
obj-$(CONFIG_LEDS_NETXBIG)		+= leds-netxbig.o
obj-$(CONFIG_LEDS_ASIC3)		+= leds-asic3.o
obj-$(CONFIG_LEDS_RENESAS_TPU)		+= leds-renesas-tpu.o
obj-$(CONFIG_LEDS_MAX8997)		+= leds-max8997.o
obj-$(CONFIG_LEDS_LM355x)		+= leds-lm355x.o
obj-$(CONFIG_LEDS_BLINKM)		+= leds-blinkm.o
obj-$(CONFIG_LEDS_SC8810_KB)		+= leds-sc8810-kb.o
# richteck
obj-$(CONFIG_FLASHLIGHT)		+= flashlight.o
obj-$(CONFIG_RT_FLASH_LED)		+= rtfled.o
obj-$(CONFIG_FLED_RT5033)       += rt5033_fled.o
#SiliconMitus
obj-$(CONFIG_FLED_SM5701)       	+= leds-sm5701.o
obj-$(CONFIG_FLED_SPRD_SM5701)       	+= leds-sprd-sm5701.o
# SPRD keyboard-backlight Drivers
obj-$(CONFIG_LEDS_SPRD_KB) += leds-sprd-kb.o
obj-$(CONFIG_SPRD_KPLED_2723) += leds-sprd-kpled-2723.o
obj-$(CONFIG_SPRD_KPLED_2731) += leds-sprd-kpled-2731.o
# SPRD bltc_rgb led Drivers
obj-$(CONFIG_LEDS_SPRD_BLTC_RGB)	+= leds-sprd-bltc-rgb.o
obj-$(CONFIG_LEDS_SPRD_BLTC_RGB_2731)	+= leds-sprd-bltc-rgb-2731.o
# SPRD rgb led Drivers
obj-$(CONFIG_LEDS_SPRD_RGB)	+= leds-sprd-rgb.o
obj-$(CONFIG_LEDS_SPRD_PWM_RGB)	+= leds-sprd-pwm.o
# LED SPI Drivers
obj-$(CONFIG_LEDS_DAC124S085)		+= leds-dac124s085.o

# LED Triggers
obj-$(CONFIG_LEDS_TRIGGERS)		+= trigger/
