#
# Makefile for the input core drivers.
#
PROJECT_NAME := $(strip $(subst ro.project =,,$(shell cat $(PWD)/../build.ini |grep ^ro.project)))
ifeq ($(strip $(PROJECT_NAME)),)
  $(warning --PROJECT_NAME -is null--please--do--lunch--$(TARGET_PRODUCT)-$(TARGET_BUILD_VARIANT)---)
endif
NEW_MAKE_FILE := $(PWD)/../vendor/tinno/$(TARGET_PRODUCT)/$(PROJECT_NAME)/cust_config/Makefile
ifneq ($(wildcard $(NEW_MAKE_FILE)),)
  include $(NEW_MAKE_FILE)
endif

ccflags-$(CONFIG_NO_ALSPS_TP_PS)	:= -DCONFIG_NO_ALSPS_TP_PS
# Each configuration option enables a list of files.
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH)  += focaltech_ts.o
#focaltech_ts-objs += focaltech.o focaltech_ctl.o focaltech_ex_fun.o
focaltech_ts-objs := focaltech.o
focaltech_ts-objs += focaltech_ctl.o
focaltech_ts-objs += focaltech_ex_fun.o
#focaltech_ts-y    += focaltech.o focaltech_ctl.o focaltech_ex_fun.o
