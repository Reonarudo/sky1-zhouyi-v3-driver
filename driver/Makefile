# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2024 Arm Technology (China) Co. Ltd.

MODULE_NAME := aipu
SRC_DIR := .
# Resolve version.mk relative to this Makefile so it works both for a direct
# `make -C driver` build and for the kernel Kbuild phase (cwd = kernel tree).
KMD_SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
-include $(KMD_SELF_DIR)version.mk
EXTRA_CFLAGS += -DKMD_VERSION=\"$(COMPASS_DRV_BTENVAR_KMD_VERSION)\" \
                   -I$(PWD)/armchina-npu/ -I$(PWD)/armchina-npu/include -I$(PWD)/armchina-npu/zhouyi
ccflags-y := -DKMD_VERSION=\"$(COMPASS_DRV_BTENVAR_KMD_VERSION)\" \
                   -I$(PWD)/armchina-npu/ -I$(PWD)/armchina-npu/include -I$(PWD)/armchina-npu/zhouyi

COMM_OBJ := armchina-npu/aipu.o \
            armchina-npu/aipu_common.o \
            armchina-npu/aipu_io.o \
            armchina-npu/aipu_irq.o \
            armchina-npu/aipu_job_manager.o \
            armchina-npu/aipu_mm.o \
            armchina-npu/aipu_dma_buf.o \
            armchina-npu/aipu_priv.o \
            armchina-npu/aipu_tcb.o \
            armchina-npu/zhouyi/zhouyi.o

    AIPU_OBJ := armchina-npu/zhouyi/v3.o \
                armchina-npu/zhouyi/v3_priv.o
    EXTRA_CFLAGS += -DCONFIG_ARMCHINA_NPU_ARCH_V3
    ccflags-y += -DCONFIG_ARMCHINA_NPU_ARCH_V3

    INIT_OBJ := armchina-npu/sky1/sky1.o
    EXTRA_CFLAGS += -DCONFIG_SKY1
    ccflags-y += -DCONFIG_SKY1

OBJS     := $(COMM_OBJ) $(AIPU_OBJ) $(SOC_OBJ)

ifneq ($(KERNELRELEASE),)
	obj-m := $(MODULE_NAME).o
	$(MODULE_NAME)-objs := $(INIT_OBJ) $(OBJS)
else
	COMPASS_DRV_BTENVAR_KPATH ?= /lib/modules/`uname -r`/build
	PWD :=$(shell pwd)

all:
	$(MAKE) -C $(COMPASS_DRV_BTENVAR_KPATH) M=$(PWD) modules
clean:
	$(MAKE) -C $(COMPASS_DRV_BTENVAR_KPATH) M=$(PWD) clean
endif
