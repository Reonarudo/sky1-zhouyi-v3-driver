/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_SOC_H__
#define __AIPU_SOC_H__

#include <linux/platform_device.h>
#include <linux/device.h>
#include <armchina_aipu.h>

int armchina_aipu_probe(struct platform_device *p_dev);
int armchina_aipu_remove(struct platform_device *p_dev);
int armchina_aipu_suspend(struct platform_device *p_dev, pm_message_t state);
int armchina_aipu_resume(struct platform_device *p_dev);
int armchina_aipu_alloc_dma_buf(struct aipu_dma_buf_request *request);
int armchina_aipu_free_dma_buf(int fd);

#endif /* __AIPU_SOC_H__ */
