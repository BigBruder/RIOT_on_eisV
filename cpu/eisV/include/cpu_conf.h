/*
 * Copyright (C) 2017 Ken Rabold
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup         cpu_fe310
 * @{
 *
 * @file
 * @brief           CPU specific configuration options
 *
 * @author          Ken Rabold
 */

#ifndef CPU_CONF_H
#define CPU_CONF_H

#include "cpu_conf_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Helper functions
#define _REG32(p, i) 			(*(volatile uint32_t *) ((p) + (i)))
#define _REG32P(p, i) 			((volatile uint32_t *) ((p) + (i)))

/**
 * @brief Base address of the CLINT
 */
#define CLINT_BASE_ADDR     (0x40000000L)
#define CLINT_MTIMECMP      (0x4000)
#define CLINT_MTIME         (0xBFF8)

#define RTC_FREQ (50000000UL)
/**
 * @brief Base address of the PLIC peripheral
 */
#define PLIC_BASE_ADDR      (0x50000000L)
#define PLIC_CTRL_ADDR      (0x50000000L)

#define INT_UART0_BASE      (2)
#define PLIC_NUM_INTERRUPTS      (32)
#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
