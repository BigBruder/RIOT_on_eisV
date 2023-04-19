/*
 * Copyright (C) 2023 E.I.S
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_arty_a735t
 * @{
 *
 * @file
 * @brief       Configuration of CPU peripherals for Arty-A7-35T board
 *
 * @author      Jingwei Li <jingweili.sh@outlook.com>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

//#include <stdint.h>

#include "cpu.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* put here the board peripherals definitions:
- Available clocks
- Timers
- UARTs
- PWMs
- SPIs
- I2C
- ADC
- RTC
- RTT
etc
 */

/**
 * @name    Timer configuration
 *
 * @{
 */
#define TIMER_NUMOF                 (1)
/** @} */

#define CLOCK_CORECLOCK             (50000000UL)
#define XTIMER_HZ                   (50000000UL)

/**
 * @name   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .addr       = (0x40020000L),
        .isr_num    = INT_UART0_BASE,
    },
};

#define UART_NUMOF                  ARRAY_SIZE(uart_config)
/** @} */



#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
