/*
 * Copyright (C) 2022 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_vexriscv
 * @{
 *
 * @brief       Implementation of the CPU initialization for VexRiscV based CPU
 *
 * @author      Koen Zandberg
 * @}
 */

#include "clk.h"
#include "cpu_common.h"
#include "periph/init.h"

#include "stdio_uart.h"

/**
 * @brief Initialize the CPU, set IRQ priorities, clocks, peripheral
 */
void cpu_init(void)
{
    /* Common RISC-V initialization */
    riscv_init();

    /* Initialize stdio */
    stdio_init();

    /* Initialize static peripheral */
    periph_init();
}
