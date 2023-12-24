/*
 * Copyright 2022
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_vexriscv
 * @{
 *
 * @file        uart.c
 * @brief       Low-level spinalhdl uart implementation
 *
 * @author      Koen Zandberg
 * @}
 */

#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <inttypes.h>

#include "irq.h"
#include "cpu_common.h"
#include "periph/uart.h"
#include "plic.h"
#include "vendor/riscv_csr.h"

/**
 * @brief   Allocate memory to store the callback functions
 */
static uart_isr_ctx_t isr_ctx[UART_NUMOF];

#define UART_REG_FIFO 0x0
#define UART_REG_STATUS 0x4

#define UART_REG_STATUS_TXINT_ENABLE (1 << 0)
#define UART_REG_STATUS_RXINT_ENABLE (1 << 1)

static inline unsigned _txfifo_avail(uart_t dev)
{
    uint32_t status = _REG32(uart_config[dev].addr, UART_REG_STATUS);
    return (status >> 16) & 0xff;
}

static inline unsigned _rxfifo_avail(uart_t dev)
{
    uint32_t status = _REG32(uart_config[dev].addr, UART_REG_STATUS);
    return (status >> 24) & 0xff;
}

static void _drain(uart_t dev)
{
    volatile uint8_t data = 0;
    while (_rxfifo_avail(dev)) {
        data = _REG32(uart_config[dev].addr, UART_REG_FIFO) & 0xFF;
    }
    (void)data;
}

static inline void _uart_isr(uart_t dev)
{
    while (_rxfifo_avail(dev)) {
        uint8_t data = _REG32(uart_config[dev].addr, UART_REG_FIFO) & 0xFF;
        if (isr_ctx[dev].rx_cb) {
            isr_ctx[dev].rx_cb(isr_ctx[dev].arg, data);
        }
    }
}

void uart_isr(int num)
{
    switch (num) {
    case INT_UART0_BASE:
        _uart_isr(0);
        break;
    default:
        break;
    }
}

int uart_init(uart_t dev, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    (void)dev;
    (void)baudrate;
    (void)rx_cb;
    (void)arg;

    /* Enable RX intr if there is a callback */
    if (rx_cb) {
        /* Disable ext interrupts when setting up */
        clear_csr(mie, MIP_MEIP);

        /* Save interrupt callback context */
        isr_ctx[dev].rx_cb = rx_cb;
        isr_ctx[dev].arg = arg;

        /* Configure UART ISR with PLIC */
        plic_set_isr_cb(uart_config[dev].isr_num, uart_isr);
        plic_enable_interrupt(uart_config[dev].isr_num);
        plic_set_priority(uart_config[dev].isr_num, UART_ISR_PRIO);

        /* avoid trap by emptying RX FIFO */
        _drain(dev);

        /* enable RX interrupt */
        _REG32(uart_config[dev].addr, UART_REG_STATUS) |= UART_REG_STATUS_RXINT_ENABLE;

        /* Re-enable ext interrupts */
        set_csr(mie, MIP_MEIP);
    }
    return UART_OK;
}

void uart_write(uart_t dev, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        /* Wait for FIFO to empty */
        while (_txfifo_avail(dev) == 0) {}

        /* Write a byte */
        _REG32(uart_config[dev].addr, UART_REG_FIFO) = data[i];
    }
}

void uart_poweron(uart_t dev)
{
    (void)dev;
}

void uart_poweroff(uart_t dev)
{
    (void)dev;
}
