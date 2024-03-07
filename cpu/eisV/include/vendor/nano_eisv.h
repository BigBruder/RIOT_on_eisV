/*
 * Copyright (C) 2023 Jingwei Li
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     eisv
 * @ingroup     drivers_periph_nanocontroller
 * @{
 *
 * @file        nano.h
 * @brief       Low-level I2C driver implementation headfile
 *
 * @author      Jingwei Li <jingwei.li@tu-braunschweig.de>
 *
 * @}
 */

#ifndef _NANO_EISV_H
#define _NANO_EISV_H

#include <stdint.h>

/* NanoController config*/
#define NANO_D_W                (9)  //nano data path width

/* Register offsets */
#define NANO_CONTROL			(0x0)
#define NANO_TIME		  	    (0x4)

/* Commands: event trigger */
#define NANO_CMD_ON             (0b0001) 
#define NANO_CMD_OFF            (0b0010)
#define NANO_CMD_100MS          (0b0100)
#define NANO_CMD_SYNC           (0b1000)

/* Mask */
#define NANO_READY              (1<<31)

/*NanoController Time Stamp structrue*/
typedef struct {
    uint32_t time;
    uint8_t  hours;
    uint8_t  mins;
    uint8_t  secs;
} nano_time_strc;

/*Functionality
  Three functions of NanoController: 
  	1. eisv_sleep: pseudo power-gating in Emulation; clock-gating + reset 
  	2. nano_wait_100ms: roughly clock-gating ~0.1 sec by count-down in RTC, (+-) one tick
  	3. nano_time_stamp: Hour:Minute:Second; from RTC in Nanocontroller 
*/

void nano_wait_100ms(const uint8_t n_times);

void nano_eisv_sleep(void);

void nano_time_stamp(nano_time_strc* t);

/*Initializing time stamp of Nano is supposed to be set as interrupt */
void nano_time_init(nano_time_strc* t_init );



/*------------------------------------------------*/
//UART of eisV definition: UART driver has been described in syscall.c of eisV.

#define UART_STATUS             (0xC)

#define UART_STATUS_TXC         (1<<6)
#define UART_STATUS_UDRE        (1<<5)

#endif /* _NANO_EISV_H */
