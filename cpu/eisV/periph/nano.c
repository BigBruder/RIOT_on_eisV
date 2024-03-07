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
 * @file        nano.c
 * @brief       Low-level I2C driver implementation headfile
 *
 * @author      Jingwei Li <jingwei.li@tu-braunschweig.de>
 *
 * @}
 */
#include <inttypes.h>
#include <errno.h>

//dir:cpu/eisV/include/vendor
#include "vendor/nano_eisv.h"
#include "vendor/platform.h"

//dir:core/lib/include
#define ENABLE_DEBUG 0
#include "debug.h" //core/lib/include

#define BUSY_TIMEOUT    (0xFFFFFF)

#if ENABLE_DEBUG == 1  
static void _wait_uart_busy(void);
#endif
static int   _wait_nano_busy(uint32_t max_timeout_counter);
static void  _nano_trigger_cmd(uint8_t nano_command);


void nano_wait_100ms(const uint8_t n_times){
//count down 6 ticks each time, 1 tick = 0.016384 sec
//precision: +- 1 tick
// 1 sec = 61 ticks
	DEBUG("_[NANO]: nano_wait:%d * 0.1 sec \n", n_times );
    #if ENABLE_DEBUG == 1  
        _wait_uart_busy();
    #endif
        
    for (uint8_t i=0; i < n_times; i++){
        _nano_trigger_cmd(NANO_CMD_100MS); //about 0.1sec once
    }

    /*waiting the last clock-gating is executing in Nano*/
    int ret = _wait_nano_busy(BUSY_TIMEOUT);
    if (ret < 0) {
        DEBUG("_[NANO] clock-gating error: return ret = %d\n", ret);
    } 
}


void nano_time_init(nano_time_strc* t_init ){
    DEBUG("_[NANO]: **Init time stamp**\n");

    t_init->time    =   (t_init->hours  <<(NANO_D_W*2)) | 
                        (t_init->mins   <<(NANO_D_W*1)) | 
                        (t_init->secs   <<(NANO_D_W*0)) ;
    /*transfer data to nanocontroller*/
    NANO_REG(NANO_TIME) = t_init->time;	
    DEBUG("_[NANO]_timeinit: sending SYNC cmd\n");
    /*trigger nano to synchronize time stamp*/
	_nano_trigger_cmd(NANO_CMD_SYNC); 
    DEBUG("_[NANO]_timeinit: sending wait 100ms cmd\n");
    /*waiting nano load data and output time stamp */
    nano_wait_100ms(1);
    DEBUG("_[NANO]_timeinit: **Init time stamp** END\n");
}

void nano_time_stamp(nano_time_strc* t){
	t->time  = NANO_REG(NANO_TIME);
	t->hours =(uint8_t)  t->time               ;
	t->mins  =(uint8_t) (t->time>> NANO_D_W )  ;
	t->secs  =(uint8_t) (t->time>>(NANO_D_W*2));
}


void nano_eisv_sleep(void){
// pseudo power gating for eisV controller
    DEBUG("_[NANO]: eisv_sleep \n");

    #if ENABLE_DEBUG == 1  
	    _wait_uart_busy();//waiting UART
    #endif

	_nano_trigger_cmd(NANO_CMD_OFF);

    /*waiting for power-gating */
	int ret =_wait_nano_busy(BUSY_TIMEOUT);
    if (ret < 0) {
        DEBUG("_[NANO] sleep error: return ret = %d\n", ret);
    } 
}

#if ENABLE_DEBUG == 1 
/* 
UART is quit slow compared to eisV clock.
_wait_uart_busy must be called after printing by using UART.
Example: 
        printf("...");
        //not allowed other actions between.
        _wait_uart_busy();
*/
static void _wait_uart_busy(void){
	DEBUG("=====>waiting UART\n");

    /*waiting UART transmitter buffer to empty state*/
    while (!(UART0_REG(UART_STATUS) & UART_STATUS_UDRE)){};

    /*flip transmit-complete (TXC) to 1 manually */
	UART0_REG(UART_STATUS) |= UART_STATUS_TXC;

    /*waiting UART transmit last byte done*/
	while (!(UART0_REG(UART_STATUS) & UART_STATUS_TXC)){};
}
#endif 

static int _wait_nano_busy(uint32_t max_timeout_counter)
{ 	
    uint32_t timeout_counter = 0;
    DEBUG("_[NANO]: waiting nano busy\n");
    while (!(NANO_REG(NANO_CONTROL) & NANO_READY))
    {
        if (++timeout_counter >= max_timeout_counter) {
            //DEBUG("_[NANO] error: transfer timeout\n");
            return -ETIMEDOUT;
        }
    }
    //DEBUG("_[NANO]: wait nano busy end\n");
    return 0;
}

static void _nano_trigger_cmd(uint8_t nano_command){
    
    DEBUG("_[NANO]: Trigger Command: %X \n",nano_command);
        
    /* Ensure last event has been triggered and executed */
    int ret = _wait_nano_busy(BUSY_TIMEOUT);
    if (ret < 0) {
        DEBUG("_[NANO]error: return ret = %d\n", ret);
    } 

    NANO_REG(NANO_CONTROL) = nano_command;
    //DEBUG("_[NANO]: command sent\n");

}
