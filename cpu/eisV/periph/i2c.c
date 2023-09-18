/*
 * Copyright (C) 2019 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     eisv
 * @ingroup     drivers_periph_i2c
 * @{
 *
 * @file        i2c.c
 * @brief       Low-level I2C driver implementation
 *
 * @author      Jingwei Li <jingwei.li@tu-braunschweig.de>
 *
 * @}
 */

#include <inttypes.h>
#include <assert.h>
#include <errno.h>

#include "clk.h"
#include "cpu.h"
#include "mutex.h"

#include "periph/i2c.h"
#include "periph_conf.h"

#include "vendor/i2c_eisv.h"
#include "vendor/platform.h"
//#include "vendor/prci_driver.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define I2C_BUSY_TIMEOUT    (0xffff)
//static const uint16_t _fe310_i2c_speed[2] = { 100U, 400U };

static inline int _wait_busy(i2c_t dev, uint32_t max_timeout_counter);
static inline int _start(i2c_t dev, uint16_t address); //address - slave addr. with read 1 or write 0
static inline int _read(i2c_t dev, uint8_t *data, int length, uint8_t stop);
static inline int _write(i2c_t dev, const uint8_t *data, int length,
                         uint8_t stop);
/**
 * @brief   Initialized bus locks
 */
static mutex_t locks[I2C_NUMOF];

static inline int DEBUG_crl_reg(i2c_t dev)
{
    uint8_t CRL_reg = _REG32(i2c_config[dev].addr, I2C_CONTROL);
	//printf("0x%X\n",CRL_reg);
    DEBUG("_[i2c] [control reg] 0x%02X\n", CRL_reg);
    return 0;
} 


void i2c_init(i2c_t dev)
{
    assert(dev < I2C_NUMOF);

    /* Initialize mutex */
    mutex_init(&locks[dev]);

    /* Select IOF0 */
    GPIO_REG(GPIO_IOF_SEL) &=
        ~((1 << i2c_config[dev].scl) | (1 << i2c_config[dev].sda));
    /* Enable IOF */
    GPIO_REG(GPIO_IOF_EN) |=
        ((1 << i2c_config[dev].scl) | (1 << i2c_config[dev].sda));

    //------------------init-------------------------------- 
    DEBUG_crl_reg(dev);
    /* set I2C_TIMER to 99 for test */
    _REG32(i2c_config[dev].addr, I2C_TIMER) = I2C_TIMER_99;
    //init_stop 0b11110000 = 0xD0
    _REG32(i2c_config[dev].addr, I2C_CONTROL) = I2C_CONTROL_START_STOP;
    // random data, no slave addr.
    _REG32(i2c_config[dev].addr, I2C_DATA) = 0b00101000; 
    //---------------------------------------------------------

    int ret=0;
    /* Wait for response on bus. */
    ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
        if (ret < 0) {
        DEBUG("_[i2c] return ret = %d\n", ret);
        //return ret;
    } 
    DEBUG_crl_reg(dev);

    /* Compute prescale: presc = (CORE_CLOCK / (5 * I2C_SPEED)) - 1 */

    DEBUG("_[i2c] init: control reg (0x%08X)\n",
          (unsigned)_REG32(i2c_config[dev].addr, I2C_CONTROL));
    DEBUG("_[i2c] initialization done\n");
}

void i2c_acquire(i2c_t dev)
{
    assert(dev < I2C_NUMOF);
    mutex_lock(&locks[dev]);
}

void i2c_release(i2c_t dev)
{
    assert(dev < I2C_NUMOF);
    mutex_unlock(&locks[dev]);
}

int i2c_read_bytes(i2c_t dev, uint16_t address, void *data, size_t length,
                   uint8_t flags) //address - slave addr. 7bit
{
    assert(length > 0);
    assert(dev < I2C_NUMOF);
    //10bit addr.
    if (flags & I2C_ADDR10) {
        return -EOPNOTSUPP;
    }

    /* Check for wrong arguments given */
    if (data == NULL || length == 0) {
        return -EINVAL;
    }

    DEBUG("_[i2c] read bytes\n");

    int ret = 0;

    if (!(flags & I2C_NOSTART)) {
        ret = _start(dev, ((address << 1) | I2C_READ));
        if (ret < 0) {
            DEBUG("_[i2c] Error: start command failed\n");
            DEBUG("_[i2c] return ret = %d\n", ret);   
            //return ret;
        }
    }
    
	/* Read dummy byte to enter ST_READ state in FSM */
	/* iobus_wr = '0' -> start_read_access <= '1' */
	uint32_t dummy = (uint32_t)(_REG32(i2c_config[dev].addr, I2C_DATA));
	DEBUG("_[i2c] read dummy byte , 0x%02lX\n", dummy);
	/* Wait for response on bus. */
	ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
    if (ret < 0) {
        DEBUG("_[i2c] return ret = %d\n", ret);
        return ret;
	} 

    /* read data and issue stop if needed */
    ret = _read(dev, data, length, (flags & I2C_NOSTOP) ? 0 : 1);
    if (ret < 0) {
        DEBUG("_[i2c] Error: read command failed\n");
        DEBUG("_[i2c] return ret = %d\n", ret);
        return ret;
        }

    _wait_busy(dev, I2C_BUSY_TIMEOUT);

    //return length;
    return 0;
}

int i2c_write_bytes(i2c_t dev, uint16_t address, const void *data,
                    size_t length,
                    uint8_t flags) //address - slave addr. 7bit
{
    assert(dev < I2C_NUMOF);

    int ret = 0;

    /* Check for unsupported operations */
    if (flags & I2C_ADDR10) {
        return -EOPNOTSUPP;
    }
    /* Check for wrong arguments given */
    if ((data == NULL) || (length == 0)) {
        return -EINVAL;
    }

    DEBUG("_[i2c] write bytes\n");

    if (!(flags & I2C_NOSTART)) {
        ret = _start(dev, (address << 1));
        if (ret < 0) {
            DEBUG("_[i2c] error: start command failed\n");
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;        
        }
    }

    ret = _write(dev, data, length, (flags & I2C_NOSTOP) ? 0 : 1);
    if (ret < 0) {
        DEBUG("_[i2c] error: write command failed\n");
        DEBUG("_[i2c] return ret = %d\n", ret);
        return ret;
    }

    return 0;
}

static inline int _wait_busy(i2c_t dev, uint32_t max_timeout_counter)
{
    uint32_t timeout_counter = 0;

    DEBUG("_[i2c] wait for transfer\n");

    while (!(_REG32(i2c_config[dev].addr, I2C_CONTROL) & I2C_CONTROL_DONE))
     {
        if (++timeout_counter >= max_timeout_counter) {
            DEBUG("_[i2c] error: transfer timeout\n");
            return -ETIMEDOUT;
            //if(ENABLE_DEBUG == 1)  break;
        }

    }
    DEBUG("_[i2c] transfer DONE\n");
    return 0;
}

static inline int _start(i2c_t dev, uint16_t address) 
//address - slave addr. with read 1 or write 0
{
    DEBUG("\n_[i2c]--_start--\n");

    _wait_busy(dev, I2C_BUSY_TIMEOUT);
    DEBUG_crl_reg(dev);
    /* start transmission */
    DEBUG("_[i2c] [send] start condition\n");
    //_REG32(i2c_config[dev].addr, I2C_CONTROL) |= I2C_CONTROL_START;
    _REG32(i2c_config[dev].addr, I2C_CONTROL) = 0b11100000; //0b11100000

    DEBUG_crl_reg(dev);
    DEBUG("_[i2c] [send] slave address, 0x%02X\n", address);
    _REG32(i2c_config[dev].addr, I2C_DATA) = address;
    DEBUG_crl_reg(dev);
  
    /* Ensure all bytes has been read */
    int ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
    if (ret < 0) {
        DEBUG("_[i2c] return ret = %d\n", ret);
        return ret;
    } 
    DEBUG_crl_reg(dev);
    DEBUG("_[i2c]--_start END--\n");
    return 0;
}

static inline int _read(i2c_t dev, uint8_t *data, int length, uint8_t stop)
{
    uint8_t count = 0;

    /* Read data buffer. */
    while (length--) {
        //uint8_t command = I2C_CMD_RD;

        /* Wait for hardware module to sync */
        int ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
         if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
        } 

        DEBUG_crl_reg(dev);
        if((length == 0) && stop){
            DEBUG("STOP read, last byte\n");
            //_REG32(i2c_config[dev].addr, I2C_CONTROL) |= I2C_CONTROL_STOP;
            _REG32(i2c_config[dev].addr, I2C_CONTROL) = 0b11010000;
        }
        DEBUG_crl_reg(dev);

        data[count] = (uint32_t)(_REG32(i2c_config[dev].addr, I2C_DATA));
        DEBUG("_[i2c] read byte #%i, 0x%02X\n", count, data[count]);

        /* Wait for response on bus. */
        ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
         if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
        } 
        DEBUG_crl_reg(dev);

        count++;
    }
    return 0;
}

static inline int _write(i2c_t dev, const uint8_t *data, int length,
                         uint8_t stop)
{
    uint8_t count = 0;

    /* Write data buffer until the end. */
    while (length--) {
        /* Wait for hardware module to sync */
        int ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
        if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
        } 

        DEBUG("_[i2c] write byte #%i, 0x%02X\n", count, data[count]);
        _REG32(i2c_config[dev].addr, I2C_DATA) = data[count++];
        
        DEBUG_crl_reg(dev);
        if ((length == 0) && stop) {
            //STOP condition
            //_REG32(i2c_config[dev].addr, I2C_CONTROL) |= I2C_CONTROL_STOP;
            //_REG32(i2c_config[dev].addr, I2C_CONTROL) = 0b11010000;
            DEBUG("_[i2c] STOP write ");
        }

        DEBUG_crl_reg(dev);
        ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
        if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
        }
        DEBUG_crl_reg(dev);
    }

    return 0;
}
