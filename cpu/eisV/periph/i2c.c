/*
 * Copyright (C) 2023 Jingwei Li
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

//dir:drivers/include/periph
#include "periph/i2c.h"  

//dir:cpu/eisV/include/vendor
#include "vendor/i2c_eisv.h"
#include "vendor/platform.h"

//dir:core/lib/include
#define ENABLE_DEBUG 0
#include "debug.h" //core/lib/include

#define I2C_BUSY_TIMEOUT    (0xffff)

static inline int _wait_busy(i2c_t dev, uint32_t max_timeout_counter);
static inline int _start(i2c_t dev, uint16_t address); //address - slave addr. with read 1 or write 0
static inline int _read(i2c_t dev, uint8_t *data, int length, uint8_t stop);
static inline int _write(i2c_t dev, const uint8_t *data, int length,
                         uint8_t stop);
/**
 * @brief   Initialized bus locks
 */
static mutex_t locks[I2C_NUMOF];

void i2c_init(i2c_t dev)
{
    assert(dev < I2C_NUMOF);

    /* Initialize mutex */
    mutex_init(&locks[dev]);

    //------------------init-------------------------------- 
    /* set I2C_TIMER to 99 for test */
    _REG32(i2c_config[dev].addr, I2C_TIMER) = I2C_TIMER_99;
    //init 0b11110000 = 0xD0
    _REG32(i2c_config[dev].addr, I2C_CONTROL) = I2C_CONTROL_START_STOP;
    // random data, no slave addr.
    _REG32(i2c_config[dev].addr, I2C_DATA) = 0b00101000; //random data
    //---------------------------------------------------------

    int ret=0;
    /* Wait for response on bus. */
    ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
        if (ret < 0) {
        DEBUG("_[i2c] return ret = %d\n", ret);
    } 

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
            return ret;
        }
    }
    
    /* Read dummy byte to enter ST_READ state in hardware FSM, 
    which is like to open a channel for the first Byte transmission */
    /* iobus_wr = '0' -> start_read_access <= '1' */
    uint32_t dummy = (uint32_t)(_REG32(i2c_config[dev].addr, I2C_DATA));
    DEBUG("_[i2c] read dummy byte , 0x%02lX\n", dummy);

    /*consider the situation: only one Byte to receive*/
    if((length == 1) && ((flags & I2C_NOSTOP) ? 0 : 1)){
        DEBUG("STOP read, last byte\n");
        _REG32(i2c_config[dev].addr, I2C_CONTROL) |= I2C_CONTROL_STOP;
    }

    /* Waiting for the first Byte data transmission */
    ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
    if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
    } 

    /* read data and issue STOP if needed */
    ret = _read(dev, data, length, (flags & I2C_NOSTOP) ? 0 : 1);
    if (ret < 0) {
        DEBUG("_[i2c] Error: read command failed\n");
        DEBUG("_[i2c] return ret = %d\n", ret);
        return ret;
    }
    _wait_busy(dev, I2C_BUSY_TIMEOUT);

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

    /*Checking control-reg DONE bit*/
    while (!(_REG32(i2c_config[dev].addr, I2C_CONTROL) & I2C_CONTROL_DONE))
     {
        if (++timeout_counter >= max_timeout_counter) {
            DEBUG("_[i2c] error: transfer timeout\n");
            return -ETIMEDOUT;
        }
    }

    return 0;
}

static inline int _start(i2c_t dev, uint16_t address) 
//address - slave addr. with read 1 or write 0
{
    DEBUG("\n_[i2c]--_start--\n");

    _wait_busy(dev, I2C_BUSY_TIMEOUT);

    /* start transmission: enable START condition by 11100000 */
    DEBUG("_[i2c] [send] start condition\n");
    _REG32(i2c_config[dev].addr, I2C_CONTROL) |= I2C_CONTROL_START;

    DEBUG("_[i2c] [send] slave address, 0x%02X\n", address);
    _REG32(i2c_config[dev].addr, I2C_DATA) = address;

    /* Ensure all bytes has been read */
    int ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
    if (ret < 0) {
        DEBUG("_[i2c] return ret = %d\n", ret);
        return ret;
    } 
    
    DEBUG("_[i2c]--_start END--\n");
    return 0;
}

static inline int _read(i2c_t dev, uint8_t *data, int length, uint8_t stop)
{
    uint8_t count = 0;

    /* Read data buffer. */
    while (length--) {

        /* Wait for hardware module to sync */
        int ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
         if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
        } 

        /*Read a real data from DATA-reg. and open channel for the next one*/
        data[count] = (uint32_t)(_REG32(i2c_config[dev].addr, I2C_DATA));
        DEBUG("_[i2c] read byte #%i, 0x%02X\n", count, data[count]);

        /*Send STOP condition, before transmission of the last byte*/
        if((length == 1) && stop){
            DEBUG("STOP read, last byte\n");
            _REG32(i2c_config[dev].addr, I2C_CONTROL) |= I2C_CONTROL_STOP;
        }
        /* Wait for the next data byte transmission */
        ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
         if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
        } 

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

        /*Send STOP condition, before transmission of the last byte*/
        if ((length == 0) && stop) {
            //STOP condition
            _REG32(i2c_config[dev].addr, I2C_CONTROL) |= I2C_CONTROL_STOP;
            DEBUG("_[i2c] STOP write \n");
        }

        DEBUG("_[i2c] write byte #%i, 0x%02X\n", count, data[count]);
        _REG32(i2c_config[dev].addr, I2C_DATA) = data[count++];
        
        /*Wait for writing done*/
        ret = _wait_busy(dev, I2C_BUSY_TIMEOUT);
        if (ret < 0) {
            DEBUG("_[i2c] return ret = %d\n", ret);
            return ret;
        }
    }

    return 0;
}
