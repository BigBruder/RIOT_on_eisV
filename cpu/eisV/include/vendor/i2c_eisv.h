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
 * @file        i2c_eisv.h
 * @brief       Low-level I2C driver implementation
 *
 * @author      Jingwei Li <jingwei.li@tu-braunschweig.de>
 *
 * @}
 */

#ifndef _EISV_I2C_H
#define _EISV_I2C_H

/* I2C memory map */
// #define I2C0_CTRL_ADDR 			(0xFFFFFFD0) //adapted to eisv i2c_master

// Helper functions
// #define _REG32(p, i) 			(*(volatile uint32_t *) ((p) + (i)))
//#define I2C0_REG(offset) 		_REG32(I2C0_CTRL_ADDR, offset)


/* Register offsets */
#define I2C_CONTROL			(0x4)
#define I2C_DATA			(0x8)
#define I2C_TIMER			(0xC)

/* CONTROL register */
/* I2C_CONTROL 8bit
  -- bit 7 (r/w): I2CIE  interrupt enable
  -- bit 6 (r/w): I2CE   enable sda and scl pins
  -- bit 5 (r/w): START  I2C start condition before next write access
  -- bit 4 (r/w): STOP   I2C stop condition after end of bus access
  -- bit 3 (r/w)         not used
  -- bit 2 (r/w): DONE   read, write or STOP done, interrupt flag
  -- bit 1 (r)  : BFREE  bus free (there was a STOP)
  -- bit 0 (r)  : ACK    Ack of last transfer
*/
#define I2C_CONTROL_I2CIE			(1 << 7) //(0b10000000)
#define I2C_CONTROL_I2CE			(1 << 6) //(0b01000000)
#define I2C_CONTROL_START			(1 << 5) //(0b00100000)
#define I2C_CONTROL_STOP			(1 << 4) //(0b00010000)

#define I2C_CONTROL_DONE			(1 << 2) //(0b00000100)
#define I2C_CONTROL_BFREE			(1 << 1) //(0b00000010)
#define I2C_CONTROL_ACK			    (1 << 0) //(0b00000001)





#endif /* _EISV_I2C_H */
