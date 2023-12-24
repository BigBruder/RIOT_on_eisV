// See LICENSE for license details.

#ifndef _SIFIVE_I2C_H
#define _SIFIVE_I2C_H

/* Register offsets */
#define I2C_PRESCALE_LO		(0x00)
#define I2C_PRESCALE_HI		(0x00)
#define I2C_CONTROL			(0x04)
#define I2C_DATA			(0x08)
#define I2C_CMD				I2C_CONTROL
#define I2C_STATUS			I2C_CONTROL
//#define I2C_STATUS			(0x10)

/* CONTROL register */
#define I2C_CONTROL_EN		(1 << 7)
#define I2C_CONTROL_IE		(1 << 6)

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

/* CMD register */
#define I2C_CMD_I2CIE			(1 << 7) //(0b10000000)
#define I2C_CMD_I2CE			(1 << 6) //(0b01000000)
#define I2C_CMD_START			(1 << 5) //(0b00100000)
#define I2C_CMD_STOP			(1 << 4) //(0b00010000)
#define I2C_CMD_DONE			(1 << 2) //(0b00000100)
#define I2C_CMD_BFREE			(1 << 1) //(0b00000010)
#define I2C_CMD_ACK			(1 << 0) //(0b00000001)
/*
#define I2C_CMD_STA			(1 << 7) //(0b10000000)
#define I2C_CMD_STO			(1 << 6) //(0b01000000)
#define I2C_CMD_RD			(1 << 5) //(0b00100000)
#define I2C_CMD_WR			(1 << 4) //(0b10000000)
#define I2C_CMD_ACK			(1 << 3) //(0b10000000)
#define I2C_CMD_IACK			(1 << 0) //(0b10000000)
*/


/* STATUS register */
#define I2C_STATUS_RXACK	(1 << 7)
#define I2C_STATUS_BUSY		(1 << 6)
#define I2C_STATUS_ALOST	(1 << 5)
#define I2C_STATUS_TIP		(1 << 1)
#define I2C_STATUS_IF		(1 << 0)

/**************************************************/
#define I2C_STATUS_DONE		(1 << 2) // eisv

#endif /* _SIFIVE_I2C_H */
