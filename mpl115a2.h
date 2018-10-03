/*
 * mpl115a2.h
 *
 *  Created on: 26 Oct 2017
 *      Author: root
 */

#ifndef MPL115A2_H_
#define MPL115A2_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <i2c/i2c.h>

#define I2C_BUS 0
#define MPL115A2_ADDR (0x60)

/* MPL115 registers */
#define MPL115_PADC_MSB 0x00
#define MPL115_PADC_LSB 0x01
#define MPL115_TADC_MSB 0x02
#define MPL115_TADC_LSB 0x03
#define MPL115_AO_MSB 0x04
#define MPL115_A0_LSB 0x05
#define MPL115_B1_MSB 0x06
#define MPL115_B1_LSB 0x07
#define MPL115_B2_MSB 0x08
#define MPL115_B2_LSB 0x09
#define MPL115_C12_MSB 0x0A
#define MPL115_C12_LSB 0x0B
#define MPL115_CONVERT_REG 0x12

float mpl115_read_pressure();

#endif /* MPL115A2_H_ */
