/*
 * t6713.h
 *
 *  Created on: 26 Oct 2017
 *      Author: root
 */

#ifndef T6713_H_
#define T6713_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <i2c/i2c.h>

#define I2C_BUS 0
#define T6713_ADDR 0x15

uint16_t t6713_read_gas_ppm();
uint16_t t6713_get_status();
uint16_t t6713_get_fw_version();

#endif /* T6713_H_ */
