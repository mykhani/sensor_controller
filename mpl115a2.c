/*
 * mpl115a2.c
 *
 *  Created on: 30 Oct 2017
 *      Author: root
 */

#include "mpl115a2.h"

//#define MPL115_DEBUG true

#ifdef MPL115_DEBUG
#include <stdio.h>
#define debug(fmt, ...) printf("%s: " fmt "\n", "TCS3471", ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

static int mpl115_read_coefficients(float *a0, float *b1, float *b2, float *c12)
{
	int16_t a0_word;
	int16_t b1_word;
	int16_t b2_word;
	int16_t c12_word;
	char data[8];
	uint8_t dest_reg;
	int ret;

	/* select start of coefficients register as destination register */
	dest_reg = MPL115_AO_MSB;

    i2c_slave_read(I2C_BUS, MPL115A2_ADDR, &dest_reg, data, 8);

	a0_word = (data[0] << 8) | data[1];
	b1_word = (data[2] << 8) | data[3];
	b2_word = (data[4] << 8) | data[5];
	c12_word = ((data[6] << 8) | (data[7])) >> 2;

	*a0 = (float) a0_word / 8.0;
	*b1 = (float) b1_word / 8192.0;
	*b2 = (float) b2_word / 16384.0;
	*c12 = (float) c12_word / 4194304.0;

    debug("MPL115 coefficients: a0: %f, b1: %f, b2: %f, c12: %f \r\n", a0, b1, b2, c12);

	return 0;
}

float mpl115_read_pressure()
{
    float pressure = 0.0f;
	uint8_t dest_reg = 0;
    char cmd[2] = {0x12, 0x00}; /* start conversion command */
	char data[4] = {0};
	int16_t Padc = 0;
	int16_t Tadc = 0;
	float a0 = 0, b1 = 0, b2 = 0, c12 = 0;
	float Pcomp = 0.0f;	/* Pressure compensation */
	int ret = -1;

	if (mpl115_read_coefficients(&a0, &b1, &b2, &c12)) {
		return -1;
	}
	
	/* send start conversion command */
    i2c_slave_write(I2C_BUS, MPL115A2_ADDR, NULL, cmd, 2);

	vTaskDelay(50 / portTICK_PERIOD_MS);
	
	dest_reg = MPL115_PADC_MSB;

    i2c_slave_read(I2C_BUS, MPL115A2_ADDR, &dest_reg, data, 4);

	/* Convert the data to 10-bits */
	Padc = ((data[0] << 8) | (data[1] & 0xC0)) >> 6;
	Tadc = ((data[2] << 8) | (data[3] & 0xC0)) >> 6;

	/* Calculate pressure compensation */
	Pcomp = a0 + (b1 + c12 * Tadc) * Padc + b2 * Tadc;

	/* Calculate Pressure (kPa) */
	pressure = (65.0 / 1023.0) * Pcomp + 50.0;

	return pressure;
}
