/*
 * t6713.c
 *
 *  Created on: 26 Oct 2017
 *      Author: root
 */

#include "t6713.h"

const char t6713_commands[3][5] = {
		{0x04, 0x13, 0x8B, 0x00, 0x01}, /* read */
		{0x04, 0x13, 0x8A, 0x00, 0x01}, /* get status */
		{0x04, 0x13, 0x89, 0x00, 0x01} /* get fw version */
};

typedef enum {
	T6713_CMD_READ_GAS_PPM,
	T6713_CMD_GET_STATUS,
	T6713_CMD_GET_FW_VERSION,
};

static int t6713_command(uint8_t type, uint8_t *buffer)
{
	int ret, i;
    uint8_t *command;

    command = t6713_commands[type];

    i2c_slave_write(I2C_BUS, T6713_ADDR, NULL, command, 5);

	vTaskDelay(2 / portTICK_PERIOD_MS);

    i2c_slave_read(I2C_BUS, T6713_ADDR, NULL, buffer, 4);

	return 0;
}

uint16_t t6713_read_gas_ppm()
{
	uint16_t co2ppm = 0;
	uint8_t data[4] = {0};

	t6713_command(T6713_CMD_READ_GAS_PPM, data);

	co2ppm = ((data[2] * 0xFF ) + data[3]);

	return co2ppm;
}

uint16_t t6713_get_status()
{
	uint16_t status = 0;
	uint8_t data[4] = {0};

	t6713_command(T6713_CMD_GET_STATUS, data);

	status = (data[2] * 0xFF) + data[3];

	return status;
}

uint16_t t6713_get_fw_version()
{
	uint16_t version = 0;
	uint8_t data[4] = {0};

	t6713_command(T6713_CMD_GET_FW_VERSION, data);

	version = (data[2] * 0xFF) + data[3];

	return version;
}

