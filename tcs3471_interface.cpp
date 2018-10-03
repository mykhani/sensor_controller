#include "tcs3471_interface.h"
#include "TCS3471.h"
#include <i2c/i2c.h>

// implementation of i2cWrite and i2cRead functions for simplest case when there is only one
// TCS2471 chip attached to Arduino's two wire bus
void i2cWrite(uint8_t address, uint8_t count, uint8_t* buffer)
{
    i2c_slave_write(I2C_BUS, address, NULL, buffer, count);

    return;
}

void i2cRead(uint8_t address, uint8_t count, uint8_t* buffer)
{
    i2c_slave_read(I2C_BUS, address, NULL, buffer, count);
    
    return;
}

TCS3471Handle_t tcs3471_create()
{
    return new TCS3471(i2cWrite, i2cRead);
}

void tcs3471_delete(TCS3471Handle_t p)
{
    delete (TCS3471 *)p;
}

bool tcs3471_detect(TCS3471Handle_t t)
{
    return ((TCS3471 *)t)->detect();
}

uint8_t tcs3471_getChipID(TCS3471Handle_t t)
{
    return ((TCS3471 *)t)->getChipID();
}

void tcs3471_setIntegrationTime(TCS3471Handle_t t, float integration_time)
{
    ((TCS3471 *)t)->setIntegrationTime(integration_time);
}

void tcs3471_setWaitTime(TCS3471Handle_t t, float wait_time)
{
    ((TCS3471 *)t)->setWaitTime(wait_time);
}

void tcs3471_setGain(TCS3471Handle_t t, uint8_t gain)
{
    ((TCS3471 *)t)->setGain((tcs3471Gain_t)gain);
}

void tcs3471_enable(TCS3471Handle_t t)
{
    ((TCS3471 *)t)->enable();
}

bool tcs3471_rgbcValid(TCS3471Handle_t t)
{
    return ((TCS3471 *)t)->rgbcValid();
}

uint16_t tcs3471_readCData(TCS3471Handle_t t)
{
    return ((TCS3471 *)t)->readCData();
}

uint16_t tcs3471_readRData(TCS3471Handle_t t)
{
    return ((TCS3471 *)t)->readRData();
}

uint16_t tcs3471_readGData(TCS3471Handle_t t)
{
    return ((TCS3471 *)t)->readGData();
}

uint16_t tcs3471_readBData(TCS3471Handle_t t)
{
    return ((TCS3471 *)t)->readBData();
}

