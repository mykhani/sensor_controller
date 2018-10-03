/*
  TCS3471.cpp - TCS3471 Color light-to-digital converter IC by TAOS/AMS library
  Copyright (c) 2012, 2013 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "TCS3471.h"

//#define TCS3471_DEBUG true

#ifdef TCS3471_DEBUG
#include <stdio.h>
#define debug(fmt, ...) printf("%s: " fmt "\n", "TCS3471", ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

TCS3471::TCS3471(void (*i2cWriteFunc)(uint8_t,uint8_t,uint8_t*),void (*i2cReadFunc)(uint8_t,uint8_t,uint8_t*))
{
    _i2cWrite = i2cWriteFunc;
    _i2cRead = i2cReadFunc;
    _detected = false;
    _i2cAddress = 0;
}

bool TCS3471::detect()
{
    if (_detected)
        return true;
    else
    {
        _i2cAddress = TCS3471_ADDRESS_1;
        uint8_t tmpaddr = read8(TCS3471_ID_REG);
        if (tmpaddr == 0x14 || tmpaddr == 0x1D)
            {
                _detected = true;
            }
        else
            {
                _i2cAddress = TCS3471_ADDRESS_2;
                tmpaddr = read8(TCS3471_ID_REG);
                if (tmpaddr == 0x14 || tmpaddr == 0x1D)
                    {
                        _detected = true;
                    }
                else
                    {
                        _i2cAddress = 0;
                    }
            }
    }
    return _detected;
}

bool TCS3471::enable()
{
    detect();
    if (_detected)
    {
        debug("Enabling TCS341 \r\n");
        write8(TCS3471_ENABLE_REG, read8(TCS3471_ENABLE_REG) | TCS3471_PON_BIT);
        vTaskDelay(3 / portTICK_PERIOD_MS);
        write8(TCS3471_ENABLE_REG, read8(TCS3471_ENABLE_REG) | TCS3471_AEN_BIT);
        return true;
    }
    else
        return false;
}

void TCS3471::disable()
{
    if (_detected)
    {
        write8(TCS3471_ENABLE_REG, read8(TCS3471_ENABLE_REG) & ~(TCS3471_PON_BIT | TCS3471_AEN_BIT));
    }
}

void TCS3471::setIntegrationTime(float integrationTime)
{
    if (_detected)
    {
        debug("Setting integration time: %f \r\n", integrationTime);
        uint16_t aTime = (uint16_t)(integrationTime * 10);
        aTime = aTime / 24;
        if (aTime > 256)
            aTime = 256;
        aTime = aTime - 256;
        write8(TCS3471_ATIME_REG, (uint8_t)aTime);
    }
}

void TCS3471::setWaitTime(float waitTime)
{
    if (_detected)
        debug("Setting wait time: %f \r\n", waitTime);
    {
        int32_t wTime = (int32_t)(waitTime * 10);
        if (wTime < 24)
        {
            write8(TCS3471_ENABLE_REG, read8(TCS3471_ENABLE_REG) & ~TCS3471_WEN_BIT);
            return;
        }
        else if (wTime > 6144)
        {
            write8(TCS3471_CONFIG_REG, TCS3471_WLONG_BIT);
            if (wTime > 73728)
                wTime = 73728;
            wTime = wTime / 288;
        }
        else
        {
            write8(TCS3471_CONFIG_REG, 0x00);
            wTime = wTime / 24;
        }
        wTime = 256 - wTime;
        write8(TCS3471_WTIME_REG, (uint8_t)wTime);
        write8(TCS3471_ENABLE_REG, read8(TCS3471_ENABLE_REG) | TCS3471_WEN_BIT);
    }
}

void TCS3471::setGain(tcs3471Gain_t gain)
{
    if (_detected)
    {
        debug("Setting gain: %d \r\n", gain);
        write8(TCS3471_CONTROL_REG, gain);
    }
}

void TCS3471::enableInterrupt()
{
    if (_detected)
    {
        write8(TCS3471_ENABLE_REG, read8(TCS3471_ENABLE_REG) | TCS3471_AIEN_BIT);
    }
}

void TCS3471::disableInterrupt()
{
    if (_detected)
    {
        write8(TCS3471_ENABLE_REG, read8(TCS3471_ENABLE_REG) & ~TCS3471_AIEN_BIT);
    }
}

void TCS3471::clearInterrupt()
{
    if (_detected)
    {
        _i2cBuffer[0] = TCS3471_COMMAND_BIT | TCS3471_SPECIAL_BIT | TCS3471_INTCLEAR_BIT;
        _i2cWrite(_i2cAddress,1,_i2cBuffer);
    }
}

void TCS3471::interruptHighThreshold(uint16_t highThreshold)
{
    if (_detected)
    {
        write16(TCS3471_AIHTL_REG,highThreshold);
    }
}

void TCS3471::interruptLowThreshold(uint16_t lowThreshold)
{
    if (_detected)
    {
        write16(TCS3471_AILTL_REG,lowThreshold);
    }
}

void TCS3471::interruptPersistence(uint8_t persistence)
{
    if (_detected)
    {
        uint8_t valueToWrite = persistence;
        if (valueToWrite > 60)
            valueToWrite = 60;
        if (valueToWrite > 3)
            valueToWrite = valueToWrite / 5 + 3;
        write8(TCS3471_PERS_REG,valueToWrite & 0x0F);
    }
}

uint8_t TCS3471::getChipID()
{
    detect();
    if (_detected)
    {
        return read8(TCS3471_ID_REG);
    }
    else
        return 0;
}

bool TCS3471::rgbcValid()
{
    debug("Reading TCS3471 status reg\r\n");
    if (_detected)
    {
        uint8_t status =  0;
       
        status = read8(TCS3471_STATUS_REG);

        debug("TCS3471 status reg: %02x\r\n", status);
        return (status & TCS3471_AVALID_BIT) == TCS3471_AVALID_BIT;
    }
    else
        return false;
}

uint16_t TCS3471::readCData()
{
    if (_detected)
    {
        return read16(TCS3471_CDATA_REG);
    }
    return 0;
}

uint16_t TCS3471::readRData()
{
    if (_detected)
    {
        return read16(TCS3471_RDATA_REG);
    }
    return 0;
}

uint16_t TCS3471::readGData()
{
    if (_detected)
    {
        return read16(TCS3471_GDATA_REG);
    }
    return 0;
}

uint16_t TCS3471::readBData()
{
    if (_detected)
    {
        return read16(TCS3471_BDATA_REG);
    }
    return 0;
}

void TCS3471::write8(uint8_t reg, uint8_t val)
{
    _i2cBuffer[0] = TCS3471_COMMAND_BIT | reg;
    _i2cBuffer[1] = val;
    _i2cWrite(_i2cAddress,2,_i2cBuffer);
}

void TCS3471::write16(uint8_t reg, uint16_t val)
{
    _i2cBuffer[0] = TCS3471_COMMAND_BIT | TCS3471_AUTOINCR_BIT | reg;
    _i2cBuffer[1] = val & 0xFF;
    _i2cBuffer[2] = (val >> 8) & 0xFF;
    _i2cWrite(_i2cAddress,3,_i2cBuffer);
}

uint8_t TCS3471::read8(uint8_t reg)
{
    _i2cBuffer[0] = TCS3471_COMMAND_BIT | reg;
    _i2cWrite(_i2cAddress, 1, _i2cBuffer);
    _i2cRead(_i2cAddress, 1, _i2cBuffer);
    return _i2cBuffer[0];
}

uint16_t TCS3471::read16(uint8_t reg)
{
    _i2cBuffer[0] = TCS3471_COMMAND_BIT | TCS3471_AUTOINCR_BIT | reg;
    _i2cWrite(_i2cAddress, 1, _i2cBuffer);
    _i2cRead(_i2cAddress, 2, _i2cBuffer);
    uint16_t ret = _i2cBuffer[1] << 8;
	ret |= _i2cBuffer[0];
    return ret;
}
