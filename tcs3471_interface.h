
#ifndef tcs3471_interface__h
#define tcs3471_interface__h

#include <stdint.h>

typedef void * TCS3471Handle_t;

#define I2C_BUS 0

enum {
    GAIN_1X = 0x00,
    GAIN_4X = 0x01,
    GAIN_16X = 0x02,
    GAIN_60X = 0x03,
};
                

#ifdef __cplusplus
extern "C" {
#endif

TCS3471Handle_t tcs3471_create();
void tcs3471_delete(TCS3471Handle_t);
bool tcs3471_detect(TCS3471Handle_t);
uint8_t tcs3471_getChipID(TCS3471Handle_t);
void tcs3471_setIntegrationTime(TCS3471Handle_t, float); //old value: 700
void tcs3471_setWaitTime(TCS3471Handle_t, float);
void tcs3471_setGain(TCS3471Handle_t, uint8_t gain);
void tcs3471_enable(TCS3471Handle_t);
bool tcs3471_rgbcValid(TCS3471Handle_t);
uint16_t tcs3471_readCData(TCS3471Handle_t);
uint16_t tcs3471_readRData(TCS3471Handle_t);
uint16_t tcs3471_readGData(TCS3471Handle_t);
uint16_t tcs3471_readBData(TCS3471Handle_t);

#ifdef __cplusplus
}
#endif

#endif
