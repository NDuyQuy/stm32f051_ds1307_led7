#ifndef _DS1307_CONTROL_H
#define _DS1307_CONTROL_H

#include "main.h"
#include "I2C_SW.h"

uint8_t DS1307_ReadByte(uint8_t u8Addr);
void DS1307_WriteByte(uint8_t u8Addr, uint8_t u8Data);
uint8_t DS1307_getSec(void);
uint8_t DS1307_getMin(void);
void DS1307_setMin(uint8_t time);
uint8_t DS1307_getHour(void);
void DS1307_setHour(uint8_t time);
#endif 