#include "I2C_SW.h"
#include "main.h"
#include "DS1307_CONTROL.h"

#define DS1307_W_ADDR	0xD0
#define DS1307_R_ADDR	0xD1

uint8_t DS1307_ReadByte(uint8_t u8Addr)
{
	I2C_SW_Start();
	I2C_SW_WriteByte(DS1307_W_ADDR);
	I2C_SW_WriteByte(u8Addr);
	I2C_SW_Start();
	I2C_SW_WriteByte(DS1307_R_ADDR);
	uint8_t dataRead = I2C_SW_ReadByte();
	I2C_SW_Stop();
	
	return dataRead;
}
void DS1307_WriteByte(uint8_t u8Addr, uint8_t u8Data)
{
	I2C_SW_Start();
	I2C_SW_WriteByte(DS1307_W_ADDR);
	I2C_SW_WriteByte(u8Addr);
	I2C_SW_WriteByte(u8Data);
	I2C_SW_Stop();
}

uint8_t convert_deciTime_to_clockTime(uint8_t dec_time)
{
	uint8_t Clock_time_format = 0;
	if(dec_time>9)
	{
		Clock_time_format = (dec_time/10)<<4 | (dec_time%10);
	}
	else
	{
		Clock_time_format = dec_time;
	}
	return Clock_time_format ;
}
uint8_t convert_clockTime_to_deciTime(uint8_t clock_time)
{
	uint8_t Decimal_time = 0;
	Decimal_time = ((clock_time&0XF0)>>4)*10+(clock_time&0X0F);
	return Decimal_time;
}
uint8_t DS1307_getSec(void)
{
	uint8_t sec = DS1307_ReadByte(0x0);
	sec = convert_clockTime_to_deciTime(sec);
	return sec;
}

uint8_t DS1307_getMin(void)
{
	uint8_t min = DS1307_ReadByte(0X01);
	min = convert_clockTime_to_deciTime(min);
	return min;
}
 
void DS1307_setMin(uint8_t time)
{
	uint8_t t = convert_deciTime_to_clockTime(time);
	DS1307_WriteByte(0X01,t);
}
uint8_t DS1307_getHour(void)
{
	uint8_t hour = DS1307_ReadByte(0X02);
	if((hour&0X40) == 0x40)
	{
		if((hour&0x20)==0x20)
		{
			hour+=12;
		}
		hour &=0x3F;
		hour += convert_clockTime_to_deciTime(hour);
		if(hour==24) hour=0;
	}
	else
	{
		hour &=0x3F;
		hour = convert_clockTime_to_deciTime(hour);
	}
	return hour;
}
void DS1307_setHour(uint8_t time)
{
	uint8_t t = convert_deciTime_to_clockTime(time);
	DS1307_WriteByte(0x02,t);
}
