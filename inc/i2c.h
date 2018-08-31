#ifndef _I2C_H
#define _I2C_H

#include "stm8l15x.h"


void I2C_Setup(void);
uint8_t I2C_Start(void);
uint8_t I2C_Stop(void);
uint8_t I2C_Write_Byte(uint8_t data);
uint8_t I2C_Write_Address(uint8_t addr);
uint8_t I2C_Read_Byte(void);
void I2C_Read_Buffer(uint8_t *buf, uint16_t len);







#endif 
