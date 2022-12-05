#ifndef I2C_H
#define I2C_H
	#ifndef __STM32L476xx_H
	#include "stm32l412xx.h"
	#endif
void I2C_init(void);
void I2C_Start(I2C_TypeDef * I2Cx, uint32_t DevAddress, uint8_t Size, uint8_t direction);
void I2C_Stop(I2C_TypeDef * I2Cx);
void I2C_WaitLineIdle(I2C_TypeDef * I2Cx);
int8_t I2C_SendData(I2C_TypeDef *I2Cx, uint8_t SlaveAddress, uint8_t *pData, uint8_t Size);
int8_t I2C_ReceiveData(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t *pData, uint8_t Size);

#endif
