#ifndef __I2C__
#define __I2C__

#include "stm32f3xx_hal.h"

#define I2C_SCL_H  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define I2C_SDA_H  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)
#define I2C_SCL_L  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)
#define I2C_SDA_L  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)
#define READ_SDA   HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
unsigned char I2C_Wait_Ack(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_Send_Byte(unsigned char txd);
unsigned char I2C_Read_Byte(void);
void Delay_us(uint32_t udelay);

#endif
