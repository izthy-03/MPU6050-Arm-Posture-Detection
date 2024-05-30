#include "stm32f10x.h"				  // Device header

#ifndef __MPU6050_H
#define __MPU6050_H

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

void MPU6050_WriteReg_Pin(uint8_t RegAddress, uint8_t Data, uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
uint8_t MPU6050_ReadReg_Pin(uint8_t RegAddress, uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);

void MPU6050_Init_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
uint8_t MPU6050_GetID_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
void MPU6050_GetData_Pin(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, 
						uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);

int MPU6050_I2C_Write_Buf(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int MPU6050_I2C_Read_Buf(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);


#endif
