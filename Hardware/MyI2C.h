#ifndef __MYI2C_H
#define __MYI2C_H

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
int MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

void MyI2C_W_SDA(uint8_t BitValue);
void MyI2C_W_SCL(uint8_t BitValue);

void MyI2C_Init_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
void MyI2C_Start_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
void MyI2C_Stop_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
void MyI2C_SendByte_Pin(uint8_t Byte, uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
uint8_t MyI2C_ReceiveByte_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
void MyI2C_SendAck_Pin(uint8_t AckBit, uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);
uint8_t MyI2C_ReceiveAck_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx);

#endif
