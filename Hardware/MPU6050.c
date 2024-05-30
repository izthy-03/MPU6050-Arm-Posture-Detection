#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Serial.h"
#include <math.h>

#define MPU6050_ADDRESS		(0xD0 + (AD0 << 1))		//MPU6050的I2C从机地址
#define q30 1073741824.0f


extern uint8_t AD0;

/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
}

void MPU6050_WriteReg_Pin(uint8_t RegAddress, uint8_t Data, uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx)
{
	MyI2C_Start_Pin(SCL_Pin, SDA_Pin, GPIOx);						
	MyI2C_SendByte_Pin(MPU6050_ADDRESS, SCL_Pin, SDA_Pin, GPIOx);	
	MyI2C_ReceiveAck_Pin(SCL_Pin, SDA_Pin, GPIOx);					
	MyI2C_SendByte_Pin(RegAddress, SCL_Pin, SDA_Pin, GPIOx);		
	MyI2C_ReceiveAck_Pin(SCL_Pin, SDA_Pin, GPIOx);					
	MyI2C_SendByte_Pin(Data, SCL_Pin, SDA_Pin, GPIOx);				
	MyI2C_ReceiveAck_Pin(SCL_Pin, SDA_Pin, GPIOx);					
	MyI2C_Stop_Pin(SCL_Pin, SDA_Pin, GPIOx);						
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();						//I2C终止
	
	return Data;
}

uint8_t MPU6050_ReadReg_Pin(uint8_t RegAddress, uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx)
{
	uint8_t Data;
	
	MyI2C_Start_Pin(SCL_Pin, SDA_Pin, GPIOx);						
	MyI2C_SendByte_Pin(MPU6050_ADDRESS, SCL_Pin, SDA_Pin, GPIOx);	
	MyI2C_ReceiveAck_Pin(SCL_Pin, SDA_Pin, GPIOx);					
	MyI2C_SendByte_Pin(RegAddress, SCL_Pin, SDA_Pin, GPIOx);		
	MyI2C_ReceiveAck_Pin(SCL_Pin, SDA_Pin, GPIOx);					
	
	MyI2C_Start_Pin(SCL_Pin, SDA_Pin, GPIOx);						
	MyI2C_SendByte_Pin(MPU6050_ADDRESS | 0x01, SCL_Pin, SDA_Pin, GPIOx);	
	MyI2C_ReceiveAck_Pin(SCL_Pin, SDA_Pin, GPIOx);					
	Data = MyI2C_ReceiveByte_Pin(SCL_Pin, SDA_Pin, GPIOx);			
	MyI2C_SendAck_Pin(1, SCL_Pin, SDA_Pin, GPIOx);					
	MyI2C_Stop_Pin(SCL_Pin, SDA_Pin, GPIOx);						
	
	return Data;
}

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
	// MyI2C_Init();									//先初始化底层的I2C
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x8);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x8);	//加速度计配置寄存器，选择满量程为±16g
}

void MPU6050_Init_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx)
{
	MyI2C_Init_Pin(SCL_Pin, SDA_Pin, GPIOx);								
	
	MPU6050_WriteReg_Pin(MPU6050_PWR_MGMT_1, 0x01, SCL_Pin, SDA_Pin, GPIOx);	
	MPU6050_WriteReg_Pin(MPU6050_PWR_MGMT_2, 0x00, SCL_Pin, SDA_Pin, GPIOx);	
	MPU6050_WriteReg_Pin(MPU6050_SMPLRT_DIV, 0x09, SCL_Pin, SDA_Pin, GPIOx);	
	MPU6050_WriteReg_Pin(MPU6050_CONFIG, 0x06, SCL_Pin, SDA_Pin, GPIOx);		
	MPU6050_WriteReg_Pin(MPU6050_GYRO_CONFIG, 0x18, SCL_Pin, SDA_Pin, GPIOx);	
	MPU6050_WriteReg_Pin(MPU6050_ACCEL_CONFIG, 0x18, SCL_Pin, SDA_Pin, GPIOx);	
}

/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

uint8_t MPU6050_GetID_Pin(uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx)
{
	return MPU6050_ReadReg_Pin(MPU6050_WHO_AM_I, SCL_Pin, SDA_Pin, GPIOx);	
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}

void MPU6050_GetData_Pin(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, 
						uint16_t SCL_Pin, uint16_t SDA_Pin, GPIO_TypeDef* GPIOx)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg_Pin(MPU6050_ACCEL_XOUT_H, SCL_Pin, SDA_Pin, GPIOx);		
	DataL = MPU6050_ReadReg_Pin(MPU6050_ACCEL_XOUT_L, SCL_Pin, SDA_Pin, GPIOx);		
	*AccX = (DataH << 8) | DataL;								
	
	DataH = MPU6050_ReadReg_Pin(MPU6050_ACCEL_YOUT_H, SCL_Pin, SDA_Pin, GPIOx);		
	DataL = MPU6050_ReadReg_Pin(MPU6050_ACCEL_YOUT_L, SCL_Pin, SDA_Pin, GPIOx);		
	*AccY = (DataH << 8) | DataL;								
	
	DataH = MPU6050_ReadReg_Pin(MPU6050_ACCEL_ZOUT_H, SCL_Pin, SDA_Pin, GPIOx);		
	DataL = MPU6050_ReadReg_Pin(MPU6050_ACCEL_ZOUT_L, SCL_Pin, SDA_Pin, GPIOx);		
	*AccZ = (DataH << 8) | DataL;								
	
	DataH = MPU6050_ReadReg_Pin(MPU6050_GYRO_XOUT_H, SCL_Pin, SDA_Pin, GPIOx);		
	DataL = MPU6050_ReadReg_Pin(MPU6050_GYRO_XOUT_L, SCL_Pin, SDA_Pin, GPIOx);		
	*GyroX = (DataH << 8) | DataL;								
	
	DataH = MPU6050_ReadReg_Pin(MPU6050_GYRO_YOUT_H, SCL_Pin, SDA_Pin, GPIOx);		
	DataL = MPU6050_ReadReg_Pin(MPU6050_GYRO_YOUT_L, SCL_Pin, SDA_Pin, GPIOx);		
	*GyroY = (DataH << 8) | DataL;

	DataH = MPU6050_ReadReg_Pin(MPU6050_GYRO_ZOUT_H, SCL_Pin, SDA_Pin, GPIOx);
	DataL = MPU6050_ReadReg_Pin(MPU6050_GYRO_ZOUT_L, SCL_Pin, SDA_Pin, GPIOx);
	*GyroZ = (DataH << 8) | DataL;
}


int MPU6050_I2C_Write_Buf(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(slave_addr);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(reg_addr);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	for (int i = 0; i < length; i++) {
		MyI2C_SendByte(data[i]);				//发送要写入寄存器的数据
		MyI2C_ReceiveAck();					//接收应答
	}
	MyI2C_Stop();						//I2C终止
	return 0;
}


int MPU6050_I2C_Read_Buf(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(slave_addr);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(reg_addr);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(slave_addr | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	for (int i = 0; i < length; i++) {
		data[i] = MyI2C_ReceiveByte();			//接收指定寄存器的数据
		if (i == length - 1) {
			MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
		} else {
			MyI2C_SendAck(0);					//发送应答，给从机应答，继续从机的数据输出
		}
	}
	MyI2C_Stop();						//I2C终止
	return 0;
}

