#include "stm32f10x.h"                  // Device header
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "Delay.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "Serial.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "bsp_i2c.h"

#define CHANGE_SLAVE(n) AD0 = n;\
						st.hw->addr = (0x68 | n);\

uint8_t AD0 = 0;								//定义用于存放AD0电平的变量
extern struct gyro_state_s st;


uint8_t ID, ID2;								//定义用于存放ID号的变量
int16_t AX, AY, AZ, GX, GY, GZ;			//定义用于存放各个数据的变量
double pitch, roll, yaw;
long q[4];

int main(void)
{
	int res;

	Serial_Init();		//串口初始化
	Serial_Printf("System Start\n");
	MyI2C_Init();		//先初始化底层的I2C
	// I2C_Bus_Init();		//初始化I2C总线

	/*模块初始化*/
	CHANGE_SLAVE(0);
	MPU6050_Init();		//MPU6050 slave0初始化
	res = mpu_dmp_init();
	Serial_Printf("MPU6050 slave 0 init: %d\n", res);

	CHANGE_SLAVE(1);
	MPU6050_Init();		//MPU6050 slave1初始化
	res = mpu_dmp_init();
	Serial_Printf("MPU6050 slave 1 init: %d\n", res);

	/*显示ID号*/
	// ID = MPU6050_GetID();				//获取MPU6050的ID号
	// ID1 = MPU6050_GetID_Pin(M1.SCL_Pin, M1.SDA_Pin, M1.GPIOx);
	// ID2 = MPU6050_GetID_Pin(M2.SCL_Pin, M2.SDA_Pin, M2.GPIOx);
	
	while (1)
	{
		char buf1[256], buf2[256];
		char buf3[256], buf4[256];

		/* Slave 0 */
		CHANGE_SLAVE(0);
		ID = MPU6050_GetID();				//获取MPU6050的ID号
		// MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);	//获取MPU6050的数据
		// sprintf(buf1, "MPU1 0x%x: %d %d %d %d %d %d", ID, AX, AY, AZ, GX, GY, GZ);

		// mpu_reset_fifo();
		// Delay_ms(5);
		while(res = mpu_dmp_get_data(&pitch, &roll, &yaw));
		sprintf(buf3, "MPU1=%d: %lf %lf %lf, ", res, pitch, roll, yaw);

		/* Slave 1 */
		CHANGE_SLAVE(1);
		ID = MPU6050_GetID();				//获取MPU6050的ID号
		// MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		// sprintf(buf2, "MPU2 0x%x: %d %d %d %d %d %d", ID, AX, AY, AZ, GX, GY, GZ);

		// mpu_reset_fifo();
		// Delay_ms(5);
		while(res = mpu_dmp_get_data(&pitch, &roll, &yaw));
		sprintf(buf4, "MPU2=%d: %lf %lf %lf", res, pitch, roll, yaw);

		// sprintf(buf1, "%s %s\n", buf1, buf2);
		sprintf(buf3, "%s %s\n", buf3, buf4);
		// Serial_SendString(buf1);	//发送数据到串口
		Serial_SendString(buf3);	//发送数据到串口

		// Delay_ms(10);		
	}
}
