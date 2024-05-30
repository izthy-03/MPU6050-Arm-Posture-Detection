/**
  ******************************************************************************
  * @file    bsp_i2c_ee.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   i2c mpu6050Ӧ�ú���bsp
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��Ե� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 


#include "bsp_i2c.h"
#include "bsp_SysTick.h"
#include "Serial.h"
#include "MyI2C.h"


#define Delay mdelay

/* STM32 I2C ����ģʽ */
#define I2C_Speed              400000  //*

/* �����ַֻҪ��STM32��ҵ�I2C������ַ��һ������ */
#define I2Cx_OWN_ADDRESS7      0X0A   


static __IO uint32_t  I2CTimeout = I2CT_LONG_TIMEOUT;    



/********************************* Prototypes *********************************/
unsigned long ST_Hard_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
unsigned long ST_Hard_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
/*******************************  Function ************************************/

//������������꣬ʹ��Ӳ��iic������ʹ������iic���������bsp_i2c.h�ļ�����
//ʹ��Ӳ��IICʱ������Һ�������ã���ΪFSMC��NADV��IIC1��SDA ��ͬһ�����ţ�����Ӱ����

#ifdef HARD_IIC

#define ST_Sensors_I2C_WriteRegister  ST_Hard_Sensors_I2C_WriteRegister
#define ST_Sensors_I2C_ReadRegister ST_Hard_Sensors_I2C_ReadRegister

static uint8_t I2C_TIMEOUT_UserCallback(uint8_t errorCode);


#else
//
#define Soft_I2C_SDA_STATE   	GPIO_ReadInputDataBit(Soft_I2C_PORT, Soft_I2C_SDA)
#define Soft_I2C_DELAY 				Soft_I2C_Delay(100000)
#define Soft_I2C_NOP					Soft_I2C_Delay(10) 
//
#define Soft_I2C_READY		0x00
#define Soft_I2C_BUS_BUSY	0x01	
#define Soft_I2C_BUS_ERROR	0x02
//
#define Soft_I2C_NACK	  0x00 
#define Soft_I2C_ACK		0x01


//
static void Soft_I2C_Configuration(void);
static void Soft_I2C_Delay(uint32_t dly);
static uint8_t Soft_I2C_START(void);
static void Soft_I2C_STOP(void);
static void Soft_I2C_SendACK(void);
static void Soft_I2C_SendNACK(void);
static uint8_t Soft_I2C_SendByte(uint8_t anbt_i2c_data);
static uint8_t Soft_I2C_ReceiveByte_WithACK(void);
static uint8_t Soft_I2C_ReceiveByte(void);
uint8_t Soft_DMP_I2C_Write(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf);
uint8_t Soft_DMP_I2C_Read(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf);


#define ST_Sensors_I2C_WriteRegister  Soft_DMP_I2C_Write
#define ST_Sensors_I2C_ReadRegister Soft_DMP_I2C_Read

#endif

#ifdef HARD_IIC
/**
  * @brief  I2C1 I/O����
  * @param  ��
  * @retval ��
  */
static void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 

	/* ʹ���� I2C1 �йص�ʱ�� */
	SENSORS_I2C_APBxClock_FUN ( SENSORS_I2C_CLK, ENABLE );
	SENSORS_I2C_GPIO_APBxClock_FUN ( SENSORS_I2C_GPIO_CLK, ENABLE );
	
    
  /* PB6-I2C1_SCL��PB7-I2C1_SDA*/
  GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // ��©���
  GPIO_Init(SENSORS_I2C_SCL_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // ��©���
  GPIO_Init(SENSORS_I2C_SDA_PORT, &GPIO_InitStructure);	
	
	
}


/**
  * @brief  I2C ����ģʽ����
  * @param  ��
  * @retval ��
  */
static void I2C_Mode_Configu(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /* I2C ���� */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	
	/* �ߵ�ƽ�����ȶ����͵�ƽ���ݱ仯 SCL ʱ���ߵ�ռ�ձ� */
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
  I2C_InitStructure.I2C_OwnAddress1 =I2Cx_OWN_ADDRESS7; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
	
	/* I2C��Ѱַģʽ */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	/* ͨ������ */
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
	/* I2C1 ��ʼ�� */
  I2C_Init(SENSORS_I2Cx, &I2C_InitStructure);
  
	/* ʹ�� I2C1 */
  I2C_Cmd(SENSORS_I2Cx, ENABLE);   
}



/**
  * @brief  ��IIC�豸�ļĴ�������д������
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫд�����ݵĳ���
  * @param  RegisterValue: Ҫָ��д�����ݵ�ָ��
  * @retval 0��������0�쳣
  */
unsigned long ST_Hard_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	 uint32_t i=0;
	
	 /* Send STRAT condition */
	  I2C_GenerateSTART(SENSORS_I2Cx, ENABLE);

		I2CTimeout = I2CT_FLAG_TIMEOUT;


	  /* Test on EV5 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	  {
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(0);
	  }

	  /* Send slave address for write */
	  I2C_Send7bitAddress(SENSORS_I2Cx, (Address<<1), I2C_Direction_Transmitter);

		I2CTimeout = I2CT_FLAG_TIMEOUT;
	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(1);
	  }

	  /* Send the slave's internal address to write to */
	  I2C_SendData(SENSORS_I2Cx, RegisterAddr);

		I2CTimeout = I2CT_FLAG_TIMEOUT;
	  /* Test on EV8 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(2);
	  }

	  /* Send the byte to be written */
	  for( i=0; i<(RegisterLen); i++)
	   {
		  	  I2CTimeout = I2CT_FLAG_TIMEOUT;
		  	  /* Test on EV8 and clear it */
		  	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		  		{
		  	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(3);
		  	  }

	     /* Prepare the register value to be sent */
	     I2C_SendData(SENSORS_I2Cx, RegisterValue[i]);
	   }

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV8 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(4);
	  }

	  /* Send STOP condition */
	  I2C_GenerateSTOP(SENSORS_I2Cx, ENABLE);

		return 0; //��������0
}


/**
  * @brief  ��IIC�豸�ļĴ���������������
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫ��ȡ�����ݳ���
  * @param  RegisterValue: ָ��洢�������ݵ�ָ��
  * @retval 0��������0�쳣
  */
unsigned long ST_Hard_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	 I2CTimeout = I2CT_LONG_TIMEOUT;

	  while(I2C_GetFlagStatus(SENSORS_I2Cx, I2C_FLAG_BUSY)) // Added by Najoua 27/08/2008
	  {
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(5);
	   }

	  I2C_GenerateSTART(SENSORS_I2Cx, ENABLE);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV5 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(6);
	   }

	  /* Send slave address for write */
	  I2C_Send7bitAddress(SENSORS_I2Cx, (Address<<1), I2C_Direction_Transmitter);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(7);
	   }

	  /* Clear EV6 by setting again the PE bit */
	  I2C_Cmd(SENSORS_I2Cx, ENABLE);

	  /* Send the slave's internal address to write to */
	  I2C_SendData(SENSORS_I2Cx, RegisterAddr);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV8 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(8);
	   }

	  /* Send STRAT condition a second time */
	  I2C_GenerateSTART(SENSORS_I2Cx, ENABLE);

		I2CTimeout = I2CT_FLAG_TIMEOUT;
	  /* Test on EV5 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(9);
	   }

	  /* Send slave address for read */
	  I2C_Send7bitAddress(SENSORS_I2Cx, (Address<<1), I2C_Direction_Receiver);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(10);
	   }

	  /* While there is data to be read */
	  while(RegisterLen)
	  {
	    if(RegisterLen == 1)
	    {
	      /* Disable Acknowledgement */
	      I2C_AcknowledgeConfig(SENSORS_I2Cx, DISABLE);

	      /* Send STOP Condition */
	      I2C_GenerateSTOP(SENSORS_I2Cx, ENABLE);
	    }

			I2CTimeout = I2CT_LONG_TIMEOUT;
			while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
			{
				if((I2CTimeout--) == 0) 
				{

						return I2C_TIMEOUT_UserCallback(10);
				}
			 }
			
	    {
	      /* Read a byte from the slave */
	      *RegisterValue = I2C_ReceiveData(SENSORS_I2Cx);

	      /* Point to the next location where the byte read will be saved */
	      RegisterValue++;

	      /* Decrement the read bytes counter */
	      RegisterLen--;
	    }

	  }

	  /* Enable Acknowledgement to be ready for another reception */
	  I2C_AcknowledgeConfig(SENSORS_I2Cx, ENABLE);

		return 0; //����������0
}






/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
static  uint8_t I2C_TIMEOUT_UserCallback(uint8_t errorCode)
{
	/*����IIC*/
  I2C_GenerateSTOP(SENSORS_I2Cx, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2Cx, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2Cx, DISABLE);
	
	I2C_Bus_Init();
	
  /* Block communication and all processes */
  MPU_ERROR("I2C Timeout error! error code = %d",errorCode);
  
  return 1;
}

#endif //endof #ifdef HARD_IIC



/**
  * @brief  I2C �����ʼ��
  * @param  ��
  * @retval ��
  */
void I2C_Bus_Init(void)
{	
	
	Set_I2C_Retry(5);
	
	#ifdef HARD_IIC
	MPU_DEBUG("hard iic");
	
  I2C_GPIO_Config(); 
 
  I2C_Mode_Configu();
	
	#else
	
  MPU_DEBUG("soft iic");
	Soft_I2C_Configuration();
	
	#endif

}






/**
  * @brief  ��IIC�豸�ļĴ�������д�����ݣ�����ʱ�������ã���mpu�ӿڵ���
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫд�����ݵĳ���
  * @param  RegisterValue: Ҫָ��д�����ݵ�ָ��
  * @retval 0��������0�쳣
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
  ret = 0;
  ret = ST_Sensors_I2C_WriteRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  return ret;
}


/**
  * @brief  ��IIC�豸�ļĴ���������������,����ʱ�������ã���mpu�ӿڵ���
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫ��ȡ�����ݳ���
  * @param  RegisterValue: ָ��洢�������ݵ�ָ��
  * @retval 0��������0�쳣
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
  ret = 0;
  ret = ST_Sensors_I2C_ReadRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec);
    goto tryReadAgain;
  }
  return ret;
}


static unsigned short RETRY_IN_MLSEC  = 55;

/**
  * @brief  ����iic����ʱ��
  * @param  ml_sec�����Ե�ʱ�䣬��λ����
  * @retval ���Ե�ʱ�䣬��λ����
  */
void Set_I2C_Retry(unsigned short ml_sec)
{
  RETRY_IN_MLSEC = ml_sec;
}

/**
  * @brief  ��ȡ���õ�iic����ʱ��
  * @param  none
  * @retval none
  */
unsigned short Get_I2C_Retry(void)
{
  return RETRY_IN_MLSEC;
}


/************************����IIC��������****************************************/

#ifndef HARD_IIC

static void Soft_I2C_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
	//
	//
  GPIO_InitStructure.GPIO_Pin = Soft_I2C_SCL | Soft_I2C_SDA;					//����ʹ�õ�I2C��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   //����I2C�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	  //����I2CΪ��©���
  GPIO_Init(Soft_I2C_PORT, &GPIO_InitStructure); 
	//
	//
	Soft_I2C_SCL_1; 
	Soft_I2C_SDA_1;
	Soft_I2C_DELAY; 
}

static void Soft_I2C_Delay(uint32_t dly) 
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

static uint8_t Soft_I2C_START(void)
{ 
	Soft_I2C_SDA_1; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	if(!Soft_I2C_SDA_STATE) return Soft_I2C_BUS_BUSY;
	//
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
  //
 	Soft_I2C_SCL_0;  
 	Soft_I2C_NOP; 
	//
 	if(Soft_I2C_SDA_STATE) return Soft_I2C_BUS_ERROR;
	//
 	return Soft_I2C_READY;
}

static void Soft_I2C_STOP(void)
{
 	Soft_I2C_SDA_0; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_NOP;
}

static void Soft_I2C_SendACK(void)
{
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_1;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_0; 
 	Soft_I2C_NOP;  
}

static void Soft_I2C_SendNACK(void)
{
	Soft_I2C_SDA_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_0; 
	Soft_I2C_NOP;  
}




/**
  * @brief  �ȴ�Ӧ���źŵ���
  * @retval ����ֵ��1������Ӧ��ʧ��
	*									0������Ӧ��ɹ�
  */
uint8_t Soft_I2C_Wait_Ack(void)
{
	uint8_t ucErrTime=0;

	Soft_I2C_SDA_1;
	Soft_I2C_NOP;	   
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;	 
	
	while(Soft_I2C_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Soft_I2C_STOP();
			return Soft_I2C_BUS_ERROR;
		}
	}
	Soft_I2C_SCL_0;//ʱ�����0 	   
	return 0;  
} 


static uint8_t Soft_I2C_SendByte(uint8_t soft_i2c_data)
{
 	uint8_t i;
 	
	Soft_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(soft_i2c_data&0x80) Soft_I2C_SDA_1;
   		else Soft_I2C_SDA_0;
			//
  		soft_i2c_data<<=1;
  		Soft_I2C_NOP;
			//
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP;
  		Soft_I2C_SCL_0;
  		Soft_I2C_NOP; 
 	}
	//
// 	Soft_I2C_SDA_1; 
// 	Soft_I2C_NOP;
// 	Soft_I2C_SCL_1;
// 	Soft_I2C_NOP;   
// 	if(Soft_I2C_SDA_STATE)
// 	{
//  		Soft_I2C_SCL_0;
//  		return Soft_I2C_NACK;
// 	}
// 	else
// 	{
//  		Soft_I2C_SCL_0;
//  		return Soft_I2C_ACK;  
// 	}  
	return Soft_I2C_Wait_Ack();  
}

static uint8_t Soft_I2C_ReceiveByte(void)
{
	uint8_t i,soft_i2c_data;
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_SCL_0; 
 	soft_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP; 
  		soft_i2c_data<<=1;
			//
  		if(Soft_I2C_SDA_STATE)	soft_i2c_data|=0x01; 
  
  		Soft_I2C_SCL_0;  
  		Soft_I2C_NOP;         
 	}
	Soft_I2C_SendNACK();
 	return soft_i2c_data;
}

static uint8_t Soft_I2C_ReceiveByte_WithACK(void)
{
	uint8_t i,soft_i2c_data;
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_SCL_0; 
 	soft_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP; 
  		soft_i2c_data<<=1;
			//
  		if(Soft_I2C_SDA_STATE)	soft_i2c_data|=0x01; 
  
  		Soft_I2C_SCL_0;  
  		Soft_I2C_NOP;         
 	}
	Soft_I2C_SendACK();
 	return soft_i2c_data;
}


//static void Soft_DMP_Delay_us(uint32_t dly)
//{
//	uint8_t i;
//	while(dly--) for(i=0;i<10;i++);
//}
////
//static void Soft_DMP_Delay_ms(uint32_t dly)
//{
//	while(dly--) Soft_DMP_Delay_us(1000);
//}
//

uint8_t Soft_DMP_I2C_Write(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf)
{		
		uint8_t i, result=0;
		Soft_I2C_START();
		result  = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Transmitter);	
		if(result != 0) return result;
	
		result = Soft_I2C_SendByte(soft_reg_addr);  
		if(result != 0) return result;
	
		for (i=0;i<soft_i2c_len;i++) 
		{
			result = Soft_I2C_SendByte(soft_i2c_data_buf[i]); 
			if (result != 0) return result;
		}
		Soft_I2C_STOP();
		return 0x00;
}

uint8_t Soft_DMP_I2C_Read(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf)
{
		uint8_t result;
// #define 	Soft_I2C_SendByte  MyI2C_SendByte
		Soft_I2C_START();
		result  = Soft_I2C_SendByte((soft_dev_addr << 1) | I2C_Direction_Transmitter);			
		if(result != 0) {
			// Serial_Printf("Soft_I2C_SendByte WRITE error at %d: %d, dev_addr=0x%x\n", __LINE__, result, soft_dev_addr);
			return result;
		}
		result = Soft_I2C_SendByte(soft_reg_addr); 
		if(result != 0) {
			// Serial_Printf("Soft_I2C_SendByte error at %d: %d, dev_addr=0x%x\n", __LINE__, result, soft_dev_addr);
			return result;
		}
		Soft_I2C_START();
		result = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Receiver);
		if(result != 0) {
			// Serial_Printf("Soft_I2C_SendByte READ at %d: %d, dev_addr=0x%x\n", __LINE__, result, soft_dev_addr);
			return result;
		}
		//
    while (soft_i2c_len)
		{
			if (soft_i2c_len==1) *soft_i2c_data_buf =Soft_I2C_ReceiveByte();  
      else *soft_i2c_data_buf =Soft_I2C_ReceiveByte_WithACK();
      soft_i2c_data_buf++;
      soft_i2c_len--;
    }
		Soft_I2C_STOP();
    return 0x00;
}

#endif //endof #ifndef HARD_IIC
/*********************************************END OF FILE**********************/

