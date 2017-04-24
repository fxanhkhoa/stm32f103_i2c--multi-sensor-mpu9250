#include "MPU9250.h"

/******************************************************************
*											Default Range
******************************************************************/
int currentGyroRange = 3,
	 currentAcceleroRange = 3;

void MPU9250_I2C_Init(int ADO_bit)
{
	if (ADO_bit == 1)
	{
		
	}
	else
	{
		
	}
}

void Read_data_buffer(u8 Register_address,u8 *pBuffer, u8 Device_address, u16 Byte_Length)
{
	/* While the bus is busy */
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY));
	
	/* Generate Start */
	I2C_GenerateSTART(MPU9250_I2C, ENABLE);
	
	/* Check if started */
	while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));
	
	/* Send 7bit address with transmitter bit */
	I2C_Send7bitAddress(MPU9250_I2C, Device_address, I2C_Direction_Transmitter); 
	
	/* Check if sent 7 bit address */
	while(!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(MPU9250_I2C, ENABLE);
	
	/* Send register address to read from */
	I2C_SendData(MPU9250_I2C, Register_address);
	
	/* Check if sent reg address */
	while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	/* Generate Start again */
	I2C_GenerateSTART(MPU9250_I2C, ENABLE);
	
	/* Check if started */
	while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));
	
	/* Send 7 bit address with Receiver bit */
	I2C_Send7bitAddress(MPU9250_I2C, Device_address, I2C_Direction_Receiver);
	
	/* Check if sent 7 bit address */
	while(!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

//	for (u8 i = 0; i < Byte_Length; i++)
//	{
//		while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
//		
//		/* Get data */
//		pBuffer[i] = I2C_ReceiveData(MPU9250_I2C);
//		
//	}
	while (Byte_Length)
	{
		if (Byte_Length == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
		}
		
		if (I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* Read a byte from the MPU9250 */
			*pBuffer = I2C_ReceiveData(MPU9250_I2C);
			
			/* Point to the next location where the byte read will be saved */
			pBuffer++;
			
			/* Decrement the read bytes counter */
			Byte_Length--;
		}
	}
	
	/* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Write_Bytes(u8 Register_address, u8 Device_address, u8 Value)
{
	/* While the bus is busy */
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY));
	
	/* Generate Start */
	I2C_GenerateSTART(MPU9250_I2C, ENABLE);
	
	/* Check if started */
	while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));
	
	/* Send 7bit address with transmitter bit */
	I2C_Send7bitAddress(MPU9250_I2C, Device_address, I2C_Direction_Transmitter); 
	
	/* Check if sent 7 bit address */
	while(!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 
	
	/* Send Register address to write to */
	I2C_SendData(MPU9250_I2C, Register_address);
	
	/* Check Sent */
	while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	/* Send value */
	I2C_SendData(MPU9250_I2C, Value);
	
	/* Check Sent and clear */
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	/* Generate Stop */
	I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
}
/***********************************************************************************
*
*
***********************************************************************************/
int Check_Connection(u8 Register_address, u8 Device_address, u8 Return_true_val)
{
	u8 *Check = new u8[1];
	Read_data_buffer(Register_address, Check, Device_address, 1);
	if (Check[0] == Return_true_val) return 1;
	else return 0;
}

/***********************************************************************************
*
*
***********************************************************************************/
void Initialize(int ado_bit)
{
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Set_Clock(ado_bit); // include stop sleep mode
	/*************test******************/
	//Write_Bytes(I2C_MST_CTRL, MPU9250_ADDRESS_DEFAULT, 0x0D);
	//Write_Bytes(USER_CTRL, MPU9250_ADDRESS_DEFAULT, 0x20);
	/************* Set up Pass-through mode *****************/
	//Write_Bytes(INT_PIN_CFG, address, 0x22);
	Write_Bytes(I2C_MST_CTRL, address, 0x00);
	/**************/
	Write_Bytes(SMPLRT_DIV, address, 0x04);
  Write_Bytes(CONFIG, address, 0x03);
	u8 *c = new u8[1];
	//Write_Bytes(USER_CTRL, MPU9250_ADDRESS_DEFAULT, 0x20);
	Read_data_buffer(ACCEL_CONFIG2,c , address, 1);
	Write_Bytes(ACCEL_CONFIG2, address,(c[0] & ~0x0F));
	Write_Bytes(ACCEL_CONFIG2, address,(c[0] | 0x03));
	Set_Gyro_Range(ado_bit);
	Set_Accel_Range(ado_bit);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Initialize_AK8963()
{
	Write_Bytes(AK8963_CNTL, AK8963_ADDRESS_DEFAULT, 0x00);
	delay(10000);
	Write_Bytes(AK8963_CNTL, AK8963_ADDRESS_DEFAULT, 0x16);
	delay(10000);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Set_Clock(int ado_bit)
{
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Bytes(PWR_MGMT_1, address, 0x00);
	delay(10000);
	Write_Bytes(PWR_MGMT_1, address, 0x02);
	delay(10000);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Set_Gyro_Range(int ado_bit)
{
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Bytes(GYRO_CONFIG, address, MPU9250_GYRO_FS_250);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Set_Accel_Range(int ado_bit)
{
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Bytes(ACCEL_CONFIG, address, MPU9250_ACCEL_FS_2);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Get_Mag(float *Mag)
{
	u8 *data = new u8[7];
	
	Read_data_buffer(AK8963_ST1, data, AK8963_ADDRESS_DEFAULT, 1);
	if (data[0] & 0x01)
	{
		Read_data_buffer(AK8963_XOUT_L, data, AK8963_ADDRESS_DEFAULT, 7);
		if (!(data[6] & 0x08))
		{
			Mag[0] = float(data[0] + (data[1] << 8));
			Mag[1] = float(data[2] + (data[3] << 8));
			Mag[2] = float(data[4] + (data[5] << 8));
		}
		Mag[0] *= 10.*4912./32760.0;
		Mag[1] *= 10.*4912./32760.0;
		Mag[2] *= 10.*4912./32760.0;
	}
	delete data;
}
/***********************************************************************************
*
*
***********************************************************************************/
void reset()
{
	Write_Bytes(PWR_MGMT_1, MPU9250_ADDRESS_DEFAULT, 0x80);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Raw_Accel_Gyro(float AccelGyro[])
{
//	/*---- Re_Init ----*/
//			I2C_InitTypeDef I2C_InitStructure;
//			I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//			I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//			I2C_InitStructure.I2C_OwnAddress1 = 0x00; // MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
//			I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//			I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//			I2C_InitStructure.I2C_ClockSpeed = MPU9250_I2C_Speed;
//			/* Apply I2C configuration after enabling it */
//				I2C_Init(MPU9250_I2C, &I2C_InitStructure);
//			/* I2C Peripheral Enable */
//				I2C_Cmd(MPU9250_I2C, ENABLE);
	
	u8 *data = new u8[14];
	//u8 data[14];
	Read_data_buffer( ACCEL_XOUT_H, data , MPU9250_ADDRESS_DEFAULT, 14);
	/**************** Calculate into AccelGyro **********************/
		/*-------- Accelero ---------*/
			AccelGyro[0] = (data[0] << 8) + data[1]; // Accel_X
			AccelGyro[1] = (data[2] << 8) + data[3]; // Accel_Y
			AccelGyro[2] = (data[4] << 8) + data[5]; // Accel_Z
		/*-------- Gyrometer --------*/
			AccelGyro[3] = (data[8] << 8) + data[9]; // Gyro_X
			AccelGyro[4] = (data[10] << 8) + data[11]; // Gyro_X
			AccelGyro[5] = (data[12] << 8) + data[13]; // Gyro_X
	/**************************************/
//	/*---------ACCELERO--------*/
//		if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_2G) {
//        AccelGyro[0]=(float) AccelGyro[0] / 16384.0 * 9.81;
//        AccelGyro[1]=(float) AccelGyro[1] / 16384.0 * 9.81;
//        AccelGyro[2]=(float) AccelGyro[2] / 16384.0 * 9.81;
//        }
//    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_4G){
//        AccelGyro[0]=(float) AccelGyro[0] / 8192.0 * 9.81;
//        AccelGyro[1]=(float) AccelGyro[1] / 8192.0 * 9.81;
//        AccelGyro[2]=(float) AccelGyro[2] / 8192.0 * 9.81;
//        }
//    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_8G){
//        AccelGyro[0]=(float) AccelGyro[0] / 4096.0 * 9.81;
//        AccelGyro[1]=(float) AccelGyro[1] / 4096.0 * 9.81;
//        AccelGyro[2]=(float) AccelGyro[2] / 4096.0 * 9.81;
//        }
//    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_16G){
//        AccelGyro[0]=(float) AccelGyro[0] / 2048.0 * 9.81;
//        AccelGyro[1]=(float) AccelGyro[1] / 2048.0 * 9.81;
//        AccelGyro[2]=(float) AccelGyro[2] / 2048.0 * 9.81;
//        }
//				AccelGyro[0]*=2;
//        AccelGyro[1]*=2;   
//        AccelGyro[2]*=2;
//		/*-----------GYRO------------*/
//		if (currentGyroRange == MPU9250_GYRO_RANGE_250) {
//        AccelGyro[3]=(float)AccelGyro[3] / 7505.7;
//        AccelGyro[4]=(float)AccelGyro[4] / 7505.7;
//        AccelGyro[5]=(float)AccelGyro[5] / 7505.7;
//        }
//    if (currentGyroRange == MPU9250_GYRO_RANGE_500){
//        AccelGyro[3]=(float)AccelGyro[3] / 3752.9;
//        AccelGyro[4]=(float)AccelGyro[4] / 3752.9;
//        AccelGyro[5]=(float)AccelGyro[5] / 3752.9;
//        }
//    if (currentGyroRange == MPU9250_GYRO_RANGE_1000){
//        AccelGyro[3]=(float)AccelGyro[3] / 1879.3;;
//        AccelGyro[4]=(float)AccelGyro[4] / 1879.3;
//        AccelGyro[5]=(float)AccelGyro[5] / 1879.3;
//        }
//    if (currentGyroRange == MPU9250_GYRO_RANGE_2000){
//        AccelGyro[3]=(float)AccelGyro[3] / 939.7;
//        AccelGyro[4]=(float)AccelGyro[4] / 939.7;
//        AccelGyro[5]=(float)AccelGyro[5] / 939.7;
//        }
		delete data;
}
/***********************************************************************************
*
*
***********************************************************************************/
void AK8963_Calibrate(float *MCali)
{
	u8 *data = new u8[3];
	Write_Bytes(AK8963_CNTL, AK8963_ADDRESS_DEFAULT, 0x1F);
	delay(10000);
	Read_data_buffer(AK8963_ASAX, data, AK8963_ADDRESS_DEFAULT, 3);
	MCali[0] = (float)(data[0] - 128)/256. +1.;
	MCali[1] = (float)(data[1] - 128)/256. +1.;
	MCali[2] = (float)(data[2] - 128)/256. +1.;
	delete data;
}
/***********************************************************************************
*
*
***********************************************************************************/
void Get_Accel(float *Accel, int ado_bit)
{
	u8 *data = new u8[6];
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Read_data_buffer(ACCEL_XOUT_H,data , address, 6);
	Accel[0] = (data[0] << 8) | data[1];
	Accel[1] = (data[2] << 8) | data[3];
	Accel[2] = (data[4] << 8) | data[5];
	delete data;
}
/***********************************************************************************
*
*
***********************************************************************************/
void Get_Gyro(float *Gyro, int ado_bit)
{
	u8 *data = new u8[6];
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Read_data_buffer(GYRO_XOUT_H, data, address, 6);
	Gyro[0] = (data[0] << 8) + data[1];
	Gyro[1] = (data[2] << 8) + data[3];
	Gyro[2] = (data[4] << 8) + data[5];
	delete data;
}
/***********************************************************************************
*
*
***********************************************************************************/
void delay(u32 k)
{
	while (k != 0)
	{
		k--;
	}
}
/***********************************************************************************
*
*
***********************************************************************************/
void AK8963_turn_off(int ado_bit)
{
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Bytes(INT_PIN_CFG, address, 0x00);
	//Write_Bytes(I2C_MST_CTRL, address, 0x00);
}

/***********************************************************************************
*
*
***********************************************************************************/
void AK8963_turn_on(int ado_bit)
{
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Bytes(INT_PIN_CFG, address, 0x22);
	//Write_Bytes(I2C_MST_CTRL, address, 0x00);
}
