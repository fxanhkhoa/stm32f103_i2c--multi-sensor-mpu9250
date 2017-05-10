#include "MPU9250.h"

/******************************************************************
*											Default Range
******************************************************************/
char currentGyroRange = 3,
	  currentAcceleroRange = 3,
		mscale = 1;

void MPU9250_I2C_Init(int ADO_bit)
{
	if (ADO_bit == 1)
	{
		
	}
	else
	{
		
	}
}

void Read_data_buffer(u8 Device_address, u8 Register_address,u8 *pBuffer,  u16 Byte_Length)
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
void Write_Byte(u8 Device_address, u8 Register_address, u8 Value)
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
	Read_data_buffer(Device_address, Register_address, Check,  1);
	if (Check[0] == Return_true_val) return 1;
	else return 0;
	//delete Check;
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
	//Write_Byte(I2C_MST_CTRL, MPU9250_ADDRESS_DEFAULT, 0x0D);
	//Write_Byte(USER_CTRL, MPU9250_ADDRESS_DEFAULT, 0x20);
	/************* Set up Pass-through mode *****************/
	//Write_Byte(INT_PIN_CFG, address, 0x22);
	Write_Byte(address, I2C_MST_CTRL, 0x00);
	/**************/
	Write_Byte(address, SMPLRT_DIV, 0x04);
  Write_Byte(address, CONFIG, 0x03);
	u8 *c = new u8[1];
	Set_Gyro_Range(ado_bit);
	Set_Accel_Range(ado_bit);
	delete c;
}
/***********************************************************************************
*
*
***********************************************************************************/
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  Write_Byte(MPU9250_ADDRESS_DEFAULT, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(1000);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  Write_Byte(MPU9250_ADDRESS_DEFAULT, PWR_MGMT_1, 0x01);  
  Write_Byte(MPU9250_ADDRESS_DEFAULT, PWR_MGMT_2, 0x00); 
  delay(2000);
  
// Configure device for bias calculation
  Write_Byte(MPU9250_ADDRESS_DEFAULT, INT_ENABLE, 0x00);   // Disable all interrupts
  Write_Byte(MPU9250_ADDRESS_DEFAULT, FIFO_EN, 0x00);      // Disable FIFO
  Write_Byte(MPU9250_ADDRESS_DEFAULT, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  Write_Byte(MPU9250_ADDRESS_DEFAULT, I2C_MST_CTRL, 0x00); // Disable I2C master
  Write_Byte(MPU9250_ADDRESS_DEFAULT, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  Write_Byte(MPU9250_ADDRESS_DEFAULT, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(1500);
  
// Configure MPU9250 gyro and accelerometer for bias calculation
  Write_Byte(MPU9250_ADDRESS_DEFAULT, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  Write_Byte(MPU9250_ADDRESS_DEFAULT, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  Write_Byte(MPU9250_ADDRESS_DEFAULT, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  Write_Byte(MPU9250_ADDRESS_DEFAULT, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  Write_Byte(MPU9250_ADDRESS_DEFAULT, USER_CTRL, 0x40);   // Enable FIFO  
  Write_Byte(MPU9250_ADDRESS_DEFAULT, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  delay(4000); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  Write_Byte(MPU9250_ADDRESS_DEFAULT, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  Read_data_buffer(MPU9250_ADDRESS_DEFAULT, FIFO_COUNTH, data, 2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    Read_data_buffer(MPU9250_ADDRESS_DEFAULT, FIFO_R_W, data, 12); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  Read_data_buffer(MPU9250_ADDRESS_DEFAULT, XA_OFFSET_H,data, 2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  Read_data_buffer(MPU9250_ADDRESS_DEFAULT, YA_OFFSET_H,data, 2);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  Read_data_buffer(MPU9250_ADDRESS_DEFAULT, ZA_OFFSET_H,data, 2);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
/***********************************************************************************
*
*
***********************************************************************************/
void Initialize_AK8963(float *destination)
{
	u8 rawData[3];
	Write_Byte(AK8963_ADDRESS_DEFAULT, AK8963_CNTL,  0x00);
	//delay(10000);
	Write_Byte(AK8963_ADDRESS_DEFAULT, AK8963_CNTL,  0x0F);
	//delay(10000);
	Read_data_buffer(AK8963_ADDRESS_DEFAULT, AK8963_ASAX, rawData,  3);
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
	Write_Byte(AK8963_ADDRESS_DEFAULT, AK8963_CNTL,  0x00);
	//delay(10000);
	
	Write_Byte(AK8963_ADDRESS_DEFAULT, AK8963_CNTL,  (mscale << 4) | 0x02);
	//delay(1000);
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
	Write_Byte(address, PWR_MGMT_1, 0x00);
	delay(1000);
	Write_Byte(address, PWR_MGMT_1, 0x01);
	delay(1000);
}
/***********************************************************************************
*
*
***********************************************************************************/
void Set_Gyro_Range(int ado_bit)
{
	u8 address, *c = new u8[1];
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Byte(address, GYRO_CONFIG,  MPU9250_GYRO_FS_250);
	
	Read_data_buffer(address, GYRO_CONFIG, c,  1);
	c[0] &=  ~0x02; // Clear Fchoice bits [1:0] 
  c[0] &=  ~0x18; // Clear AFS bits [4:3]
	c[0] |= currentGyroRange << 3; // Set full scale range for the gyro	
	Write_Byte(address, GYRO_CONFIG,  c[0]);
	
	delete c;
}
/***********************************************************************************
*
*
***********************************************************************************/
void Set_Accel_Range(int ado_bit)
{
	u8 address, *c = new u8[1];
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Byte(address, ACCEL_CONFIG,  MPU9250_ACCEL_FS_2);
	
	Read_data_buffer(address, ACCEL_CONFIG, c,  1);
	c[0] &= ~0x18; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	c[0] |= currentAcceleroRange << 3; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	Write_Byte(address, ACCEL_CONFIG,  c[0]);
	
	Read_data_buffer(address, ACCEL_CONFIG2, c,  1);
	c[0] &= ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	c[0] |=  0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	Write_Byte(address, ACCEL_CONFIG2,  c[0]);
	
	delete c;
}
/***********************************************************************************
*
*
***********************************************************************************/
void Get_Mag(float *Mag)
{
	u8 *data = new u8[7];
	//u16 temp;
	//Write_Byte(AK8963_ADDRESS_DEFAULT, AK8963_CNTL, 0x01);
	Read_data_buffer(AK8963_ADDRESS_DEFAULT, AK8963_ST1, data,  1);
	if (data[0] & 0x01)
	{
		Read_data_buffer(AK8963_ADDRESS_DEFAULT, AK8963_XOUT_L, data,  7);
		if (!(data[6] & 0x08))
		{
			Mag[0] = float(int16_t(data[1] << 8) | data[0]);
			Mag[1] = float(int16_t(data[3] << 8) | data[2]);
			Mag[2] = float(int16_t(data[5] << 8) | data[4]);
		}
		switch (mscale)
		{
			case MFS_14BITS:
				Mag[0] *= 10.*4912./8190.;
				Mag[1] *= 10.*4912./8190.;
				Mag[2] *= 10.*4912./8190.;
			break;
			case MFS_16BITS:
				Mag[0] *= 10.*4912./32760.0;
				Mag[1] *= 10.*4912./32760.0;
				Mag[2] *= 10.*4912./32760.0;
			break;
		}
	}
	delete data;
}
/***********************************************************************************
*
*
***********************************************************************************/
void reset()
{
	Write_Byte(MPU9250_ADDRESS_DEFAULT, PWR_MGMT_1,  0x80);
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
	Read_data_buffer(MPU9250_ADDRESS_DEFAULT, ACCEL_XOUT_H, data ,  14);
	/**************** Calculate into AccelGyro **********************/
		/*-------- Accelero ---------*/
			AccelGyro[0] = int16_t((data[0] << 8) + data[1]); // Accel_X
			AccelGyro[1] = int16_t((data[2] << 8) + data[3]); // Accel_Y
			AccelGyro[2] = int16_t((data[4] << 8) + data[5]); // Accel_Z
		/*-------- Gyrometer --------*/
			AccelGyro[3] = (data[8] << 8) + data[9]; // Gyro_X
			AccelGyro[4] = (data[10] << 8) + data[11]; // Gyro_X
			AccelGyro[5] = (data[12] << 8) + data[13]; // Gyro_X
	/**************************************/
	/*---------ACCELERO--------*/
		if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_2G) {
        AccelGyro[0]=(float) AccelGyro[0] / 16384.0 * 9.81;
        AccelGyro[1]=(float) AccelGyro[1] / 16384.0 * 9.81;
        AccelGyro[2]=(float) AccelGyro[2] / 16384.0 * 9.81;
        }
    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_4G){
        AccelGyro[0]=(float) AccelGyro[0] / 8192.0 * 9.81;
        AccelGyro[1]=(float) AccelGyro[1] / 8192.0 * 9.81;
        AccelGyro[2]=(float) AccelGyro[2] / 8192.0 * 9.81;
        }
    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_8G){
        AccelGyro[0]=(float) AccelGyro[0] / 4096.0 * 9.81;
        AccelGyro[1]=(float) AccelGyro[1] / 4096.0 * 9.81;
        AccelGyro[2]=(float) AccelGyro[2] / 4096.0 * 9.81;
        }
    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_16G){
        AccelGyro[0]=(float) AccelGyro[0] / 2048.0 * 9.81;
        AccelGyro[1]=(float) AccelGyro[1] / 2048.0 * 9.81;
        AccelGyro[2]=(float) AccelGyro[2] / 2048.0 * 9.81;
        }
				AccelGyro[0]*=2;
        AccelGyro[1]*=2;   
        AccelGyro[2]*=2;
		/*-----------GYRO------------*/
		if (currentGyroRange == MPU9250_GYRO_RANGE_250) {
        AccelGyro[3]=(float)AccelGyro[3] / 7505.7;
        AccelGyro[4]=(float)AccelGyro[4] / 7505.7;
        AccelGyro[5]=(float)AccelGyro[5] / 7505.7;
        }
    if (currentGyroRange == MPU9250_GYRO_RANGE_500){
        AccelGyro[3]=(float)AccelGyro[3] / 3752.9;
        AccelGyro[4]=(float)AccelGyro[4] / 3752.9;
        AccelGyro[5]=(float)AccelGyro[5] / 3752.9;
        }
    if (currentGyroRange == MPU9250_GYRO_RANGE_1000){
        AccelGyro[3]=(float)AccelGyro[3] / 1879.3;;
        AccelGyro[4]=(float)AccelGyro[4] / 1879.3;
        AccelGyro[5]=(float)AccelGyro[5] / 1879.3;
        }
    if (currentGyroRange == MPU9250_GYRO_RANGE_2000){
        AccelGyro[3]=(float)AccelGyro[3] / 939.7;
        AccelGyro[4]=(float)AccelGyro[4] / 939.7;
        AccelGyro[5]=(float)AccelGyro[5] / 939.7;
        }
		delete data;
}
/***********************************************************************************
*
*
***********************************************************************************/
void AK8963_Calibrate(float *MCali)
{
	u8 *data = new u8[3];
	Write_Byte(AK8963_ADDRESS_DEFAULT, AK8963_CNTL,  0x1F);
	delay(10000);
	Read_data_buffer(AK8963_ADDRESS_DEFAULT, AK8963_ASAX, data,  3);
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
	int16_t temp;
	u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Read_data_buffer(address, ACCEL_XOUT_H,data ,  6);
	temp = int16_t((data[0] << 8) | data[1]);
	Accel[0] = float(temp);
	
	temp = int16_t((data[2] << 8) | data[3]);
	Accel[1] = float(temp);
	
	temp = int16_t((data[4] << 8) | data[5]);
	Accel[2] = float(temp);
	
	/*---------ACCELERO--------*/
		if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_2G) {
        Accel[0] *= 2.0/32768.0;
        Accel[1] *= 2.0/32768.0;
        Accel[2] *= 2.0/32768.0;
        }
    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_4G){
        Accel[0] *= 4.0/32768.0;
        Accel[1] *= 4.0/32768.0;
        Accel[2] *= 4.0/32768.0;
        }
    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_8G){
        Accel[0] *= 8.0/32768.0;
        Accel[1] *= 8.0/32768.0;
        Accel[2] *= 8.0/32768.0;
        }
    if (currentAcceleroRange == MPU9250_ACCELERO_RANGE_16G){
        Accel[0] *= 16.0/32768.0;
        Accel[1] *= 16.0/32768.0;
        Accel[2] *= 16.0/32768.0;
        }
	
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
	int16_t temp;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Read_data_buffer(address, GYRO_XOUT_H, data,  6);
	temp = int16_t((data[0] << 8) | data[1]);
	Gyro[0] = float(temp);
	
	temp = int16_t((data[2] << 8) | data[3]);
	Gyro[1] = float(temp);
	
	temp = int16_t((data[4] << 8) | data[5]);
	Gyro[2] = float(temp);
	
	/*-----------GYRO------------*/
		if (currentGyroRange == MPU9250_GYRO_RANGE_250) {
        Gyro[0] *= 250.0/32768.0;
        Gyro[1] *= 250.0/32768.0;
        Gyro[2] *= 250.0/32768.0;
        }
    if (currentGyroRange == MPU9250_GYRO_RANGE_500){
        Gyro[0] *= 500.0/32768.0;
        Gyro[1] *= 500.0/32768.0;
        Gyro[2] *= 500.0/32768.0;
        }
    if (currentGyroRange == MPU9250_GYRO_RANGE_1000){
        Gyro[0] *= 1000.0/32768.0;
        Gyro[1] *= 1000.0/32768.0;
        Gyro[2] *= 1000.0/32768.0;
        }
    if (currentGyroRange == MPU9250_GYRO_RANGE_2000){
        Gyro[0] *= 2000.0/32768.0;
        Gyro[1] *= 2000.0/32768.0;
        Gyro[2] *= 2000.0/32768.0;
        }
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
	Write_Byte(address, INT_PIN_CFG,  0x00);
	//Write_Byte(I2C_MST_CTRL, address, 0x00);
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
	Write_Byte(address, INT_PIN_CFG,  0x22);
	//Write_Byte(I2C_MST_CTRL, address, 0x00);
}

void MPU9250_Reset(int ado_bit) 
{
  // reset device
  u8 address;
	if (ado_bit == 0) address = MPU9250_ADDRESS_DEFAULT;
	else address = MPU9250_ADDRESS_DEFAULT_ADO_HIGH;
	Write_Byte(address, PWR_MGMT_1,  0x80);
  delay(10000);
 }
