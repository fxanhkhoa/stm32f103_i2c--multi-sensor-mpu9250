#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "usart_print.h"	
#include "MPU9250.h"
#include "stm32f10x_tim.h"
#include "Main.h"
#include "MPU6050.h"
#include <math.h>
#include "Kalman.c"
#include "Kalman_New.h"


#define Rad2Dree       57.295779513082320876798154814105
#define PI	3.1415926535897932384626433832795
#define declinationAngle  0.007563
#define alpha  0.5

//static void prvSetupHardware( void );
void sleep(long i);
void led_toggle(void);
float Distance(float x, float y);
int Get_Central(kalman p, kalman r, kalman y);
void LowPass_Accel(Accel fAcc[]);
void LowPass_Gyro(Gyro fGyro[]);
void LowPass_Mag(Mag fMag[]);
void Print_Bias();
void Interrupt_Init();

uint16_t timer, time_pre =0, time_now = 0;
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias

	
int main()
{
	//prvSetupHardware();
	/* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(MPU9250_I2C_RCC_Periph, ENABLE);
    RCC_APB2PeriphClockCmd(MPU9250_I2C_RCC_Port, ENABLE);
/***********************************************************************************************
*											GPIO_Init						
***********************************************************************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	/* Set Led */
		GPIO_InitTypeDef GPIO_InitStruct,GPIO_InitStructure,GPIO;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		//GPIO_ResetBits(GPIOC, GPIO_Pin_13);

/*************************************************************************************
*																	USART_Init
**************************************************************************************/
			
	USART_InitTypeDef USART;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	USART.USART_BaudRate = 9600;
	USART.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART.USART_StopBits = USART_StopBits_1;
	USART.USART_WordLength = USART_WordLength_8b;
	USART.USART_Parity = USART_Parity_No;
	USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	/*---- Configure USART1 ----*/
			USART_Init(USART1, &USART);
	/*---- Enable RXNE interrupt ----*/
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	/*---- USART ENABLE ----*/
			USART_Cmd(USART1, ENABLE);
		/*------ TX-Pin PA9 & RX-Pin PA10 -----*/
			
			GPIO.GPIO_Pin = GPIO_Pin_9;
			GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, & GPIO);
			
			GPIO.GPIO_Pin = GPIO_Pin_10;
			GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO);
			
/************************************************************************************************
*															I2C_Init
************************************************************************************************/
		I2C_InitTypeDef I2C_InitStructure;
    //GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = MPU9250_I2C_SCL_Pin | MPU9250_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(MPU9250_I2C_Port, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;// MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = MPU9250_I2C_Speed;

    /* Apply I2C configuration after enabling it */
    I2C_Init(MPU9250_I2C, &I2C_InitStructure);
    /* I2C Peripheral Enable */
    I2C_Cmd(MPU9250_I2C, ENABLE);
		
		MPU6050_I2C_Init();
/**************************************************************************************************
*											Timer
**************************************************************************************************/
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure; 
		timerInitStructure.TIM_Prescaler = 36000;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period = 2-1;
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &timerInitStructure);
		TIM_Cmd(TIM4, ENABLE);
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
		/*----- NVIC Timer interrupt -----*/
			NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

/**************************************************************************************************
*											Interrupt_Init
**************************************************************************************************/
		Interrupt_Init();
/**************************************************************************************************
*											Check_Connection
**************************************************************************************************/
	/*---- Reset MPU9250 ----*/
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	MPU9250_Reset(0);
	MPU9250_Reset(1);
	
		U_Print_Char(USART1,"let's go\n");
			
		/*----- Check Connection of MPU9250 -----*/
		if (Check_Connection(WHO_AM_I_MPU9250, MPU9250_ADDRESS_DEFAULT, 0x73))
		{
			U_Print_Char(USART1, "Found MPU9250 - number 1 ");
			calibrateMPU9250(gyroBias, accelBias);
			Print_Bias();
			Initialize(0);
			//Write_Byte(INT_PIN_CFG, MPU9250_ADDRESS_DEFAULT, 0x22);
			//u8 *a = new u8[1];
			//Read_data_buffer(INT_PIN_CFG, a, MPU9250_ADDRESS_DEFAULT, 1);
			//U_Print(USART1, a[0]);
			U_Print_Char(USART1, "Initialized");
		}
		else
			U_Print_Char(USART1, "Check Connection MPU");
		
		AK8963_turn_on(0);
		/*----- Check Connection of AK8963 -----*/
		if (Check_Connection(WHO_AM_I_AK8963, (AK8963_ADDRESS_DEFAULT), 0x48))
		{
			U_Print_Char(USART1, "Found AK8963\n");
			Initialize_AK8963(magCalibration);
			U_Print_Char(USART1, "AK8963 Initialized...\n");
		}
		else
			U_Print_Char(USART1, "Check Connection AK\n");
		
//		AK8963_turn_off(0);
//		
//		U_Print_Char(USART1,"let's go - 2\n");
//			//MPU6050_I2C_Init();
//		/*----- Check Connection of MPU9250 -----*/
//		if (Check_Connection(WHO_AM_I_MPU9250, MPU9250_ADDRESS_DEFAULT_ADO_HIGH, 0x73))
//		{
//			U_Print_Char(USART1, "Found MPU9250 - number 2 ");
//			Initialize(1);
//			//Write_Byte(INT_PIN_CFG, MPU9250_ADDRESS_DEFAULT, 0x22);
//			//u8 *a = new u8[1];
//			//Read_data_buffer(INT_PIN_CFG, a, MPU9250_ADDRESS_DEFAULT, 1);
//			//U_Print(USART1, a[0]);
//			U_Print_Char(USART1, "Initialized");
//		}
//		else
//			U_Print_Char(USART1, "Check Connection MPU");
//		
//		AK8963_turn_on(1);
//		/*----- Check Connection of AK8963 -----*/
//		if (Check_Connection(WHO_AM_I_AK8963, (AK8963_ADDRESS_DEFAULT), 0x48))
//		{
//			U_Print_Char(USART1, "Found AK8963\n");
//			Initialize_AK8963(magCalibration);
//			U_Print_Char(USART1, "AK8963 Initialized...\n");
//		}
//		else
//			U_Print_Char(USART1, "Check Connection AK\n");
//		AK8963_turn_off(1);
//		
//		if (MPU6050_TestConnection())
//			U_Print_Char(USART1, "Found MPU6050 \n");
//		MPU6050_Initialize();
//		U_Print_Char(USART1, "Initialized...\n");
		
/**************************************************************************************************
*											Variables Init
**************************************************************************************************/
	Accel fAcc[3];
	Gyro fGyro[3];
	Mag fMag[2];
	Angle angle[3];
	char flag = 0;			
	/********* Kalman Init *********/
		kalman filter_pitch;
		kalman filter_roll;
		kalman filter_yaw;	
				
//		kalman filter_pitch1;
//		kalman filter_roll1;
//		kalman filter_yaw1;	
//		
//		kalman filter_pitch2;
//		kalman filter_roll2;
//		kalman filter_yaw2;
	
		time_pre = 0;
		time_now = 0;
		
		magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
		
		
/**************************************************************************************************
*											Function Init
**************************************************************************************************/	
	/*********** Kalman Filter Init ***********/
		kalman_init(&filter_pitch, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		kalman_init(&filter_roll, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		kalman_init(&filter_yaw, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		
//		kalman_init(&filter_pitch1, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
//		kalman_init(&filter_roll1, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
//		kalman_init(&filter_yaw1, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
//		
//		kalman_init(&filter_pitch2, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
//		kalman_init(&filter_roll2, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
//		kalman_init(&filter_yaw2, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		
		//Kalman kalmanPitch;
		//Kalman kalmanRoll;
	while (1)
	{
/**************************************************************************************************
*											Get Central
**************************************************************************************************/
		if (((angle[1].pitch *Rad2Dree) < 100) && ((angle[2].yaw *Rad2Dree) < 300))	flag ++;
/**************************************************************************************************
*											Main Processing
**************************************************************************************************/
		//Create memory
		
		//Get_Accel(RawAccel, 0);Get_Accel(RawAccel1, 1);
		//Get_Gyro(RawGyro, 0);Get_Gyro(RawGyro1, 1);
		//MPU6050_GetRawAccelGyro(AccelGyro);
		//U_Print_Char(USART1, "ACCEL & GYRO\n");
		LowPass_Accel(fAcc);
		LowPass_Gyro(fGyro);
	
		//U_Print_Char(USART1, "MAG\n");
		LowPass_Mag(fMag);
			
			
		float R = sqrt(float((fAcc[0].x * fAcc[0].x + fAcc[0].y * fAcc[0].y + fAcc[0].z * fAcc[0].z )));
//		float R1 = sqrt(float((fAcc[1].x * fAcc[1].x + fAcc[1].y * fAcc[1].y + fAcc[1].z * fAcc[1].z )));
//		float R2 = sqrt(float((fAcc[2].x * fAcc[2].x + fAcc[2].y * fAcc[2].y + fAcc[2].z * fAcc[2].z )));
		//U_Print_float(USART1, R);
		//U_Print_Char(USART1, "\n");
//		U_Print_float(USART1, acos(Mag[1]/Distance(Mag[0],Mag[1])));
//		U_Print_Char(USART1, "\n");
//		U_Print_float(USART1, Distance(Mag[0],Mag[1]));
//		U_Print_Char(USART1, "\n");
//		U_Print_float(USART1, Mag[1]/Distance(Mag[0],Mag[1]));
//		U_Print_Char(USART1, "\n");
		
//		for (char i = 0; i < 3; i++)
//		{
//			if (fAcc[i].x > 32768) 
//			{
//				fAcc[i].x -= 65536;
//				//reverse = -1;
//			}
//			if (fAcc[i].y > 32768) 
//			{
//				fAcc[i].y -= 65536;
//				//reverse = -1;
//			}
//			if (fAcc[i].z > 32768) 
//			{
//				fAcc[i].z -= 65536;
//				//reverse = -1;
//			}
//			if (fGyro[i].x > 32768) 
//			{
//				fGyro[i].x -= 65536;
//				//reverse = -1;
//			}
//			if (fGyro[i].y > 32768) 
//			{
//				fGyro[i].y -= 65536;
//				//reverse = -1;
//			}
//			if (fGyro[i].z > 32768) 
//			{
//				fGyro[i].z -= 65536;
//				//reverse = -1;
//			}
//		}
//		if (fMag[0].x > 32768)				fMag[1].x -= 98304;
//			
//		if (fMag[0].y > 32768)				fMag[1].y -= 98304;
//			
//		if (fMag[0].z > 32768)				fMag[1].z -= 98304;
//		
//		if (fMag[1].x > 32768)				fMag[1].x -= 98304;
//			
//		if (fMag[1].y > 32768)				fMag[1].y -= 98304;
//			
//		if (fMag[1].z > 32768)				fMag[1].z -= 98304;

		
		
		
		/************************** Put in Kalman **************************/	
		//kalman_predict(&filter_pitch, RawGyro[0],  ( time_now - time_pre));
    //kalman_update(&filter_pitch, acos((RawAccel[0]/R)));
		kalman_predict(&filter_pitch, fGyro[0].x,  ( time_now - time_pre));
    kalman_update(&filter_pitch, acos(fAcc[0].x/R));
    kalman_predict(&filter_roll, fGyro[0].y,  (time_now - time_pre));
    kalman_update(&filter_roll, acos(fAcc[0].y/R));
		
//		kalman_predict(&filter_pitch1, fGyro[1].x,  ( time_now - time_pre));
//    kalman_update(&filter_pitch1, acos(fAcc[1].x/R));
//    kalman_predict(&filter_roll1, fGyro[1].y,  (time_now - time_pre));
//    kalman_update(&filter_roll1, acos(fAcc[1].y/R1));
//		
//		kalman_predict(&filter_pitch1, fGyro[2].x,  ( time_now - time_pre));
//    kalman_update(&filter_pitch2, acos(fAcc[2].x/R2));
//    kalman_predict(&filter_roll2, fGyro[2].y,  (time_now - time_pre));
//    kalman_update(&filter_roll2, acos(fAcc[2].y/R2));
		
		//angle[0].pitch = kalmanPitch.GetAngle(acos(fAcc[0].x / R) * Rad2Dree, fGyro[0].x, (time_now - time_pre));
		
//		/************************** Put in Kalman **************************/	
//		kalman_predict(&filter_pitch, RawGyro[0],  ( time_now - time_pre));
//    kalman_update(&filter_pitch, float(atan(RawAccel[0] / sqrt(RawAccel[1]*RawAccel[1] + RawAccel[2]*RawAccel[2]))));
//    kalman_predict(&filter_roll, RawGyro[1],  (time_now - time_pre));
//    kalman_update(&filter_roll, float(atan2(RawAccel[1],RawAccel[2])));
		
		/************************** Get Pitch, Roll, Yaw ***********************/
		angle[0].pitch = kalman_get_angle(&filter_pitch);
		angle[0].roll = kalman_get_angle(&filter_roll);
//		angle[1].pitch = kalman_get_angle(&filter_pitch1);
//		angle[1].roll = kalman_get_angle(&filter_roll1);
//		angle[2].pitch = kalman_get_angle(&filter_pitch2);
//		angle[2].roll = kalman_get_angle(&filter_roll2);
		
		//float xh,yh;
		//xh= fMag[0].x*cos(angle[0].roll)+ fMag[0].y * sin(angle[0].pitch) * sin(angle[0].roll) - fMag[0].z * cos(angle[0].pitch) * sin(angle[0].roll);
		//yh= fMag[0].y * cos(angle[0].pitch) + fMag[0].z * sin(angle[0].pitch);
			
		//float heading = atan2(xh,yh);	
		float heading = atan2(fMag[0].y, fMag[0].x);
		//float heading1 = atan2(fMag[1].z, fMag[1].y);
		//float heading1 = atan2(fMag[1].y, fMag[1].x);
		heading += declinationAngle; // declination get WEST
		//heading1 -= declinationAngle;
		if (heading < 0) heading += 2*PI;
		if (heading > 2*PI) heading -= 2*PI;
//		if (heading1 < 0) heading1 += 2*PI;
//		if (heading1 > 2*PI) heading1 -= 2*PI;
		//U_Print_float(USART1, heading * Rad2Dree);
		//U_Print_Char(USART1, "   ");
		//U_Print_float(USART1, heading1 * Rad2Dree);
		//U_Print_Char(USART1, "\n");
		
		kalman_predict(&filter_yaw, fGyro[0].z,  (time_now - time_pre));
    kalman_update(&filter_yaw, heading);
//		kalman_predict(&filter_yaw1, fGyro[1].z,  (time_now - time_pre));
//    kalman_update(&filter_yaw1, heading1);
//		kalman_predict(&filter_yaw, (float)acos(fMag[0].y/Distance(fMag[0].x,fMag[0].y)),  (time_now - time_pre));
//    kalman_update(&filter_yaw, (float)acos(fMag[0].y/Distance(fMag[1].x,fMag[1].y)));
		time_pre = time_now; // get current time
		
		if (time_pre == 0xffffffffffffffff)
			time_pre = time_now = 0;
		
		angle[0].yaw = kalman_get_angle(&filter_yaw);
		//angle[1].yaw = kalman_get_angle(&filter_yaw1);
		
		//if (flag == 1)
//		{
			//U_Print_Char(USART1, "Pitch ");
			//U_Print_float(USART1, angle[0].roll * Rad2Dree);// pitch mpu 1
		U_Print(USART1,int( angle[0].roll * Rad2Dree));// pitch mpu 1
			//U_Print_float(USART1, acos(fAcc[0].x/R) * Rad2Dree);//pitch mpu1 no fill
//			U_Print_Char(USART1, "  ");
//			U_Print_float(USART1, angle[1].yaw * Rad2Dree);// yaw mpu 2
//			U_Print_Char(USART1, "  ");
//			U_Print_float(USART1, angle[2].pitch * Rad2Dree);// roll mpu 3
			//U_Print_Char(USART1, " \n");
		U_Print_Char(USART1," "); // Print Space
			//U_Print_Char(USART1, "Roll ");
			//U_Print_float(USART1, angle[0].pitch * Rad2Dree);// roll mpu1
		U_Print(USART1,int (angle[0].pitch * Rad2Dree));// roll mpu1
			//U_Print_float(USART1, acos(fAcc[0].y/R));// roll mpu1 no fill
			U_Print_Char(USART1, " "); // Print Space
//			U_Print_float(USART1, angle[1].pitch * Rad2Dree);//
			//U_Print_Char(USART1, " \n");
			//U_Print_Char(USART1, "Yaw ");
			//U_Print_float(USART1, angle[0].yaw * Rad2Dree);// yaw mpu 1
			U_Print(USART1,int( angle[0].yaw * Rad2Dree));// yaw mpu 1
			//U_Print_float(USART1, atan2(fMag[0].y, fMag[0].x) * Rad2Dree);// yaw mpu1 no fill
			
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))
			{
				U_Print_Char(USART1, "*");
			}
			else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))
			{
				U_Print_Char(USART1, "@");
			}
			else
			{
				U_Print_Char(USART1, "!");
			}
//			U_Print_float(USART1, angle[1].roll * Rad2Dree);// pitch mpu 2
//			U_Print_Char(USART1, "  ");
//			U_Print_float(USART1, angle[2].roll * Rad2Dree);// pitch mpu 3
			//U_Print_Char(USART1, " \n");
//		}
		//else U_Print_Char(USART1, "not \n");
		timer = 0;
		while (timer < 50);
		led_toggle();
	}
}


//static void prvSetupHardware( void )
//{
//	/* Start with the clocks in their expected state. */
//	RCC_DeInit();

//	/* Enable HSE (high speed external clock). */
//	RCC_HSEConfig( RCC_HSE_ON );

//	/* Wait till HSE is ready. */
//	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
//	{
//	}

//	/* 2 wait states required on the flash. */
//	*( ( unsigned long * ) 0x40022000 ) = 0x02;

//	/* HCLK = SYSCLK */
//	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

//	/* PCLK2 = HCLK */
//	RCC_PCLK2Config( RCC_HCLK_Div1 );

//	/* PCLK1 = HCLK/2 */
//	RCC_PCLK1Config( RCC_HCLK_Div2 );

//	/* PLLCLK = 8MHz * 9 = 72 MHz. */
//	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

//	/* Enable PLL. */
//	RCC_PLLCmd( ENABLE );

//	/* Wait till PLL is ready. */
//	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//	{
//	}

//	/* Select PLL as system clock source. */
//	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

//	/* Wait till PLL is used as system clock source. */
//	while( RCC_GetSYSCLKSource() != 0x08 )
//	{
//	}

//	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
//							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

//	/* SPI2 Periph clock enable */
//	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


//	/* Set the Vector Table base address at 0x08000000 */
//	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

//	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

//	/* Configure HCLK clock as SysTick clock source. */
//	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

//	//vParTestInitialise();
//}

void sleep(long i)
{
	for (long k = 0 ; k < i; k++);
}

extern "C" void TIM4_IRQHandler()
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
				timer ++;
				time_now++;
    }
}

void led_toggle(void)
		{
				/* Read LED output (GPIOA PIN8) status */
				uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
			 
				/* If LED output set, clear it */
				if(led_bit == (uint8_t)Bit_SET)
				{
						GPIO_ResetBits(GPIOC, GPIO_Pin_13);
				}
				/* If LED output clear, set it */
				else
				{
						GPIO_SetBits(GPIOC, GPIO_Pin_13);
				}
		}
		
float Distance(float x, float y)
{
	return sqrt(x*x + y*y);
}

//int Get_Central(kalman p, kalman r, kalman y)
//{
//	float *RawAccel = new float[3],
//				*RawGyro = new float[3],
//				*Mag = new float[3],
//				angle[3];
//	Get_Accel(RawAccel,0);
//	Get_Gyro(RawGyro,0);
//	Get_Mag(Mag);
//	double R = sqrt(float((RawAccel[0] * RawAccel[0] + RawAccel[1] * RawAccel[1] + RawAccel[2] * RawAccel[2] )));
//	if ((Mag[0] > 32768) && ( Mag[1] < 32768))
//		{
//			Mag[0] -= 98304; // 65536 + 32768
//			U_Print_float(USART1, Mag[0]);
//			U_Print_Char(USART1, "\n");
//		}
//		if ((Mag[0] > 32768) && ( Mag[1] > 32768))
//		{
//			Mag[0] -= 98304; // 65536 + 32768
//			Mag[1] -= 98304;
//			U_Print_float(USART1, Mag[0]);
//			U_Print_Char(USART1, "\n");
//		}
//		float heading = atan2(Mag[1], Mag[0]);
//		heading += declinationAngle;
//		if (heading < 0) heading += 2*PI;
//		if (heading > 2*PI) heading -= 2*PI;
//		kalman_predict(&p, RawGyro[0],  ( time_now - time_pre));
//    kalman_update(&p, acos((RawAccel[1]/R)));
//    kalman_predict(&r, RawGyro[1],  (time_now - time_pre));
//    kalman_update(&r, acos(RawAccel[1]/R));
//		angle[0] = kalman_get_angle(&p);
//		angle[1] = kalman_get_angle(&r);
//		
//		
//		kalman_predict(&y, RawGyro[2],  (time_now - time_pre));
//    kalman_update(&y, heading);
//		time_pre = time_now; // get current time
//		
//		angle[2] = kalman_get_angle(&y);
//		
//		U_Print_Char(USART1, "Pitch ");
//		U_Print_float(USART1, angle[1] * Rad2Dree);
//		U_Print_Char(USART1, " \n");
//		U_Print_Char(USART1, "Roll ");
//		U_Print_float(USART1, angle[0] * Rad2Dree);
//		U_Print_Char(USART1, " \n");
//		U_Print_Char(USART1, "Yaw ");
//		U_Print_float(USART1, angle[2] * Rad2Dree);
//		U_Print_Char(USART1, " \n");
//		
//		
//		if (((angle[2] * Rad2Dree) < 300) && ((angle[1] *Rad2Dree) <= 90))
//			return 1;
//		else return 0;
//		delete RawAccel;
//		delete RawGyro;
//		delete Mag;
//}

void LowPass_Accel(Accel fAcc[])
{
	float RawAccel[6];
		Get_Accel(RawAccel, 0);
		fAcc[0].x = RawAccel[0] * alpha + (fAcc[0].x * (1.0 - alpha));
		fAcc[0].y = RawAccel[1] * alpha + (fAcc[0].y * (1.0 - alpha));
		fAcc[0].z = RawAccel[2] * alpha + (fAcc[0].z * (1.0 - alpha));
	
		fAcc[0].x = RawAccel[0] - accelBias[0];
		fAcc[0].y = RawAccel[1] - accelBias[1];
		fAcc[0].z = RawAccel[2] - accelBias[2];
		
//		Get_Accel(RawAccel, 1);
//		fAcc[1].x = RawAccel[0] * alpha + (fAcc[1].x * (1.0 - alpha));
//		fAcc[1].y = RawAccel[1] * alpha + (fAcc[1].y * (1.0 - alpha));
//		fAcc[1].z = RawAccel[2] * alpha + (fAcc[1].z * (1.0 - alpha));
//		
//		MPU6050_GetRawAccelGyro(RawAccel);
//		fAcc[2].x = RawAccel[0] * alpha + (fAcc[2].x * (1.0 - alpha));
//		fAcc[2].y = RawAccel[1] * alpha + (fAcc[2].y * (1.0 - alpha));
//		fAcc[2].z = RawAccel[2] * alpha + (fAcc[2].z * (1.0 - alpha));
		
//		U_Print_Char(USART1, "Accel MPU 0: ");
//		U_Print_float(USART1, fAcc[0].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fAcc[0].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fAcc[0].z);
//		U_Print_Char(USART1, "\n");
//		
//		U_Print_Char(USART1, "Accel MPU 1: ");
//		U_Print_float(USART1, fAcc[1].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fAcc[1].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fAcc[1].z);
//		U_Print_Char(USART1, "\n");
//		
//		U_Print_Char(USART1, "Accel MPU 2: ");
//		U_Print_float(USART1, fAcc[2].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fAcc[2].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fAcc[2].z);
//		U_Print_Char(USART1, "\n");
}

void LowPass_Gyro(Gyro fGyro[])
{
	float Raw[6];
		Get_Gyro(Raw, 0);
		fGyro[0].x = Raw[0] * alpha + (fGyro[0].x * (1.0 - alpha));
		fGyro[0].y = Raw[1] * alpha + (fGyro[0].y * (1.0 - alpha));
		fGyro[0].z = Raw[2] * alpha + (fGyro[0].z * (1.0 - alpha));
//		fGyro[0].x = Raw[0] - gyroBias[0];
//		fGyro[0].y = Raw[1] - gyroBias[1];
//		fGyro[0].z = Raw[2]	- gyroBias[2];
		
//		Get_Gyro(Raw, 1);
//		fGyro[1].x = Raw[0] * alpha + (fGyro[1].x * (1.0 - alpha));
//		fGyro[1].y = Raw[1] * alpha + (fGyro[1].y * (1.0 - alpha));
//		fGyro[1].z = Raw[2] * alpha + (fGyro[1].z * (1.0 - alpha));
//		
//		MPU6050_GetRawAccelGyro(Raw);
//		fGyro[2].x = Raw[3] * alpha + (fGyro[2].x * (1.0 - alpha));
//		fGyro[2].y = Raw[4] * alpha + (fGyro[2].y * (1.0 - alpha));
//		fGyro[2].z = Raw[5] * alpha + (fGyro[2].z * (1.0 - alpha));
		
//		U_Print_Char(USART1, "Gyro MPU 0: ");
//		U_Print_float(USART1, fGyro[0].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fGyro[0].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fGyro[0].z);
//		U_Print_Char(USART1, "\n");
//		
//		U_Print_Char(USART1, "Gyro MPU 1: ");
//		U_Print_float(USART1, fGyro[1].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fGyro[1].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fGyro[1].z);
//		U_Print_Char(USART1, "\n");
//		
//		U_Print_Char(USART1, "Gyro MPU 2: ");
//		U_Print_float(USART1, fGyro[2].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fGyro[2].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fGyro[2].z);
//		U_Print_Char(USART1, "\n");
}

void LowPass_Mag(Mag fMag[])
{
		float Raw[3];
		AK8963_turn_on(0);
		//Initialize_AK8963(Raw);
		Get_Mag(Raw);
		//AK8963_turn_off(0);
	
//		Raw[0] = Raw[0]*magCalibration[0] - magbias[0];
//		Raw[1] = Raw[1]*magCalibration[1] - magbias[1];
//		Raw[2] = Raw[2]*magCalibration[2] - magbias[2];
	
		fMag[0].x = Raw[0] * alpha + (fMag[0].x * (1.0 - alpha));
		fMag[0].y = Raw[1] * alpha + (fMag[0].y * (1.0 - alpha));
		fMag[0].z = Raw[2] * alpha + (fMag[0].z * (1.0 - alpha));
	
//		U_Print_Char(USART1, "Mag MPU 0: ");
//		U_Print_float(USART1, fMag[0].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fMag[0].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fMag[0].z);
//		U_Print_Char(USART1, "\n");
		
//		AK8963_turn_on(1);
//		//Initialize_AK8963(Raw);
//		Get_Mag(Raw);
//		AK8963_turn_off(1);
//		
//		fMag[1].x = Raw[0] * alpha + (fMag[1].x * (1.0 - alpha));
//		fMag[1].y = Raw[1] * alpha + (fMag[1].y * (1.0 - alpha));
//		fMag[1].z = Raw[2] * alpha + (fMag[1].z * (1.0 - alpha));
	
//		U_Print_Char(USART1, "Mag MPU 1: ");
//		U_Print_float(USART1, fMag[1].x);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fMag[1].y);
//		U_Print_Char(USART1, "  ");
//		U_Print_float(USART1, fMag[1].z);
//		U_Print_Char(USART1, "\n");
}

void Print_Bias()
{
	U_Print_float(USART1, accelBias[0]);
	U_Print_Char(USART1,"   ");
	U_Print_float(USART1, accelBias[0]);
	U_Print_Char(USART1,"   ");
	U_Print_float(USART1, accelBias[0]);
	U_Print_Char(USART1,"\n");
			
	U_Print_float(USART1, gyroBias[0]);
	U_Print_Char(USART1,"   ");
	U_Print_float(USART1, gyroBias[0]);
	U_Print_Char(USART1,"   ");
	U_Print_float(USART1, gyroBias[0]);
	U_Print_Char(USART1,"   ");
}

void Interrupt_Init()
{
NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
/*---- NVIC EXTERNAL INTERRUPT ----*/
	/*---- EXTI0 ----*/
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	/*---- EXTI1 ----*/
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	/*---- EXTI3 ----*/
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
/*---- EXTERNAL INTERRUPT ----*/
	// Configure PA0, PA1, PA3 as input with internal pullup resistor
	GPIO_InitTypeDef GPIO;
	GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
	GPIO.GPIO_Mode = GPIO_Mode_IPU;
	GPIO.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO);
	// Configure PB0, PB1 as input with internal pullup resistor
	GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO.GPIO_Mode = GPIO_Mode_IPU;
	GPIO.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO);
	/*---- Connect Pin With Interrupt_NVIC ----*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); // Pin A0
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); // Pin A1
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2); // Pin A2
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0); // Pin B0
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1); // Pin B1
	/*---- Confirgure EXTI sturct ----*/
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line3;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
}

extern "C" void EXTI0_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) // judge whether a line break
	{
		if (!(GPIO_ReadInputData(GPIOA) & GPIO_Pin_0))
		{
			U_Print_Char(USART1, "LEFT********");
			timer = 10;
			while (timer < 10);
		}
		if (!(GPIO_ReadInputData(GPIOB) & GPIO_Pin_0))
		{
			U_Print_Char(USART1, "LEFT");
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

extern "C" void EXTI1_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) // judge whether a line break
	{
		if (!(GPIO_ReadInputData(GPIOA) & GPIO_Pin_1))
		{
			//led_toggle();
			U_Print_Char(USART1, "RIGHT*******");
			timer = 10;
			while (timer < 10);
		}
		if (!(GPIO_ReadInputData(GPIOB) & GPIO_Pin_1))
		{
			U_Print_Char(USART1, "LEFT");
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

extern "C" void EXTI3_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line3) != RESET) // judge whether a line break
	{
		if (!(GPIO_ReadInputData(GPIOA) & GPIO_Pin_3))
		{
			//led_toggle();
			U_Print_Char(USART1, "CALIBRATION*");
		}
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
	
}
