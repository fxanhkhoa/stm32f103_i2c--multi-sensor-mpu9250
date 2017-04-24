#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "usart_print.h"	
#include "MPU9250.h"
#include "stm32f10x_tim.h"
#include "Main.h"
//#include "MPU6050.h"
#include <math.h>
#include "Kalman.c"

#define Rad2Dree       57.295779513082320876798154814105
#define PI	3.1415926535897932384626433832795
#define declinationAngle  0.007563
#define alpha  0.5

static void prvSetupHardware( void );
void sleep(long i);
void led_toggle(void);
float Distance(float x, float y);
int Get_Central(kalman p, kalman r, kalman y);

uint32_t timer, time_pre =0, time_now = 0;


	
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
	USART.USART_BaudRate = 38400;
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
*											Check_Connection
**************************************************************************************************/
	float *MCali = new float[3];
		U_Print_Char(USART1,"let's go\n");
			//MPU6050_I2C_Init();
		/*----- Check Connection of MPU9250 -----*/
		if (Check_Connection(WHO_AM_I_MPU9250, MPU9250_ADDRESS_DEFAULT, 0x73))
		{
			U_Print_Char(USART1, "Found MPU9250 - number 1 ");
			Initialize(0);
			//Write_Bytes(INT_PIN_CFG, MPU9250_ADDRESS_DEFAULT, 0x22);
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
			Initialize_AK8963();
			U_Print_Char(USART1, "AK8963 Initialized...\n");
		}
		else
			U_Print_Char(USART1, "Check Connection AK\n");
		
		AK8963_turn_off(0);
		
		U_Print_Char(USART1,"let's go - 2\n");
			//MPU6050_I2C_Init();
		/*----- Check Connection of MPU9250 -----*/
		if (Check_Connection(WHO_AM_I_MPU9250, MPU9250_ADDRESS_DEFAULT_ADO_HIGH, 0x73))
		{
			U_Print_Char(USART1, "Found MPU9250 - number 2 ");
			Initialize(1);
			//Write_Bytes(INT_PIN_CFG, MPU9250_ADDRESS_DEFAULT, 0x22);
			//u8 *a = new u8[1];
			//Read_data_buffer(INT_PIN_CFG, a, MPU9250_ADDRESS_DEFAULT, 1);
			//U_Print(USART1, a[0]);
			U_Print_Char(USART1, "Initialized");
		}
		else
			U_Print_Char(USART1, "Check Connection MPU");
		
		AK8963_turn_on(1);
		/*----- Check Connection of AK8963 -----*/
		if (Check_Connection(WHO_AM_I_AK8963, (AK8963_ADDRESS_DEFAULT), 0x48))
		{
			U_Print_Char(USART1, "Found AK8963\n");
			Initialize_AK8963();
			U_Print_Char(USART1, "AK8963 Initialized...\n");
		}
		else
			U_Print_Char(USART1, "Check Connection AK\n");
		AK8963_turn_off(1);
/**************************************************************************************************
*											Variables Init
**************************************************************************************************/
	float angle[3],
				*RawAccel = new float[3],
				*RawGyro = new float[3],
				*Mag = new float[3],
				*fRawAccel = new float[3],
				*fRawGyro = new float[3],
				*fMag = new float[3];
				//declinationAngle = 0.0404;
				Mag[0] = Mag[1] = Mag[2] = 0;
				angle[0] = angle[1] = angle[2] = 900;
	
	float angle1[3],
				*RawAccel1 = new float[3],
				*RawGyro1 = new float[3],
				*Mag1 = new float[3],
				*fRawAccel1 = new float[3],
				*fRawGyro1 = new float[3],
				*fMag1 = new float[3];
				//declinationAngle = 0.0404;
				Mag1[0] = Mag1[1] = Mag1[2] = 0;
				angle1[0] = angle1[1] = angle1[2] = 900;
	int flag = -5;			
	/********* Kalman Init *********/
		kalman filter_pitch;
		kalman filter_roll;
		kalman filter_yaw;	
				
		kalman filter_pitch1;
		kalman filter_roll1;
		kalman filter_yaw1;	
				
		time_pre = 0;
		time_now = 0;
/**************************************************************************************************
*											Function Init
**************************************************************************************************/	
	/*********** Kalman Filter Init ***********/
		kalman_init(&filter_pitch, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		kalman_init(&filter_roll, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		kalman_init(&filter_yaw, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		
		kalman_init(&filter_pitch1, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		kalman_init(&filter_roll1, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
		kalman_init(&filter_yaw1, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
	while (1)
	{
/**************************************************************************************************
*											Get Central
**************************************************************************************************/
		if (((angle[1] *Rad2Dree) < 100) && ((angle[2] *Rad2Dree) < 300))	flag ++;
/**************************************************************************************************
*											Main Processing
**************************************************************************************************/
	
		Get_Accel(RawAccel, 0);Get_Accel(RawAccel1, 1);
		Get_Gyro(RawGyro, 0);Get_Gyro(RawGyro1, 1);
		U_Print_Char(USART1, "ACCEL & GYRO\n");
		for (int i = 0; i < 3; i++)
			{
				fRawAccel[i] = RawAccel[i] * alpha + (fRawAccel[i] * (1.0 - alpha));
				fRawAccel1[i] = RawAccel1[i] * alpha + (fRawAccel1[i] * (1.0 - alpha));
				U_Print_float(USART1, fRawAccel[i]);
				U_Print_Char(USART1, "   ");
				U_Print_float(USART1, fRawAccel1[i]);
				U_Print_Char(USART1, "\n");
			}
		for (int i = 0; i < 3; i++)
		{
			fRawGyro[i] = RawGyro[i] * alpha + (fRawGyro[i] * (1.0 - alpha));
			fRawGyro1[i] = RawGyro1[i] * alpha + (fRawGyro1[i] * (1.0 - alpha));
			U_Print_float(USART1, fRawGyro[i]);
			U_Print_Char(USART1, "   ");
			U_Print_float(USART1, fRawGyro1[i]);
			U_Print_Char(USART1, "\n");
		}
		U_Print_Char(USART1, "MAG\n");
		
		//AK8963_turn_off(1);
		//AK8963_turn_off(0);
		
		AK8963_turn_on(0);
		Initialize_AK8963();
		Get_Mag(Mag);
		AK8963_turn_off(0);
		
		AK8963_turn_on(1);
		Initialize_AK8963();
		Get_Mag(Mag1);
		AK8963_turn_off(1);
		
		for (int i =0 ; i < 3; i++)
			{
				fMag[i] = Mag[i] * alpha + (fMag[i] * (1.0 - alpha));
				fMag1[i] = Mag1[i] * alpha + (fMag1[i] * (1.0 - alpha));
				U_Print_float(USART1, fMag[i]);
				U_Print_Char(USART1, "   ");
				U_Print_float(USART1, fMag1[i]);
				U_Print_Char(USART1, "\n");
			}
		double R = sqrt(float((RawAccel[0] * RawAccel[0] + RawAccel[1] * RawAccel[1] + RawAccel[2] * RawAccel[2] )));
		double R1 = sqrt(float((RawAccel1[0] * RawAccel1[0] + RawAccel1[1] * RawAccel1[1] + RawAccel1[2] * RawAccel1[2] )));
		//U_Print_float(USART1, R);
		//U_Print_Char(USART1, "\n");
//		U_Print_float(USART1, acos(Mag[1]/Distance(Mag[0],Mag[1])));
//		U_Print_Char(USART1, "\n");
//		U_Print_float(USART1, Distance(Mag[0],Mag[1]));
//		U_Print_Char(USART1, "\n");
//		U_Print_float(USART1, Mag[1]/Distance(Mag[0],Mag[1]));
//		U_Print_Char(USART1, "\n");
		
		for (int i = 0; i < 3; i++)
		{
			if (RawAccel[i] > 32768) 
			{
				RawAccel[i] -= 65536;
				//reverse = -1;
			}
			if (RawGyro[i] > 32768) 
			{
				RawGyro[i] -= 65536;
				//reverse = -1;
			}
			if (Mag[i] > 32768)
			{
				Mag[i] -= 98304;
			}
			if (RawAccel1[i] > 32768) 
			{
				RawAccel1[i] -= 65536;
				//reverse = -1;
			}
			if (RawGyro1[i] > 32768) 
			{
				RawGyro1[i] -= 65536;
				//reverse = -1;
			}
			if (Mag1[i] > 32768)
			{
				Mag1[i] -= 98304;
			}
		}
		float heading = atan2(Mag[1], Mag[0]);
		float heading1 = atan2(Mag1[1], Mag1[0]);
		heading += declinationAngle;
		heading1 += declinationAngle;
		if (heading < 0) heading += 2*PI;
		if (heading > 2*PI) heading -= 2*PI;
		if (heading1 < 0) heading1 += 2*PI;
		if (heading1 > 2*PI) heading1 -= 2*PI;
		//U_Print_float(USART1, heading * Rad2Dree);
		//U_Print_Char(USART1, "   ");
		//U_Print_float(USART1, heading1 * Rad2Dree);
		//U_Print_Char(USART1, "\n");
		
		float pitch = acos((RawAccel[0]/R));
		pitch += declinationAngle;
		if (pitch < 0) pitch += 2*PI;
		if (pitch > 2*PI) pitch -= 2*PI;
		
		/************************** Put in Kalman **************************/	
		kalman_predict(&filter_pitch, RawGyro[0],  ( time_now - time_pre));
    kalman_update(&filter_pitch, acos((RawAccel[0]/R)));
    kalman_predict(&filter_roll, RawGyro[1],  (time_now - time_pre));
    kalman_update(&filter_roll, acos(RawAccel[1]/R));
		
		kalman_predict(&filter_pitch1, RawGyro1[0],  ( time_now - time_pre));
    kalman_update(&filter_pitch1, acos((RawAccel1[0]/R)));
    kalman_predict(&filter_roll1, RawGyro1[1],  (time_now - time_pre));
    kalman_update(&filter_roll1, acos(RawAccel1[1]/R));
		
//		/************************** Put in Kalman **************************/	
//		kalman_predict(&filter_pitch, RawGyro[0],  ( time_now - time_pre));
//    kalman_update(&filter_pitch, float(atan(RawAccel[0] / sqrt(RawAccel[1]*RawAccel[1] + RawAccel[2]*RawAccel[2]))));
//    kalman_predict(&filter_roll, RawGyro[1],  (time_now - time_pre));
//    kalman_update(&filter_roll, float(atan2(RawAccel[1],RawAccel[2])));
		
		/************************** Get Pitch, Roll, Yaw ***********************/
		angle[0] = kalman_get_angle(&filter_pitch);
		angle[1] = kalman_get_angle(&filter_roll);
		angle1[0] = kalman_get_angle(&filter_pitch);
		angle1[1] = kalman_get_angle(&filter_roll);
		
		
		kalman_predict(&filter_yaw, RawGyro[2],  (time_now - time_pre));
    kalman_update(&filter_yaw, heading);
		kalman_predict(&filter_yaw1, RawGyro1[2],  (time_now - time_pre));
    kalman_update(&filter_yaw1, heading1);
		//kalman_predict(&filter_yaw, (float)acos(Mag[1]/Distance(Mag[0],Mag[1])),  (time_now - time_pre));
    //kalman_update(&filter_yaw, (float)acos(Mag[1]/Distance(Mag[0],Mag[1])));
		time_pre = time_now; // get current time
		
		angle[2] = kalman_get_angle(&filter_yaw);
		angle1[2] = kalman_get_angle(&filter_yaw1);
		
		//if (flag == 1)
		{
			U_Print_Char(USART1, "Pitch ");
			U_Print_float(USART1, angle[1] * Rad2Dree);
			U_Print_Char(USART1, "  ");
			U_Print_float(USART1, angle1[2] * Rad2Dree);
			U_Print_Char(USART1, " \n");
			U_Print_Char(USART1, "Roll ");
			U_Print_float(USART1, angle[0] * Rad2Dree);
			U_Print_Char(USART1, "  ");
			U_Print_float(USART1, angle1[0] * Rad2Dree);
			U_Print_Char(USART1, " \n");
			U_Print_Char(USART1, "Yaw ");
			U_Print_float(USART1, angle[2] * Rad2Dree);
			U_Print_Char(USART1, "  ");
			U_Print_float(USART1, angle1[1] * Rad2Dree);
			U_Print_Char(USART1, " \n");
		}
		//else U_Print_Char(USART1, "not \n");
		timer = 0;
		while (timer < 250);
		led_toggle();
	}
}


static void prvSetupHardware( void )
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
	{
	}

	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = 8MHz * 9 = 72 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	//vParTestInitialise();
}

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

int Get_Central(kalman p, kalman r, kalman y)
{
	float *RawAccel = new float[3],
				*RawGyro = new float[3],
				*Mag = new float[3],
				angle[3];
	Get_Accel(RawAccel,0);
	Get_Gyro(RawGyro,0);
	Get_Mag(Mag);
	double R = sqrt(float((RawAccel[0] * RawAccel[0] + RawAccel[1] * RawAccel[1] + RawAccel[2] * RawAccel[2] )));
	if ((Mag[0] > 32768) && ( Mag[1] < 32768))
		{
			Mag[0] -= 98304; // 65536 + 32768
			U_Print_float(USART1, Mag[0]);
			U_Print_Char(USART1, "\n");
		}
		if ((Mag[0] > 32768) && ( Mag[1] > 32768))
		{
			Mag[0] -= 98304; // 65536 + 32768
			Mag[1] -= 98304;
			U_Print_float(USART1, Mag[0]);
			U_Print_Char(USART1, "\n");
		}
		float heading = atan2(Mag[1], Mag[0]);
		heading += declinationAngle;
		if (heading < 0) heading += 2*PI;
		if (heading > 2*PI) heading -= 2*PI;
		kalman_predict(&p, RawGyro[0],  ( time_now - time_pre));
    kalman_update(&p, acos((RawAccel[1]/R)));
    kalman_predict(&r, RawGyro[1],  (time_now - time_pre));
    kalman_update(&r, acos(RawAccel[1]/R));
		angle[0] = kalman_get_angle(&p);
		angle[1] = kalman_get_angle(&r);
		
		
		kalman_predict(&y, RawGyro[2],  (time_now - time_pre));
    kalman_update(&y, heading);
		time_pre = time_now; // get current time
		
		angle[2] = kalman_get_angle(&y);
		
		U_Print_Char(USART1, "Pitch ");
		U_Print_float(USART1, angle[1] * Rad2Dree);
		U_Print_Char(USART1, " \n");
		U_Print_Char(USART1, "Roll ");
		U_Print_float(USART1, angle[0] * Rad2Dree);
		U_Print_Char(USART1, " \n");
		U_Print_Char(USART1, "Yaw ");
		U_Print_float(USART1, angle[2] * Rad2Dree);
		U_Print_Char(USART1, " \n");
		
		
		if (((angle[2] * Rad2Dree) < 300) && ((angle[1] *Rad2Dree) <= 90))
			return 1;
		else return 0;
		delete RawAccel;
		delete RawGyro;
		delete Mag;
}
