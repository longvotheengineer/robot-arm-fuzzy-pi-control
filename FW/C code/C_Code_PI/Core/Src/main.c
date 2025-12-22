/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//=============INCLUDE LIBRARY=============//
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//=============DEFINE VARIABLE=============//
#define MCP4725_ADDR (0x60 << 1) 

uint8_t RX_Data[17], TX_Data[29], Leech_Motor_Speed [3], LED_Status_Buffer [2], Each_Eeprom_Element[1];
uint8_t UART1_RX_Flag, LED_1_Status_Flag, LED_2_Status_Flag, LED_Status_Tracking_Flag, Store_LED_Status_Flag, Timer3_Delay_Flag, Store_Motor_Speed_Flag, Recover_Data_Flag;
uint8_t Forward_45D_Flag, Forward_90D_Flag, Forward_180D_Flag, Forward_360D_Flag, Motor_Position_Tracking_Flag, Control_Motor_Speed_Flag;
uint8_t Mode_00_Flag, Mode_01_Flag, Mode_02_Flag, Mode_03_Flag, Mode_04_Flag, Mode_05_Flag, Mode_06_Flag;

char *Char_RX_Data, Down_Line[] = "\r \n", RX_Hex_Data[29];
char Pulse_Buffer[13], Label_LED_Status_Buffer[13];

int pre_pulse, pulse, PWM_Duty_Cycle, Label_LED_Status;
int count, i;
int Iden_Control_Motor;

float setpoint = 0, Output_Voltage_Setpoint;
float pre_error, error;
float integral, derivative, duty_cycle_output;
float Kp = 10, Ki = 0.5, Kd = 0.2;
float Delta_t = 0.01;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=======================================EEPROM=========================================//
//======================================================================================//
//======================================================================================//

void Recover_Data ()
{
	Recover_Data_Flag = 1;
	
	for(int i = 0, j = 0; i <= 2; i++)
	{
		HAL_I2C_Mem_Read(&hi2c1, 0xA0, i, 2, Each_Eeprom_Element, 1, 100);
		LED_Status_Buffer[j] = Each_Eeprom_Element[0];
		j++;
	}
	
	if (LED_Status_Buffer[0] == 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		
		Label_LED_Status = 1000;
		int len = sprintf(Label_LED_Status_Buffer, "%d\r\n", Label_LED_Status);
		HAL_UART_Transmit (&huart1, (uint8_t *)Label_LED_Status_Buffer, len, HAL_MAX_DELAY);
	}
	else if (LED_Status_Buffer[0] == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
		
		Label_LED_Status = 1001;
		int len = sprintf(Label_LED_Status_Buffer, "%d\r\n", Label_LED_Status);
		HAL_UART_Transmit (&huart1, (uint8_t *)Label_LED_Status_Buffer, len, HAL_MAX_DELAY);
	}

	if (LED_Status_Buffer[1] == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		
		Label_LED_Status = 1010;
		int len = sprintf(Label_LED_Status_Buffer, "%d\r\n", Label_LED_Status);
		HAL_UART_Transmit (&huart1, (uint8_t *)Label_LED_Status_Buffer, len, HAL_MAX_DELAY);
	}
	else if (LED_Status_Buffer[1] == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		
		Label_LED_Status = 1011;
		int len = sprintf(Label_LED_Status_Buffer, "%d\r\n", Label_LED_Status);
		HAL_UART_Transmit (&huart1, (uint8_t *)Label_LED_Status_Buffer, len, HAL_MAX_DELAY);
	}
}

//========================================DAC===========================================//
//======================================================================================//
//======================================================================================//

void MCP4725_SetVoltage(int Output_Voltage_Setpoint)
{
	uint8_t Output_Voltage_Buffer[3];

  Output_Voltage_Buffer[0] = 0x40;                      
  Output_Voltage_Buffer[1] = (Output_Voltage_Setpoint>> 4) & 0xFF;  
  Output_Voltage_Buffer[2] = (Output_Voltage_Setpoint& 0x0F) << 4;  

  HAL_I2C_Master_Transmit(&hi2c2, MCP4725_ADDR, Output_Voltage_Buffer, 3, HAL_MAX_DELAY);
}

void Change_Output_Voltage ()
{
	if (strncmp(Char_RX_Data, "00060000011", sizeof(RX_Data)) == 0)
	{
		Mode_06_Flag = 0;
	}
	
	Output_Voltage_Setpoint = (RX_Data[6] - '0') + (RX_Data[7] - '0') / 10.0;
	MCP4725_SetVoltage ((Output_Voltage_Setpoint * 4095) / 3);         
}

//========================LED STATUS & MOTOR POSITION TRACKING==========================//
//======================================================================================//
//======================================================================================//

void Active_Inactive_Status_Tracking ()
{
	Timer3_Delay_Flag = 0;
	count ++;
	
	if (count == 1)
	{
		setpoint = 0;
	}
	
	if (count == 2)
	{
		setpoint = 50;
	}
	
	if (count == 3)
	{
		setpoint = 100;
	}
	
	if (count == 4)
	{
		setpoint = 150;
		count = 0;
	}	
}

//=====================================DRAW GRAPH=======================================//
//======================================================================================//
//======================================================================================//

void Pulse_Transmit_Via_Com ()
{
	int len = sprintf(Pulse_Buffer, "%d\r\n",pulse);
	HAL_UART_Transmit (&huart1, (uint8_t *)Pulse_Buffer, len, HAL_MAX_DELAY);
	HAL_Delay(5);
}

//====================================CONTROL MOTOR=====================================//
//======================================================================================//
//======================================================================================//

//=============CHANGE MOTOR POSITION=============//
void Change_Motor_Position ()
{
	Mode_01_Flag = 0;
	
	if (Iden_Control_Motor % 2 == 1)
	{	
		//FORWARD 45 DEG//
		if (Forward_45D_Flag == 1)
		{
			setpoint = 224/8;
			Kp = 10;
			Ki = 0.5;
			Kd = 0.1;
			
			Forward_45D_Flag = 0;
		}
		
		//FORWARD 90 DEG//
		if (Forward_90D_Flag == 1)
		{
			setpoint = 224/4;
			Kp = 10;
			Ki = 0.5;
			Kd = 0.1;
			
			Forward_90D_Flag = 0;
}
		
		//FORWARD 180 DEG//
		if (Forward_180D_Flag == 1)
		{
			setpoint = 224/2;
			Kp = 20;
			Ki = 0.5;
			Kd = 0.1;
			
			Forward_180D_Flag = 0;
		}
		
		//FORWARD 360 DEG//
		if (Forward_360D_Flag == 1)
		{
			setpoint = 224/1;
			Kp = 8;
			Ki = 6;
			Kd = 0.4;
			
			Forward_360D_Flag = 0;
		}
	}
}

//=============GENERATE PWM SIGNAL BY TIMER 1=============//

void Generate_PWM (TIM_HandleTypeDef *htim, uint32_t Channel, float Duty_Cycle)
{
	Duty_Cycle = Duty_Cycle / 100 * htim->Instance->ARR;
	__HAL_TIM_SET_COMPARE(&htim1, Channel, (uint16_t)Duty_Cycle);
}

//=============CONTROL POSITION=============//

void Control_Forward ()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
}

void Control_Reverse ()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
}

void Control_Stop ()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
}

void Control_Position ()
{
	if (duty_cycle_output > 0)
	{
		Control_Forward();
		Generate_PWM(&htim1, TIM_CHANNEL_1, duty_cycle_output);
	}
	
	if (duty_cycle_output < 0)
	{
		Control_Reverse();
		Generate_PWM(&htim1, TIM_CHANNEL_1, - duty_cycle_output);
	}	
		
	if (duty_cycle_output == 0)
	{
		Control_Stop();
		Generate_PWM(&htim1, TIM_CHANNEL_1, duty_cycle_output);
	}
}

//=============CONTROL MOTOR SPEED=============//

void Control_Motor_Speed ()
{
	Mode_04_Flag = 0;
	
	if (Control_Motor_Speed_Flag == 1)
	{
		HAL_TIM_Base_Stop_IT (&htim2);	//Disable encoder timer2 response interupt avoid PID conflict
		memcpy(Leech_Motor_Speed, &RX_Data[5], 3);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	
		Control_Forward ();
		PWM_Duty_Cycle = atoi((char*)Leech_Motor_Speed);
		Generate_PWM (&htim1, TIM_CHANNEL_1, PWM_Duty_Cycle);
	}
	else if (Control_Motor_Speed_Flag == 0)
	{
		HAL_TIM_Base_Start_IT (&htim2);
		Generate_PWM (&htim1, TIM_CHANNEL_1, 0);
		pulse = 0;
	}
}

//=============ENCODER PULSE COUNTING=============//

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0)
	{
		pulse++;
	}
	else
	{
		pulse--;
	}
}

//=============PID CACULATION & ACTIVE TIMER INTERUPT=============//

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	if (htim == &htim2)
	{
		error = setpoint - pulse;
		integral = integral + (error * Delta_t);
		derivative = (error - pre_error) / Delta_t;
		duty_cycle_output = 	(Kp * error) + (Ki * integral);
		
		if (duty_cycle_output > 100)
		{
			duty_cycle_output = 100;
		}
	
		if (duty_cycle_output < -100)
		{
			duty_cycle_output = -100;
		}
		
		pre_error = error;
		
		Control_Position();
	}
	
	if (htim == &htim3)
	{
		Timer3_Delay_Flag = 1;
	}
}

//=================================UART COMMUNICATTION==================================//
//======================================================================================//
//======================================================================================//

//=============UART RECEIVE CHECKING=============//

void HAL_UARTEx_RxEventCallback (UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{
			UART1_RX_Flag = 1;	
	}
}

//=============UART RECEIVE PROCESSING=============//

void UART1_RX_Processing ()
{
	UART1_RX_Flag = 0;
	
	HAL_UARTEx_ReceiveToIdle_DMA (&huart1, RX_Data, sizeof(RX_Data));
	
	Char_RX_Data = (char*)RX_Data;
	
	//MODE 00 - CONTROL LED//
	if ((strncmp(Char_RX_Data, "0000", 4) == 0) && (strncmp(&Char_RX_Data[9], "1", 1) == 0))
	{
		Mode_00_Flag = 1;	
		
		if (strncmp(Char_RX_Data, "00000000011", sizeof(RX_Data)) == 0)
		{
			LED_1_Status_Flag = 0;
		}
		else if (strncmp(Char_RX_Data, "00000001011", sizeof(RX_Data)) == 0)
		{
			LED_1_Status_Flag = 1;
		}
		
		if (strncmp(Char_RX_Data, "00000010011", sizeof(RX_Data)) == 0)
		{
			LED_2_Status_Flag = 0;
		}
		else if (strncmp(Char_RX_Data, "00000011011", sizeof(RX_Data)) == 0)
		{
			LED_2_Status_Flag = 1;
		}
	}
		
	//MODE 01 - CONTROL MOTOR POSITION//
	if ((strncmp(Char_RX_Data, "0001", 4) == 0) && (strncmp(&Char_RX_Data[9], "1", 1) == 0))
	{
		Mode_01_Flag = 1;
		
		if (strncmp(Char_RX_Data, "0001002D011", sizeof(RX_Data)) == 0)
		{
			Forward_45D_Flag = 1;
		}
		
		if (strncmp(Char_RX_Data, "0001005A011", sizeof(RX_Data)) == 0)
		{
			Forward_90D_Flag = 1;
		}
		
		if (strncmp(Char_RX_Data, "000100B4011", sizeof(RX_Data)) == 0)
		{
			Forward_180D_Flag = 1;
		}
		
		if (strncmp(Char_RX_Data, "00010168011", sizeof(RX_Data)) == 0)
		{
			Forward_360D_Flag = 1;
		}
	}
	
	//MODE 02 - ACTIVE/INACTIVE PULSE TRANSFERING//
	if ((strncmp(Char_RX_Data, "0002", 4) == 0) && (strncmp(&Char_RX_Data[9], "1", 1) == 0))
	{
		if (strncmp(Char_RX_Data, "00020000011", sizeof(RX_Data)) == 0)
		{
			Mode_02_Flag = 0;	
		}
	
		if (strncmp(Char_RX_Data, "00020001011", sizeof(RX_Data)) == 0) 
		{
			Mode_02_Flag = 1;	
		}
	}
	
	//MODE 03 - ACTIVE/INACTIVE STATUS TRACKING//
	if ((strncmp(Char_RX_Data, "0003", 4) == 0) && (strncmp(&Char_RX_Data[9], "1", 1) == 0))
	{
		Mode_03_Flag = 1;
		
		if (strncmp(Char_RX_Data, "00030000011", sizeof(RX_Data)) == 0)
		{
			LED_Status_Tracking_Flag = 0;
		}
		
		else if (strncmp(Char_RX_Data, "00030001011", sizeof(RX_Data)) == 0)
		{
			LED_Status_Tracking_Flag = 1;
		}
		
		if (strncmp(Char_RX_Data, "00030010011", sizeof(RX_Data)) == 0)
		{
			Motor_Position_Tracking_Flag = 0;
		}
		
		else if (strncmp(Char_RX_Data, "00030011011", sizeof(RX_Data)) == 0)
		{
			Motor_Position_Tracking_Flag = 1;
		}
	}
	
	//MODE 04 - CONTROL MOTOR SPEED//
	if ((strncmp(Char_RX_Data, "0004", 4) == 0) && (strncmp(&Char_RX_Data[9], "1", 1) == 0))
	{
		Mode_04_Flag = 1;
		
		if (memcmp(&Char_RX_Data[4], "0", 1) == 0)
		{
			Control_Motor_Speed_Flag = 1;
		}
		else if (memcmp(&Char_RX_Data[4], "FFFF", 4) == 0)
		{
			Control_Motor_Speed_Flag = 0;
		}
	}
	
	//MODE 05 - STORE LED STATUS & MOTOR SPEED//
	if ((strncmp(Char_RX_Data, "0005", 4) == 0) && (strncmp(&Char_RX_Data[9], "1", 1) == 0))
	{
		Mode_05_Flag = 1;
		
		if (strncmp(Char_RX_Data, "00050001011", sizeof(RX_Data)) == 0)
		{
			Store_LED_Status_Flag = 1;
		}
		
		if (strncmp(Char_RX_Data, "00050011011", sizeof(RX_Data)) == 0)
		{
			Store_Motor_Speed_Flag = 1;
		}		
	}
	//MODE 06 - DAC//
	if ((strncmp(Char_RX_Data, "0006", 4) == 0) && (strncmp(&Char_RX_Data[9], "1", 1) == 0))
	{
		if (memcmp(&Char_RX_Data[4], "00", 2) == 0)
		{
			Mode_06_Flag = 1;
		}
	}
}

//=============EEPROM PROCESSING=============//

void EEPROM_Processing ()
{
	Mode_05_Flag = 0;
	
	if (Store_LED_Status_Flag == 1)
	{
		for(int i = 0; i <= 2; i++)
	{
		HAL_I2C_Mem_Write(&hi2c1, 0xA0, i, 2, &LED_Status_Buffer[i], 1, 100);
		HAL_Delay(5);
	}
	}
}

//=====================================CONTROL LED======================================//
//======================================================================================//
//======================================================================================//

void LED_Control ()
{
	Mode_00_Flag = 0;
	
	if (LED_1_Status_Flag == 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		LED_Status_Buffer [0] = 0;
	}
	else if (LED_1_Status_Flag == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
		LED_Status_Buffer [0] = 1;
	}

	if (LED_2_Status_Flag == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		LED_Status_Buffer [1] = 0;
	}
	else if (LED_2_Status_Flag == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		LED_Status_Buffer [1] = 1;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	//=============ACTIVE UART RECEIVING WITH IDLE AND DMA MODE=============//
	HAL_UARTEx_ReceiveToIdle_DMA (&huart1, RX_Data, sizeof(RX_Data));
	
	//=============ACTIVE PWM MODE VIA TIMER=============//
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
	
	//=============ACTIVE TIMER INTERUPT=============//
	HAL_TIM_Base_Start_IT (&htim2);
	HAL_TIM_Base_Start_IT (&htim3);	
	
	//HAL_TIM_Base_Start_IT(&htim1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		uint8_t Opto_Digital_Input = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_15);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (UART1_RX_Flag == 1)
		{
			UART1_RX_Flag = 0;
			UART1_RX_Processing ();
		}
		
		if (Mode_00_Flag == 1)
		{
			LED_Control ();
		}
				
		if (Mode_01_Flag == 1)
		{
			Iden_Control_Motor++;
			Change_Motor_Position ();
		}
		
		if (Mode_02_Flag == 1)
		{
			Pulse_Transmit_Via_Com ();
		}
		
		if ((Mode_03_Flag == 1) && (Timer3_Delay_Flag == 1))
		{
			Active_Inactive_Status_Tracking ();
		}
		
		if (Mode_04_Flag == 1)
		{
			Control_Motor_Speed ();
		}
		
		if (Mode_05_Flag == 1) 
		{
			EEPROM_Processing ();
		}
		
		if (Recover_Data_Flag == 0)
		{
			Recover_Data ();
		}
		
		if (Mode_06_Flag == 1)
		{
			Change_Output_Voltage ();
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
