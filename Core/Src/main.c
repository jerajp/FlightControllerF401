/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "nrf24.h"
#include "MPU9250.h"
#include "MPUcalc.h"
#include "stm32f4xx_it.h"
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

/* USER CODE BEGIN PV */

uint32_t watch1,watch2,watch3,watch4,watch5,watch6;

//UART DEBUG
char UartTXbuff0[100];

//NRF24
uint8_t nRF24_payloadTX[32]; //TX buffer
uint8_t nRF24_payloadRX[32]; //RX buffer
const uint8_t nRF24_ADDR[3] = {5, 3, 5 }; //Address
uint32_t wifiOK;
uint8_t RXstpaketov=0;

//NRF24 data
uint32_t Ljoyupdown;
uint32_t Ljoyleftright;
uint32_t Djoyupdown;
uint32_t Djoyleftright;
uint32_t potenc1;
uint32_t potenc2;
uint32_t togg1;
uint32_t togg2;
uint32_t togg3;
uint32_t togg4;
uint32_t togg5;
uint32_t togg6;

//MOTOR
uint32_t MotorStatus=MOTORINIT;
uint32_t GyroCalibStatus=0;

//Gyro
struct MPUstr mpuDataStr;

struct FlashDatastruct FlashDataDefault; //Default constant embedded in Code
struct FlashDatastruct FlashDataFlash;  //Constants read from Flash
struct FlashDatastruct FlashDataActive; //Active constants

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start(&hadc1);

  HAL_Delay(400);//wait for stable power

  //NRF24 INIT-----------------------------------
  SPI2->CR1|=SPI_CR1_SPE; //enable SPI

  nRF24_CE_L(); // RX/TX disabled

  wifiOK=nRF24_Check();

  nRF24_Init(); //Default init

  // Disable ShockBurst for all RX pipes
  nRF24_DisableAA(0xFF);

  // Set RF channel
  nRF24_SetRFChannel(15); //2400Mhz + 15Mhz

  // Set data rate
  nRF24_SetDataRate(nRF24_DR_250kbps);

  // Set CRC scheme
  nRF24_SetCRCScheme(nRF24_CRC_1byte);

  // Set address width, its common for all pipes (RX and TX)
  nRF24_SetAddrWidth(3);

  nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); //PROGRAM PIPE1!! for RX

  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 8); // Auto-ACK: disabled


  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR);

  // Set TX power
  nRF24_SetTXPower(nRF24_TXPWR_18dBm);

  // Set operational mode
  nRF24_SetOperationalMode(nRF24_MODE_RX);

  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();

  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);

  nRF24_CE_H();//Enable RX

  //Gyro Init
  MPU9250_Init();


  HAL_Delay(5000);//wait to connect battery

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  MotorStatus=MOTOROFF;

  HAL_TIM_Base_Start_IT(&htim2);//Start at the END of Main Initialization

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //own function is used for UART TX, very basic function for direct Register write one char at the time
	  //HAL_UART_Transmit ( &huart1, UartTXbuff0, strlen( UartTXbuff0 ), 1 ); //removed ->missing bytes on occasion

	  HAL_Delay(100);

	  sprintf(UartTXbuff0,T_HIDE_CUR);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0,T_GO_TO,1,1); //Go to start Screen
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0,T_CLR_SCREEN);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Motor Status %u \n\r",MotorStatus);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "ThrottleIN %.2f \n\r",ThrottleINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "PitchIN %.2f \n\r",PitchINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "RollIN %.2f \n\r",RollINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "YawIN %.2f \n\r",YawINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch=%.2f deg\n\r",mpuDataStr.Pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll=%.2f deg\n\r",mpuDataStr.Roll);
	  WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Pitch=%.2f deg\n\r",mpuDataStr.Angle_Gyro_Pitch);
	  //WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Roll=%.2f deg\n\r",mpuDataStr.Angle_Gyro_Roll);
	  //WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw-Gyro=%.2f deg\n\r",mpuDataStr.Angle_Gyro_Yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Accel Pitch=%.2f deg\n\r",mpuDataStr.Angle_Accel_Pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Accel Roll=%.2f deg\n\r",mpuDataStr.Angle_Accel_Roll);
	  WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro X speed=%.2f deg/s\n\r",mpuDataStr.AngleSpeed_Gyro_X);
	 //WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Y speed=%.2f deg/s\n\r",mpuDataStr.AngleSpeed_Gyro_Y);
	  //WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Z speed=%.2f deg/s\n\r",mpuDataStr.AngleSpeed_Gyro_Z);
	  //WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Acc Raw %d %d %d \n\r",mpuDataStr.Accelerometer_X_RAW,mpuDataStr.Accelerometer_Y_RAW,mpuDataStr.Accelerometer_Z_RAW);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Gyro Raw %d %d %d \n\r",mpuDataStr.Gyroscope_X_RAW,mpuDataStr.Gyroscope_Y_RAW,mpuDataStr.Gyroscope_Z_RAW);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Gyro Offset %.1f %.1f %.1f \n\r",mpuDataStr.Offset_Gyro_X,mpuDataStr.Offset_Gyro_Y,mpuDataStr.Offset_Gyro_Z);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Gyro Cal %.1f %.1f %.1f \n\r",mpuDataStr.Gyroscope_X_Cal,mpuDataStr.Gyroscope_Y_Cal,mpuDataStr.Gyroscope_Z_Cal);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "PWM 1:%u  2:%u  3:%u  4:%u   \n\r",PWM_Mot1,PWM_Mot2,PWM_Mot3,PWM_Mot4);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Toggle %d %d %d %d %d %d  ",togg1,togg2,togg3,togg4,togg5,togg6);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Potenc %d %d  ",potenc1,potenc2);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "YL %d %d  YD %d %d \n\r",Ljoyupdown, Ljoyleftright, Djoyupdown, Djoyleftright);
	  WriteString(UartTXbuff0);

	  //ACTIVE PID CONSTANTS
	  sprintf(UartTXbuff0, "\n\rPID Constants Active  \n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch P=%.2f I=%.5f D=%.2f \n\r",FlashDataActive.pid_p_gain_pitch, FlashDataActive.pid_i_gain_pitch, FlashDataActive.pid_d_gain_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll P=%.2f I=%.5f D=%.2f \n\r",FlashDataActive.pid_p_gain_roll, FlashDataActive.pid_i_gain_roll, FlashDataActive.pid_d_gain_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw P=%.2f I=%.5f D=%.2f \n\r",FlashDataActive.pid_p_gain_yaw, FlashDataActive.pid_i_gain_yaw, FlashDataActive.pid_d_gain_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max %d Max I %d \n\r",FlashDataActive.pid_max_pitch, FlashDataActive.pid_i_max_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max %d Max I %d \n\r",FlashDataActive.pid_max_roll, FlashDataActive.pid_i_max_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max %d Max I %d \n\r",FlashDataActive.pid_max_yaw, FlashDataActive.pid_i_max_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max Degree %.2f \n\r",FlashDataActive.maxpitchdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max Degree %.2f \n\r",FlashDataActive.maxrolldegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max Degree %.2f \n\r",FlashDataActive.maxyawdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Throttle Max %.2f Min I %.2f \n\r",FlashDataActive.maxthrottle, FlashDataActive.minthrottle);
	  WriteString(UartTXbuff0);


	  //FLASH PID CONSTANTS
	  sprintf(UartTXbuff0, "\n\rPID Constants Flash  \n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch P=%.2f I=%.5f D=%.2f \n\r",FlashDataFlash.pid_p_gain_pitch, FlashDataFlash.pid_i_gain_pitch, FlashDataFlash.pid_d_gain_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll P=%.2f I=%.5f D=%.2f \n\r",FlashDataFlash.pid_p_gain_roll, FlashDataFlash.pid_i_gain_roll, FlashDataFlash.pid_d_gain_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw P=%.2f I=%.5f D=%.2f \n\r",FlashDataFlash.pid_p_gain_yaw, FlashDataFlash.pid_i_gain_yaw, FlashDataFlash.pid_d_gain_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max %d Max I %d \n\r",FlashDataFlash.pid_max_pitch, FlashDataFlash.pid_i_max_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max %d Max I %d \n\r",FlashDataFlash.pid_max_roll, FlashDataFlash.pid_i_max_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max %d Max I %d \n\r",FlashDataFlash.pid_max_yaw, FlashDataFlash.pid_i_max_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max Degree %.2f \n\r",FlashDataFlash.maxpitchdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max Degree %.2f \n\r",FlashDataFlash.maxrolldegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max Degree %.2f \n\r",FlashDataFlash.maxyawdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Throttle Max %.2f Min I %.2f \n\r",FlashDataFlash.maxthrottle, FlashDataFlash.minthrottle);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "watch %d %d %d %d %d %d\n\r",watch1,watch2,watch3,watch4,watch5,watch6);
	  WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "watch %.2f %.2f %.2f %.2f \n\r",watch1fl,watch2fl,watch3fl,watch4fl);
	  //WriteString(UartTXbuff0);


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */

//Write Data into Flash starting from given address------------------------------------------------------------------
void WriteFlashData(uint32_t StartAddr, struct FlashDatastruct *p)
{
	FLASH_EraseInitTypeDef EraseInitStruct;

	uint32_t PageError;

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = StartAddr;
	EraseInitStruct.NbSectors     = 1;

	HAL_FLASH_Unlock();

	//FLASH_PageErase(0x800FC00); //doesn't handle all registers PER regiser in CR is not cleared

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr, p->controlData);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+4, (uint32_t) ( p->pid_p_gain_pitch * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+8, (uint32_t) ( p->pid_i_gain_pitch * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+12,(uint32_t) ( p->pid_d_gain_pitch * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+16,(uint32_t) ( p->pid_p_gain_roll * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+20,(uint32_t) ( p->pid_i_gain_roll * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+24,(uint32_t) ( p->pid_d_gain_roll * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+28,(uint32_t) ( p->pid_p_gain_yaw * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+32,(uint32_t) ( p->pid_i_gain_yaw * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+36,(uint32_t) ( p->pid_d_gain_yaw * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+40, p->pid_max_pitch);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+44, p->pid_i_max_pitch);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+48, p->pid_max_roll);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+52, p->pid_i_max_roll);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+56, p->pid_max_yaw);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+60, p->pid_i_max_yaw);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+64,(uint32_t)(p->maxpitchdegree) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+68,(uint32_t)(p->maxrolldegree) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+72,(uint32_t)(p->maxyawdegree) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+76,(uint32_t)(p->minthrottle) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+80,(uint32_t)(p->maxthrottle) );

	HAL_FLASH_Lock();
}

void EraseFlashData(uint32_t StartAddr)
{
	FLASH_EraseInitTypeDef EraseInitStruct;

	uint32_t PageError;

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = StartAddr;
	EraseInitStruct.NbSectors     = 1;

	HAL_FLASH_Unlock();

	//FLASH_PageErase(0x800FC00); //doesn't handle all registers PER regiser in CR is not cleared

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	HAL_FLASH_Lock();
}


//Check if Data on given address matches control word
uint32_t CheckFlashData(uint32_t StartAddr)
{
	return *(( uint32_t *) (StartAddr) );
}

//Read Data from Flash
void ReadFlashData(uint32_t StartAddr, struct FlashDatastruct *p)
{
	p->controlData= *(( uint32_t *) (StartAddr) );
	p->pid_p_gain_pitch=(float)( (*(( uint32_t *) (StartAddr+4) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_i_gain_pitch=(float)( (*(( uint32_t *) (StartAddr+8) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_d_gain_pitch=(float)( (*(( uint32_t *) (StartAddr+12) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_p_gain_roll=(float)( (*(( uint32_t *) (StartAddr+16) ))  )/FLASHCONSTANTMULTIPLIER;
	p->pid_i_gain_roll=(float)( (*(( uint32_t *) (StartAddr+20) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_d_gain_roll=(float)( (*(( uint32_t *) (StartAddr+24) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_p_gain_yaw=(float)( (*(( uint32_t *) (StartAddr+28) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_i_gain_yaw=(float)( (*(( uint32_t *) (StartAddr+32) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_d_gain_yaw=(float)( (*(( uint32_t *) (StartAddr+36) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_max_pitch=*(( uint32_t *) (StartAddr+40) );
	p->pid_i_max_pitch=*(( uint32_t *) (StartAddr+44) );
	p->pid_max_roll=*(( uint32_t *) (StartAddr+48) );
	p->pid_i_max_roll=*(( uint32_t *) (StartAddr+52) );
	p->pid_max_yaw=*(( uint32_t *) (StartAddr+56) );
	p->pid_i_max_yaw=*(( uint32_t *) (StartAddr+60) );
	p->maxpitchdegree=(float)( (*(( uint32_t *) (StartAddr+64) )) );
	p->maxrolldegree=(float)( (*(( uint32_t *) (StartAddr+68) )) );
	p->maxyawdegree=(float)( (*(( uint32_t *) (StartAddr+72) )) );
	p->minthrottle=(float)( (*(( uint32_t *) (StartAddr+76) )) );
	p->maxthrottle=(float)( (*(( uint32_t *) (StartAddr+80) )) );
}

/* Print character to array */
void WriteString(char string[])
{
  unsigned int i=0;
  while (string[i])
  {
	PrintCharUart (string[i]);
	i++;
  }
}

/* Write character to Serial Port    */
void PrintCharUart (int ch)
{
	while (!(USART1->SR & USART_SR_TXE));
	USART1->DR = (USART_DR_DR & ch);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
