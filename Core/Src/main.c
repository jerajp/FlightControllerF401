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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

//UART DEBUG
char UartTXbuff0[100];

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */



  HAL_TIM_Base_Start_IT(&htim2);//Start at the END of Main Initialization

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */

	  //own function is used for UART TX, very basic function for direct Register write one char at the time
	  //HAL_UART_Transmit ( &huart1, UartTXbuff0, strlen( UartTXbuff0 ), 1 ); //removed ->missing bytes on occasion

	  HAL_Delay(100);

	  sprintf(UartTXbuff0,T_HIDE_CUR);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0,T_GO_TO,1,1); //Go to start Screen
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0,T_CLR_SCREEN);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "test test");
	  WriteString(UartTXbuff0);

	  /*sprintf(UartTXbuff0, "Motor Status %u MPU=%u \n\r",MotorStatus,MPU6050rezulatat);
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

	  sprintf(UartTXbuff0, "Pitch=%.2f deg\n\r",mpu6050DataStr.Pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll=%.2f deg\n\r",mpu6050DataStr.Roll);
	  WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Pitch=%.2f deg\n\r",mpu6050DataStr.Angle_Gyro_Pitch);
	  //WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Roll=%.2f deg\n\r",mpu6050DataStr.Angle_Gyro_Roll);
	  //WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw-Gyro=%.2f deg\n\r",mpu6050DataStr.Angle_Gyro_Yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Accel Pitch=%.2f deg\n\r",mpu6050DataStr.Angle_Accel_Pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Accel Roll=%.2f deg\n\r",mpu6050DataStr.Angle_Accel_Roll);
	  WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro X speed=%.2f deg/s\n\r",mpu6050DataStr.AngleSpeed_Gyro_X);
	 //WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Y speed=%.2f deg/s\n\r",mpu6050DataStr.AngleSpeed_Gyro_Y);
	  //WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "Gyro Z speed=%.2f deg/s\n\r",mpu6050DataStr.AngleSpeed_Gyro_Z);
	  //WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Acc Raw %d %d %d \n\r",mpu6050DataStr.Accelerometer_X_RAW,mpu6050DataStr.Accelerometer_Y_RAW,mpu6050DataStr.Accelerometer_Z_RAW);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Gyro Raw %d %d %d \n\r",mpu6050DataStr.Gyroscope_X_RAW,mpu6050DataStr.Gyroscope_Y_RAW,mpu6050DataStr.Gyroscope_Z_RAW);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Gyro Offset %.1f %.1f %.1f \n\r",mpu6050DataStr.Offset_Gyro_X,mpu6050DataStr.Offset_Gyro_Y,mpu6050DataStr.Offset_Gyro_Z);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Gyro Cal %.1f %.1f %.1f \n\r",mpu6050DataStr.Gyroscope_X_Cal,mpu6050DataStr.Gyroscope_Y_Cal,mpu6050DataStr.Gyroscope_Z_Cal);
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

	  sprintf(UartTXbuff0, "watch %.2f %.2f %.2f %.2f \n\r",watch1fl,watch2fl,watch3fl,watch4fl);
	  WriteString(UartTXbuff0);*/

    /* USER CODE BEGIN 3 */
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
