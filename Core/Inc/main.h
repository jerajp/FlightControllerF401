/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPUcalc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

//MotorStates
#define MOTORINIT 		0
#define MOTOROFF 		1
#define MOTORSTARTING 	2
#define MOTORRUNNING	3

//WIfi message type selector
#define COMMCONTROLDATA 	0  //normal control data
#define COMMPARAMACTIVE		1  //Param Send to Drone ->MSG saves in Active Structure - And Drone Returns Value
#define COMMPARAMFLASH  	2  //Get Parameter from Drone Flash
#define COMMERASEFLASHDR	3
#define COMMWRITEFLASHDR	4
#define FLASHOPERATIONTIMEOUT 2000

#define VDIVRESISTOR1	6800 //Battery voltage dividers
#define VDIVRESISTOR2 	1475
#define BATTADCTOMV	(float)( (float)((VDIVRESISTOR1 + VDIVRESISTOR2) * 3300) / (float)((VDIVRESISTOR2 * 4095)) )
#define BATTAVERAGETIME 50 //50 msec average
#define MINMSGPERSEC   10

#define T_CLR_SCREEN 	"\x1b[2J"
#define T_GO_TO			"\x1b[%d;%dH"
#define T_SHOW_CUR      "\33[?25h"
#define T_HIDE_CUR      "\33[?25l"

#define DEGREESTORADIANS (float)(0.017453292)
#define RADIANSTODEGREES (float)(57.2957795)

//DMP ABSOULUTE DEGREES OFFSET
#define PITCHOFFSET (float) (0.0)
#define ROLLOFFSET  (float) (0.0)
#define YAWOFFSET  (float) (0.0)

//Accel Manual offsets
#define ACCELPITCHMANUALOFFSET (float)(-0.7)
#define ACCELROLLMANUALOFFSET (float)(0.15)

#define GYROCALIBVALUES (int32_t) (1000)

//GYRO
//#define ACCELCONSTANT (float)(16384.0)  //uncomment for AFS_SEL=0 (+-2G) used in init procedure
//#define ACCELCONSTANT (float)(8192.0)  //uncomment for AFS_SEL=1 (+-4G) used in init procedure
#define ACCELCONSTANT (float)(4096.0)  //uncomment for AFS_SEL=2 (+-8G) used in init procedure
//#define ACCELCONSTANT (float)(2048.0)  //uncomment for AFS_SEL=3 (+-16G) used in init procedure

//#define GYROCONSTANT (float)(131.0)  //uncomment for FS_SEL=0 (+-250) used in init procedure
//#define GYROCONSTANT (float)(65.5)  //uncomment for FS_SEL=1 (+-500) used in init procedure
//#define GYROCONSTANT (float)(32.8)  //uncomment for FS_SEL=2 (+-1000) used in init procedure
#define GYROCONSTANT (float)(16.4)  //uncomment for FS_SEL=3 (+-2000) used in init procedure

//SAFETY THROTTLE CHECK AT STARTUP
#define MOTORSTARTBLOCKTHRESHOLD	200  //max throttle stick position to allow start

//FLASH DATA SPECIFIC
#define FLASHCONSTADDR  	0x08020000
#define FLASHCONSTSECTOR	FLASH_SECTOR_5

//STM32F401xC
//	Sector 0 0x0800 0000 - 0x0800 3FFF 16 Kbytes
//	Sector 1 0x0800 4000 - 0x0800 7FFF 16 Kbytes
//	Sector 2 0x0800 8000 - 0x0800 BFFF 16 Kbytes
//	Sector 3 0x0800 C000 - 0x0800 FFFF 16 Kbytes
//	Sector 4 0x0801 0000 - 0x0801 FFFF 64 Kbytes
//	Sector 5 0x0802 0000 - 0x0803 FFFF 128 Kbytes
//	Constants in Sector 5 - available room for program 128kbyte

#define CONTROLWORD 7 //control word to check if Flash constants are present
#define FLASHCONSTANTMULTIPLIER 100000

#define PARAM1 1
#define PARAM2 2
#define PARAM3 3
#define PARAM4 4
#define PARAM5 5
#define PARAM6 6
#define PARAM7 7
#define PARAM8 8
#define PARAM9 9
#define PARAM10 10
#define PARAM11 11
#define PARAM12 12
#define PARAM13 13
#define PARAM14 14
#define PARAM15 15
#define PARAM16 16
#define PARAM17 17
#define PARAM18 18
#define PARAM19 19
#define PARAM20 20
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

struct FlashDatastruct
{
	uint32_t controlData;
	float pid_p_gain_pitch;  //Gain setting for the pitch P-controller
	float pid_i_gain_pitch;  //Gain setting for the pitch I-controller
	float pid_d_gain_pitch;  //Gain setting for the pitch D-controller
	float pid_p_gain_roll;   //Gain setting for the roll P-controller
	float pid_i_gain_roll;   //Gain setting for the roll I-controller
	float pid_d_gain_roll;   //Gain setting for the roll D-controller
	float pid_p_gain_yaw;    //Gain setting for the pitch P-controller
	float pid_i_gain_yaw;    //Gain setting for the pitch I-controller
	float pid_d_gain_yaw;    //Gain setting for the pitch D-controller
	int pid_max_pitch; 		 //Maximum output of the PID-controller (+/-)
	int pid_i_max_pitch; 	 //Maximum output of the Integral part
	int pid_max_roll;        //Maximum output of the PID-controller (+/-)
	int pid_i_max_roll;      //Maximum output of the Integral part
	int pid_max_yaw;         //Maximum output of the PID-controller (+/-)
	int pid_i_max_yaw;       //Maximum output of the Integral part
	float maxpitchdegree;    //degrees
	float maxrolldegree;     //degrees
	float maxyawdegree;      //degrees
	float minthrottle;       //80counts of 1000 to keep rotors spinning
	float maxthrottle;       //800counts of 1000 (80%)
};

void WriteFlashData(uint32_t flashstartaddr,uint32_t sectornumb, struct FlashDatastruct *p);
void ReadFlashData(uint32_t flashstartaddr, struct FlashDatastruct *p);
void EraseFlashData(uint32_t sectornumb);
uint32_t CheckFlashData(uint32_t StartAddr);
void WriteString(char string[]);
void PrintCharUart (int ch);

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

extern uint32_t watch1,watch2,watch3,watch4,watch5,watch6;

extern uint32_t MotorStatus;
extern uint32_t GyroCalibStatus;

//NRF24 data
extern uint8_t nRF24_payloadTX[32]; //TX buffer
extern uint8_t nRF24_payloadRX[32]; //RX buffer
extern uint8_t RXstpaketov;
extern const uint8_t nRF24_ADDR[3]; //Address
extern uint32_t Ljoyupdown;
extern uint32_t Ljoyleftright;
extern uint32_t Djoyupdown;
extern uint32_t Djoyleftright;
extern uint32_t potenc1;
extern uint32_t potenc2;
extern uint32_t togg1;
extern uint32_t togg2;
extern uint32_t togg3;
extern uint32_t togg4;
extern uint32_t togg5;
extern uint32_t togg6;

//MPU structure
extern struct MPUstr mpuDataStr;

//Inputs
extern float ThrottleINscaled;
extern float PitchINscaled;
extern float RollINscaled;
extern float YawINscaled;

//PID motor control
extern uint32_t PWM_Mot1;
extern uint32_t PWM_Mot2;
extern uint32_t PWM_Mot3;
extern uint32_t PWM_Mot4;

//PID
extern float pid_output_pitch;
extern float pid_output_roll;
extern float pid_output_yaw;

//Flash structure
extern struct FlashDatastruct FlashDataActive;
extern struct FlashDatastruct FlashDataFlash;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define GYRO_CSN_Pin GPIO_PIN_4
#define GYRO_CSN_GPIO_Port GPIOA
#define NRF24_CE_Pin GPIO_PIN_12
#define NRF24_CE_GPIO_Port GPIOB
#define NRF24_IRQ_Pin GPIO_PIN_12
#define NRF24_IRQ_GPIO_Port GPIOA
#define NRF24_CSN_Pin GPIO_PIN_15
#define NRF24_CSN_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
