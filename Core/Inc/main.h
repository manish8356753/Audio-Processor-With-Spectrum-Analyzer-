/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define Fc_pin_Pin GPIO_PIN_0
#define Fc_pin_GPIO_Port GPIOA
#define filterUpdate_button_Pin GPIO_PIN_12
#define filterUpdate_button_GPIO_Port GPIOC
#define filterUpdate_button_EXTI_IRQn EXTI15_10_IRQn
#define RST_Pin GPIO_PIN_2
#define RST_GPIO_Port GPIOD
#define DCX_Pin GPIO_PIN_4
#define DCX_GPIO_Port GPIOD
#define CSX_Pin GPIO_PIN_6
#define CSX_GPIO_Port GPIOD

#define BLOCK_SIZE_FLOAT 2048
#define BLOCK_SIZE_U16 8192

#define NO_FILTER 0
#define LPF_FILTER 1
#define HPF_FILTER 2

#define FREQ_SLOTS 9
#define INIT_Y_VAL 0

#define POT_TURN_LEVELS 80
#define POT_LEVELS_FREQ 50
#define POT_SAMPLES 50
#define MAX_POT 4095

#define OFFSET 100

#define MIN_LEVEL 3
#define MAX_LEVEL 320

#define FALSE 0
#define TRUE 1

void StartApplyFilterTxTask(void *argument);
void StartLCDdisplayTask(void *argument);
void StartDoFFTtask(void *argument);
void StartupdateCutOffTask(void *argument);
void StartCalcBiquadTask(void *argument);
void SystemClock_Config(void);
void LCDdisplay_Init(void);
float complexABS(float real, float compl);

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
