/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <unistd.h>
#include "I2Cdev.h"
#include "hml.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	sample_init();
	sample_calibration();
	sample_hold_run();
	//sample_get_filter_distance();
}







