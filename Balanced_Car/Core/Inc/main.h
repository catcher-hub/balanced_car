/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//�򻯱����Ķ���
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

//C���Կ����ͷ�ļ�
#include "stdio.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_RES_Pin GPIO_PIN_3
#define OLED_RES_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_4
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SDAB5_Pin GPIO_PIN_5
#define OLED_SDAB5_GPIO_Port GPIOB
#define OLED_SDAB6_Pin GPIO_PIN_6
#define OLED_SDAB6_GPIO_Port GPIOB
#define IIC_SCL_Pin GPIO_PIN_8
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_9
#define IIC_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
