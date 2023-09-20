/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
typedef struct
{
    struct
    { 
        signed short ch0;
        signed short ch1;
        signed short ch2;
        signed short ch3;
        unsigned char s1;
        unsigned char s2;
        
        unsigned short sw;
    }rc;
}DBUS;

typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_AUXILIARY_ALL_ID = 0x1FF,
  motor1 = 0x201,
  motor2 = 0x202,
  motor3 = 0x203,
  motor4 = 0x204,
}can_msg_id;
 
typedef struct 
{
    uint16_t angle_value;
    int16_t speed_rpm;
    int16_t real_current;
    uint8_t temperate;
    int16_t real_angle;
}motor_measure_t; 

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
