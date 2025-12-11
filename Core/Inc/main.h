/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utils_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t tamaga_encoder_data[6];
extern uint16_t ADC_Value[14];
extern uint16_t spi1_rx_buffer, spi2_rx_buffer;
extern float main_magnetic_abs, sec_magnetic_abs;
extern uint32_t tamagawa_abs;
extern float tamagawa_serial_abs;
extern float tamagawa_elec_angle, tamagawa_rotor_pos, tamagawa_output_pos;
extern float sincos_angle, serial_angle;
extern float sincos_elec_angle, sincos_rotor_pos, sincos_output_pos;
extern float magnetic_elec_angle, magnetic_rotor_pos, magnetic_output_pos;
extern float tamagawa_sincos_diff, tamagawa_magnetic_diff, sincos_magnetic_diff;
extern float motor_pole_pairs;
extern float motor_gear_ratio;
extern float s_gain_;
extern float s_offset_;
extern float c_gain_;
extern float c_offset_;
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
#define magnetic_spi_Pin GPIO_PIN_9
#define magnetic_spi_GPIO_Port GPIOC
#define tamaga_485_Pin GPIO_PIN_8
#define tamaga_485_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define SIN_MIN_VALUE 1101 
#define SIN_MAX_VALUE 3004
#define COS_MIN_VALUE 933
#define COS_MAX_VALUE 3175

#define MAIN_GEAR_RATIO 32
#define SEC_GEAR_RATIO 31

#define MAIN_ENCODER_CPR 65536
#define SEC_ENCODER_CPR 65536

#define INVERT_MAIN_DIRECTION 0
#define INVERT_SEC_DIRECTION 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
