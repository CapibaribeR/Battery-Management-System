/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Cabeçalho principal – tipos, macros e definições de pinos.
  *
  * ADIÇÕES em relação ao arquivo anterior:
  *   1. Defines de TOTAL_CELL, TOTAL_IC.
  *   2. Macros BIT_SET/CLEAR/CHECK/FLIP (usados em comm.c).
  *   3. Pin defines dos pinos BMS (SPI CS, FAULT, SDC, FAN, WAKE etc.)
  *   4. BMS_StatusTypeDef unificada com BMS_status_t do common.h
  *      (ambos os nomes são aceitos via typedef alias).
  *
  * Modificado em: Mar 2026
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

#include "stm32g4xx_nucleo.h"
#include <stdio.h>
#include "../Analog_libs/inc/common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* Alias para compatibilidade com código legado */
typedef BMS_StatusTypeDef BMS_status_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef  htim2;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* -------------------------------------------------------------------
 * Configuração da cadeia daisy chain
 * ------------------------------------------------------------------- */
#define TOTAL_CELL      16          /* Células por ADBMS6830B (fixo)      */
#define TOTAL_IC        1           /* Número de ICs na cadeia            */

/* -------------------------------------------------------------------
 * Macros de bit
 * ------------------------------------------------------------------- */
#define BIT_SET(reg, n)    ((reg) |=  (UINT32_C(1) << (n)))
#define BIT_CLEAR(reg, n)  ((reg) &= ~(UINT32_C(1) << (n)))
#define BIT_CHECK(reg, n)  ((reg) &   (UINT32_C(1) << (n)))
#define BIT_FLIP(reg, n)   ((reg) ^=  (UINT32_C(1) << (n)))

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define BMS_CS_Pin GPIO_PIN_0
#define BMS_CS_GPIO_Port GPIOC
#define BMS_SCK_Pin GPIO_PIN_13
#define BMS_SCK_GPIO_Port GPIOB
#define BMS_MISO_Pin GPIO_PIN_14
#define BMS_MISO_GPIO_Port GPIOB
#define BMS_MOSI_Pin GPIO_PIN_15
#define BMS_MOSI_GPIO_Port GPIOB
#define BMS_CS2_Pin GPIO_PIN_7
#define BMS_CS2_GPIO_Port GPIOC
#define BMS_LED_Pin GPIO_PIN_6
#define BMS_LED_GPIO_Port GPIOC
#define SDC_CTRL_Pin GPIO_PIN_8
#define SDC_CTRL_GPIO_Port GPIOC
#define SDC_SENSE_Pin GPIO_PIN_9
#define SDC_SENSE_GPIO_Port GPIOC
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
