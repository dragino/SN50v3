/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains all hardware driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /******************************************************************************
  * @file    bsp.h
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   contains all hardware driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H__
#define __BSP_H__
#include <stdint.h>
#include "stm32wlxx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* --------------------------- USER KEY HW definition -------------------------------*/
//#define USE_LHT65_BASE_BOARD
#define USE_RS485_BL_BASE_BOARD
//#define USE_LSN50_V3_BASE_BOARD
//#define USE_SIB_BASE_BOARD

// LA66<.....>LA66S

// PA1        PC13
// PA0        PB1
// PA3        PB2
// PA2        BOOT0
// PA6        PA13
// PA7        PA14
// PA14       PA12
// PA15       PA11
// PB7        PA0
// PB9        PA9
// PB8        PA10
// PB11       PC3
// PB10       PC2
// PB13       PB13

// PB12       PB12
// PB14       PB14
// PB15       PB15
// PC12       PB7
// PC13       PB6
// PD14       PC5
// PD10       PB8
// PA12       NA
// PA13       NA
// PA9        PB9

// PC3        PB10
// PC4        PB11
// PC0        PA1
// PC1        PC4
// PC5        PC1
// PC8        PA8
// PC9        PB5
// PA4        PB4
// PA5        PA15
// PA8        PB3
// PA11       NC
// PB1        PA2
// PB0        PA3


#define GPIO_USERKEY_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define GPIO_USERKEY_PORT                GPIOA
#define GPIO_USERKEY_PIN                 GPIO_PIN_8

#define LED_RGB_PORT                     GPIOB	
#define LED_RED_PIN                      GPIO_PIN_13 
#define LED_BLUE_PIN    			           GPIO_PIN_12 
#define LED_GREEN_PIN    			           GPIO_PIN_14

#define GPIO_BT24_RESET_PORT             GPIOB
#define GPIO_BT24_RESET_PIN              GPIO_PIN_8

#define GPIO_BT24_KEY_PORT               GPIOC
#define GPIO_BT24_KEY_PIN                GPIO_PIN_5

#define GPIO_BT24_LINK_PORT              GPIOC
#define GPIO_BT24_LINK_PIN               GPIO_PIN_4

#define GPIO_BT24_WORK_PORT              GPIOC
#define GPIO_BT24_WORK_PIN               GPIO_PIN_1

#define GPIO_SOLAR_POWER_PORT            GPIOA
#define GPIO_SOLAR_POWER_PIN    			   GPIO_PIN_15

#define POWER_5V_PORT            	       GPIOA	
#define POWER_5V_PIN    			           GPIO_PIN_0

/* ---------------------------  GPIO EXTI definition -------------------------------*/
#define GPIO_EXTI4_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_EXTI4_PORT           GPIOB	 
#define GPIO_EXTI4_PIN            GPIO_PIN_4
#define GPIO_EXTI8_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIO_EXTI8_PORT           GPIOB	 
#define GPIO_EXTI8_PIN            GPIO_PIN_3
#define GPIO_EXTI15_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIO_EXTI15_PORT          GPIOB	 
#define GPIO_EXTI15_PIN           GPIO_PIN_15

/* ---------------------------  ADC_definition -------------------------------*/
#define GPIO_ADC_IN1_PORT         GPIOB
#define GPIO_ADC_IN1_PIN          GPIO_PIN_4
#define GPIO_ADC_IN1_CHANNEL      ADC_CHANNEL_3
#define GPIO_ADC_IN2_PORT         GPIOA
#define GPIO_ADC_IN2_PIN    		  GPIO_PIN_15
#define GPIO_ADC_IN2_CHANNEL      ADC_CHANNEL_11
#define GPIO_ADC_IN3_PORT         GPIOB
#define GPIO_ADC_IN3_PIN    		  GPIO_PIN_3
#define GPIO_ADC_IN3_CHANNEL      ADC_CHANNEL_2

/*Definition LoRa Sensor Pins*/
/* ---------------------------  DS18B20-1 HW definition -------------------------------*/
#define DOUT1_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define DOUT1_PORT          GPIOB	 
#define DOUT1_PIN           GPIO_PIN_6
#define DOUT1_READ()        HAL_GPIO_ReadPin(DOUT1_PORT,DOUT1_PIN)
#define DOUT1_0             HAL_GPIO_WritePin(DOUT1_PORT,DOUT1_PIN,GPIO_PIN_RESET)
#define DOUT1_1             HAL_GPIO_WritePin(DOUT1_PORT,DOUT1_PIN,GPIO_PIN_SET)

/* ---------------------------  DS18B20-2 HW definition -------------------------------*/
#define DOUT2_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define DOUT2_PORT          GPIOA	 
#define DOUT2_PIN           GPIO_PIN_9
#define DOUT2_READ()        HAL_GPIO_ReadPin(DOUT2_PORT,DOUT2_PIN)
#define DOUT2_0             HAL_GPIO_WritePin(DOUT2_PORT,DOUT2_PIN,GPIO_PIN_RESET)
#define DOUT2_1             HAL_GPIO_WritePin(DOUT2_PORT,DOUT2_PIN,GPIO_PIN_SET)

/* ---------------------------  DS18B20-3 HW definition -------------------------------*/
#define DOUT3_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define DOUT3_PORT          GPIOA	 
#define DOUT3_PIN           GPIO_PIN_10
#define DOUT3_READ()        HAL_GPIO_ReadPin(DOUT3_PORT,DOUT3_PIN)
#define DOUT3_0             HAL_GPIO_WritePin(DOUT3_PORT,DOUT3_PIN,GPIO_PIN_RESET)
#define DOUT3_1             HAL_GPIO_WritePin(DOUT3_PORT,DOUT3_PIN,GPIO_PIN_SET)

/* --------------------------- I2C HW definition -------------------------------*/
#define I2Cx                            I2C2
#define RCC_PERIPHCLK_I2Cx              RCC_PERIPHCLK_I2C2
#define RCC_I2CxCLKSOURCE_SYSCLK        RCC_I2C2CLKSOURCE_SYSCLK
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C2_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_12
#define I2Cx_SCL_GPIO_PORT              GPIOA
#define I2Cx_SDA_PIN                    GPIO_PIN_11
#define I2Cx_SDA_GPIO_PORT              GPIOA
#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C2

typedef struct{
	
  bool in1;
	
	bool exit_pa4;
	
	bool exit_pa8;
	
	bool exit_pb15;
	
	float temp1;

	float temp2;

	float temp3;
	
	uint16_t ADC_4; 

	uint16_t ADC_5; 
	
	uint16_t ADC_8; 

	float temp_sht;
	
	float hum_sht;

	float temp_tmp117;
	
	uint16_t illuminance;	
	
  uint16_t distance_mm;
	
	uint16_t distance_signal_strengh;
	
	int32_t Weight;
	
	uint16_t bat_mv;
	
	uint32_t count_pa4;
	
	uint32_t count_pa8;
	
  uint16_t pwm_freq;
	
	uint16_t pwm_duty;
	
  /**more may be added*/
} sensor_t;

typedef struct{
	
	float temp_sht;
	float hum_sht;

} sht3x_data_t;


/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  initialises the sensor
 *
 * @note
 * @retval None
 */
void  BSP_sensor_Init( void  );

/**
 * @brief  sensor  read. 
 *
 * @note none
 * @retval sensor_data
 */
void BSP_sensor_Read( sensor_t *sensor_data , uint8_t message);

uint16_t battery_voltage_measurement(void);

void sht3x_get_data(sht3x_data_t *sht3x_data);

void GPIO_EXTI8_IoInit(uint8_t state);

void display_sht31_message(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
