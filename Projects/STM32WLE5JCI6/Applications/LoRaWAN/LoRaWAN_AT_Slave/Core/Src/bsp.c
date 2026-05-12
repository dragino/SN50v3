 /******************************************************************************
  * @file    bsp.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   manages the sensors on the application
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
  
  /* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <math.h>
#include "bsp.h"
#include "sys_app.h"
#include "ds18b20.h"
#include "sht3x.h"
#include "adc_if.h"
#include "usart.h"
#include "usart_if.h"
#include "mcp3426.h"
#include "lora_app.h"
#include "mw_log_conf.h"
#include "sht3x.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
void GPIO_Sensor_IoInit(void);
void rename_ble(void);
static uint32_t AD_code3=0;

extern uint8_t inmode,inmode2,inmode3;
extern uint32_t batteryLevel_mV;
static bool sht31_status=1;
static uint8_t rxdatas[6];
float sht31_tem,sht31_hum;
I2C_HandleTypeDef I2cHandle2;
#define I2C_TIMING    0x10A13E56

uint32_t ADCxConvertedValues = 0, batterymv = 0, value = 0;

#if defined (USE_RS485_BL_BASE_BOARD) || defined (USE_SIB_BASE_BOARD)
uint8_t data_uart_soil[256] = {0xFE, 0x03, 0x00, 0x00, 0x00, 0x03, 0x11, 0xC4};
#endif

uint16_t bat_record=3400;
extern uint8_t extpower_logic;
static float gxht30_temp_record=10,gxht30_hum_record=40,tmp117_temp_record=10,ext_sht31_temp_record=10,ext_sht31_hum_record=40;
extern uint8_t Ext;
extern uint16_t power_time;
extern uint32_t count;
extern bool ds18b20_connect_status;

void ADC_Dxpd(uint32_t adc_nums[])
{
	uint32_t i, j, temp, isSorted;  
	for(i=0; i<6-1; i++)
	{
		isSorted = 1;  
		for(j=0; j<6-1-i; j++)
		{
			if(adc_nums[j] > adc_nums[j+1])
			{
				temp = adc_nums[j];
				adc_nums[j] = adc_nums[j+1];
				adc_nums[j+1] = temp;
				isSorted = 0; 
			}
		}
		if(isSorted) break;
	}
}

uint16_t ADC_Average(uint32_t adc_nums[])
{
	uint32_t sum = 0;

	ADC_Dxpd(adc_nums);
	
	for(uint8_t i=1; i<5; i++)
	{
		sum = sum + adc_nums[i];
	}
	
	return sum/4;
}

void rename_ble(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_BT24_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_BT24_RESET_PORT, &GPIO_InitStruct);	
  HAL_GPIO_WritePin(GPIO_BT24_RESET_PORT,GPIO_BT24_RESET_PIN,GPIO_PIN_RESET);	
  HAL_Delay(300);	 
  HAL_GPIO_WritePin(GPIO_BT24_RESET_PORT,GPIO_BT24_RESET_PIN,GPIO_PIN_SET);	
		 
  HAL_Delay(200);	

	uint8_t buf[8];
	lora_config_deveui_get(buf);	
	MW_LOG(TS_OFF, VLEVEL_M, "AT+NAME%02X%02X%02X%02X%02X%02X%02X%02X\r\n", 
							buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

	HAL_Delay(50);
		
	HAL_GPIO_WritePin(GPIO_BT24_RESET_PORT,GPIO_BT24_RESET_PIN,GPIO_PIN_RESET);	
	HAL_Delay(200);	 
	HAL_GPIO_WritePin(GPIO_BT24_RESET_PORT,GPIO_BT24_RESET_PIN,GPIO_PIN_SET);	
}

void BSP_sensor_Read( sensor_t *sensor_data , uint8_t message)
{
	batteryLevel_mV=battery_voltage_measurement();
  if((sensor_data->bat_mv>4300)||(sensor_data->bat_mv<2500)||(sensor_data->bat_mv-bat_record>200)||(bat_record-sensor_data->bat_mv>200))
	{
	  sensor_data->bat_mv=battery_voltage_measurement();
		
		if((sensor_data->bat_mv>4300)||(sensor_data->bat_mv<2500))
		{
			if((sensor_data->bat_mv-bat_record>200)||(bat_record-sensor_data->bat_mv>200))
			{
				sensor_data->bat_mv=battery_voltage_measurement();
				
				if((sensor_data->bat_mv>4300)||(sensor_data->bat_mv<2500))
				{
					sensor_data->bat_mv=bat_record;					
				}
			}
		}
	}
	
	if((sensor_data->bat_mv<4300)&&(sensor_data->bat_mv>2500))
	{
		bat_record=sensor_data->bat_mv;
	}
	
	sensor_data->bat_mv=batteryLevel_mV;

	sht3x_data_t sht3x_data;
	sht3x_get_data(&sht3x_data);
	sensor_data->temp_sht=sht3x_data.temp_sht;
	sensor_data->hum_sht=sht3x_data.hum_sht;

	HAL_Delay(50);
  uint32_t adcdata[3][6];	
	for(uint8_t z=0;z<6;z++)
	{
	 uint32_t ADCxConvertedValues=0;
	 
   ADCxConvertedValues = ADC_ReadChannels( GPIO_ADC_IN1_CHANNEL );	
	 adcdata[2][z] = __LL_ADC_CALC_DATA_TO_VOLTAGE(batteryLevel_mV, ADCxConvertedValues, LL_ADC_RESOLUTION_12B);
	 HAL_Delay(10);				 
	}		 
	AD_code3=ADC_Average(adcdata[2]);		 
 sensor_data->ADC_4=AD_code3;  

 sensor_data->temp1=DS18B20_GetTemp_SkipRom(1);
	
 sensor_data->exit_pa8= HAL_GPIO_ReadPin(GPIO_EXTI8_PORT,GPIO_EXTI8_PIN);
	
 sensor_data->in1=HAL_GPIO_ReadPin(GPIO_EXTI15_PORT,GPIO_EXTI15_PIN);
	
 if(message==1)
 {
	MW_LOG(TS_OFF, VLEVEL_M,"\r\nBat_voltage:%d mV\r\n",sensor_data->bat_mv);
  HAL_Delay(20);
	
	MW_LOG(TS_OFF, VLEVEL_M,"PA8_status:%d\r\n",sensor_data->exit_pa8);
	MW_LOG(TS_OFF, VLEVEL_M,"PB15_status:%d\r\n",sensor_data->in1);
	 
	MW_LOG(TS_OFF, VLEVEL_M,"ADC_PA4:%.3f V\r\n",AD_code3/1000.0);
	HAL_Delay(20);
	
  MW_LOG(TS_OFF, VLEVEL_M,"SHT3x_temp:%.1f,SHT3x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
	HAL_Delay(20);
	 
	if((sensor_data->temp1>=-55)&&(sensor_data->temp1<=125))
	{
		MW_LOG(TS_OFF, VLEVEL_M,"DS18B20_temp1:%.1f\r\n",sensor_data->temp1);
	}
	else
	{
		MW_LOG(TS_OFF, VLEVEL_M,"DS18B20_temp1:null\r\n");
	}
 }
}

void BSP_sensor_Init(void)
{
  GPIO_Sensor_IoInit();
	GPIO_EXTI8_IoInit(inmode);
	rename_ble();	
	SHT3X_Init(0x44);
}

void GPIO_Sensor_IoInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_USERKEY_PIN;
	HAL_GPIO_Init(GPIO_USERKEY_PORT, &GPIO_InitStruct);
	
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_BT24_KEY_PIN;
	HAL_GPIO_Init(GPIO_BT24_KEY_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIO_BT24_KEY_PORT, GPIO_BT24_KEY_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_BT24_LINK_PIN | GPIO_BT24_WORK_PIN;
	HAL_GPIO_Init(GPIO_BT24_LINK_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN;
	HAL_GPIO_Init(LED_RGB_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_RGB_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN, GPIO_PIN_RESET);	

	GPIO_InitStruct.Mode =GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_SOLAR_POWER_PIN;
	HAL_GPIO_Init(GPIO_SOLAR_POWER_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = POWER_5V_PIN;
	HAL_GPIO_Init(POWER_5V_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(POWER_5V_PORT, POWER_5V_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Mode =GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_ADC_IN1_PIN;
	HAL_GPIO_Init(GPIO_ADC_IN1_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode =GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_ADC_IN2_PIN;
	HAL_GPIO_Init(GPIO_ADC_IN2_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Mode =GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_ADC_IN3_PIN;
	HAL_GPIO_Init(GPIO_ADC_IN3_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_EXTI15_PIN;
	HAL_GPIO_Init(GPIO_EXTI15_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Mode =GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_SOLAR_POWER_PIN;
	HAL_GPIO_Init(GPIO_SOLAR_POWER_PORT, &GPIO_InitStruct);
	
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStruct.Pin = GPIO_EXTI8_PIN;
//	HAL_GPIO_Init(GPIO_EXTI8_PORT, &GPIO_InitStruct);
//	
//	HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
//  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void GPIO_EXTI8_IoInit(uint8_t state)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_EXTI4_CLK_ENABLE();	
	
	GPIO_InitStruct.Pin = GPIO_EXTI8_PIN;	
	
	if(state == 0)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 1)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 2)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 3)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}	
	
	HAL_GPIO_Init(GPIO_EXTI8_PORT, &GPIO_InitStruct);
	
	if(state != 0)
	{
		/* EXTI interrupt init*/
		HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}
}

uint16_t battery_voltage_measurement(void)
{
	  uint16_t bat_mv=0;
	
		bat_mv=SYS_GetBatteryLevel();		
	  #if defined LB_LS
    ADCxConvertedValues = ADC_ReadChannels(ADC_CHANNEL_11);
	  bat_mv = __LL_ADC_CALC_DATA_TO_VOLTAGE(bat_mv, ADCxConvertedValues, LL_ADC_RESOLUTION_12B);
	  bat_mv = bat_mv * 6;
	  #endif		
	  return bat_mv;
}

void sht3x_get_data(sht3x_data_t *sht3x_data)
{	
	etError   error;
  uint8_t times = 0;
	
	do
	{
		times++;
	  error = SHT3X_GetTempAndHumiClkStretch(&sht3x_data->temp_sht, &sht3x_data->hum_sht, REPEATAB_HIGH, 50);
	}while(times < 4 && error != NO_ERROR);

  if(error != NO_ERROR || sht3x_data->temp_sht-gxht30_temp_record>30 || sht3x_data->temp_sht-gxht30_temp_record<-30
			|| sht3x_data->hum_sht<20 || sht3x_data->hum_sht>=100 || sht3x_data->hum_sht-gxht30_hum_record>20 || sht3x_data->hum_sht-gxht30_hum_record<-20)
	{
		error = SHT3X_SoftReset();
		times = 0;
		HAL_Delay(50);
		do
		{
			times++;
			error = SHT3X_GetTempAndHumiClkStretch(&sht3x_data->temp_sht, &sht3x_data->hum_sht, REPEATAB_HIGH, 50);
		}while(times < 4 && error != NO_ERROR);
	}
	
  if(error == NO_ERROR)
	{
		if(sht3x_data->temp_sht>125)
		{
			sht3x_data->temp_sht=125;
			sht3x_data->temp_sht=gxht30_temp_record;
		}
		else if(sht3x_data->temp_sht<-40)
		{
			sht3x_data->temp_sht=-40;
			sht3x_data->temp_sht=gxht30_temp_record;
		}
		
		if(sht3x_data->hum_sht>100)
		{
			sht3x_data->hum_sht=100;
			sht3x_data->hum_sht=gxht30_hum_record;
		}
		else if(sht3x_data->hum_sht<0)
		{
			sht3x_data->hum_sht=0;
			sht3x_data->hum_sht=gxht30_hum_record;
		}
		else if(sht3x_data->hum_sht==100)
		{
			sht3x_data->hum_sht=gxht30_hum_record;
		}
		
		gxht30_temp_record=sht3x_data->temp_sht;
		gxht30_hum_record=sht3x_data->hum_sht;
	}
	else
	{
		sht3x_data->temp_sht=3276.7;
		sht3x_data->hum_sht=6553.5;
	}
}

void display_sht31_message(void)
{
	HAL_Delay(50);
	float bat_temp=0.0;
  bat_temp=battery_voltage_measurement()/1000.0;
  if(bat_temp>3750)
	{
	  bat_temp=battery_voltage_measurement()/1000.0;
	}			
	MW_LOG(TS_OFF, VLEVEL_M,"Battery: %.3f V\r\n",bat_temp);
  HAL_Delay(50);				
	
	sht3x_data_t get_sht3x_data;
	sht3x_get_data(&get_sht3x_data);
  if((fabs(get_sht3x_data.temp_sht-3276.7)< 0.01f)&&(fabs(get_sht3x_data.hum_sht-6553.5)< 0.01f))	
	{
		MW_LOG(TS_OFF, VLEVEL_M,"Temperature: NULL ;Humidity: NULL\r\n");
	}
	else
	{
		MW_LOG(TS_OFF, VLEVEL_M,"Temperature: %.1f ;Humidity: %.1f\r\n",get_sht3x_data.temp_sht,get_sht3x_data.hum_sht);
	}
	HAL_Delay(50);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
