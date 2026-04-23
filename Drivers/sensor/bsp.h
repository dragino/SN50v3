#ifndef __BSP_H__
#define __BSP_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
	 
typedef struct{
  bool in1;
	
	bool exit_pa4;
	
	bool exit_pa8;
	
	bool exit_pb15;
	
	float temp1;

	float temp2;

	float temp3;
	
	float ADC_4; 

	float ADC_5; 
	
	float ADC_8; 

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
void BLE_power_Init(void);
void  BSP_sensor_Init( void  );
void BSP_sensor_Read( sensor_t *sensor_data , uint8_t message ,uint8_t mod_temp);
float DS18B20_Read(uint8_t temp,uint8_t message);
uint16_t ADC_Read(uint8_t temp,uint8_t message);
bool Digital_input_Read(uint8_t temp,uint8_t message);
uint16_t battery_voltage_measurement(void);
uint16_t middle_value(uint16_t value[]);
void display_message(void);
/**
 * @brief  sensor  read. 
 *
 * @note none
 * @retval sensor_data
 */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
