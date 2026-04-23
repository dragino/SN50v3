#ifndef __I2C_SENSOR_H__
#define __I2C_SENSOR_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h> 
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  
 *
 * @note
 * @retval None
 */
	 
#include "bsp.h"

typedef struct{
	
	float temp_sht;
	
	float hum_sht;
  /**more may be added*/
} sht20_t;

uint8_t SHT31_CheckSum_CRC8(uint8_t* Result,uint8_t num);
void SHT31_Read(sht3x_data_t *sht3x_data);
uint8_t check_sht31_connect(void);
uint8_t SHT20_CheckSum_CRC8(uint8_t* Result);
float SHT20_RH(void);
float SHT20_RT(void);
uint8_t check_sht20_connect(void);
uint16_t bh1750_read(void);
void I2C_read_data(sensor_t *sensor_data,uint8_t flag_temp, uint8_t message);
void LidarLite_init(void);
uint16_t LidarLite(void);
uint16_t waitbusy(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_A_H__ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
