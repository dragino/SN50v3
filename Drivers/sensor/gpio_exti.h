#ifndef __GPIO_EXTI_H__
#define __GPIO_EXTI_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

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
	 
#include "tremo_gpio.h"
	 
void POWER_open_time(uint16_t times);
void POWER_IoInit(void);
void POWER_IoDeInit(void);	 
void GPIO_EXTI4_IoInit(uint8_t state);
void GPIO_EXTI8_IoInit(uint8_t state);
void GPIO_EXTI15_IoInit(uint8_t state);	 
void GPIO_BLE_STATUS_Ioinit(void);
uint16_t adc_in1_measurement(gpio_t* gpiox, uint8_t gpio_pin,uint8_t adc_channel);

#ifdef __cplusplus
}
#endif

#endif 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
