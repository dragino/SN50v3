#ifndef __TFSENSOR_H__
#define __TFSENSOR_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
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
typedef struct{
  uint16_t distance_cm;
	uint16_t distance_signal_strengh;
	int temperature;
} tfsensor_reading_t; 

void send_uart_data(uint8_t *txdata,uint8_t txdatalen);
void HAL_UART_RxCallback(uint8_t *rxbuff);
void uart2_IoInit(void);
void uartsersion1_IoInit(uint8_t mode);
void BSP_tfsensor_Init(void);
uint8_t check_deceive(void); 
void tfsensor_read_distance(tfsensor_reading_t *tfsensor_reading);
void read_distance(uint8_t rxdata10[]);
uint8_t getCheckSum(uint8_t *pack, uint8_t pack_len);
void at_tfmini_data_receive(uint8_t rxdatatemp[],uint16_t delayvalue);

#ifdef __cplusplus
}
#endif

#endif 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
