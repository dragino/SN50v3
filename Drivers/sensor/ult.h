#ifndef __ULT_H__
#define __ULT_H__

void GPIO_ULT_INPUT_Init(void);
void GPIO_ULT_OUTPUT_Init(void);
void GPIO_ULT_INPUT_DeInit(void);
void GPIO_ULT_OUTPUT_DeInit(void);
void bstimer_onepulse(void);
void btim0_IRQHandler(void);
uint16_t ULT_test(void);
uint16_t ult_test_temp();

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

#ifdef __cplusplus
}
#endif

#endif
