#ifndef __WEIGHT_H
#define __WEIGHT_H

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

void WEIGHT_SCK_Init(void);
void WEIGHT_DOUT_Init(void);
void WEIGHT_SCK_DeInit(void);
void WEIGHT_DOUT_DeInit(void);
uint32_t HX711_Read(void);
void Get_Maopi(void);
int32_t Get_Weight(void);

#ifdef __cplusplus
}
#endif

#endif

