#ifndef __PWM_H
#define __PWM_H

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

void gptimer_pwm_output(uint16_t time,uint32_t freq_hz,uint8_t duty);
void gptimer_pwm_input_capture(uint8_t pwmmd);
void gptimer_pwm_Iodeinit(void);
	 
#ifdef __cplusplus
}
#endif

#endif

