 /******************************************************************************
  * @file    pwm.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-April-2019
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
#include "string.h"		
#include "lora_config.h"
#include "pwm.h"
#include "tremo_timer.h"
#include "tremo_iwdg.h"
#include "tremo_delay.h"

extern uint16_t IC1Value,IC2Value;

void gptimer_pwm_output(uint16_t time,uint32_t freq_hz,uint8_t duty)
{
	uint16_t freq_temp=0;
  uint16_t prese_temp=0;
	uint32_t pulse_temp=0;
	
	if(((1000000/freq_hz)-1)<=65000)
	{
		prese_temp=23; //1us
		freq_temp=(1000000/freq_hz)-1;
		pulse_temp=(freq_temp+1)*duty/100;
	}
	else
	{
		prese_temp=23999; //1ms
		freq_temp=(1000/freq_hz)-1;
		pulse_temp=(freq_temp+1)*duty/100;
	}
	
  PWM_RCC_ENABLE();
	PWM_CLK_ENABLE();
  gpio_set_iomux(PWM_PORT, PWM_PIN, 6);  // TIMER1 CH0			
	
  timer_init_t timerx_init;
  timer_oc_init_t oc_init;

  memset(&oc_init, 0, sizeof(oc_init));
  oc_init.oc_mode.oc0m_mode = TIMER_OC0M_PWM1;
  oc_init.pulse             = pulse_temp;
	oc_init.high_level        = true;
  oc_init.oc_fast           = false;

  timerx_init.prescaler          = prese_temp;   
  timerx_init.counter_mode       = TIMER_COUNTERMODE_UP;
  timerx_init.period             = freq_temp;  
  timerx_init.clock_division     = TIMER_CKD_FPCLK_DIV1;
  timerx_init.autoreload_preload = false;
	
  timer_config_pwm(TIMER1, &oc_init, &timerx_init, 0);
	TIMER_ENABLE();

	for(uint16_t i=0;i<=(uint16_t)(time/100);i++)
	{
		delay_ms(100);
    if((i%99==0)&&(i!=0))
	  {
			iwdg_reload();		 
	  }				 
	}
	
	gptimer_pwm_Iodeinit();
}

void gptimer_pwm_input_capture(uint8_t pwmmd)
{
	uint16_t prese_temp=0;
	uint16_t freq_temp=0;
	
	if(pwmmd==0)
	{
		prese_temp = 23;  //1us
		freq_temp  = 50999;  //51ms count reload
	}
	else
	{
		prese_temp = 23999;  //1ms
		freq_temp  = 1099;  //1100ms count reload
	}

	IC1Value=0;
	IC2Value=0;
	PWM_RCC_ENABLE();
	PWM_CLK_ENABLE();
	gpio_set_iomux(PWM_PORT, PWM_PIN, 6);  // TIMER1 CH0		
	
  timer_init_t timerx_init;
  timerx_init.prescaler          = prese_temp;   
  timerx_init.counter_mode       = TIMER_COUNTERMODE_UP;
  timerx_init.period             = freq_temp;  
  timerx_init.clock_division     = TIMER_CKD_FPCLK_DIV1;
  timerx_init.autoreload_preload = false;
  timer_init(TIMER1, &timerx_init);
	
  timer_config_ti(TIMER1, (uint32_t)TIMER_CC0P_RISING, TIMER_CC0S_INPUT_SAME, TIMER_IC0F_0, 0);  
  timer_config_ti(TIMER1, (uint32_t)TIMER_CC1P_FALLING, TIMER_CC1S_INPUT_NEAR, TIMER_IC1F_0, 1);  
	
	timer_config_interrupt(TIMER1, TIMER_DIER_CC0IE, ENABLE);
	timer_clear_status(TIMER1, TIMER_SR_CC0IF);   
	timer_config_ts(TIMER1, TIMER_TS_TI0FP0);   
	
  TIMER1->SMCR &= (uint32_t) ~(uint32_t)0x07;
  TIMER1->SMCR |= (uint32_t)TIMER_SMS_RESET;	  

  TIMER1->SMCR &= (uint32_t) ~(uint32_t)0x80;
  TIMER1->SMCR |= (uint32_t)TIMER_MSM_SYNC;	  
		
  TIMER_ENABLE();
	NVIC_EnableIRQ(TIMER1_IRQn);   
  NVIC_SetPriority(TIMER1_IRQn, 2);
}

void gptimer_pwm_Iodeinit(void)
{
	PWM_RCC_DISABLE();	
  TIMER_DISABLE();
	gpio_set_iomux(PWM_PORT, PWM_PIN, 0); 
  gpio_init(PWM_PORT, PWM_PIN, GPIO_MODE_ANALOG);	
	timer_clear_status(TIMER1, TIMER_SR_CC0IF);   		
  NVIC_DisableIRQ(TIMER1_IRQn);   
}
