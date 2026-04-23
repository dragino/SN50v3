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
