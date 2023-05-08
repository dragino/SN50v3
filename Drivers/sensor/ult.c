#include "stdio.h"	
#include "tremo_bstimer.h"
#include "lora_config.h"
#include "ult.h"
#include "timer.h"
#include "tremo_delay.h"
#include "log.h"

void GPIO_ULT_INPUT_Init(void)
{
	ULT_Echo_CLK_ENABLE();
	gpio_set_iomux(ULT_Echo_PORT, ULT_Echo_PIN, 0);	
	gpio_init(ULT_Echo_PORT, ULT_Echo_PIN, GPIO_MODE_INPUT_PULL_UP);		
}

void GPIO_ULT_OUTPUT_Init(void)
{
	ULT_TRIG_CLK_ENABLE();
	gpio_set_iomux(ULT_TRIG_PORT, ULT_TRIG_PIN, 0);				
  gpio_init(ULT_TRIG_PORT, ULT_TRIG_PIN,GPIO_MODE_OUTPUT_PP_LOW);
}

void GPIO_ULT_INPUT_DeInit(void)
{
	ULT_Echo_CLK_ENABLE();
	gpio_set_iomux(ULT_Echo_PORT, ULT_Echo_PIN, 0);	
	gpio_init(ULT_Echo_PORT, ULT_Echo_PIN, GPIO_MODE_ANALOG);	
}

void GPIO_ULT_OUTPUT_DeInit(void)
{
	ULT_TRIG_CLK_ENABLE();
	gpio_set_iomux(ULT_TRIG_PORT, ULT_TRIG_PIN, 0);				
  gpio_init(ULT_TRIG_PORT, ULT_TRIG_PIN,GPIO_MODE_ANALOG);	
}

void bstimer_onepulse(void)
{
  bstimer_init_t bstimer_init_config;

  bstimer_init_config.bstimer_mms        = BSTIMER_MMS_ENABLE;
  bstimer_init_config.period             = 10000-1;  //time period is ((1 / 2.4k) * (2399 + 1)) ; 
  bstimer_init_config.prescaler          = 239;  //sysclock defaults to 24M, is divided by (prescaler + 1) to 2.4k ; 
  bstimer_init_config.autoreload_preload = false;
  bstimer_init(BSTIMER0, &bstimer_init_config);

	bstimer_config_one_pulse(BSTIMER0, ENABLE);
}

uint16_t ULT_test(void)
{
	uint16_t distance;
	uint8_t  ult_flags=0;
	
	if(gpio_read(ULT_Echo_PORT, ULT_Echo_PIN)==0)
	{
		 ult_flags=0;
	}
	else
	{
		 ult_flags=1;
	}
	
	if(ult_flags==0)
	{	
		distance=ult_test_temp();
	
		if((distance<240)||(distance>6000))
		{
				distance=0;
				return distance;
		}
		else
		{
				return distance;
		}
	}	
	else
	{
		distance=65535;
		return distance;
	}
}

uint16_t ult_test_temp(void)
{	
	uint16_t temp_1;
	uint16_t ultdata=0;
  TimerTime_t currentTime;
	
	gpio_init(ULT_TRIG_PORT, ULT_TRIG_PIN,GPIO_MODE_OUTPUT_PP_HIGH); //give a signal
	delay_us(20);	
	gpio_init(ULT_TRIG_PORT, ULT_TRIG_PIN,GPIO_MODE_OUTPUT_PP_LOW); 
		
	currentTime = TimerGetCurrentTime();			
	while(!gpio_read(ULT_Echo_PORT, ULT_Echo_PIN)) //wait the response signal  
	{
		if(TimerGetElapsedTime(currentTime) >= 1000)
		{
			break;
		}			
	}      
	
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_BSTIMER0, true);					
	bstimer_onepulse();
	bstimer_config_overflow_update(BSTIMER0, ENABLE);
  bstimer_generate_event(BSTIMER0, BSTIMER_EGR_UG, ENABLE); //in order to make prescaler work in the first time period
	bstimer_cmd(BSTIMER0, true);
	
	currentTime = TimerGetCurrentTime();
	while(gpio_read(ULT_Echo_PORT, ULT_Echo_PIN))  //wait the response signal
	{
		if(TimerGetElapsedTime(currentTime) >= 1000)
		{
			break;
		}					
	}		
	
	temp_1=BSTIMER0->CNT;
	
//	LOG_PRINTF(LL_DEBUG,"time:%d\r\n",temp_1);
	
	bstimer_deinit(BSTIMER0);
	bstimer_cmd(BSTIMER0, false);
	
	ultdata=((temp_1*0.343)/2.0)*10.0; //(time(10us)*0.343/us)/2	
	
	return ultdata;
}
