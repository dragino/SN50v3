#include "stdio.h"	
#include "gpio_exti.h"
#include "lora_config.h"
#include "tremo_delay.h"
#include "tremo_iwdg.h"
#include "tremo_adc.h"

extern __IO bool wakeup_pa8_flag,wakeup_pa4_flag,wakeup_pb15_flag;

void POWER_open_time(uint16_t times)
{
	if(times!=0)
	{
	  POWER_IoInit();//Enable 5v power supply
		for(uint16_t i=0;i<=(uint16_t)(times/100);i++)
		{
			 delay_ms(100);
       if((i%99==0)&&(i!=0))
			 {
					iwdg_reload();		 
			 }				 
		}
	}	
}

void POWER_IoInit(void)
{
	POWER_CLK_ENABLE();
	gpio_set_iomux(POWER_PORT, POWER_5V_PIN, 0);
	gpio_init(POWER_PORT, POWER_5V_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	delay_ms(20);  
}

void POWER_IoDeInit(void)
{
	POWER_CLK_ENABLE();
	gpio_set_iomux(POWER_PORT, POWER_5V_PIN, 0);
	gpio_init(POWER_PORT, POWER_5V_PIN,GPIO_MODE_OUTPUT_PP_HIGH);
}

void GPIO_EXTI4_IoInit(uint8_t state)
{
	GPIO_EXTI4_CLK_ENABLE();	
	gpio_set_iomux(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN, 0);	
	gpio_init(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN, GPIO_MODE_INPUT_FLOATING);			
	
  if((state == 1)||(state == 4))
	{
		gpio_config_stop3_wakeup(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN,true,GPIO_LEVEL_HIGH);		
		gpio_config_interrupt(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN, GPIO_INTR_RISING_FALLING_EDGE);
	}
	else if(state == 2)
	{
		gpio_config_stop3_wakeup(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN,true,GPIO_LEVEL_HIGH);			
		gpio_config_interrupt(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN, GPIO_INTR_FALLING_EDGE);
	}
	else if(state == 3)
	{
		gpio_config_stop3_wakeup(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN,true,GPIO_LEVEL_HIGH);			
		gpio_config_interrupt(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN, GPIO_INTR_RISING_EDGE);		
	}	
	else 
	{
		gpio_config_stop3_wakeup(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN,false,GPIO_LEVEL_HIGH);		
	}

	wakeup_pa4_flag=gpio_read(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN);
	
	if(state != 0)
	{
		NVIC_SetPriority(GPIO_IRQn, 3);
		NVIC_EnableIRQ(GPIO_IRQn);
	}		
}

void GPIO_EXTI8_IoInit(uint8_t state)
{
	GPIO_EXTI8_CLK_ENABLE();	
	gpio_set_iomux(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN, 0);	
	gpio_init(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN, GPIO_MODE_INPUT_FLOATING);			
	
  if((state == 1)||(state == 4))
	{
		gpio_config_stop3_wakeup(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN,true,GPIO_LEVEL_HIGH);		
		gpio_config_interrupt(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN, GPIO_INTR_RISING_FALLING_EDGE);
	}
	else if(state == 2)
	{
		gpio_config_stop3_wakeup(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN,true,GPIO_LEVEL_HIGH);			
		gpio_config_interrupt(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN, GPIO_INTR_FALLING_EDGE);
	}
	else if(state == 3)
	{
		gpio_config_stop3_wakeup(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN,true,GPIO_LEVEL_HIGH);			
		gpio_config_interrupt(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN, GPIO_INTR_RISING_EDGE);		
	}	
	else 
	{
		gpio_config_stop3_wakeup(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN,false,GPIO_LEVEL_HIGH);		
	}

	wakeup_pa8_flag=gpio_read(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN);
	
	if(state != 0)
	{
		NVIC_SetPriority(GPIO_IRQn, 3);
		NVIC_EnableIRQ(GPIO_IRQn);
	}	
}

void GPIO_EXTI15_IoInit(uint8_t state)
{
	GPIO_EXTI15_CLK_ENABLE();	
	gpio_set_iomux(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN, 0);	
	gpio_init(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN, GPIO_MODE_INPUT_FLOATING);			
	
  if((state == 1)||(state == 4))
	{
		gpio_config_stop3_wakeup(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN,true,GPIO_LEVEL_HIGH);		
		gpio_config_interrupt(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN, GPIO_INTR_RISING_FALLING_EDGE);
	}
	else if(state == 2)
	{
		gpio_config_stop3_wakeup(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN,true,GPIO_LEVEL_HIGH);			
		gpio_config_interrupt(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN, GPIO_INTR_FALLING_EDGE);
	}
	else if(state == 3)
	{
		gpio_config_stop3_wakeup(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN,true,GPIO_LEVEL_HIGH);			
		gpio_config_interrupt(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN, GPIO_INTR_RISING_EDGE);		
	}	
	else 
	{
		gpio_config_stop3_wakeup(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN,false,GPIO_LEVEL_HIGH);		
	}

	wakeup_pb15_flag=gpio_read(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN);
	
	if(state != 0)
	{
		NVIC_SetPriority(GPIO_IRQn, 3);
		NVIC_EnableIRQ(GPIO_IRQn);
	}			
}	

void GPIO_BLE_STATUS_Ioinit(void)
{
	DX_BT24_STATUS_ENABLE();
	gpio_set_iomux(DX_BT24_STATUS_PORT,DX_BT24_LINK_PIN, 0);
	gpio_init(DX_BT24_STATUS_PORT,DX_BT24_LINK_PIN, GPIO_MODE_INPUT_FLOATING);

	gpio_set_iomux(DX_BT24_STATUS_PORT,DX_BT24_WORK_PIN, 0);
	gpio_init(DX_BT24_STATUS_PORT,DX_BT24_WORK_PIN, GPIO_MODE_INPUT_FLOATING);  	
}

uint16_t adc_in1_measurement(gpio_t* gpiox, uint8_t gpio_pin,uint8_t adc_channel)
{
  uint16_t adc_in1_temp=0;
  uint8_t i;
  float gain_value;
  float dco_value;
	float calibrated_sample_1[25] = {0.0};
  uint16_t adc_data_1[25] = {0};
	float sum_cab = 0.0f;
	uint32_t adc_data_sum = 0;
	
  rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_PCLK1);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);	
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);	
	
  adc_get_calibration_value(false, &gain_value, &dco_value);
  gpio_init(gpiox, gpio_pin, GPIO_MODE_ANALOG);
	
  adc_init();
  delay_ms(10);
  adc_config_clock_division(20); //sample frequence 150K
  adc_config_sample_sequence(0, adc_channel);
  adc_config_conv_mode(ADC_CONV_MODE_CONTINUE);
	
  adc_enable(true);		
	adc_start(true);
	
  for (i = 0; i < 25; i++)
  {
    while(!adc_get_interrupt_status(ADC_ISR_EOC));
    adc_data_1[i] = adc_get_data();	
		adc_data_sum+=adc_data_1[i];
  }

  adc_start(false);
  adc_enable(false);
	
  for (i = 0; i < 25; i++)
  {//calibration sample value
     calibrated_sample_1[i] = ((1.2/4096) * adc_data_1[i] - dco_value) / gain_value;
		 sum_cab+=calibrated_sample_1[i];
  }

	if(adc_data_sum==0)
	{
		adc_in1_temp = 0;
	}	
	else
	{
		sum_cab = sum_cab/25;
		adc_in1_temp = sum_cab*1000;
	}
	
	if(adc_in1_temp>=1200)
	{
		adc_in1_temp=1200;
	}
//	LOG_PRINTF(LL_DEBUG,"adc=%d mV\r\n",adc_in1_temp>=1200);	
	
	return adc_in1_temp;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
