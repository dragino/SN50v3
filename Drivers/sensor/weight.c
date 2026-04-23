#include <stdio.h>
#include "lora_config.h"
#include "weight.h"
#include "tremo_gpio.h"
#include "timer.h"

uint32_t HX711_Buffer=0;
uint32_t Weight_Maopi=0;
float GapValue=400.0;

void WEIGHT_SCK_Init(void)
{
	WEIGHT_SCK_CLK_ENABLE();
	gpio_set_iomux(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN, 0);				
  gpio_init(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN,GPIO_MODE_OUTPUT_PP_LOW);
	HX711_SCK_0; 
}

void WEIGHT_DOUT_Init(void)
{
	WEIGHT_DOUT_CLK_ENABLE();
	gpio_set_iomux(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, 0);	
	gpio_init(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, GPIO_MODE_INPUT_PULL_DOWN);			
}

void WEIGHT_SCK_DeInit(void)
{
	WEIGHT_SCK_CLK_ENABLE();
	gpio_set_iomux(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN, 0);				
  gpio_init(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN,GPIO_MODE_ANALOG);
}

void WEIGHT_DOUT_DeInit(void)
{
	WEIGHT_DOUT_CLK_ENABLE();
	gpio_set_iomux(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, 0);	
	gpio_init(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, GPIO_MODE_ANALOG);			
}

uint32_t HX711_Read(void)	
{
	uint32_t count; 
	uint8_t i;   
	HX711_SCK_0;  
  count=0; 
	
	TimerTime_t currentTime = TimerGetCurrentTime();	
  while(gpio_read(WEIGHT_DOUT_PORT,WEIGHT_DOUT_PIN)!=0)
	{
			if(TimerGetElapsedTime(currentTime) >= 2000)
			{
				break;
			}				
	}
  for(i=0;i<24;i++)
	{ 
	  	HX711_SCK_1; 	
	  	count=count<<1; 
			HX711_SCK_0; 
	  	if(gpio_read(WEIGHT_DOUT_PORT,WEIGHT_DOUT_PIN)==1)
			count++;	
	} 
 	HX711_SCK_1; 
  count=count^0x800000;
	HX711_SCK_0; 
	return(count);
}

void Get_Maopi(void)
{
	Weight_Maopi = HX711_Read();	
} 

int32_t Get_Weight(void)
{
	int32_t Weight_Shiwu=0;
	
	HX711_Buffer = HX711_Read();
	if(HX711_Buffer != Weight_Maopi)			
	{
		Weight_Shiwu = HX711_Buffer;
		Weight_Shiwu = Weight_Shiwu - Weight_Maopi;				
	
		Weight_Shiwu = (int32_t)((float)Weight_Shiwu/GapValue); 	
	}
	else
	{
		Weight_Shiwu =0;
	}

	return Weight_Shiwu;
}
