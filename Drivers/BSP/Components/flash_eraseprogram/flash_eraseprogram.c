 /******************************************************************************
  * @file    oil_float.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    01-June-2017
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
#include "stdio.h"	
#include "flash_eraseprogram.h"
#include "timer.h"
#include "mw_log_conf.h"
#include "flash_if.h"
#include "stm32_systime.h"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t PAGEError = 0;

__IO float data32_comp_year_mon_day_hour_min = 0;

__IO uint32_t data_on_add10 = 0,data_on_add = 0, data_on_add04=0,data_on_add12=0,data_on_add16=0;

char printf_buff[100]={"\0"};	

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

void  FLASH_erase_all_sensor_data_storage(uint32_t page_address)
{
  for(uint32_t addr=FLASH_SENSOR_DATA_START_ADDR;addr<FLASH_SENSOR_DATA_END_ADDR;)
	{
		__disable_irq();
		if(FLASH_IF_Erase((void *)addr, FLASH_PAGE_SIZE) == FLASH_IF_ERROR)
		{
			MW_LOG(TS_OFF, VLEVEL_M, "erase all sensor data storage error\r\n");
		}
		__enable_irq();
		HAL_Delay(100);
		addr=addr+0x1000;
	}
}

void  FLASH_program(uint32_t add, uint8_t *data, uint8_t count)
{
	__disable_irq();

	//pDestination pointer of flash address to write. It has to be 8 bytes aligned
	if(FLASH_IF_Write((void *)add, data, count) ==FLASH_IF_OK)
	{
		MW_LOG(TS_OFF, VLEVEL_M, "write config error\r\n");
	}
	__enable_irq();
}

uint32_t find_addr_first_after_systemreset(uint32_t addr)
{
	uint32_t current_address,address_record=0;
	uint32_t data_comp;
	current_address=addr;
	
	if(current_address<FLASH_SENSOR_DATA_END_ADDR)
	{	
		data_on_add=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
		
		while(data_on_add!=0xFFFFFFFF)
		{
			current_address = current_address+16;
			if(current_address>=FLASH_SENSOR_DATA_END_ADDR)
			{
				current_address=FLASH_SENSOR_DATA_START_ADDR;
				data_comp=data_on_add=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));

				while(current_address<FLASH_SENSOR_DATA_END_ADDR)
				{
					data_on_add=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
					if(data_on_add>=data_comp)
					{
							address_record=current_address;
							data_comp=data_on_add;
					}
					current_address = current_address+16;	
				}
				current_address=((address_record-FLASH_BASE)/4096);
				current_address=current_address*4096+FLASH_BASE;//将地址归为当前page的首地址
				current_address+=4096;//next page
				break;
			}
			data_on_add=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
		}
  }
	
	if(current_address>=FLASH_SENSOR_DATA_END_ADDR)
	{
		__disable_irq();
		FLASH_IF_Erase((void *)FLASH_SENSOR_DATA_START_ADDR, FLASH_PAGE_SIZE);
		__enable_irq();
		HAL_Delay(5);
		current_address=FLASH_SENSOR_DATA_START_ADDR;
	}
	
//	MW_LOG(TS_OFF, VLEVEL_M, "find address= %0X\r\n",(unsigned int)current_address);
	return current_address;
}

uint32_t find_addr(uint32_t addr)
{
	uint32_t current_address;
	current_address=addr;
	
	data_on_add=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
	
	if(current_address<FLASH_SENSOR_DATA_END_ADDR)
	{
		if(data_on_add!=0xFFFFFFFF)
		{				
			current_address=((current_address-FLASH_BASE)/4096);
			current_address=current_address*4096+FLASH_BASE;//将地址归为当前page的首地址
      __disable_irq();
//			flash_erase_page(current_address);//擦除当前地址所在的整个page
			FLASH_IF_Erase((void *)current_address, FLASH_PAGE_SIZE);
      __enable_irq();
			HAL_Delay(5);
		}
  }
	
	if(current_address>=FLASH_SENSOR_DATA_END_ADDR)
	{
		__disable_irq();
//		flash_erase_page(FLASH_SENSOR_DATA_START_ADDR);
		FLASH_IF_Erase((void *)FLASH_SENSOR_DATA_START_ADDR, FLASH_PAGE_SIZE);
		__enable_irq();
		HAL_Delay(5);
		current_address=FLASH_SENSOR_DATA_START_ADDR;
	}
//	MW_LOG(TS_OFF, VLEVEL_M, "find address= %0X\r\n",(unsigned int)current_address);
	return current_address;
}

uint32_t store_sensor_data(uint32_t add,uint8_t *sensor_data)
{
	__disable_irq();
//	flash_program_bytes(add,sensor_data,16);
	//pDestination pointer of flash address to write. It has to be 8 bytes aligned
	if(FLASH_IF_Write((void *)add, sensor_data, 16) == FLASH_IF_OK)	
	__enable_irq();
	add=add+16;
	
	return add;
}

uint8_t read_data_on_flash_buff(uint16_t address_start,uint16_t address_end)
{
	uint8_t buff[16];
  uint32_t timestamp;
	uint32_t cnt;
	uint8_t de,le,ex;
	
	uint16_t ba,illu;

	float T_sht20,H_sht20,T_ds18b20,H_sht31;
	float adc;
	
	short t1,h1,t2,h2;
	
  uint16_t startpage=address_start;
  uint16_t endpage=address_end;
	
	__IO uint32_t add1=FLASH_SENSOR_DATA_START_ADDR+(128*(startpage-1));
	__IO uint32_t end_address=FLASH_SENSOR_DATA_START_ADDR+(128*(endpage));
	
	for (;add1 < end_address;)
  {
		MW_LOG(TS_OFF, VLEVEL_M, "%0X ",(unsigned int)add1);
    
		for(int i=0;i<16;i++)
		{
      buff[i] = *(uint8_t *)add1;
      add1 = add1 + 1;
		}	

		timestamp=buff[0]<<24|buff[1]<<16|buff[2]<<8|buff[3];

		struct tm localtime;
		SysTimeLocalTime( timestamp, &localtime );
		localtime.tm_mon = localtime.tm_mon + 1;
		
		de=buff[5] & 0x0f;
		
		ba=buff[6]<<8 | buff[7];
		
		t1=(buff[8]<<8) | buff[9];
		if(t1<0)
		{
			T_sht20=(~t1+1)/-100.0;
		}
		else
		  T_sht20=t1/100.0;
		
		h1=buff[10]<<8 | buff[11];
		if(h1<0)
		{
			H_sht20=(~h1+1)/-10.0;
		}
		else
			H_sht20=h1/10.0;
		
		/*
		 dev=0x01,ds18b20
		 */
		if(de==0x00)
		{
			if(ba!=0)
			{
			  snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			
			  MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
			}
		}
		else if(de==0x01 || de==0x09)
		{
			t2=buff[12]<<8 | buff[13];
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f ds_temp=%0.2f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
	  }
		else if(de==0x02 || de==0x0A)
		{
			t2=buff[12]<<8 | buff[13];
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f tmp_temp=%0.2f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
	  }			
		else if(de==0x04)
		{
			le=buff[12];
			ex=buff[13];
			
			if(le==0x00)
			{
				if(ex==0x00)
				{
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:low status:false\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
				}
				else
				{
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:low status:true\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);					
				}
			}	
      else if(le==0x01)
			{
				if(ex==0x00)
				{				
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:high status:false\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
				}
				else
				{
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:high status:true\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);					
				}
			}		
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);			
		}	
		else if(de==0x05)
		{
			illu=(buff[12]<<8) | buff[13];
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f illu_lux=%d\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,illu);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);		
		}					
		else if(de==0x06)
		{
			adc=(buff[12]<<8 | buff[13])/1000.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f adc_v=%0.3f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,adc);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);	
		}	
		else if((de==0x08)||(de==0x0e))
		{
			cnt= buff[12]<<24 | buff[13]<<16 | buff[14]<<8 | buff[15];
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f count_times=%u\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,cnt);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);					
		}			
		else if(de==0x0B)
		{
			t2=buff[12]<<8 | buff[13];
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			h2=buff[14]<<8 | buff[15];
			if(h2<0)
			{
				H_sht31=(~h2+1)/-10.0;
			}
			else
				H_sht31=h2/10.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f s31_temp=%0.2f s31_hum=%0.1f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20,H_sht31);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);				
		}		
		else
		{
			if(ba!=0xFFFF)
			{
				snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
				
				MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
			}
		}	
		
		while (1 != UTIL_ADV_TRACE_IsBufferEmpty())
		{
			/* Wait that all printfs are completed*/
		}
		
		MW_LOG(TS_OFF, VLEVEL_M, "\n\r");
		
		for(uint8_t k=0;k<16;k++)
		{
		  buff[k]=0;
		}
  }
	
	return 1;
}

uint8_t read_data_on_flash_buff_last_sets_data(uint32_t address_start,uint16_t last_sets_data)
{
	uint8_t buff[16];
  uint32_t timestamp;
	uint32_t cnt;
	uint8_t de,le,ex;
	
	uint16_t ba,illu;

	float T_sht20,H_sht20,T_ds18b20,H_sht31;
	float adc;
	
	short t1,h1,t2,h2;
	
	__IO uint32_t add1=address_start;

	for (uint16_t j=1;j <= last_sets_data;j++)
  {
		MW_LOG(TS_OFF, VLEVEL_M, "%04d ",j);

		for(int i=0;i<16;i++)
		{
      buff[i] = *(uint8_t *)add1;
      add1 = add1 + 1;
			if(add1>=FLASH_SENSOR_DATA_END_ADDR)
			{
				add1=FLASH_SENSOR_DATA_START_ADDR;
			}
		}	
		
		timestamp=buff[0]<<24|buff[1]<<16|buff[2]<<8|buff[3];

		struct tm localtime;
		SysTimeLocalTime( timestamp, &localtime );
		localtime.tm_mon = localtime.tm_mon + 1;
		
		de=buff[5] & 0x0f;
		
		ba=buff[6]<<8 | buff[7];
		
		t1=(buff[8]<<8) | buff[9];
		if(t1<0)
		{
			T_sht20=(~t1+1)/-100.0;
		}
		else
		  T_sht20=t1/100.0;
		
		h1=buff[10]<<8 | buff[11];
		if(h1<0)
		{
			H_sht20=(~h1+1)/-10.0;
		}
		else
			H_sht20=h1/10.0;
		
		/*
		 dev=0x01,ds18b20
		 */
		if(de==0x00)
		{
			if(ba!=0)
			{
			  snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			
			  MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
			}
		}
		else if(de==0x01 || de==0x09)
		{
			t2=buff[12]<<8 | buff[13];
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f ds_temp=%0.2f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
	  }
		else if(de==0x02 || de==0x0A)
		{
			t2=buff[12]<<8 | buff[13];
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f tmp_temp=%0.2f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
	  }		
		else if(de==0x04)
		{
			le=buff[12];
			ex=buff[13];
			
			if(le==0x00)
			{
				if(ex==0x00)
				{
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:low status:false\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
				}
				else
				{
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:low status:true\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);					
				}
			}	
      else if(le==0x01)
			{
				if(ex==0x00)
				{				
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:high status:false\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
				}
				else
				{
					sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:high status:true\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);					
				}
			}		
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);			
		}		
		else if(de==0x05)
		{
			illu=(buff[12]<<8) | buff[13];
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f illu_lux=%d\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,illu);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);		
		}				
		else if(de==0x06)
		{
			adc=(buff[12]<<8 | buff[13])/1000.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f adc_v=%0.3f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,adc);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);	
		}
		else if((de==0x08)||(de==0x0e))
		{
			cnt= buff[12]<<24 | buff[13]<<16 | buff[14]<<8 | buff[15];
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f count_times=%u\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,cnt);
			
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);					
		}				
		else if(de==0x0B)
		{
			t2=buff[12]<<8 | buff[13];
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			h2=buff[14]<<8 | buff[15];
			if(h2<0)
			{
				H_sht31=(~h2+1)/-10.0;
			}
			else
				H_sht31=h2/10.0;
			
			snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f s31_temp=%0.2f s31_hum=%0.1f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20,H_sht31);
				
			MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);				
		}		
		else
		{
			if(ba!=0xFFFF)
			{
				snprintf(printf_buff,100,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
				
				MW_LOG(TS_OFF, VLEVEL_M, "%s",printf_buff);
			}
		}	
		
		while (1 != UTIL_ADV_TRACE_IsBufferEmpty())
		{
			/* Wait that all printfs are completed*/
		}
		
		MW_LOG(TS_OFF, VLEVEL_M, "\n\r");
		
		for(uint8_t k=0;k<16;k++)
		{
		  buff[k]=0;
		}
  }
	
	return 1;
}

uint32_t find_LoRaMacDevNonce_addr_first_after_systemreset(uint32_t addr)
{
	uint32_t current_address,address_record=0;
	uint32_t data_comp;
	current_address=addr;
	
	if(current_address<FLASH_LoRaMacDevNonce_END_ADDR)
	{	
		data_on_add10=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
		
		while(data_on_add10!=0xFFFFFFFF)
		{
			current_address = current_address+16;
			if(current_address>=FLASH_LoRaMacDevNonce_END_ADDR)
			{
				current_address=FLASH_LoRaMacDevNonce_START_ADDR;
				data_comp=data_on_add10=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));

				while(current_address<FLASH_LoRaMacDevNonce_END_ADDR)
				{
					data_on_add10=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
					if(data_on_add10>=data_comp)
					{
							address_record=current_address;
							data_comp=data_on_add10;
					}
					current_address = current_address+16;	
				}
				current_address=((address_record-FLASH_BASE)/4096);
				current_address=current_address*4096+FLASH_BASE;//将地址归为当前page的首地址
				current_address+=4096;//next page
				break;
			}
			data_on_add10=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
		}
  }
	
	if(current_address>=FLASH_LoRaMacDevNonce_END_ADDR)
	{
		__disable_irq();
		FLASH_IF_Erase((void *)FLASH_LoRaMacDevNonce_START_ADDR, FLASH_PAGE_SIZE);
		__enable_irq();
		HAL_Delay(5);		
		current_address=FLASH_LoRaMacDevNonce_START_ADDR;
	}
	
//	LOG_PRINTF(LL_DEBUG,"find address= %0X\r\n",(unsigned int)current_address);
	return current_address;
}

uint32_t find_LoRaMacDevNonce_addr(uint32_t addr)
{
	uint32_t current_address;
	current_address=addr;
	
	data_on_add10=(*((uint8_t *) (current_address)))<<24|(*((uint8_t *) (current_address+1)))<<16|(*((uint8_t *) (current_address+2)))<<8|(*((uint8_t *) (current_address+3)));
	
	if(current_address<FLASH_LoRaMacDevNonce_END_ADDR)
	{
		if(data_on_add10!=0xFFFFFFFF)
		{				
			current_address=((current_address-FLASH_BASE)/4096);
			current_address=current_address*4096+FLASH_BASE;//将地址归为当前page的首地址
      __disable_irq();			
			FLASH_IF_Erase((void *)current_address, FLASH_PAGE_SIZE);//擦除当前地址所在的整个page
			HAL_Delay(5);
      __enable_irq();			
		}
  }
	
	if(current_address>=FLASH_LoRaMacDevNonce_END_ADDR)
	{
		__disable_irq();
		FLASH_IF_Erase((void *)FLASH_LoRaMacDevNonce_START_ADDR, FLASH_PAGE_SIZE);
		__enable_irq();
		HAL_Delay(5);
		current_address=FLASH_LoRaMacDevNonce_START_ADDR;
	}
//	LOG_PRINTF(LL_DEBUG,"find address= %0X\r\n",(unsigned int)current_address);
	return current_address;
}

void read_lastet_count(uint32_t address,uint16_t *cnt1)
{
	uint8_t buff[16];
	
	if(address<=FLASH_LoRaMacDevNonce_START_ADDR)
	{
		address=FLASH_LoRaMacDevNonce_END_ADDR;
	}
	address=address-16;
	
	for(int i=0;i<16;i++)
	{
     buff[i] = *(uint8_t *)address;
		 address = address + 1;
	}

	if((buff[0]==0xFF) && (buff[1]==0xFF) & (buff[2]==0xFF) && (buff[3]==0xFF))
	{
		*cnt1=0;
	}
	else
	{
		*cnt1= buff[4]<<8 | buff[5];
	}
}

void read_lastet_fcnt(uint32_t address,uint32_t *cnt1)
{
	uint8_t buff[16];
	
	if(address<=FLASH_LoRaMacDevNonce_START_ADDR)
	{
		address=FLASH_LoRaMacDevNonce_END_ADDR;
	}
	address=address-16;
	
	for(int i=0;i<16;i++)
	{
     buff[i] = *(uint8_t *)address;
		 address = address + 1;
	}

		if((buff[0]==0xFF) && (buff[1]==0xFF) & (buff[2]==0xFF) && (buff[3]==0xFF))
	{
		*cnt1=0;
	}
	else
	{
		*cnt1= buff[4]<<24 | buff[5]<<16 | buff[6]<<8 | buff[7];
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
