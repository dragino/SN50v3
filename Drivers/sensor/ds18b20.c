#include "tremo_gpio.h"
#include "tremo_delay.h"
#include "lora_config.h"
#include "log.h"
#include "ds18b20.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
void DS18B20_delay(uint16_t time)
{
	delay_us(time);
}

uint8_t DS18B20_Init(uint8_t num)
{ 
	DS18B20_Rst(num);

  return DS18B20_Presence(num);
}

void DS18B20_Mode_IPU(uint8_t num)
{
	if(num==1)
	{
		DOUT1_CLK_ENABLE();
		gpio_init(DOUT1_PORT, DOUT1_PIN, GPIO_MODE_INPUT_PULL_UP);
	}
  else if(num==2)
	{
		DOUT2_CLK_ENABLE();
		gpio_init(DOUT2_PORT, DOUT2_PIN, GPIO_MODE_INPUT_PULL_UP);	
	}
	else if(num==3)
	{
		DOUT3_CLK_ENABLE();
		gpio_init(DOUT3_PORT, DOUT3_PIN, GPIO_MODE_INPUT_PULL_UP);		
	}
}

void DS18B20_Mode_Out_PP(uint8_t num)
{
	if(num==1)
	{	
		DOUT1_CLK_ENABLE();
		gpio_init(DOUT1_PORT, DOUT1_PIN, GPIO_MODE_OUTPUT_PP_LOW);	
  }
	else if(num==2)
	{	
		DOUT2_CLK_ENABLE();
		gpio_init(DOUT2_PORT, DOUT2_PIN, GPIO_MODE_OUTPUT_PP_LOW);	 
  }
	else if(num==3)
	{	
		DOUT3_CLK_ENABLE();
		gpio_init(DOUT3_PORT, DOUT3_PIN, GPIO_MODE_OUTPUT_PP_LOW);	  
  }	
}

void DS18B20_IoDeInit(uint8_t num)
{
	if(num==1)
	{
		DOUT1_CLK_ENABLE();
		gpio_init(DOUT1_PORT, DOUT1_PIN, GPIO_MODE_ANALOG);	
	}
	else if(num==2)
	{
		DOUT2_CLK_ENABLE();
		gpio_init(DOUT2_PORT, DOUT2_PIN, GPIO_MODE_ANALOG);		
	}
	else if(num==3)
	{
		DOUT3_CLK_ENABLE();
		gpio_init(DOUT3_PORT, DOUT3_PIN, GPIO_MODE_ANALOG);		
	}
}

void DS18B20_Rst(uint8_t num)
{
	if(num==1)
	{
    DS18B20_Mode_Out_PP(1);
        
    DOUT1_0;
  
    DS18B20_delay(750);
        
    DOUT1_1;
               
    DS18B20_delay(30);
	}
	else if(num==2)
	{
    DS18B20_Mode_Out_PP(2);
        
    DOUT2_0;
  
    DS18B20_delay(750);
        
    DOUT2_1;
               
    DS18B20_delay(30);
	}
	else if(num==3)
	{
    DS18B20_Mode_Out_PP(3);
        
    DOUT3_0;
  
    DS18B20_delay(750);
        
    DOUT3_1;
               
    DS18B20_delay(30);
	}
}

uint8_t DS18B20_Presence(uint8_t num)
{
  uint8_t pulse_time = 0;
	
  if(num==1)
	{        
    DS18B20_Mode_IPU(1);

    while( DOUT1_READ() && pulse_time<100 )
    {
       pulse_time++;
       DS18B20_delay(1);
    }        

    if( pulse_time >=100 )
		{
       return 1;
		}
    else
       pulse_time = 0;
        
				
    while( !DOUT1_READ() && pulse_time<240 )
    {
       pulse_time++;
       DS18B20_delay(1);
    }        
    if( pulse_time >=240 )
		{
       return 1;
		}
    else			
       return 0;
	}
  else if(num==2)
	{        
    DS18B20_Mode_IPU(2);

    while( DOUT2_READ() && pulse_time<100 )
    {
      pulse_time++;
      DS18B20_delay(1);
    }        

    if( pulse_time >=100 )
		{
       return 1;
		}
    else
       pulse_time = 0;
        
				
    while( !DOUT2_READ() && pulse_time<240 )
    {
      pulse_time++;
      DS18B20_delay(1);
    }        
    if( pulse_time >=240 )
		{
      return 1;
		}
    else			
      return 0;
	 }
   else if(num==3)
	 {        
      DS18B20_Mode_IPU(3);

      while( DOUT3_READ() && pulse_time<100 )
      {
         pulse_time++;
         DS18B20_delay(1);
      }        

      if( pulse_time >=100 )
			{
         return 1;
			}
      else
         pulse_time = 0;
        
				
      while( !DOUT3_READ() && pulse_time<240 )
      {
         pulse_time++;
         DS18B20_delay(1);
      }        
      if( pulse_time >=240 )
		  {
         return 1;
			}
      else			
         return 0;
	 }
				
	 return 0;
}

uint8_t DS18B20_ReadBit(uint8_t num)
{
   uint8_t dat=0;
	
	 if(num==1)
	 {               
     DS18B20_Mode_Out_PP(1);

     DOUT1_0;
     DS18B20_delay(10);
     DS18B20_Mode_IPU(1);
        
     if( DOUT1_READ()==GPIO_LEVEL_HIGH)
        dat = 1;
     else
        dat = 0;
	  }    
	  else if(num==2)
		{               
      DS18B20_Mode_Out_PP(2);

      DOUT2_0;
      DS18B20_delay(10);
      DS18B20_Mode_IPU(2);
        
      if( DOUT2_READ()==GPIO_LEVEL_HIGH)
          dat = 1;
      else
          dat = 0;
		}  
	  else if(num==3)
	  {               
      DS18B20_Mode_Out_PP(3);

      DOUT3_0;
      DS18B20_delay(10);
      DS18B20_Mode_IPU(3);
        
      if( DOUT3_READ()==GPIO_LEVEL_HIGH)
          dat = 1;
      else
          dat = 0;
		}  
			
    DS18B20_delay(45);
        
    return dat;
}

uint8_t DS18B20_ReadByte(uint8_t num)
{
   uint8_t i, j, dat = 0;        
        
   for(i=0; i<8; i++) 
   {
      j = DS18B20_ReadBit(num); 	               								
      dat = (dat) | (j<<i);
   }
        
   return dat;
}

void DS18B20_WriteByte(uint8_t dat,uint8_t num)
{
   uint8_t i, testb;
	 if(num==1)
	 {
     DS18B20_Mode_Out_PP(1);
        
     for( i=0; i<8; i++ )
     {
       testb = dat&0x01;
       dat = dat>>1;                
               
       if (testb)
       {                        
         DOUT1_0;
         /* 1us < ???? < 15us */
         DS18B20_delay(8);
                        
         DOUT1_1;
         DS18B20_delay(58);
       }                
       else
       {                        
         DOUT1_0;
         /* 60us < Tx 0 < 120us */
         DS18B20_delay(90);
                        
         DOUT1_1;                
         /* 1us < Trec(????) < ???*/
         DS18B20_delay(2);
       }
      }
	  }
	  else if(num==2)
		{
      DS18B20_Mode_Out_PP(2);
        
      for( i=0; i<8; i++ )
      {
        testb = dat&0x01;
        dat = dat>>1;                
               
        if (testb)
        {                        
          DOUT2_0;
          /* 1us < ???? < 15us */
          DS18B20_delay(8);
                        
          DOUT2_1;
          DS18B20_delay(58);
        }                
        else
        {                        
          DOUT2_0;
          /* 60us < Tx 0 < 120us */
          DS18B20_delay(90);
                        
          DOUT2_1;                
          /* 1us < Trec(????) < ???*/
          DS18B20_delay(2);
        }
      }
		}
	  else if(num==3)
		{
      DS18B20_Mode_Out_PP(3);
        
      for( i=0; i<8; i++ )
      {
        testb = dat&0x01;
        dat = dat>>1;                
               
        if (testb)
        {                        
          DOUT3_0;
          /* 1us < ???? < 15us */
          DS18B20_delay(8);
                        
          DOUT3_1;
          DS18B20_delay(58);
        }                
        else
        {                        
          DOUT3_0;
          /* 60us < Tx 0 < 120us */
          DS18B20_delay(90);
                        
          DOUT3_1;                
          /* 1us < Trec(????) < ???*/
          DS18B20_delay(2);
        }
      }
	  }
}

void DS18B20_SkipRom(uint8_t num)
{       
	 DS18B20_Rst(num);  	
   DS18B20_Presence(num);                 
   DS18B20_WriteByte(0XCC,num);       
}

float DS18B20_GetTemp_SkipRom (uint8_t num)
{
	 bool read_status=0;
	 uint8_t k=0,j=0,flag_ftem=0;
   uint8_t rxdata[9];
   uint8_t tpmsb, tplsb;
   short s_tem;
   float f_tem;        
        
	 do
	 {
		 read_status=0;
		 if(DS18B20_Init(num)==0)
		 {		 
				DS18B20_SkipRom(num);	
				DS18B20_WriteByte(0X4E,num); 
				DS18B20_WriteByte(0X7F,num); 
				DS18B20_WriteByte(0X00,num); 
				DS18B20_WriteByte(0X7F,num); 
			 			 
			  DS18B20_SkipRom(num);
				DS18B20_WriteByte(0X44,num);		
			  delay_ms(770);
				DS18B20_SkipRom (num);
				DS18B20_WriteByte(0XBE,num);                              
					
			  for(uint8_t i=0;i<9;i++)
			  {
					rxdata[i]=DS18B20_ReadByte(num);
				}
				
				if(crcCalc(rxdata,8)==rxdata[8])
				{
					tplsb = rxdata[0];                 
					tpmsb = rxdata[1];         
					
					s_tem = tpmsb<<8;
					s_tem = s_tem | tplsb;
					
					if( s_tem < 0 )  
					{					
						f_tem = (~s_tem+1) * -0.0625;        
					}	
					else
					{
						f_tem = s_tem * 0.0625;
					}
				
					if(f_tem==85.0)
					{
						flag_ftem++;
						if(flag_ftem==2)
						{
							k=3;
							break;
						}
						read_status=1;
					}
					else
					{
						k=3;
						break;
					}					
				}
				else
				{
					read_status=1;
				}
			}
			else
			{
				k++;
				read_status=1;				
			}	
			j++;	
		}while((j<3)&&(k<3));
		
		DS18B20_IoDeInit(num);
		
		if(read_status==1)
		{
			f_tem=3276.7;		
		}
		else
		{
			if(f_tem<-55 || f_tem>125)
			{
				f_tem=3276.7;
			}		
		}		
	
  return f_tem;         
}

uint8_t crcCalc(void *src, uint8_t size)
{
	uint8_t ret = 0;
	uint8_t *p;
	int i = 0;
	uint8_t pBuf = 0;
	p = (uint8_t*)src;
 
	while(size--)
	{
		pBuf = *p ++;
 
		for ( i = 0; i < 8; i ++ )
		{
			if ((ret ^ (pBuf)) & 0x01)
			{
				ret ^= 0x18;
				ret >>= 1;
				ret |= 0x80;
			}
			else
			{
				ret >>= 1;
			}
 
			pBuf >>= 1;
		}
	}
 
	return ret;
}
/*******END OF FILE********/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
