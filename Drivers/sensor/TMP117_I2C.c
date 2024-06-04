/**
  ******************************************************************************
  * @file    
  * @author  Dragino
  * @version 
  * @date    
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "TMP117_I2C.h"
#include "tremo_delay.h"

#define GPIO_PORT_TMP117_I2C	 GPIOA			
#define TMP117_I2C_SCL_PIN		 GPIO_PIN_14		
#define TMP117_I2C_SDA_PIN		 GPIO_PIN_15

#define TMP117_I2C_SCL_1  gpio_write(GPIO_PORT_TMP117_I2C, TMP117_I2C_SCL_PIN, 1)		
#define TMP117_I2C_SCL_0  gpio_write(GPIO_PORT_TMP117_I2C, TMP117_I2C_SCL_PIN, 0)	
	
#define TMP117_I2C_SDA_1  gpio_write(GPIO_PORT_TMP117_I2C, TMP117_I2C_SDA_PIN, 1)		/* SDA = 1 */
#define TMP117_I2C_SDA_0  gpio_write(GPIO_PORT_TMP117_I2C, TMP117_I2C_SDA_PIN, 0)		/* SDA = 0 */
	
#define TMP117_I2C_SDA_READ()  gpio_read(GPIO_PORT_TMP117_I2C, TMP117_I2C_SDA_PIN)	

extern bool tmp117_connect_status;

void TMP117_I2C_GPIO_MODE_Config(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
	
  gpio_init(GPIO_PORT_TMP117_I2C, TMP117_I2C_SCL_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
	gpio_init(GPIO_PORT_TMP117_I2C, TMP117_I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
}

uint8_t TMP117_I2C_Write_Byte(uint8_t addr,uint8_t data)
{
    TMP117_I2C_Start();
    TMP117_I2C_SendByte((addr<<1)|0); 
    if(TMP117_I2C_WaitAck())          
    {
        TMP117_I2C_Stop();
        return 1;
    }

    TMP117_I2C_SendByte(data);        
    if(TMP117_I2C_WaitAck())         
    {
        TMP117_I2C_Stop();
        return 1;
    }
    TMP117_I2C_Stop();
    return 0;
}

uint8_t TMP117_I2C_Read_Byte(uint8_t addr)
{
    uint8_t res;

	  TMP117_I2C_Start();                
    TMP117_I2C_SendByte((addr<<1)|1); 
    TMP117_I2C_WaitAck();             
    res=TMP117_I2C_ReadByte(0);		
    TMP117_I2C_Stop();                 
    return res;  
} 

uint8_t TMP117_I2C_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    TMP117_I2C_Start();
     TMP117_I2C_SendByte((addr<<1)|0); 
    if(TMP117_I2C_WaitAck())          
    {
        TMP117_I2C_Stop();
        return 1;
    }
     TMP117_I2C_SendByte(reg);        
    TMP117_I2C_WaitAck();             
    for(i=0;i<len;i++)
    {
         TMP117_I2C_SendByte(buf[i]); 
        if(TMP117_I2C_WaitAck())      
        {
            TMP117_I2C_Stop();
            return 1;
        }
    }
    TMP117_I2C_Stop();
    return 0;
} 

uint8_t TMP117_I2C_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{             
	  TMP117_I2C_Start();                
     TMP117_I2C_SendByte((addr<<1)|0); 
        if(TMP117_I2C_WaitAck())      
        {
            TMP117_I2C_Stop();
            return 1;
        }  

     TMP117_I2C_SendByte(reg);         
    TMP117_I2C_WaitAck(); 
			
	  TMP117_I2C_Start();                
     TMP117_I2C_SendByte((addr<<1)|1); 
    TMP117_I2C_WaitAck();
				
    while(len)
    {
        if(len==1)*buf=TMP117_I2C_ReadByte(0);
		else *buf=TMP117_I2C_ReadByte(1);		
		len--;
		buf++;  
    }
    TMP117_I2C_Stop();                 
    return 0;      
}

void TMP117_I2C_SDA_IN(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_TMP117_I2C, TMP117_I2C_SDA_PIN, GPIO_MODE_INPUT_FLOATING);
}

void TMP117_I2C_SDA_OUT(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_TMP117_I2C, TMP117_I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
}

void TMP117_I2C_Delay(void)
{
  volatile int i = 5;
    while (i)

    i--;
}
void TMP117_I2C_Start(void)
{
	TMP117_I2C_SDA_OUT();	
	TMP117_I2C_SDA_1;
	TMP117_I2C_SCL_1;
	TMP117_I2C_Delay();
	TMP117_I2C_SDA_0;
	TMP117_I2C_Delay();
	TMP117_I2C_SCL_0;
	TMP117_I2C_Delay();
}	
void TMP117_I2C_Stop(void)
{
	TMP117_I2C_SDA_OUT();
	TMP117_I2C_SCL_0;
	TMP117_I2C_SDA_0;
	TMP117_I2C_Delay();
	TMP117_I2C_SCL_1;
	TMP117_I2C_SDA_1;
	TMP117_I2C_Delay();
}
uint8_t TMP117_I2C_WaitAck(void)
{
	uint8_t ucErrTime=0;
  TMP117_I2C_SDA_IN();
	TMP117_I2C_SDA_1;	
	TMP117_I2C_Delay();
	TMP117_I2C_SCL_1;	
	TMP117_I2C_Delay();
	while (TMP117_I2C_SDA_READ())	
	{
			ucErrTime++;
		if(ucErrTime>250)
		{
			TMP117_I2C_Stop();
			return 1;
		}
	}

	TMP117_I2C_SCL_0;
	TMP117_I2C_Delay();
	return 0;
}
void TMP117_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	TMP117_I2C_SDA_OUT();
	TMP117_I2C_SCL_0;
	for (i = 0; i < 8; i++)
	{		
		if (Byte & 0x80)
		{
			TMP117_I2C_SDA_1;
		}
		else
		{
			TMP117_I2C_SDA_0;
		}
		TMP117_I2C_Delay();
		TMP117_I2C_SCL_1;
		TMP117_I2C_Delay();	
		TMP117_I2C_Delay();
		TMP117_I2C_SCL_0;
//		TMP117_I2C_SDA_1; 
		Byte <<= 1;	
		TMP117_I2C_Delay();
	}
}
uint8_t TMP117_I2C_ReadByte(unsigned char ack)
{
	uint8_t i;
	uint8_t value;
  TMP117_I2C_SDA_IN();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		TMP117_I2C_SCL_0;
		TMP117_I2C_Delay();
		TMP117_I2C_Delay();
		TMP117_I2C_SCL_1;
		
		if (TMP117_I2C_SDA_READ())
		{
			value++;
		}
		TMP117_I2C_Delay();
	}
	
	if (!ack)
	{
			TMP117_I2C_NAck();
	}
	else
	{
			TMP117_I2C_Ack(); 
	}
		
	return value;
}	

void TMP117_I2C_Ack(void)
{
	TMP117_I2C_SCL_0;
	TMP117_I2C_SDA_OUT();
	TMP117_I2C_SDA_0;
	TMP117_I2C_Delay();
	TMP117_I2C_SCL_1;
	TMP117_I2C_Delay();
	TMP117_I2C_SCL_0;
}

void TMP117_I2C_NAck(void)
{
	TMP117_I2C_SCL_0;
	TMP117_I2C_SDA_OUT();
	TMP117_I2C_SDA_1;
	TMP117_I2C_Delay();
	TMP117_I2C_SCL_1;
	TMP117_I2C_Delay();
	TMP117_I2C_SCL_0;	
}

uint16_t TMP117_Set_Config(void)
{
	uint8_t data[2] = {0x0C, 0x20};
	uint8_t read_status=0,check_number=0;
	
	TMP117_I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(TMP117_I2C_Write_Len(0x48,0x01,2,data)==1)
		{
			read_status=0;
						
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	return 0;
}

float get_tmp117_temp(void)
{
	short AD_code;
	float current_temp;
	uint8_t data[2] = {0x0C, 0x20};
	uint8_t read_status=0,check_number=0;
	
  do
	{
		read_status=1;
		check_number++;
	  if(TMP117_I2C_Write_Len(0x48,0x01,2,data)==1)
		{
			read_status=0;
						
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(130);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(TMP117_I2C_Read_Len(0x48,0x00,2,data)==1)
			{
				read_status=0;
								
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}
	
	if(read_status==1)
	{
		tmp117_connect_status=1;
		AD_code=(data[0]<<8)|data[1];
		
		if(AD_code <0)
		{
			current_temp=-(~AD_code+1)*0.0078125;
		}
		else 
			current_temp=AD_code*0.0078125;
		
		if(current_temp>150)
		{
			tmp117_connect_status=0;			
			current_temp=150;
		}
		else if(current_temp<-55)
		{
			tmp117_connect_status=0;			
			current_temp=-55;
		}
	}
	else
	{
		current_temp=327.67;
  	tmp117_connect_status=0;
	}
	
	return current_temp;
}

void TMP117_soft_reset(void)
{
	uint8_t data[2] = {0x0C, 0x22};
	uint8_t read_status=0,check_number=0;
	
	do
	{
		read_status=1;
		check_number++;
	  if(TMP117_I2C_Write_Len(0x48,0x01,2,data)==1)
		{
			read_status=0;
			
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	delay_ms(20);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
