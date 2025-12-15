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
#include "I2C_A.h"
#include "lora_config.h"

bool iic_noack=0;

void I2C_GPIO_MODE_Config(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
	
  gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
}

void I2C_GPIO_MODE_ANALOG(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
  gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_ANALOG);
	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_ANALOG);	
}

uint8_t I2C_Write_Byte(uint8_t addr,uint8_t data)
{
    I2C_Start();
    I2C_SendByte((addr<<1)|0); 
    if(I2C_WaitAck())          
    {
        I2C_Stop();
			  iic_noack=1;
        return 1;
    }

    I2C_SendByte(data);        
    if(I2C_WaitAck())         
    {
        I2C_Stop();
			  iic_noack=1;
        return 1;
    }
    I2C_Stop();
    return 0;
}

uint8_t I2C_Read_Byte(uint8_t addr)
{
    uint8_t res;

	  I2C_Start();                
    I2C_SendByte((addr<<1)|1); 
    I2C_WaitAck();             
    res=I2C_ReadByte(0);		
    I2C_Stop();                 
    return res;  
}

uint8_t I2C_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    I2C_Start();
     I2C_SendByte((addr<<1)|0); 
    if(I2C_WaitAck())          
    {
        I2C_Stop();
        return 1;
    }
//     I2C_SendByte(reg);        
//    I2C_WaitAck();             
    for(i=0;i<len;i++)
    {
         I2C_SendByte(buf[i]); 
        if(I2C_WaitAck())      
        {
            I2C_Stop();
            return 1;
        }
    }
    I2C_Stop();
    return 0;
} 

uint8_t I2C_Write_reg_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    I2C_Start();
     I2C_SendByte((addr<<1)|0); 
    if(I2C_WaitAck())          
    {
        I2C_Stop();
        return 1;
    }
     I2C_SendByte(reg);        
    I2C_WaitAck();             
    for(i=0;i<len;i++)
    {
         I2C_SendByte(buf[i]); 
        if(I2C_WaitAck())      
        {
            I2C_Stop();
            return 1;
        }
    }
    I2C_Stop();
    return 0;
} 

uint8_t I2C_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
//    I2C_Start();
//     I2C_SendByte((addr<<1)|0); 
//    if(I2C_WaitAck())          
//    {
//        I2C_Stop();
//        return 1;
//    }
//     I2C_SendByte(reg);         
//    I2C_WaitAck();            
	  I2C_Start();                
     I2C_SendByte((addr<<1)|1); 
        if(I2C_WaitAck())      
        {
            I2C_Stop();
            return 1;
        }             
    while(len)
    {
        if(len==1)*buf=I2C_ReadByte(0);
		else *buf=I2C_ReadByte(1);		
		len--;
		buf++;  
    }
    I2C_Stop();                 
    return 0;      
}

uint8_t I2C_Read_reg_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    I2C_Start();
     I2C_SendByte((addr<<1)|0); 
    if(I2C_WaitAck())          
    {
        I2C_Stop();
        return 1;
    }
     I2C_SendByte(reg);         
    I2C_WaitAck();            
	  I2C_Start();                
     I2C_SendByte((addr<<1)|1); 
        if(I2C_WaitAck())      
        {
            I2C_Stop();
            return 1;
        }             
    while(len)
    {
        if(len==1)*buf=I2C_ReadByte(0);
		else *buf=I2C_ReadByte(1);		
		len--;
		buf++;  
    }
    I2C_Stop();                 
    return 0;      
}

void SDA_IN(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_INPUT_FLOATING);
}

void SDA_OUT(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
}

void I2C_Delay(void)
{
  volatile int i = 2;
    while (i)

    i--;
}
void I2C_Start(void)
{
	SDA_OUT();	
	I2C_SDA_1;
	I2C_SCL_1;
	I2C_Delay();
	I2C_SDA_0;
	I2C_Delay();
	I2C_SCL_0;
	I2C_Delay();
}	
void I2C_Stop(void)
{
	SDA_OUT();
	I2C_SCL_0;
	I2C_SDA_0;
	I2C_Delay();
	I2C_SCL_1;
	I2C_SDA_1;
	I2C_Delay();
}
uint8_t I2C_WaitAck(void)
{
	uint8_t ucErrTime=0;
  SDA_IN();
	I2C_SDA_1;	
	I2C_Delay();
	I2C_SCL_1;	
	I2C_Delay();
	while (I2C_SDA_READ())	
	{
			ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_Stop();
			return 1;
		}
	}

	I2C_SCL_0;
	I2C_Delay();
	return 0;
}
void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	SDA_OUT();
	I2C_SCL_0;
	for (i = 0; i < 8; i++)
	{		
		if (Byte & 0x80)
		{
			I2C_SDA_1;
		}
		else
		{
			I2C_SDA_0;
		}
		I2C_Delay();
		I2C_SCL_1;
		I2C_Delay();
    I2C_Delay();		
		I2C_SCL_0;
//		I2C_SDA_1; 
		Byte <<= 1;	
		I2C_Delay();
	}
}
uint8_t I2C_ReadByte(unsigned char ack)
{
	uint8_t i;
	uint8_t value;
  SDA_IN();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_0;
		I2C_Delay();
		I2C_Delay();
		I2C_SCL_1;
		
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_Delay();
	}
	
	if (!ack)
	{
			I2C_NAck();
	}
	else
	{
			I2C_Ack(); 
	}
		
	return value;
}	

void I2C_Ack(void)
{
	I2C_SCL_0;
	SDA_OUT();
	I2C_SDA_0;
	I2C_Delay();
	I2C_SCL_1;
	I2C_Delay();
	I2C_SCL_0;
}

void I2C_NAck(void)
{
	I2C_SCL_0;
	SDA_OUT();
	I2C_SDA_1;
	I2C_Delay();
	I2C_SCL_1;
	I2C_Delay();
	I2C_SCL_0;	
}
//-- Static function prototypes -----------------------------------------------
static etError I2c_WaitWhileClockStreching(uint8_t timeout);

void SDA_INPUT(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_INPUT_FLOATING);
}

void SDA_OUTPUT(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
}
	
void SCL_INPUT(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_INPUT_FLOATING);
}

void SCL_OUTPUT(void)
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

	gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
}

void DelayMicroSeconds(uint16_t delay)
{
  volatile int i = delay;
    while (i)

    i--;
}

//-----------------------------------------------------------------------------
void I2c_hal_Init(void)                      /* -- adapt the init for your uC -- */
{
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
	
  gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
	gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ);
}

//-----------------------------------------------------------------------------
void I2c_hal_StartCondition(void)
{
  SDA_OPEN();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(10);  // hold time start condition (t_HD;STA)
  SCL_LOW();
  DelayMicroSeconds(10);
}

//-----------------------------------------------------------------------------
void I2c_hal_StopCondition(void)
{
  SCL_LOW();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(10);  // set-up time stop condition (t_SU;STO)
  SDA_OPEN();
  DelayMicroSeconds(10);
}

//-----------------------------------------------------------------------------
etError I2c_hal_WriteByte(uint8_t txByte)
{
  etError error = NO_ERROR;
  uint8_t     mask;
  for(mask = 0x80; mask > 0; mask >>= 1)// shift bit for masking (8 times)
  {
    if((mask & txByte) == 0) SDA_LOW(); // masking txByte, write bit to SDA-Line
    else                     SDA_OPEN();
    DelayMicroSeconds(2);               // data set-up time (t_SU;DAT)
    SCL_OPEN();                         // generate clock pulse on SCL
    DelayMicroSeconds(13);               // SCL high time (t_HIGH)
    SCL_LOW();
    DelayMicroSeconds(2);               // data hold time(t_HD;DAT)
  }
  SDA_OPEN();                           // release SDA-line
  SCL_OPEN();                           // clk #9 for ack
  DelayMicroSeconds(2);                 // data set-up time (t_SU;DAT)
	SDA_INPUT();
  if(SDA_READ) error = ACK_ERROR;       // check ack from i2c slave
  SCL_LOW();
  DelayMicroSeconds(20);                // wait to see byte package on scope
  return error;                         // return error code
}

//-----------------------------------------------------------------------------
etError I2c_hal_ReadByte(uint8_t *rxByte, etI2cAck ack, uint8_t timeout)
{
  etError error = NO_ERROR;
  uint8_t mask;
  *rxByte = 0x00;
  SDA_OPEN();                            // release SDA-line
  for(mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
  { 
    SCL_OPEN();                          // start clock on SCL-line
    DelayMicroSeconds(2);                // clock set-up time (t_SU;CLK)
    error = I2c_WaitWhileClockStreching(timeout);// wait while clock streching
    DelayMicroSeconds(3);                // SCL high time (t_HIGH)
		SDA_INPUT();
    if(SDA_READ) *rxByte |= mask;        // read bit
    SCL_LOW();
    DelayMicroSeconds(12);                // data hold time(t_HD;DAT)
  }
  if(ack == ACK) SDA_LOW();              // send acknowledge if necessary
  else           SDA_OPEN();
  DelayMicroSeconds(2);                  // data set-up time (t_SU;DAT)
  SCL_OPEN();                            // clk #9 for ack
  DelayMicroSeconds(13);                  // SCL high time (t_HIGH)
  SCL_LOW();
  SDA_OPEN();                            // release SDA-line
  DelayMicroSeconds(20);                 // wait to see byte package on scope
  
  return error;                          // return with no error
}

//-----------------------------------------------------------------------------
etError I2c_hal_GeneralCallReset(void)
{
  etError error;
  
  I2c_hal_StartCondition();
                        error = I2c_hal_WriteByte(0x00);
  if(error == NO_ERROR) error = I2c_hal_WriteByte(0x06);
  
  return error;
}

//-----------------------------------------------------------------------------
static etError I2c_WaitWhileClockStreching(uint8_t timeout)
{
  etError error = NO_ERROR;
  
	SCL_INPUT();
  while(SCL_READ == 0)
  {
    if(timeout-- == 0) return TIMEOUT_ERROR;
    DelayMicroSeconds(1000);
  }
  
  return error;
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
