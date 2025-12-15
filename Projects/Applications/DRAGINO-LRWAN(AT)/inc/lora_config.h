#ifndef __LORA_CONFIG_H
#define __LORA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tremo_gpio.h"
#include "tremo_rtc.h"
	
#define CONFIG_LORA_RFSW_CTRL_GPIOX GPIOD
#define CONFIG_LORA_RFSW_CTRL_PIN   GPIO_PIN_11

#define CONFIG_LORA_RFSW_VDD_GPIOX GPIOA
#define CONFIG_LORA_RFSW_VDD_PIN   GPIO_PIN_10

/* ---------------------------  BAT_pin definition -------------------------------*/	
#define ADC_BAT_LEVEL_PORT        GPIOA                                                 
#define ADC_BAT_LEVEL_PIN         GPIO_PIN_11
#define GPIO_ADC_BAT_CHANNEL      1
#define ADC_BAT_OUTPUT_PORT       GPIOA                                                 
#define ADC_BAT_OUTPUT_PIN        GPIO_PIN_12
	
#define ADC_SOLAR_LEVEL_PORT        GPIOA                                                 
#define ADC_SOLAR_LEVEL_PIN         GPIO_PIN_5
#define GPIO_SOLAR_BAT_CHANNEL      3

/* ---------------------------  LED_pin definition -------------------------------*/
#define LED_CLK_ENABLE()					rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true)
#define LED_RGB_PORT              GPIOB	
#define LED_RED_PIN               GPIO_PIN_13 
#define LED_BLUE_PIN    			    GPIO_PIN_12 
#define LED_GREEN_PIN    			    GPIO_PIN_14 

/* ---------------------------  POWER_pin definition -------------------------------*/
#define POWER_CLK_ENABLE()	  		rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true)
#define POWER_PORT            		GPIOB	
#define POWER_5V_PIN    			  	GPIO_PIN_7 

/* ---------------------------  EXIT_pin definition -------------------------------*/
#define GPIO_EXTI4_CLK_ENABLE()		rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)
#define GPIO_EXTI4_PORT        	  GPIOA	
#define GPIO_EXTI4_PIN    			  GPIO_PIN_4 
#define GPIO_EXTI8_CLK_ENABLE()		rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)
#define GPIO_EXTI8_PORT        	  GPIOA	
#define GPIO_EXTI8_PIN    			  GPIO_PIN_8 
#define GPIO_EXTI15_CLK_ENABLE()	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true)
#define GPIO_EXTI15_PORT        	GPIOB	
#define GPIO_EXTI15_PIN    			  GPIO_PIN_15 
#define USERKEY_CLK_ENABLE()	    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true)
#define GPIO_USERKEY_PORT         GPIOC	
#define GPIO_USERKEY_PIN    		  GPIO_PIN_8 

/* ---------------------------  UART TX_RX_pin definition -------------------------------*/
#define UART2_RCC_ENABLE()      rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART2, true)
#define USAR2_CLK_ENABLE()	    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true)
#define USAR2_TX_GPIO_PORT      GPIOB	
#define USAR2_TX_PIN            GPIO_PIN_9 
#define USAR2_RX_GPIO_PORT      GPIOB	
#define USAR2_RX_PIN            GPIO_PIN_8
#define USAR2_TX_RX_MUX         1

/* ---------------------------  DS18B20 HW definition -------------------------------*/
#define DOUT1_CLK_ENABLE()  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true)  
#define DOUT1_PORT          GPIOC	 
#define DOUT1_PIN           GPIO_PIN_13
#define DOUT1_READ()        gpio_read(DOUT1_PORT,DOUT1_PIN)
#define DOUT1_0             gpio_write(DOUT1_PORT,DOUT1_PIN,0)
#define DOUT1_1             gpio_write(DOUT1_PORT,DOUT1_PIN,1)
#define DOUT2_CLK_ENABLE()  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true)  
#define DOUT2_PORT          GPIOB	 
#define DOUT2_PIN           GPIO_PIN_9
#define DOUT2_READ()        gpio_read(DOUT2_PORT,DOUT2_PIN)
#define DOUT2_0             gpio_write(DOUT2_PORT,DOUT2_PIN,0)
#define DOUT2_1             gpio_write(DOUT2_PORT,DOUT2_PIN,1)
#define DOUT3_CLK_ENABLE()  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true)  
#define DOUT3_PORT          GPIOB	 
#define DOUT3_PIN           GPIO_PIN_8
#define DOUT3_READ()        gpio_read(DOUT3_PORT,DOUT3_PIN)
#define DOUT3_0             gpio_write(DOUT3_PORT,DOUT3_PIN,0)
#define DOUT3_1             gpio_write(DOUT3_PORT,DOUT3_PIN,1)

/* ---------------------------  ADC_IN definition -------------------------------*/
#define ADC_IN1_LEVEL_PORT        GPIOA                                                 
#define ADC_IN1_LEVEL_PIN         GPIO_PIN_8
#define GPIO_ADC_IN1_CHANNEL      2
#define ADC_IN2_LEVEL_PORT        GPIOA       
#define ADC_IN2_LEVEL_PIN         GPIO_PIN_5
#define GPIO_ADC_IN2_CHANNEL      3
#define ADC_IN3_LEVEL_PORT        GPIOA      
#define ADC_IN3_LEVEL_PIN         GPIO_PIN_4
#define GPIO_ADC_IN3_CHANNEL      4

/* ---------------------------  I2C HW definition -------------------------------*/
#define GPIO_PORT_I2C	 GPIOA			
#define I2C_SCL_PIN		 GPIO_PIN_14		
#define I2C_SDA_PIN		 GPIO_PIN_15
#define I2C_SCL_1  gpio_write(GPIO_PORT_I2C, I2C_SCL_PIN, 1)		
#define I2C_SCL_0  gpio_write(GPIO_PORT_I2C, I2C_SCL_PIN, 0)	
#define I2C_SDA_1  gpio_write(GPIO_PORT_I2C, I2C_SDA_PIN, 1)		/* SDA = 1 */
#define I2C_SDA_0  gpio_write(GPIO_PORT_I2C, I2C_SDA_PIN, 0)		/* SDA = 0 */
#define I2C_SDA_READ()  gpio_read(GPIO_PORT_I2C, I2C_SDA_PIN)	

#define SDA_LOW()  gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_PP_LOW) // set SDA to low
#define SDA_OPEN() gpio_init(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_MODE_OUTPUT_OD_HIZ); // set SDA to open-drain
#define SDA_READ   gpio_read(GPIO_PORT_I2C, I2C_SDA_PIN)     // read SDA

#define SCL_LOW()  gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUTPUT_PP_LOW) // set SCL to low
#define SCL_OPEN() gpio_init(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_MODE_OUTPUT_OD_HIZ); // set SCL to open-drain
#define SCL_READ   gpio_read(GPIO_PORT_I2C, I2C_SCL_PIN)     // read SCL

/* ---------------------------  WEIGHT HW definition -------------------------------*/
#define WEIGHT_SCK_CLK_ENABLE()    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)
#define WEIGHT_SCK_PORT       		 GPIOA	 
#define WEIGHT_SCK_PIN       			 GPIO_PIN_14 
#define WEIGHT_DOUT_CLK_ENABLE()   rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)
#define WEIGHT_DOUT_PORT      		 GPIOA
#define WEIGHT_DOUT_PIN     			 GPIO_PIN_15  
#define HX711_SCK_0          			 gpio_write(WEIGHT_SCK_PORT,WEIGHT_SCK_PIN ,0)
#define HX711_SCK_1          			 gpio_write(WEIGHT_SCK_PORT,WEIGHT_SCK_PIN ,1)

/* ---------------------------  ULT definition -------------------------------*/
#define ULT_TRIG_CLK_ENABLE()     rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)
#define ULT_TRIG_PORT             GPIOA
#define ULT_TRIG_PIN              GPIO_PIN_14
#define ULT_Echo_CLK_ENABLE()     rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)
#define ULT_Echo_PORT             GPIOA
#define ULT_Echo_PIN              GPIO_PIN_15

/* ---------------------------  PWM  definition -------------------------------*/
#define PWM_CLK_ENABLE()     		rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)
#define PWM_RCC_ENABLE()     		rcc_enable_peripheral_clk(RCC_PERIPHERAL_TIMER1, true)
#define PWM_RCC_DISABLE()     	rcc_enable_peripheral_clk(RCC_PERIPHERAL_TIMER1, false)
#define TIMER_ENABLE()     			timer_cmd(TIMER1, true)
#define TIMER_DISABLE()     		timer_cmd(TIMER1, false)
#define PWM_PORT     						GPIOA
#define PWM_PIN      						GPIO_PIN_15
	
/* ---------------------------  DX-BT24_pin definition -------------------------------*/
#define DX_BT24_CLK_ENABLE()		rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true)
#define DX_BT24_PORT          	GPIOD	
#define DX_BT24_RST_PIN    	  	GPIO_PIN_10 
#define DX_BT24_KEY_PIN    	  	GPIO_PIN_14 
#define DX_BT24_STATUS_ENABLE()	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true)
#define DX_BT24_STATUS_PORT     GPIOC	
#define DX_BT24_LINK_PIN    	  GPIO_PIN_1 
#define DX_BT24_WORK_PIN    	  GPIO_PIN_5 

#ifdef __cplusplus
}
#endif

#endif /* __LORA_CONFIG_H */
