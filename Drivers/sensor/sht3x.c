//=============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//=============================================================================
// Project   :  SHT3x Sample Code (V1.1)
// File      :  sht3x.c (V1.1)
// Author    :  RFU
// Date      :  6-Mai-2015
// Controller:  STM32F100RB
// IDE       :  µVision V5.12.0.0
// Compiler  :  Armcc
// Brief     :  Sensor Layer: Implementation of functions for sensor access.
//=============================================================================

//-- Includes -----------------------------------------------------------------

#include "stdio.h"
#include "I2C_A.h"
#include "tremo_delay.h"
#include "sht3x.h"
#include "log.h"

//-- Defines ------------------------------------------------------------------
// Generator polynomial for CRC
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

//-- Global variables ---------------------------------------------------------
static uint8_t _i2cAddress; // I2C Address

//-- Static function prototypes -----------------------------------------------
static etError SHT3X_WriteAlertLimitData(float humidity, float temperature);
static etError SHT3X_ReadAlertLimitData(float* humidity, float* temperature);
static etError SHT3X_StartWriteAccess(void);
static etError SHT3X_StartReadAccess(void);
static void SHT3X_StopAccess(void);
static etError SHT3X_WriteCommand(etCommands command);
static etError SHT3X_Read2BytesAndCrc(uint16_t* data, etI2cAck finaleAckNack,
                                      uint8_t timeout);
static etError SHT3X_Write2BytesAndCrc(uint16_t data);
static uint8_t SHT3X_CalcCrc(uint8_t data[], uint8_t nbrOfBytes);
static etError SHT3X_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
static float SHT3X_CalcTemperature(uint16_t rawValue);
static float SHT3X_CalcHumidity(uint16_t rawValue);
static uint16_t SHT3X_CalcRawTemperature(float temperature);
static uint16_t SHT3X_CalcRawHumidity(float humidity);

//-----------------------------------------------------------------------------
void SHT3X_Init(uint8_t i2cAddress)          /* -- adapt the init for your uC -- */
{ 
  I2c_hal_Init(); // init I2C
  SHT3X_SetI2cAdr(i2cAddress);
}

//-----------------------------------------------------------------------------
void SHT3X_SetI2cAdr(uint8_t i2cAddress)
{
  _i2cAddress = i2cAddress;
}

//-----------------------------------------------------------------------------
etError SHT3x_ReadSerialNumber(uint32_t* serialNumber)
{
  etError error; // error code
  uint16_t serialNumWords[2];
  
  error = SHT3X_StartWriteAccess();
  
  // write "read serial number" command
  error |= SHT3X_WriteCommand(CMD_READ_SERIALNBR);
  // if no error, start read access
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();
  // if no error, read first serial number word
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[0], ACK, 100);
  // if no error, read second serial number word
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[1], NACK, 0);
  
  SHT3X_StopAccess();
  
  // if no error, calc serial number as 32-bit integer
  if(error == NO_ERROR)
  {
    *serialNumber = (serialNumWords[0] << 16) | serialNumWords[1];
  }
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_ReadStatus(uint16_t* status)
{
  etError error; // error code
  
  error = SHT3X_StartWriteAccess();
  
  // if no error, write "read status" command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_READ_STATUS);
  // if no error, start read access
  if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
  // if no error, read status
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(status, NACK, 0);
  
  SHT3X_StopAccess();
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_ClearAllAlertFlags(void)
{
  etError error; // error code
  
  error = SHT3X_StartWriteAccess();
  
  // if no error, write clear status register command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_CLEAR_STATUS);
  
  SHT3X_StopAccess();
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumi(float* temperature, float* humidity,
                             etRepeatability repeatability, etMode mode,
                             uint8_t timeout)
{
  etError error;
                               
  switch(mode)
  {    
    case MODE_CLKSTRETCH: // get temperature with clock stretching mode
      error = SHT3X_GetTempAndHumiClkStretch(temperature, humidity,
                                             repeatability, timeout);
      break;
    case MODE_POLLING:    // get temperature with polling mode
      error = SHT3X_GetTempAndHumiPolling(temperature, humidity,
                                          repeatability, timeout);
      break;
    default:              
      error = PARM_ERROR;
      break;
  }
  
  return error;
}


//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiClkStretch(float* temperature, float* humidity,
                                       etRepeatability repeatability,
                                       uint8_t timeout)
{
  etError error;        // error code
  uint16_t    rawValueTemp; // temperature raw value from sensor
  uint16_t    rawValueHumi; // humidity raw value from sensor
  
  error = SHT3X_StartWriteAccess();
  
  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in clock stretching mode
    // use depending on the required repeatability, the corresponding command
    switch(repeatability)
    {
      case REPEATAB_LOW:
        error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_L);
        break;
      case REPEATAB_MEDIUM:
        error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_M);
        break;
      case REPEATAB_HIGH:
        error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_H);
        break;
      default:
        error = PARM_ERROR;
        break;
    }
  }

	delay_ms(20);//Measurement duration
	
  // if no error, start read access
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();
  // if no error, read temperature raw values
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, timeout);
  // if no error, read humidity raw values
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
  
  SHT3X_StopAccess();
  
  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR)
  {
    *temperature = SHT3X_CalcTemperature(rawValueTemp);
    *humidity = SHT3X_CalcHumidity(rawValueHumi);
		if(SHT3X_CalcHumidity(rawValueHumi)>=100)
		{
			error = PARM_ERROR;
		}
  }
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiPolling(float* temperature, float* humidity,
                                    etRepeatability repeatability,
                                    uint8_t timeout)
{
  etError error;           // error code
  uint16_t    rawValueTemp;    // temperature raw value from sensor
  uint16_t    rawValueHumi;    // humidity raw value from sensor
  
  error  = SHT3X_StartWriteAccess();
  
  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in polling mode
    // use depending on the required repeatability, the corresponding command
    switch(repeatability)
    {
      case REPEATAB_LOW:
        error = SHT3X_WriteCommand(CMD_MEAS_POLLING_L);
        break;
      case REPEATAB_MEDIUM:
        error = SHT3X_WriteCommand(CMD_MEAS_POLLING_M);
        break;
      case REPEATAB_HIGH:
        error = SHT3X_WriteCommand(CMD_MEAS_POLLING_H);
        break;
      default:
        error = PARM_ERROR;
        break;
    }
  }
  
  // if no error, wait until measurement ready
  if(error == NO_ERROR)
  {
    // poll every 1ms for measurement ready until timeout
    while(timeout--)
    {
      // check if the measurement has finished
      error = SHT3X_StartReadAccess();
  
      // if measurement has finished -> exit loop
      if(error == NO_ERROR) break;
  
      // delay 1ms
      delay_ms(1);
    }
    
    // check for timeout error
    if(timeout == 0) error = TIMEOUT_ERROR;
  }
  
  // if no error, read temperature and humidity raw values
  if(error == NO_ERROR)
  {
    error |= SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
    error |= SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
  }
  
  SHT3X_StopAccess();
  
  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR)
  {
    *temperature = SHT3X_CalcTemperature(rawValueTemp);
    *humidity = SHT3X_CalcHumidity(rawValueHumi);
  }
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_StartPeriodicMeasurment(etRepeatability repeatability,
                                      etFrequency frequency)
{
  etError error;        // error code
  
  error = SHT3X_StartWriteAccess();
  
  // if no error, start periodic measurement 
  if(error == NO_ERROR)
  {
    // use depending on the required repeatability and frequency,
    // the corresponding command
    switch(repeatability)
    {
      case REPEATAB_LOW: // low repeatability
        switch(frequency)
        {
          case FREQUENCY_HZ5:  // low repeatability,  0.5 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_L);
            break;          
          case FREQUENCY_1HZ:  // low repeatability,  1.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_L);
            break;          
          case FREQUENCY_2HZ:  // low repeatability,  2.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_L);
            break;          
          case FREQUENCY_4HZ:  // low repeatability,  4.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_L);
            break;          
          case FREQUENCY_10HZ: // low repeatability, 10.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_L);
            break;          
          default:
            error |= PARM_ERROR;
            break;
        }
        break;
        
      case REPEATAB_MEDIUM: // medium repeatability
        switch(frequency)
        {
          case FREQUENCY_HZ5:  // medium repeatability,  0.5 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_M);
			break;
          case FREQUENCY_1HZ:  // medium repeatability,  1.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_M);
			break;        
          case FREQUENCY_2HZ:  // medium repeatability,  2.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_M);
			break;        
          case FREQUENCY_4HZ:  // medium repeatability,  4.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_M);
			break;      
          case FREQUENCY_10HZ: // medium repeatability, 10.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_M);
			break;
          default:
            error |= PARM_ERROR;
			break;
        }
        break;
        
      case REPEATAB_HIGH: // high repeatability
        switch(frequency)
        {
          case FREQUENCY_HZ5:  // high repeatability,  0.5 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_H);
            break;
          case FREQUENCY_1HZ:  // high repeatability,  1.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_H);
            break;
          case FREQUENCY_2HZ:  // high repeatability,  2.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_H);
            break;
          case FREQUENCY_4HZ:  // high repeatability,  4.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_H);
            break;
          case FREQUENCY_10HZ: // high repeatability, 10.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_H);
            break;
          default:
            error |= PARM_ERROR;
            break;
        }
        break;
      default:
        error |= PARM_ERROR;
        break;
    }
  }

  SHT3X_StopAccess();

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_ReadMeasurementBuffer(float* temperature, float* humidity)
{
  etError  error;        // error code
  uint16_t     rawValueTemp; // temperature raw value from sensor
  uint16_t     rawValueHumi; // humidity raw value from sensor

  error = SHT3X_StartWriteAccess();

  // if no error, read measurements
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_FETCH_DATA);
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();  
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);

  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR)
  {
    *temperature = SHT3X_CalcTemperature(rawValueTemp);
    *humidity = SHT3X_CalcHumidity(rawValueHumi);
  }

  SHT3X_StopAccess();

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_EnableHeater(void)
{
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write heater enable command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_ENABLE);

  SHT3X_StopAccess();

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_DisableHeater(void)
{
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write heater disable command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_DISABLE);

  SHT3X_StopAccess();

  return error;
}


//-----------------------------------------------------------------------------
etError SHT3X_SetAlertLimits(float humidityHighSet,   float temperatureHighSet,
                             float humidityHighClear, float temperatureHighClear,
                             float humidityLowClear,  float temperatureLowClear,
                             float humidityLowSet,    float temperatureLowSet)
{
  etError  error;  // error code
  
  // write humidity & temperature alter limits, high set
  error = SHT3X_StartWriteAccess();
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_HS);
  if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityHighSet,
                                                          temperatureHighSet);
  SHT3X_StopAccess();

  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, high clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_HC);
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityHighClear,
                                                            temperatureHighClear);
    SHT3X_StopAccess();
  }

  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, low clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_LC);
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityLowClear,
                                                            temperatureLowClear);
    SHT3X_StopAccess();
  }
  
  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, low set
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_LS);
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityLowSet,
                                                            temperatureLowSet);
    SHT3X_StopAccess();
  }

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_GetAlertLimits(float* humidityHighSet,   float* temperatureHighSet,
                             float* humidityHighClear, float* temperatureHighClear,
                             float* humidityLowClear,  float* temperatureLowClear,
                             float* humidityLowSet,    float* temperatureLowSet)
{
  etError  error;  // error code
  
  // read humidity & temperature alter limits, high set
  error = SHT3X_StartWriteAccess();
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_HS);
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();
  if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityHighSet,
                                                         temperatureHighSet);
  SHT3X_StopAccess();

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, high clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_HC);
    if(error == NO_ERROR) error = SHT3X_StartReadAccess();
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityHighClear,
                                                           temperatureHighClear);
    SHT3X_StopAccess();
  }

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, low clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_LC);
    if(error == NO_ERROR) error = SHT3X_StartReadAccess();
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityLowClear,
                                                           temperatureLowClear);
    SHT3X_StopAccess();
  }

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, low set
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_LS);
    if(error == NO_ERROR) error = SHT3X_StartReadAccess();
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityLowSet,
                                                           temperatureLowSet);
    SHT3X_StopAccess();
  }
 
  return error;
}
                             
//-----------------------------------------------------------------------------
etError SHT3X_SoftReset(void)
{
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // write reset command
  error |= SHT3X_WriteCommand(CMD_SOFT_RESET);

  SHT3X_StopAccess();
  
  // if no error, wait 50 ms afloater reset
  if(error == NO_ERROR) delay_ms(50); 

  return error;
}

//-----------------------------------------------------------------------------
void SHT3X_HardReset(void)
{
//  // set reset low
//  RESET_LOW();

//  // wait 100 ms
//  delay_ms(100000);
//  
//  // release reset
//  RESET_HIGH();
//  
//  // wait 50 ms afloater reset
//  delay_ms(50000);
}

                             
//-----------------------------------------------------------------------------
static etError SHT3X_WriteAlertLimitData(float humidity, float temperature)
{
  etError  error;           // error code
  
  int16_t rawHumidity;
  int16_t rawTemperature;
  
  if((humidity < 0.0f) || (humidity > 100.0f) 
  || (temperature < -45.0f) || (temperature > 130.0f))
  {
    error = PARM_ERROR;
  }
  else
  {
    rawHumidity    = SHT3X_CalcRawHumidity(humidity);
    rawTemperature = SHT3X_CalcRawTemperature(temperature);

    error = SHT3X_Write2BytesAndCrc((rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF));
  }
  
  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_ReadAlertLimitData(float* humidity, float* temperature)
{
  etError  error;           // error code
  uint16_t     data;
  
  error = SHT3X_Read2BytesAndCrc(&data, NACK, 0);
  
  if(error == NO_ERROR)
  {
    *humidity = SHT3X_CalcHumidity(data & 0xFE00);
    *temperature = SHT3X_CalcTemperature(data << 7);
  }
  
  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_StartWriteAccess(void)
{
  etError error; // error code

  // write a start condition
  I2c_hal_StartCondition();

  // write the sensor I2C address with the write flag
  error = I2c_hal_WriteByte(_i2cAddress << 1);

  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_StartReadAccess(void)
{
  etError error; // error code

  // write a start condition
  I2c_hal_StartCondition();

  // write the sensor I2C address with the read flag
  error = I2c_hal_WriteByte(_i2cAddress << 1 | 0x01);

  return error;
}

//-----------------------------------------------------------------------------
static void SHT3X_StopAccess(void)
{
  // write a stop condition
  I2c_hal_StopCondition();
}

//-----------------------------------------------------------------------------
static etError SHT3X_WriteCommand(etCommands command)
{
  etError error; // error code

  // write the upper 8 bits of the command to the sensor
  error  = I2c_hal_WriteByte(command >> 8);

  // write the lower 8 bits of the command to the sensor
  error |= I2c_hal_WriteByte(command & 0xFF);

  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_Read2BytesAndCrc(uint16_t* data, etI2cAck finaleAckNack,
                                      uint8_t timeout)
{
  etError error;    // error code
  uint8_t     bytes[2]; // read data array
  uint8_t     checksum; // checksum byte
 
  // read two data bytes and one checksum byte
                        error = I2c_hal_ReadByte(&bytes[0], ACK, timeout);
  if(error == NO_ERROR) error = I2c_hal_ReadByte(&bytes[1], ACK, 0);
  if(error == NO_ERROR) error = I2c_hal_ReadByte(&checksum, finaleAckNack, 0);
  
  // verify checksum
  if(error == NO_ERROR) error = SHT3X_CheckCrc(bytes, 2, checksum);
  
  // combine the two bytes to a 16-bit value
  *data = (bytes[0] << 8) | bytes[1];
  
  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_Write2BytesAndCrc(uint16_t data)
{
  etError error;    // error code
  uint8_t     bytes[2]; // read data array
  uint8_t     checksum; // checksum byte
  
  bytes[0] = data >> 8;
  bytes[1] = data & 0xFF;
  checksum = SHT3X_CalcCrc(bytes, 2);
 
  // write two data bytes and one checksum byte
                        error = I2c_hal_WriteByte(bytes[0]); // write data MSB
  if(error == NO_ERROR) error = I2c_hal_WriteByte(bytes[1]); // write data LSB
  if(error == NO_ERROR) error = I2c_hal_WriteByte(checksum); // write checksum
  
  return error;
}

//-----------------------------------------------------------------------------
static uint8_t SHT3X_CalcCrc(uint8_t data[], uint8_t nbrOfBytes)
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else           crc = (crc << 1);
    }
  }
  
  return crc;
}

//-----------------------------------------------------------------------------
static etError SHT3X_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  uint8_t crc;     // calculated checksum
  
  // calculates 8-Bit checksum
  crc = SHT3X_CalcCrc(data, nbrOfBytes);
  
  // verify checksum
  if(crc != checksum) return CHECKSUM_ERROR;
  else                return NO_ERROR;
}

//-----------------------------------------------------------------------------
static float SHT3X_CalcTemperature(uint16_t rawValue)
{
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

//-----------------------------------------------------------------------------
static float SHT3X_CalcHumidity(uint16_t rawValue)
{
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * (float)rawValue / 65535.0f;
}

//-----------------------------------------------------------------------------
static uint16_t SHT3X_CalcRawTemperature(float temperature)
{
  // calculate raw temperature [ticks]
  // rawT = (temperature + 45) / 175 * (2^16-1)
  return (temperature + 45.0f) / 175.0f * 65535.0f;
}

//-----------------------------------------------------------------------------
static uint16_t SHT3X_CalcRawHumidity(float humidity)
{
  // calculate raw relative humidity [ticks]
  // rawRH = humidity / 100 * (2^16-1)
  return humidity / 100.0f * 65535.0f;
}
