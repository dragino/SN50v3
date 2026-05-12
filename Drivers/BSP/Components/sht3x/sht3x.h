//=============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//=============================================================================
// Project   :  SHT3x Sample Code (V1.1)
// File      :  sht3x.h (V1.1)
// Author    :  RFU
// Date      :  6-Mai-2015
// Controller:  STM32F100RB
// IDE       :  µVision V5.12.0.0
// Compiler  :  Armcc
// Brief     :  Sensor Layer: Definitions of commands and functions for sensor
//                            access.
//=============================================================================

#ifndef SHT3X_H
#define SHT3X_H

//-- Includes -----------------------------------------------------------------
#include "stm32wlxx_hal.h"
#include "i2c_hal.h"

//-- Enumerations -------------------------------------------------------------
// Sensor Commands
typedef enum{
  CMD_READ_SERIALNBR  = 0x3780, // read serial number
  CMD_READ_STATUS     = 0xF32D, // read status register
  CMD_CLEAR_STATUS    = 0x3041, // clear status register
  CMD_HEATER_ENABLE   = 0x306D, // enabled heater
  CMD_HEATER_DISABLE  = 0x3066, // disable heater
  CMD_SOFT_RESET      = 0x30A2, // soft reset
  CMD_MEAS_CLOCKSTR_H = 0x2C06, // measurement: clock stretching, high repeatability
  CMD_MEAS_CLOCKSTR_M = 0x2C0D, // measurement: clock stretching, medium repeatability
  CMD_MEAS_CLOCKSTR_L = 0x2C10, // measurement: clock stretching, low repeatability
  CMD_MEAS_POLLING_H  = 0x2400, // measurement: polling, high repeatability
  CMD_MEAS_POLLING_M  = 0x240B, // measurement: polling, medium repeatability
  CMD_MEAS_POLLING_L  = 0x2416, // measurement: polling, low repeatability
  CMD_MEAS_PERI_05_H  = 0x2032, // measurement: periodic 0.5 mps, high repeatability
  CMD_MEAS_PERI_05_M  = 0x2024, // measurement: periodic 0.5 mps, medium repeatability
  CMD_MEAS_PERI_05_L  = 0x202F, // measurement: periodic 0.5 mps, low repeatability
  CMD_MEAS_PERI_1_H   = 0x2130, // measurement: periodic 1 mps, high repeatability
  CMD_MEAS_PERI_1_M   = 0x2126, // measurement: periodic 1 mps, medium repeatability
  CMD_MEAS_PERI_1_L   = 0x212D, // measurement: periodic 1 mps, low repeatability
  CMD_MEAS_PERI_2_H   = 0x2236, // measurement: periodic 2 mps, high repeatability
  CMD_MEAS_PERI_2_M   = 0x2220, // measurement: periodic 2 mps, medium repeatability
  CMD_MEAS_PERI_2_L   = 0x222B, // measurement: periodic 2 mps, low repeatability
  CMD_MEAS_PERI_4_H   = 0x2334, // measurement: periodic 4 mps, high repeatability
  CMD_MEAS_PERI_4_M   = 0x2322, // measurement: periodic 4 mps, medium repeatability
  CMD_MEAS_PERI_4_L   = 0x2329, // measurement: periodic 4 mps, low repeatability
  CMD_MEAS_PERI_10_H  = 0x2737, // measurement: periodic 10 mps, high repeatability
  CMD_MEAS_PERI_10_M  = 0x2721, // measurement: periodic 10 mps, medium repeatability
  CMD_MEAS_PERI_10_L  = 0x272A, // measurement: periodic 10 mps, low repeatability
  CMD_FETCH_DATA      = 0xE000, // readout measurements for periodic mode
  CMD_R_AL_LIM_LS     = 0xE102, // read alert limits, low set
  CMD_R_AL_LIM_LC     = 0xE109, // read alert limits, low clear
  CMD_R_AL_LIM_HS     = 0xE11F, // read alert limits, high set
  CMD_R_AL_LIM_HC     = 0xE114, // read alert limits, high clear
  CMD_W_AL_LIM_HS     = 0x611D, // write alert limits, high set
  CMD_W_AL_LIM_HC     = 0x6116, // write alert limits, high clear
  CMD_W_AL_LIM_LC     = 0x610B, // write alert limits, low clear
  CMD_W_AL_LIM_LS     = 0x6100, // write alert limits, low set
  CMD_NO_SLEEP        = 0x303E,
}etCommands;

// Measurement Repeatability
typedef enum{
  REPEATAB_HIGH,   // high repeatability
  REPEATAB_MEDIUM, // medium repeatability
  REPEATAB_LOW,    // low repeatability
}etRepeatability;

// Measurement Mode
typedef enum{
  MODE_CLKSTRETCH, // clock stretching
  MODE_POLLING,    // polling
}etMode;

typedef enum{
  FREQUENCY_HZ5,  //  0.5 measurements per seconds
  FREQUENCY_1HZ,  //  1.0 measurements per seconds
  FREQUENCY_2HZ,  //  2.0 measurements per seconds
  FREQUENCY_4HZ,  //  4.0 measurements per seconds
  FREQUENCY_10HZ, // 10.0 measurements per seconds
}etFrequency;

//-- Typedefs -----------------------------------------------------------------
// Status-Register
typedef union {
  u16t u16;
  struct{
    #ifdef LITTLE_ENDIAN  // bit-order is little endian
    u16t CrcStatus     : 1; // write data checksum status
    u16t CmdStatus     : 1; // command status
    u16t Reserve0      : 2; // reserved
    u16t ResetDetected : 1; // system reset detected
    u16t Reserve1      : 5; // reserved
    u16t T_Alert       : 1; // temperature tracking alert
    u16t RH_Alert      : 1; // humidity tracking alert
    u16t Reserve2      : 1; // reserved
    u16t HeaterStatus  : 1; // heater status
    u16t Reserve3      : 1; // reserved
    u16t AlertPending  : 1; // alert pending status 
    #else                 // bit-order is big endian
    u16t AlertPending  : 1;
    u16t Reserve3      : 1;
    u16t HeaterStatus  : 1;
    u16t Reserve2      : 1;
    u16t RH_Alert      : 1;
    u16t T_Alert       : 1;
    u16t Reserve1      : 5;
    u16t ResetDetected : 1;
    u16t Reserve0      : 2;
    u16t CmdStatus     : 1;
    u16t CrcStatus     : 1;
    #endif
  }bit;
} regStatus;

//=============================================================================
// Initializes the I2C bus for communication with the sensor.
//-----------------------------------------------------------------------------
// input: i2cAddress    I2C address, 0x44 ADDR pin low / 0x45 ADDR pin high
//-----------------------------------------------------------------------------
void SHT3X_Init(u8t i2cAddress);


//=============================================================================
// Sets the I2C address.
//-----------------------------------------------------------------------------
// input: i2cAddress    I2C address, 0x44 ADDR pin low / 0x45 ADDR pin high
//-----------------------------------------------------------------------------
void SHT3X_SetI2cAdr(u8t i2cAddress);


//=============================================================================
// Reads the serial number from sensor.
//-----------------------------------------------------------------------------
// input: serialNumber  pointer to serialNumber
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3x_ReadSerialNumber(u32t* serialNumber);


//=============================================================================
// Reads the status register from the sensor.
//-----------------------------------------------------------------------------
// input: status        pointer to status
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_ReadStatus(u16t* status);


//=============================================================================
// Clears all alert flags in status register from sensor.
//-----------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_ClearAllAlertFlags(void);


//=============================================================================
// Gets the temperature [°C] and the relative humidity [%RH] from the sensor.
//-----------------------------------------------------------------------------
// input: temperature   pointer to temperature
//        humiditiy     pointer to humidity
//        repeatability repeatability for the measurement [low, medium, high]
//        mode          command mode [clock stretching, polling]
//        timeout       timeout in milliseconds
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumi(ft* temperature, ft* humiditiy,
                             etRepeatability repeatability, etMode mode,
                             u8t timeout);


//=============================================================================
// Gets the temperature [°C] and the relative humidity [%RH] from the sensor.
// This function uses the i2c clock stretching for waiting until measurement is
// ready.
//-----------------------------------------------------------------------------
// input: temperature   pointer to temperature
//        humiditiy     pointer to humidity
//        repeatability repeatability for the measurement [low, medium, high]
//        timeout       clock stretching timeout in milliseconds
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiClkStretch(ft* temperature, ft* humiditiy,
                                       etRepeatability repeatability,
                                       u8t timeout);


//=============================================================================
// Gets the temperature [°C] and the relative humidity [%RH] from the sensor.
// This function polls every 1ms until measurement is ready.
//-----------------------------------------------------------------------------
// input: temperature   pointer to temperature
//        humiditiy     pointer to humidity
//        repeatability repeatability for the measurement [low, medium, high]
//        timeout       polling timeout in milliseconds
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiPolling(ft* temperature, ft* humiditiy,
                                    etRepeatability repeatability,
                                    u8t timeout);


//=============================================================================
// Starts periodic measurement.
//-----------------------------------------------------------------------------
// input: repeatability repeatability for the measurement [low, medium, high]
//        frequency     measurement frequency [0.5, 1, 2, 4, 10] Hz
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      PARM_ERROR     = parameter out of range
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_StartPeriodicMeasurment(etRepeatability repeatability,
                                      etFrequency frequency);


//=============================================================================
// Reads last measurement from the sensor buffer
//-----------------------------------------------------------------------------
// input: temperature   pointer to temperature
//        humidity      pointer to humidity
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_ReadMeasurementBuffer(ft* temperature, ft* humidity);


//=============================================================================
// Enables the heater on sensor
//-----------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_EnableHeater(void);


//=============================================================================
// Disables the heater on sensor
//-----------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_DisableHeater(void);

//=============================================================================
// 
//-----------------------------------------------------------------------------
etError SHT3X_SetAlertLimits(ft humidityHighSet,   ft temperatureHighSet,
                             ft humidityHighClear, ft temperatureHighClear,
                             ft humidityLowClear,  ft temperatureLowClear,
                             ft humidityLowSet,    ft temperatureLowSet);

//=============================================================================
// 
//-----------------------------------------------------------------------------
etError SHT3X_GetAlertLimits(ft* humidityHighSet,   ft* temperatureHighSet,
                             ft* humidityHighClear, ft* temperatureHighClear,
                             ft* humidityLowClear,  ft* temperatureLowClear,
                             ft* humidityLowSet,    ft* temperatureLowSet);

//=============================================================================
// Returns the state of the Alert-Pin.
//-----------------------------------------------------------------------------
// return:              true:  Alert-Pin is high
//                      false: Alter-Pin is low
//-----------------------------------------------------------------------------
bt SHT3X_ReadAlert(void);

//=============================================================================
// Calls the soft reset mechanism that forces the sensor into a well-defined
// state without removing the power supply.
//-----------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      TIMEOUT_ERROR  = timeout
//                      NO_ERROR       = no error
//-----------------------------------------------------------------------------
etError SHT3X_SoftReset(void);


//=============================================================================
// Resets the sensor by pulling down the reset pin.
//-----------------------------------------------------------------------------
void SHT3X_HardReset(void);
#endif
