/*!
 *  @file Adafruit_AS7341.h

 *  @mainpage Adafruit AS7341 11-Channel Spectral Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the AS7341 11-Channel Spectral Sensor
 *
 * 	This is a library for the Adafruit AS7341 breakout:
 * 	https://www.adafruit.com/product/4698
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  Simone Tollardo - ESP32 C library porting (2022)
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *  Simone Tollardo
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#ifndef __ESP32_AS7341_H__
#define __ESP32_AS7341_H__

#include <stdint.h>
#include "driver/i2c.h"
#include <esp_err.h>

/* NOTE: In SPM or SYNS mode, it is recommended to use the ASTATUS register 0x94
and spectral data register 0x94 to 0xA0. In SYND mode, **it is possible** to use
register 0x60 to 0x6F for easier implementation.
---> Since **it is possible** and not recommended, the library will always
use register 0x94 to 0xA0.
*/

/* NOTE: In order to access registers from 0x60 to 0x74 bit REG_BANK in register
CFG0 (0xA9) needs to be set to “1”. */

#define AS7341_ADDRESS 0x39         ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI

#define AS7341_ITIME_L 0x63    ///< AS7341_ITIME_L
#define AS7341_ITIME_M 0x64    ///< AS7341_ITIME_M
#define AS7341_ITIME_H 0x65    ///< AS7341_ITIME_H
#define AS7341_CONFIG 0x70 ///< Enables LED control and sets light sensing mode
#define AS7341_STAT 0x71   ///< AS7341_STAT
#define AS7341_EDGE 0x72   ///< AS7341_EDGE
#define AS7341_GPIO 0x73   ///< Connects photo diode to GPIO or INT pins
#define AS7341_LED 0x74    ///< LED Register; Enables and sets current limit
#define AS7341_ENABLE                                                          \
  0x80 ///< Main enable register. Controls SMUX, Flicker Detection, Spectral
       ///< Measurements and Power
#define AS7341_ATIME 0x81       ///< Sets ADC integration step count
#define AS7341_WTIME 0x83       ///< AS7341_WTIME
#define AS7341_SP_LOW_TH_L 0x84 ///< Spectral measurement Low Threshold low byte
#define AS7341_SP_LOW_TH_H                                                     \
  0x85 ///< Spectral measurement Low Threshold high byte
#define AS7341_SP_HIGH_TH_L                                                    \
  0x86 ///< Spectral measurement High Threshold low byte
#define AS7341_SP_HIGH_TH_H                                                    \
  0x87                    ///< Spectral measurement High Threshold low byte
#define AS7341_AUXID 0x90 ///< AS7341_AUXID
#define AS7341_REVID 0x91 ///< AS7341_REVID
#define AS7341_ID 0x92    ///< AS7341_ID
#define AS7341_STATUS                                                          \
  0x93 ///< Interrupt status registers. Indicates the occourance of an interrupt
#define AS7341_ASTATUS 0x94    ///< AS7341_ASTATUS
#define AS7341_CH0_DATA_L 0x95 ///< ADC Channel Data
#define AS7341_CH0_DATA_H 0x96 ///< ADC Channel Data
#define AS7341_CH1_DATA_L 0x97 ///< ADC Channel Data
#define AS7341_CH1_DATA_H 0x98 ///< ADC Channel Data
#define AS7341_CH2_DATA_L 0x99 ///< ADC Channel Data
#define AS7341_CH2_DATA_H 0x9A ///< ADC Channel Data
#define AS7341_CH3_DATA_L 0x9B ///< ADC Channel Data
#define AS7341_CH3_DATA_H 0x9C ///< ADC Channel Data
#define AS7341_CH4_DATA_L 0x9D ///< ADC Channel Data
#define AS7341_CH4_DATA_H 0x9E ///< ADC Channel Data
#define AS7341_CH5_DATA_L 0x9F ///< ADC Channel Data
#define AS7341_CH5_DATA_H 0xA0 ///< ADC Channel Data
#define AS7341_STATUS2 0xA3 ///< Measurement status flags; saturation, validity
#define AS7341_STATUS3                                                         \
  0xA4 ///< Spectral interrupt source, high or low threshold
#define AS7341_STATUS5 0xA6 ///< AS7341_STATUS5
#define AS7341_STATUS6 0xA7 ///< AS7341_STATUS6
#define AS7341_CFG0                                                            \
  0xA9 ///< Sets Low power mode, Register bank, and Trigger lengthening
#define AS7341_CFG1 0xAA ///< Controls ADC Gain
#define AS7341_CFG3 0xAC ///< AS7341_CFG3 (unused)
#define AS7341_CFG6 0xAF ///< Used to configure Smux
#define AS7341_CFG8 0xB1 ///< AS7341_CFG8 (unused)
#define AS7341_CFG9                                                            \
  0xB2 ///< Enables flicker detection and smux command completion system
       ///< interrupts
#define AS7341_CFG10 0xB3 ///< AS7341_CFG10 (unused)
#define AS7341_CFG12                                                           \
  0xB5 ///< Spectral threshold channel for interrupts, persistence and auto-gain
#define AS7341_PERS                                                            \
  0xBD ///< Number of measurement cycles outside thresholds to trigger an
       ///< interupt
#define AS7341_GPIO2                                                           \
  0xBE ///< GPIO Settings and status: polarity, direction, sets output, reads
       ///< input
#define AS7341_ASTEP_L 0xCA      ///< Integration step size ow byte
#define AS7341_ASTEP_H 0xCB      ///< Integration step size high byte
#define AS7341_AGC_GAIN_MAX 0xCF ///< AS7341_AGC_GAIN_MAX (unused)
#define AS7341_AZ_CONFIG 0xD6    ///< AS7341_AZ_CONFIG (unused)
#define AS7341_FD_TIME1 0xD8 ///< Flicker detection integration time low byte
#define AS7341_FD_TIME2 0xDA ///< Flicker detection gain and high nibble
#define AS7341_FD_CFG0 0xD7  ///< AS7341_FD_CFG0 (unused)
#define AS7341_FD_STATUS                                                       \
  0xDB ///< Flicker detection status; measurement valid, saturation, flicker
       ///< type
#define AS7341_INTENAB 0xF9  ///< Enables individual interrupt types
#define AS7341_CONTROL 0xFA  ///< Auto-zero, fifo clear, clear SAI active
#define AS7341_FIFO_MAP 0xFC ///< AS7341_FIFO_MAP (unused)
#define AS7341_FIFO_LVL 0xFD ///< AS7341_FIFO_LVL (unused)
#define AS7341_FDATA_L 0xFE  ///< AS7341_FDATA_L (unused)
#define AS7341_FDATA_H 0xFF  ///< AS7341_FDATA_H (unused)

#define AS7341_SPECTRAL_INT_HIGH_MSK                                           \
  0b00100000 ///< bitmask to check for a high threshold interrupt
#define AS7341_SPECTRAL_INT_LOW_MSK                                            \
  0b00010000 ///< bitmask to check for a low threshold interrupt

/**
 * @brief Allowable gain multipliers for `setGain`
 *
 */
typedef enum {
  AS7341_GAIN_0_5X,
  AS7341_GAIN_1X,
  AS7341_GAIN_2X,
  AS7341_GAIN_4X,
  AS7341_GAIN_8X,
  AS7341_GAIN_16X,
  AS7341_GAIN_32X,
  AS7341_GAIN_64X,
  AS7341_GAIN_128X,
  AS7341_GAIN_256X,
  AS7341_GAIN_512X,
} as7341_gain_t;

/**
 * @brief Available SMUX configuration commands
 *
 */
typedef enum {
  AS7341_SMUX_CMD_ROM_RESET,  ///< ROM code initialization of SMUX
  AS7341_SMUX_CMD_READ,       ///< Read SMUX configuration to RAM from SMUX chain
  AS7341_SMUX_CMD_WRITE,      ///< Write SMUX configuration from RAM to SMUX chain
} as7341_smux_cmd_t;
/**
 * @brief ADC Channel specifiers for configuration
 *
 */
typedef enum {
  AS7341_ADC_CHANNEL_0,
  AS7341_ADC_CHANNEL_1,
  AS7341_ADC_CHANNEL_2,
  AS7341_ADC_CHANNEL_3,
  AS7341_ADC_CHANNEL_4,
  AS7341_ADC_CHANNEL_5,
} as7341_adc_channel_t;
/**
 * @brief Spectral Channel specifiers for configuration and reading
 *
 */
typedef enum {
  AS7341_CHANNEL_410nm_F1,
  AS7341_CHANNEL_440nm_F2,
  AS7341_CHANNEL_470nm_F3,
  AS7341_CHANNEL_510nm_F4,
  AS7341_CHANNEL_CLEAR_0,
  AS7341_CHANNEL_NIR_0,
  AS7341_CHANNEL_550nm_F5,
  AS7341_CHANNEL_583nm_F6,
  AS7341_CHANNEL_620nm_F7,
  AS7341_CHANNEL_670nm_F8,
  AS7341_CHANNEL_CLEAR,
  AS7341_CHANNEL_NIR,
} as7341_color_channel_t;

/**
 * @brief The number of measurement cycles with spectral data outside of a
 * threshold required to trigger an interrupt
 *
 */
typedef enum {
  AS7341_INT_COUNT_ALL, ///< 0
  AS7341_INT_COUNT_1,   ///< 1
  AS7341_INT_COUNT_2,   ///< 2
  AS7341_INT_COUNT_3,   ///< 3
  AS7341_INT_COUNT_5,   ///< 4
  AS7341_INT_COUNT_10,  ///< 5
  AS7341_INT_COUNT_15,  ///< 6
  AS7341_INT_COUNT_20,  ///< 7
  AS7341_INT_COUNT_25,  ///< 8
  AS7341_INT_COUNT_30,  ///< 9
  AS7341_INT_COUNT_35,  ///< 10
  AS7341_INT_COUNT_40,  ///< 11
  AS7341_INT_COUNT_45,  ///< 12
  AS7341_INT_COUNT_50,  ///< 13
  AS7341_INT_COUNT_55,  ///< 14
  AS7341_INT_COUNT_60,  ///< 15
} as7341_int_cycle_count_t;

/**
 * @brief Pin directions to set how the GPIO pin is to be used
 *
 */
typedef enum {
  AS7341_GPIO_OUTPUT, ///< THhe GPIO pin is configured as an open drain output
  AS7341_GPIO_INPUT,  ///< The GPIO Pin is set as a high-impedence input
} as7341_gpio_dir_t;

/**
 * @brief Wait states for async reading
 */
typedef enum {
  AS7341_WAITING_START, //
  AS7341_WAITING_LOW,   //
  AS7341_WAITING_HIGH,  //
  AS7341_WAITING_DONE,  //
} as7341_waiting_t;

typedef enum {
  AS7341_MODE_SPM = 0,
  AS7341_MODE_SYNS = 1,
  AS7341_MODE_SYND = 3,
} as7341_mode_t;


/**
 *  @brief  Struct that stores the states of the AS7341 sensor.
 */
typedef struct{
  uint8_t i2c_port;
  bool init;
  bool enabled;       //AS enabled state
  as7341_mode_t mode;
  as7341_gain_t gain;
  as7341_int_cycle_count_t APERS;
  uint8_t ATIME;
  uint16_t ASTEP;
  uint16_t HT;                  // high threshold
  uint16_t LT;                  // low threshold
  as7341_adc_channel_t TH_CH;   // threshold channel
  as7341_gpio_dir_t gpio_dir;
  bool gpio_inv;
  bool out_gpio_value;
  bool reg_bank;
  uint16_t data[12];
} ESP32_AS7341;

/* Data array structure:
 * data[0] = 410nm
 * data[1] = 440nm
 * data[2] = 470nm
 * data[3] = 510nm
 * data[4] = Clear
 * data[5] = NIR
 * data[6] = 550nm
 * data[7] = 583nm
 * data[8] = 620nm
 * data[9] = 670nm
 * data[10] = Clear
 * data[11] = NIR
 */

esp_err_t AS_init(ESP32_AS7341 *AS, uint8_t i2c_port);
void AS_enable(ESP32_AS7341 *AS);
void AS_disableAll(ESP32_AS7341 *AS);
void AS_enableSpectralMeasurement(ESP32_AS7341 *AS, bool enable_measurement);

void AS_setMode(ESP32_AS7341 *AS, as7341_mode_t mode);
void AS_setASTEP(ESP32_AS7341 *AS, uint16_t ASTEP);
void AS_setATIME(ESP32_AS7341 *AS, uint8_t ATIME);
void AS_setGain(ESP32_AS7341 *AS, as7341_gain_t gain);
void AS_setAPERS(ESP32_AS7341 *AS, as7341_int_cycle_count_t APERS);
void AS_setGPIO(ESP32_AS7341 *AS, as7341_gpio_dir_t dir, bool out_gpio_inverted, bool gpio_value);
void AS_setBank(ESP32_AS7341 *AS, bool sel); // low true gives access to 0x60 to 0x74

long AS_getTINT(ESP32_AS7341 *AS);
long AS_getADC_FS(ESP32_AS7341 *AS);
float AS_toBasicCounts(ESP32_AS7341 *AS, uint16_t raw);

void AS_setHighThreshold(ESP32_AS7341 *AS, uint16_t threshold);
void AS_setLowThreshold(ESP32_AS7341 *AS, uint16_t threshold);
void AS_setThresholdChannel(ESP32_AS7341 *AS, as7341_adc_channel_t channel);
// bool AS_enableSpectralInterrupt(bool enable_int);
// bool AS_enableSystemInterrupt(bool enable_int);
// bool AS_clearInterruptStatus(void);
// bool AS_spectralInterruptTriggered(void);
// uint8_t AS_spectralInterruptSource(void);
// bool AS_spectralLowTriggered(void);
// bool AS_spectralHighTriggered(void);

void AS_read_ADC_Channel(ESP32_AS7341 *AS,as7341_adc_channel_t channel);
void AS_readAllChannels(ESP32_AS7341 *AS);
uint16_t AS_getChannel(ESP32_AS7341 *AS, as7341_color_channel_t channel);
void AS_getAllChannels(ESP32_AS7341 *AS, uint16_t data[12]);
void AS_delayForData(ESP32_AS7341 *AS, int waitTime);
bool AS_getIsDataReady(ESP32_AS7341 *AS);

// uint16_t AS_detectFlickerHz(void);

#endif
