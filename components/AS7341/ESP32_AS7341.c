/*!
 *  @file ESP32_AS7341.c
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
 *  Copyright 2020 Bryan Siepert for Adafruit Industries
 *  Copyright 2022 Simone Tollardo
 *
 * 	BSD (see license.txt)
 */

/* TODO: Check if Gain register AS7341_CFG1 change values when AGC enabled
or AGC changes only ASTATUS register AS7341_ASTATUS
*/


#include "ESP32_AS7341.h"
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

static const char* TAG="ESP32_AS7341";

// ---------------- I2C AUXILIARY FUNCTIONS ----------------

/**
 *  @brief  write an 8 bit value over I2C
 *  @param  reg
 *  @param  value
 */
static void write8(uint8_t reg, uint8_t value) {

    uint8_t buffer[2] = {reg, value};
    ESP_ERROR_CHECK(i2c_master_write_to_device(AS_I2C_PORT, AS7341_ADDRESS, buffer, 2, 1000 / portTICK_PERIOD_MS));
}

/**
 *  @brief  write a 16 bit value over I2C
 *  @param  reg
 *  @param  value
 */
static void write16(uint8_t reg, uint16_t value) {

    uint8_t buffer[3] = {reg, (uint8_t)(value), (uint8_t)(value >> 8)};
    ESP_ERROR_CHECK(i2c_master_write_to_device(AS_I2C_PORT, AS7341_ADDRESS, buffer, 3, 1000 / portTICK_PERIOD_MS));
}

/**
 *  @brief  Reads an 8 bit value over I2C
 *  @param  reg
 *  @return value
 */
static uint8_t read8(uint8_t reg) {
    uint8_t buffer[1] = {reg};
    ESP_ERROR_CHECK(i2c_master_write_to_device(AS_I2C_PORT, AS7341_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_read_from_device(AS_I2C_PORT, AS7341_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS));
    return buffer[0];
}

/**
 *  @brief  Reads a 16 bit value over I2C
 *  @param  reg
 *  @return value
 */
static uint16_t read16(uint8_t reg) {
    uint8_t buffer[2] = {reg, 0};
    ESP_ERROR_CHECK(i2c_master_write_to_device(AS_I2C_PORT, AS7341_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_read_from_device(AS_I2C_PORT, AS7341_ADDRESS, buffer, 2, 1000 / portTICK_PERIOD_MS));
    return buffer[0] | (buffer[1] << 8);
}

/**
 *  @brief  Reads <SIZE> bytes over I2C
 *  @param  reg
 *  @param  data pointer to the data buffer
 *  @param  size number of bytes to read
 *  @return value
 */
static void read_data(uint8_t reg, uint8_t *data, size_t size) {
    uint8_t buffer[1] = {reg};
    ESP_ERROR_CHECK(i2c_master_write_to_device(AS_I2C_PORT, AS7341_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_read_from_device(AS_I2C_PORT, AS7341_ADDRESS, data, size, 1000 / portTICK_PERIOD_MS));
}

/**
 * @brief Write bits in a register
 *
 * @param reg register to write to
 * @param mask mask to apply
 * @param value value to write
 */
static void writeBits(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t tmp = read8(reg);
    tmp &= ~mask;
    tmp |= (value & mask);
    write8(reg, tmp);
}

// ---------------- INITIALIZATION FUNCTIONS ----------------

/**
 *  @brief  Initializes the AS7341 sensor
 *  @param  AS Pointer to the AS7341 sensor structure
 *  @return void
 */
void AS_init(ESP32_AS7341 *AS) {
    if(AS->init!=true){
        //Load default values
        AS->mode = AS7341_MODE_SPM;
        AS->ATIME = 29;
        AS->ASTEP = 599;
        AS->gain = AS7341_GAIN_256X;
        AS->APERS = AS7341_INT_COUNT_ALL;
        AS->LT = 0;
        AS->HT = 0;
        AS->TH_CH = AS7341_ADC_CHANNEL_0;
        AS->gpio_dir = AS7341_GPIO_OUTPUT;
        AS->gpio_inv = false;
        AS->out_gpio_value = 1;
        AS->reg_bank = 0;

        //Set I2C configuration
        AS->conf.mode = I2C_MODE_MASTER;
        AS->conf.sda_io_num = AS_SDA_PIN;
        AS->conf.scl_io_num = AS_SCL_PIN;
        AS->conf.sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
        AS->conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
        AS->conf.master.clk_speed = 400000;               //I2C Full Speed

        ESP_ERROR_CHECK(i2c_param_config(AS_I2C_PORT, &(AS->conf))); //set I2C Config

        ESP_ERROR_CHECK(i2c_driver_install(AS_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

        /* Make sure we're actually connected */
        uint8_t ID = read8(AS7341_ID) >> 2;
        if (ID != AS7341_CHIP_ID) {
            ESP_LOGE(TAG, "AS7341 not found, ID 0x%02x != 0x09", ID);
        }
        AS->init = true;

        /* Set default values */
        AS_setMode(AS, AS->mode);
        AS_setATIME(AS, AS->ATIME);
        AS_setASTEP(AS, AS->ASTEP);
        AS_setGain(AS, AS->gain);
        AS_setAPERS(AS, AS->APERS);
        AS_setLowThreshold(AS, AS->LT);
        AS_setHighThreshold(AS, AS->HT);
        AS_setThresholdChannel(AS, AS->TH_CH);
        AS_setGPIO(AS, AS->gpio_dir, AS->gpio_inv, AS->out_gpio_value);
        AS_setBank(AS, AS->reg_bank);

        AS_enable(AS);
    }
    else{
        ESP_LOGE(TAG, "AS7341 already initialized");
    }
}

/**
 *  @brief  Delete the I2C driver for the AS7341
 */
void AS_delete(){
    ESP_ERROR_CHECK(i2c_driver_delete(AS_I2C_PORT));
}

/**
 *  @brief  Power On the device
 *  @param  AS Pointer to the AS7341 structure
 *  @return void
 */
void AS_enable(ESP32_AS7341 *AS){
    writeBits(AS7341_ENABLE, 0x01, 0x01);   //PON=1
    AS->enabled=true;
}

/**
 * @brief Disable Spectral reading, flicker detection, and power
 *
 * */
void AS_disableAll(ESP32_AS7341 *AS) {
    writeBits(AS7341_ENABLE, 0x5b, 0x00);  //PON=0, SP_EN=0, W_EN=0, SMUX_EN=0, FD_EN=0
    AS->enabled=0;
}

/**
 * @brief Enables measurement of spectral data
 *
 * @param enable_measurement true: enabled false: disabled
 * @return void
 */
void AS_enableSpectralMeasurement(bool enable_measurement) {
    writeBits(AS7341_ENABLE, 0x02, (uint8_t)enable_measurement << 1); //SP_EN=enable_measurement
}

// ---------------- CONFIGURATION FUNCTIONS ----------------

/**
 *  @brief  Set device mode: SPM, SYNS, SYND
 *  @param  AS Pointer to the AS7341 structure
 *  @param  mode
 *  @return void
 */
void AS_setMode(ESP32_AS7341 *AS, as7341_mode_t mode){
    writeBits(AS7341_CONFIG, 0x03, (uint8_t)mode);
    AS->mode = mode;
}

/**
 * @brief Sets the integration time step count
 * @param AS Pointer to the AS7341 structure
 * @param ATIME Integration time step count
 * @return void
 */
void AS_setATIME(ESP32_AS7341 *AS, uint8_t ATIME){
    write8(AS7341_ATIME, ATIME);
    AS->ATIME = ATIME;
}

/**
 * @brief Sets the integration time step size
 * @param AS Pointer to the AS7341 structure
 * @param ASTEP Integration time step size
 * @return void
 */
void AS_setASTEP(ESP32_AS7341 *AS, uint16_t ASTEP){
    write16(AS7341_ASTEP_L, ASTEP);
    AS->ASTEP = ASTEP;
}

/**
 * @brief Sets the ADC gain multiplier
 * @param AS Pointer to the AS7341 structure
 * @param gain ADC gain multiplier
 * @return void
 */
void AS_setGain(ESP32_AS7341 *AS, as7341_gain_t gain){
    writeBits(AS7341_CFG1, 0x1F, (uint8_t)gain);
    AS->gain = gain;
}

// Spectral Interrupt Persistence.
// Defines a filter for the number of consecutive
// occurrences that spectral data must remain outside
// the threshold range between SP_TH_L and
// SP_TH_H before an interrupt is generated. The
// spectral data channel used for the persistence filter
// is set by SP_TH_CHANNEL. Any sample that is
// inside the threshold range resets the counter to 0.
/**
 * @brief Sets the number of times an interrupt threshold must be exceeded
 * before an interrupt is triggered
 * @param AS Pointer to the AS7341 structure
 * @param cycle_count The number of cycles to trigger an interrupt
 * @return void
 */
void AS_setAPERS(ESP32_AS7341 *AS, as7341_int_cycle_count_t APERS){
    writeBits(AS7341_PERS, 0x0F, (uint8_t)APERS);
    AS->APERS = APERS;
}

/**
 * @brief Sets the threshold below which spectral measurements will trigger
 * interrupts when the APERS count is reached
 * @param AS Pointer to the AS7341 structure
 * @param LT The low threshold
 * @return void
 */
void AS_setLowThreshold(ESP32_AS7341 *AS, uint16_t threshold){
    write16(AS7341_SP_LOW_TH_L, threshold);
    AS->LT = threshold;
}

/**
 * @brief Sets the threshold above which spectral measurements will trigger
 * interrupts when the APERS count is reached
 * @param AS Pointer to the AS7341 structure
 * @param HT The high threshold
 * @return void
 */
void AS_setHighThreshold(ESP32_AS7341 *AS, uint16_t threshold){
    write16(AS7341_SP_HIGH_TH_L, threshold);
    AS->HT = threshold;
}

/**
 * @brief Set the ADC channel to use for spectral thresholds including
 * interrupts, automatic gain control, and persistance settings
 * @param AS Pointer to the AS7341 structure
 * @param channel The channel to use for spectral thresholds. Must be a
 * as7341_adc_channel_t **except for** `AS7341_ADC_CHANNEL_5`
 * @return void
 */
void AS_setThresholdChannel(ESP32_AS7341 *AS, as7341_adc_channel_t channel){
    if (channel == AS7341_ADC_CHANNEL_5) {
    ESP_LOGE(TAG, "AS7341_ADC_CHANNEL_5 is not a valid channel for spectral thresholds");
    }
    writeBits(AS7341_CFG12, 0x07, (uint8_t)channel);
}

/**
 * @brief Set the GPIO : direction, Polarity and value.
 *
 * @param dir: the GPIO direction.
 * @param out_gpio_inverted: the GPIO polarity (inverts the logic of the GPIO)
 * @param val: the GPIO output value (if GPIO is set to input it will be written
 * but ignored until the next write to the GPIO direction register)
 *
 * @return void
 */
void AS_setGPIO(ESP32_AS7341 *AS, as7341_gpio_dir_t dir, bool gpio_inverted, bool out_gpio_value){

    uint8_t buffer;
    buffer = (uint8_t)(out_gpio_value << 1) | (uint8_t)(dir <<2) |  (uint8_t)(gpio_inverted << 3);
    writeBits(AS7341_GPIO2, 0x0F, buffer);

    AS->gpio_dir = dir;
    AS->gpio_inv = gpio_inverted;
    AS->out_gpio_value = out_gpio_value;
}

/**
 * @brief Sets the active register bank
 *
 * The AS7341 uses banks to organize the register making it nescessary to set
 * the correct bank to access a register.
 *
 * @param sel
 * true: Register access to register 0x60 to 0x74
 * false: Register access to register 0x80 and above
 *
 * @return void
 */
void AS_setBank(ESP32_AS7341 *AS, bool sel) {
    writeBits(AS7341_CFG0, 0x10, (uint8_t)sel << 4);
    AS->reg_bank=sel;
}

// ---------------- SENSOR READING AUXILIARY FUNCTIONS ----------------

/**
 * @brief Configure SMUX for sensors F1-4, Clear and NIR (6 ADC Channels available)
 * See AS7341_EvalSW_Reflection_v1-26-3.zip for explanations
 */
static void setup_F1F4_Clear_NIR() {
    // SMUX Config for F1,F2,F3,F4,NIR,Clear
    write8(0x00, 0x30); // F3 left set to ADC2
    write8(0x01, 0x01); // F1 left set to ADC0
    write8(0x02, 0x00); // Reserved or disabled
    write8(0x03, 0x00); // F8 left disabled
    write8(0x04, 0x00); // F6 left disabled
    write8(0x05, 0x42); // F4 left connected to ADC3/f2 left connected to ADC1
    write8(0x06, 0x00); // F5 left disbled
    write8(0x07, 0x00); // F7 left disbled
    write8(0x08, 0x50); // CLEAR connected to ADC4
    write8(0x09, 0x00); // F5 right disabled
    write8(0x0A, 0x00); // F7 right disabled
    write8(0x0B, 0x00); // Reserved or disabled
    write8(0x0C, 0x20); // F2 right connected to ADC1
    write8(0x0D, 0x04); // F4 right connected to ADC3
    write8(0x0E, 0x00); // F6/F8 right disabled
    write8(0x0F, 0x30); // F3 right connected to AD2
    write8(0x10, 0x01); // F1 right connected to AD0
    write8(0x11, 0x50); // CLEAR right connected to AD4
    write8(0x12, 0x00); // Reserved or disabled
    write8(0x13, 0x06); // NIR connected to ADC5
}

/**
 * @brief Configure SMUX for sensors F5-8, Clear and NIR (6 ADC Channels available)
 * See AS7341_EvalSW_Reflection_v1-26-3.zip for explanations
 */
static void setup_F5F8_Clear_NIR() {
    // SMUX Config for F5,F6,F7,F8,NIR,Clear
    write8(0x00, 0x00); // F3 left disable
    write8(0x01, 0x00); // F1 left disable
    write8(0x02, 0x00); // reserved/disable
    write8(0x03, 0x40); // F8 left connected to ADC3
    write8(0x04, 0x02); // F6 left connected to ADC1
    write8(0x05, 0x00); // F4/ F2 disabled
    write8(0x06, 0x10); // F5 left connected to ADC0
    write8(0x07, 0x03); // F7 left connected to ADC2
    write8(0x08, 0x50); // CLEAR Connected to ADC4
    write8(0x09, 0x10); // F5 right connected to ADC0
    write8(0x0A, 0x03); // F7 right connected to ADC2
    write8(0x0B, 0x00); // Reserved or disabled
    write8(0x0C, 0x00); // F2 right disabled
    write8(0x0D, 0x00); // F4 right disabled
    write8(0x0E, 0x24); // F8 right connected to ADC2/ F6 right connected to ADC1
    write8(0x0F, 0x00); // F3 right disabled
    write8(0x10, 0x00); // F1 right disabled
    write8(0x11, 0x50); // CLEAR right connected to AD4
    write8(0x12, 0x00); // Reserved or disabled
    write8(0x13, 0x06); // NIR connected to ADC5
}

/**
 * @brief Configure SMUX for flicker detection
 * See AS7341_EvalSW_Reflection_v1-26-3.zip for explanations
 */
static void FDConfig() {
    // SMUX Config for Flicker- register (0x13)left set to ADC6 for flicker
    // detection
    write8(0x00, 0x00);  // disabled
    write8(0x01, 0x00);  // disabled
    write8(0x02, 0x00);  // reserved/disabled
    write8(0x03, 0x00);  // disabled
    write8(0x04, 0x00);  // disabled
    write8(0x05, 0x00);  // disabled
    write8(0x06, 0x00);  // disabled
    write8(0x07, 0x00);  // disabled
    write8(0x08, 0x00);  // disabled
    write8(0x09, 0x00);  // disabled
    write8(0x0A, 0x00);  // disabled
    write8(0x0B, 0x00);  // Reserved or disabled
    write8(0x0C, 0x00);  // disabled
    write8(0x0D, 0x00);  // disabled
    write8(0x0E, 0x00);  // disabled
    write8(0x0F, 0x00);  // disabled
    write8(0x10, 0x00);  // disabled
    write8(0x11, 0x00);  // disabled
    write8(0x12, 0x00);  // Reserved or disabled
    write8(0x13, 0x60); // Flicker connected to ADC5 to left of 0x13
}

static void setSMUXCommand(as7341_smux_cmd_t command) {
    writeBits(AS7341_CFG6, 0x18, (uint8_t)(command << 3));
}

static void enableSMUX(void) {

    writeBits(AS7341_ENABLE, 0x10, 0x01 << 4);

    // Arbitrary value, but if it takes 1000 milliseconds then something is wrong
    int timeOut = 1000;
    int count = 0;
    // wait for SMUX_EN bit to be reset (after SMUX operation) or timeout
    while (!!((read8(AS7341_ENABLE) & 0x10) >> 4) && count < timeOut) {
        vTaskDelay(1/portTICK_PERIOD_MS);
        count++;
    }
    if (count >= timeOut) {
        ESP_LOGE(TAG, "SMUX enable timeout");
    }
}

static void setSMUXLowChannels(bool lowChan) {
    AS_enableSpectralMeasurement(false);
    setSMUXCommand(AS7341_SMUX_CMD_WRITE);
    if (lowChan) {
        setup_F1F4_Clear_NIR();
    } else {
        setup_F5F8_Clear_NIR();
    }
    enableSMUX();
}

/**
 * @brief Check if spectral measurement is completed
 *
 * @return true: success false: failure
 */
bool AS_getIsDataReady() {
    //Check if AVALID bit is set
    return !!((read8(AS7341_STATUS2) & 0x40) >> 6);
}

/**
 * @brief Delay while waiting for data, with option to time out and recover
 *
 * @param waitTime the maximum amount of time to wait
 * @return void
 */
void AS_delayForData(int waitTime) {
    if (waitTime == 0) // Wait forever
    {
        while (!AS_getIsDataReady()) {
        vTaskDelay(1/portTICK_PERIOD_MS);
        }
        return;
    }
    if (waitTime > 0) // Wait for that many milliseconds
    {
        uint32_t elapsedMillis = 0;
        while (!AS_getIsDataReady() && elapsedMillis < waitTime) {
        vTaskDelay(1/portTICK_PERIOD_MS);
        elapsedMillis++;
        }
        ESP_LOGE(TAG, "Data not ready after %d milliseconds", elapsedMillis);
    }
    if (waitTime < 0) {
        // For future use?
        return;
    }
}

// ---------------- SENSOR READING FUNCTIONS ----------------

/**
 * @brief Update the ADC data in the AS structure for a given **ADC channel**
 * @param AS Pointer to the AS7341 structure
 * @param channel The ADC channel to read
 * @return void
 */
void AS_read_ADC_Channel(ESP32_AS7341 *AS, as7341_adc_channel_t channel) {
    // each channel has two bytes, so offset by two for each next channel
    AS->data[channel]=read16(AS7341_CH0_DATA_L + 2 * channel);
}

/**
 * @brief Update the ADC data in the AS structure for all **Spectral
 * channels F1-8, Clear and NIR**, not ADC Channels!
 * @param AS Pointer to the AS7341 structure
 * @return void
 */
void AS_readAllChannels(ESP32_AS7341 *AS) {

    setSMUXLowChannels(true);             // Configure SMUX to read low channels
    AS_enableSpectralMeasurement(true);   // Start integration
    AS_delayForData(0);                   // I'll wait for you for all time

    uint8_t readings_buffer[12];
    read_data(AS7341_CH0_DATA_L, readings_buffer, 12);  // Read all 12 bytes
    // Convert the readings to uint16_t and store in the AS structure
    AS->data[0] = readings_buffer[0] | readings_buffer[1] << 8;
    AS->data[1] = readings_buffer[2] | readings_buffer[3] << 8;
    AS->data[2] = readings_buffer[4] | readings_buffer[5] << 8;
    AS->data[3] = readings_buffer[6] | readings_buffer[7] << 8;
    AS->data[4] = readings_buffer[8] | readings_buffer[9] << 8;
    AS->data[5] = readings_buffer[10] | readings_buffer[11] << 8;

    setSMUXLowChannels(false);            // Configure SMUX to read high channels
    AS_enableSpectralMeasurement(true);   // Start integration
    AS_delayForData(0);                   // I'll wait for you for all time

    read_data(AS7341_CH0_DATA_L, readings_buffer, 12);
    // Convert the readings to uint16_t and store in the AS structure
    AS->data[6] = readings_buffer[0] | readings_buffer[1] << 8;
    AS->data[7] = readings_buffer[2] | readings_buffer[3] << 8;
    AS->data[8] = readings_buffer[4] | readings_buffer[5] << 8;
    AS->data[9] = readings_buffer[6] | readings_buffer[7] << 8;
    AS->data[10] = readings_buffer[8] | readings_buffer[9] << 8;
    AS->data[11] = readings_buffer[10] | readings_buffer[11] << 8;
}

/**
 * @brief Returns the reading data for the specified color channel taken
 * from the AS Strcture
 *
 *  call `readAllChannels` before reading to update the stored readings
 *
 * @param AS Pointer to the AS7341 structure
 * @param channel The color sensor channel to read
 * @return uint16_t The measured data for the selected sensor channel
 */
uint16_t AS_getChannel(ESP32_AS7341 *AS, as7341_color_channel_t channel) {
    return AS->data[channel];
}

// ---------------- NOT IMPLEMENTED FUNCTIONS ----------------

// // maybe return a typedef enum
// /**
//  * @brief Returns the flicker detection status
//  *
//  * @return int8_t
//  */
// int8_t getFlickerDetectStatus(void) {
//   Adafruit_BusIO_Register flicker_val =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_FD_STATUS);
//   return (int8_t)flicker_val.read();
// }

// bool enableFlickerDetection(bool enable_fd) {

//   Adafruit_BusIO_Register enable_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
//   Adafruit_BusIO_RegisterBits fd_enable_bit =
//       Adafruit_BusIO_RegisterBits(&enable_reg, 1, 6);
//   return fd_enable_bit.write(enable_fd);
// }

// /**
//  * @brief Enable control of an attached LED on the LDR pin
//  *
//  * @param enable_led true: LED enabled false: LED disabled
//  * @return true: success false: failure
//  */
// bool enableLED(bool enable_led) {
//   Adafruit_BusIO_Register config_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_CONFIG);
//   // Enables control of the LED via the LDR pin
//   // 1=control enabled 0 = control disabled
//   Adafruit_BusIO_RegisterBits led_sel_bit =
//       Adafruit_BusIO_RegisterBits(&config_reg, 1, 3);

//   Adafruit_BusIO_Register led_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_LED);
//   // turns the LED on or off
//   Adafruit_BusIO_RegisterBits led_act_bit =
//       Adafruit_BusIO_RegisterBits(&led_reg, 1, 7);

//   setBank(true); // Access 0x60-0x74
//   bool result = led_sel_bit.write(enable_led) && led_act_bit.write(enable_led);
//   setBank(false); // Access registers 0x80 and above (default)
//   return result;
// }

// /**
//  * @brief Set the current limit for the LED
//  *
//  * @param led_current_ma the value to set in milliamps. With a minimum of 4. Any
//  * amount under 4 will be rounded up to 4
//  *
//  * Range is 4mA to 258mA
//  * @return true: success false: failure
//  */
// bool setLEDCurrent(uint16_t led_current_ma) {
//   // check within permissible range
//   if (led_current_ma > 258) {
//     return false;
//   }
//   if (led_current_ma < 4) {
//     led_current_ma = 4;
//   }
//   setBank(true); // Access 0x60 0x74

//   Adafruit_BusIO_Register led_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_LED);

//   // true = led on , false = off
//   Adafruit_BusIO_RegisterBits led_current_bits =
//       Adafruit_BusIO_RegisterBits(&led_reg, 7, 0);

//   bool result = led_current_bits.write((uint8_t)((led_current_ma - 4) / 2));
//   setBank(false); // Access registers 0x80 and above (default)
//   return result;
// }

// /**
//  * @brief Enable Interrupts based on spectral measurements
//  *
//  * @param enable_int true: enable false: disable
//  * @return true: success false: falure
//  */
// bool enableSpectralInterrupt(bool enable_int) {
//   Adafruit_BusIO_Register int_enable_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_INTENAB);
//   Adafruit_BusIO_RegisterBits sp_int_bit =
//       Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 3);
//   return sp_int_bit.write(enable_int);
// }

// /**
//  * @brief Enabled system interrupts
//  *
//  * @param enable_int Set to true to enable system interrupts
//  * @return true: success false: failure
//  */
// bool enableSystemInterrupt(bool enable_int) {
//   Adafruit_BusIO_Register int_enable_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_INTENAB);
//   Adafruit_BusIO_RegisterBits sien_int_bit =
//       Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 0);
//   return sien_int_bit.write(enable_int);
// }


// /**
//  * @brief Returns the current value of the Interupt status register
//  *
//  * @return uint8_t
//  */
// uint8_t getInterruptStatus(void) {
//   Adafruit_BusIO_Register int_status_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);
//   return (uint8_t)int_status_reg.read();
// }

// /**
//  * @brief Returns the status of the spectral measurement threshold interrupts
//  *
//  * @return true: interrupt triggered false: interrupt not triggered
//  */
// bool spectralInterruptTriggered(void) {
//   Adafruit_BusIO_Register int_status_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);
//   Adafruit_BusIO_RegisterBits aint_bit =
//       Adafruit_BusIO_RegisterBits(&int_status_reg, 1, 3);

//   return aint_bit.read();
// }

// /**
//  * @brief Clear the interrupt status register
//  *
//  * @return true: success false: failure
//  */
// bool clearInterruptStatus(void) {
//   Adafruit_BusIO_Register int_status_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);

//   return int_status_reg.write(0xFF);
// }

// /**
//  * @brief The current state of the spectral measurement interrupt status
//  * register
//  *
//  * @return uint8_t The current status register
//  */
// uint8_t spectralInterruptSource(void) {
//   Adafruit_BusIO_Register status3_reg =
//       Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS3);

//   uint8_t spectral_int_source = status3_reg.read();
//   last_spectral_int_source = spectral_int_source;
//   return spectral_int_source;
// }

// /**
//  * @brief The status of the low threshold interrupt
//  *
//  * @return true: low interrupt triggered false: interrupt not triggered
//  */
// bool spectralLowTriggered(void) {
//   return (last_spectral_int_source & AS7341_SPECTRAL_INT_LOW_MSK > 0);
// }

// /**
//  * @brief The status of the high threshold interrupt
//  *
//  * @return true: high interrupt triggered false: interrupt not triggered
//  */
// bool spectralHighTriggered(void) {
//   return (last_spectral_int_source & AS7341_SPECTRAL_INT_HIGH_MSK > 0);
// }

// /**
//  * @brief Detect a flickering light
//  * @return The frequency of a detected flicker or 1 if a flicker of
//  * unknown frequency is detected
//  */
// uint16_t detectFlickerHz(void) {
//   bool isEnabled = true;
//   bool isFdmeasReady = false;

//   // disable everything; Flicker detect, smux, wait, spectral, power
//   disableAll();
//   // re-enable power
//   powerEnable(true);

//   // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
//   // to CFG6)
//   setSMUXCommand(AS7341_SMUX_CMD_WRITE);

//   // Write new configuration to all the 20 registers for detecting Flicker
//   FDConfig();

//   // Start SMUX command
//   enableSMUX();

//   // Enable SP_EN bit
//   enableSpectralMeasurement(true);

//   // Enable flicker detection bit
//   writeRegister(byte(AS7341_ENABLE), byte(0x41));
//   delay(500); // SF 2020-08-12 Does this really need to be so long?
//   uint16_t flicker_status = getFlickerDetectStatus();
//   enableFlickerDetection(false);
//   switch (flicker_status) {
//   case 44:
//     return 1;
//   case 45:
//     return 100;
//   case 46:
//     return 120;
//   default:
//     return 0;
//   }
// }

// ---------------- GET VALUES FUNCTIONS ----------------

/**
 * @brief Reads the ITIME value from AS, used to calculate integration time
 * in SYND mode
 * @return ITIME value
 */
static uint32_t AS_getITIME(void){
    uint8_t buffer[3];
    buffer[0]=read8(AS7341_ITIME_L);
    buffer[1]=read8(AS7341_ITIME_M);
    buffer[2]=read8(AS7341_ITIME_H);
    return (uint32_t)(buffer[2]<<16)|(uint32_t)(buffer[1]<<8)|(uint32_t)buffer[0];
}

/**
 * @brief Returns the integration time
 *
 * The integration time is `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @return long The current integration time in ms
 */
long AS_getTINT(ESP32_AS7341 *AS) {
    if(AS->mode==AS7341_MODE_SPM || AS->mode==AS7341_MODE_SYNS){
        return (AS->ATIME + 1) * (AS->ASTEP + 1) * 2.78 / 1000;
    }
    else {
        return AS_getITIME() * 2.78 / 1000;
    }
}

/**
 * @brief Returns the ADC Full Scale value
 *
 * The ADC Full Scale value is `(ATIME + 1) * (ASTEP + 1)`
 *
 * @return long The current integration time in ms
 */
long AS_getADC_FS(ESP32_AS7341 *AS) {
    if(AS->mode==AS7341_MODE_SPM || AS->mode==AS7341_MODE_SYNS){
        return (AS->ATIME + 1) * (AS->ASTEP + 1);
    }
    else {
        return 0; //if SYND mode, return 0
    }
}

/**
 * @brief Converts raw ADC values to basic counts
 *
 * The basic counts are `RAW/(GAIN * TINT)`
 *
 * @param raw The raw ADC values to convert
 *
 * @return float of the basic counts
 */
float AS_toBasicCounts(ESP32_AS7341 *AS, uint16_t raw) {
    float gain_val = 0;
    as7341_gain_t gain = AS->gain;
    switch (gain) {
    case AS7341_GAIN_0_5X:
        gain_val = 0.5;
        break;
    case AS7341_GAIN_1X:
        gain_val = 1;
        break;
    case AS7341_GAIN_2X:
        gain_val = 2;
        break;
    case AS7341_GAIN_4X:
        gain_val = 4;
        break;
    case AS7341_GAIN_8X:
        gain_val = 8;
        break;
    case AS7341_GAIN_16X:
        gain_val = 16;
        break;
    case AS7341_GAIN_32X:
        gain_val = 32;
        break;
    case AS7341_GAIN_64X:
        gain_val = 64;
        break;
    case AS7341_GAIN_128X:
        gain_val = 128;
        break;
    case AS7341_GAIN_256X:
        gain_val = 256;
        break;
    case AS7341_GAIN_512X:
        gain_val = 512;
        break;
    }
    return raw / (gain_val * (AS->ATIME + 1) * (AS->ASTEP + 1) * 2.78 / 1000);
}

