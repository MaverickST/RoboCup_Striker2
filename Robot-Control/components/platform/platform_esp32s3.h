/**
 * \file        platform_esp32s3.h
 * \brief
 * \details     I2C and ADC HAL for ESP32-S3
 * 
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/12/2024
 * \copyright   Unlicensed
 */

#ifndef __HAL_ESP32S3__
#define __HAL_ESP32S3__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"

// -------------------------------------------------------------
// ---------------------- I2C MASTER ---------------------------
// -------------------------------------------------------------

#include "driver/i2c_master.h"
#define I2C_TIMEOUT_MS 100

static const char* TAG_I2C = "i2c";

/**
 * @brief I2C master driver structure
 * 
 */
typedef struct 
{
    uint8_t addr; // I2C device address
    uint32_t clk_speed_hz;

    i2c_port_t i2c_num; // I2C port number
    uint8_t gpio_scl;
    uint8_t gpio_sda;

    // I2C configuration structure for ESP32 - S3
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t bus_handle;
} i2c_t;

/**
 * @brief Initialize the I2C master driver
 * 
 * @param i2c 
 * @param i2c_num 
 * @param gpio_scl 
 * @param gpio_sda 
 * @param clk_speed_hz 
 * @param addr 
 * @return true if the I2C is initialized correctly
 * @return false if the I2C initialization failed
 */
bool i2c_init(i2c_t *i2c, uint8_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr);

/**
 * @brief Initialize a new I2C bus. 
 * 
 * After calling this function you can add a new device to the bus using the i2c_init_new_device function.
 * 
 * @param i2c 
 * @param i2c_num 
 * @param gpio_scl 
 * @param gpio_sda 
 * @return true 
 * @return false 
 */
bool i2c_init_new_bus(i2c_t *i2c, uint8_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda);

/**
 * @brief Add a new device to the I2C bus. 
 * 
 * The i2c bus must be initialized before calling this function, and the i2c struct must 
 * contain the bus handle. You can use the i2c_get_bus_handle function to get the bus handle.
 * 
 * To use this function, you can one of two ways:
 * 
 * 1. Call the i2c_init_new_bus function to initialize the bus and then call this function to add a new device.
 * 
 * 2. Call the i2c_get_bus_handle function to get the bus handle and then call this function to add a new device.
 * 
 * @param i2c 
 * @param addr 
 * @param clk_speed_hz 
 * @return true 
 * @return false 
 */
bool i2c_init_new_device(i2c_t *i2c, uint8_t i2c_num ,uint8_t addr, uint32_t clk_speed_hz);

/**
 * @brief Get the I2C bus handle. The I2C bus must exist before calling this function.
 * 
 * @param i2c_num 
 * @param ret_bus_handle 
 * @return true if the I2C bus handle is obtained correctly
 * @return false if the I2C bus handle is not obtained correctly
 */
bool i2c_get_bus_handle(uint8_t i2c_num, i2c_master_bus_handle_t *ret_bus_handle);

/**
 * @brief Deinitialize the I2C master driver
 * 
 * @param i2c 
 * @return true if the I2C is deinitialized correctly
 * @return false if the I2C deinitialization failed
 */
bool i2c_deinit(i2c_t *i2c);

/**
 * @brief Given a register address, read the data from the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 * @return true  if the read operation was successful
 * @return false  if the read operation failed
 */
bool i2c_read_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Given a register address, write the data to the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 * @return true if the write operation was successful
 * @return false if the write operation failed
 */
bool i2c_write_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Write data to the I2C bus
 * 
 * @param i2c 
 * @param data 
 * @param len 
 */
void i2c_write(i2c_t *i2c, uint8_t *data, size_t len);

/**
 * @brief Given a 2 bytes register address, read the data from the register.
 *
 * @param i2c
 * @param reg
 * @param data
 * @param len
 */
void i2c_read_reg16bits(i2c_t *i2c, uint16_t reg, uint8_t *data, size_t len);

/**
 * @brief Given a 2 bytes register address, write the data to the register.
 *
 * @param i2c
 * @param reg
 * @param data
 * @param len
 */
void i2c_write_reg16bits(i2c_t *i2c, uint16_t reg, uint8_t *data, size_t len);




// -------------------------------------------------------------
// ---------------------- ADC ---------------------------------
// -------------------------------------------------------------

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "sdkconfig.h"
#include "stdatomic.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#define ADC_CONF_UNIT           ADC_UNIT_1   /*!< ADC unit for ADC1 */
#define ADC_RESOLUTION_12_BIT   4095         /*!< 12-bit resolution for ADC */  

#define ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_CHANNEL_COUNT       1

static const char* TAG_ADC = "adc";

typedef struct
{
    uint8_t gpio_out;
    adc_channel_t chan;
    adc_unit_t unit;
    bool is_calibrated;

    adc_cali_handle_t adc_cali_handle;
    adc_oneshot_unit_handle_t adc_handle;
} adc_t;


/**
 * @brief Initialize the ADC driver
 * 
 * @param adc struct to store the ADC configuration
 * @param gpio_out GPIO pin connected to the ADC output
 * @return true if the ADC is initialized correctly
 * @return false if the ADC initialization failed
 */
bool adc_init(adc_t *adc, uint8_t gpio_out);

/**
 * @brief Deinitialize the ADC driver
 * 
 * @param adc struct to store the ADC configuration
 * @return true if the ADC is deinitialized correctly
 * @return false if the ADC deinitialization failed
 */
bool adc_deinit(adc_t *adc);

/**
 * @brief Create a new ADC unit.
 * 
 * @param handle pointer to the ADC unit handle to be created
 * @param unit_id it can be either ADC_UNIT_1 or ADC_UNIT_2
 * @return true 
 * @return false 
 */
bool adc_create_unit(adc_oneshot_unit_handle_t *handle, uint8_t unit_id);

/**
 * @brief Configure the ADC channel for a specific GPIO pin.
 * ADC unit must be initialized before calling this function, using adc_create_unit.
 * 
 * @param adc 
 * @param gpio 
 * @param unit_id it can be either ADC_UNIT_1 or ADC_UNIT_2
 * @return true 
 * @return false 
 */
bool adc_config_channel(adc_t *adc, uint8_t gpio, uint8_t unit_id);

/**
 * @brief Read the raw ADC value. Range: 0 - 4095
 * 
 * @param adc 
 * @param raw 
 */
void adc_read_raw(adc_t *adc, int *raw);

/**
 * @brief Read the ADC value and convert it to millivolts
 * 
 * @param adc 
 * @param raw 
 * @param angle 
 */
void adc_read_mvolt(adc_t *adc, uint16_t *mvolt);


// -------------------------------------------------------------
// ---------------------- GPIO ---------------------------------
// -------------------------------------------------------------

#include "driver/gpio.h"
#include "esp_err.h"

static const char* TAG_GPIO = "gpio";

typedef struct
{
    uint8_t gpio_num;
    gpio_int_type_t intr_type;
    gpio_mode_t mode;
    gpio_pull_mode_t pullup_en;
    gpio_pull_mode_t pulldown_en;
} gpio_t;

/**
 * @brief Initialize the GPIO driver
 * 
 * @param gpio struct to store the GPIO configuration
 * @param gpio_num GPIO pin number
 * @param mode 1 = INPUT, 2 = OUTPUT, 
 * @param pulldown_en 0 = disable, 1 = enable
 * @param pullup_en 0 = disable, 1 = enable
 * @return true if the GPIO is initialized correctly
 * @return false if the GPIO initialization failed
 */
bool gpio_init_basic(gpio_t *gpio, uint8_t gpio_num, uint8_t mode, bool pulldown_en, bool pullup_en);

/**
 * @brief Deinitialize the GPIO driver
 * 
 * @param gpio struct to store the GPIO configuration
 * @return true if the GPIO is deinitialized correctly
 * @return false if the GPIO deinitialization failed
 */
bool gpio_deinit(gpio_t *gpio);

/**
 * @brief Set the GPIO pin to HIGH
 * 
 * @param gpio struct to store the GPIO configuration
 */
void gpio_set_high(gpio_t *gpio);

/**
 * @brief Set the GPIO pin to LOW
 * 
 * @param gpio struct to store the GPIO configuration
 */
void gpio_set_low(gpio_t *gpio);

// -------------------------------------------------------------
// ---------------------- UART ---------------------------------
// -------------------------------------------------------------

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"

// Constants for UART configuration
#define UART_PIN_NO_USE UART_PIN_NO_CHANGE // Do not use pin for UART configuration UART_PIN_NO_CHANGE is from SDK or ESP-IDF

/**
 * @brief Structure for UART configuration
 * 
 */
typedef struct {
    uint32_t baud_rate;         // Baud rate for UART communication
    uint16_t buffer_size;       // Buffer size for UART communication
    uint8_t gpio_tx;            // GPIO pin for UART TX
    uint8_t gpio_rx;            // GPIO pin for UART RX
    uint8_t gpio_rts;           // GPIO pin for UART RTS
    uint8_t gpio_cts;           // GPIO pin for UART CTS

    // UART configuration structure for ESP32-S3
    uart_port_t uart_num;       // UART port number
    uart_config_t uart_config;  // UART configuration structure
    uart_event_t uart_event;    // UART event structure
    uint32_t event_queue_size;  // Event queue size for UART communication
    QueueHandle_t event_queue;  // Event queue for UART communication
} uart_t;

/**
 * @brief Initialize UART peripheral with custom configuration
 * 
 * @param uart_config UART configuration structure
 * @param baud_rate Baud rate for UART communication
 * @param buffer_size Buffer size for UART communication
 * @param gpio_tx GPIO pin for UART TX
 * @param gpio_rx GPIO pin for UART RX
 * @param gpio_rts GPIO pin for UART RTS
 * @param gpio_cts GPIO pin for UART CTS
 * 
 * @return true if UART initialization was successful and false if UART initialization failed
 */
int uart_init(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, int8_t gpio_tx, int8_t gpio_rx, int8_t gpio_rts, int8_t gpio_cts);

/**
 * @brief Initialize UART peripheral with default configuration
 * 
 * @param uart_t UART configuration structure
 * @param baud_rate Baud rate for UART communication
 * @param buffer_size Buffer size for UART communication
 * @param gpio_tx GPIO pin for UART TX
 * @param gpio_rx GPIO pin for UART RX
 * 
 * @return true if UART initialization was successful and false if UART initialization failed
 */
static inline bool uart_init_with_defaults(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, int8_t gpio_tx, int8_t gpio_rx) {
    return uart_init(uart_config, baud_rate, buffer_size, gpio_tx, gpio_rx, UART_PIN_NO_USE, UART_PIN_NO_USE);
}

/**
 * @brief Write data to the UART
 * 
 * @param uart_config Pointer to the uart_t structure with initialized configuration
 * @param data Pointer to the data buffer to be written
 * @param length Length of the data buffer
 * @return int Number of bytes written, or -1 if an error occurred
 */
int uart_write(uart_t *uart_config, const uint8_t *data, size_t length);

/**
 * @brief Read data from the UART
 * 
 * @param uart_config Pointer to the uart_t structure with initialized configuration
 * @param buffer Pointer to the buffer to store the read data
 * @param length Maximum number of bytes to read into the buffer
 * @param timeout_ms Time in milliseconds to wait for data
 * @return int Number of bytes read, or -1 if an error occurred
 */
int uart_read(uart_t *uart_config, uint8_t *buffer, size_t length, int timeout_ms);

/**
 * @brief Flush the UART buffer
 * 
 * @param uart_config Pointer to the uart_t structure with initialized configuration
 * @return int 0 if the buffer was flushed successfully, or -1 if an error occurred
 */
int uart_clear(uart_t *uart_config);

#endif // __HAL_ESP32__