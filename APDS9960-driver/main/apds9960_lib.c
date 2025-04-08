#include "apds9960_lib.h"
#include <stdlib.h>

/**
    * @brief      read bytes
    * @param[in]  *handle pointer to an apds9960 handle structure
    * @param[in]  reg iic register address
    * @param[out] *data pointer to a data buffer
    * @param[in]  len data length
    * @return     status code
    *             - 0 success
    *             - 1 read failed
    * @note       none
    */
void APDS9960_Init(APDS9960_t *handle, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda)
{
        if(!i2c_init(&handle->i2c_handle, i2c_num, gpio_scl, gpio_sda, I2C_MASTER_FREQ_HZ, APDS9960_ADDRESS)) {
                printf("I2C initialization failed\n");
                return;
        }
}

void APDS9960_Deinit(APDS9960_t *handle)
{
        i2c_deinit(&handle->i2c_handle);
}

void APDS9960_Read_Rgbc(APDS9960_t *handle, uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear)
{
        uint8_t data[8]; // Buffer to store the register data

        // Read the registers for CDATAL (Clear), RDATAL (Red), GDATAL (Green), and BDATAL (Blue)
        i2c_read_reg(&handle->i2c_handle, APDS9960_REG_CDATAL, data, 8);

        // Combine the 8-bit values into 16-bit integers
        *clear = (data[1] << 8) | data[0];  // Clear
        *red = (data[3] << 8) | data[2];    // Red
        *green = (data[5] << 8) | data[4];  // Green
        *blue = (data[7] << 8) | data[6];   // Blue
}

void APDS9960_Configure(APDS9960_t *handle)
{
        uint8_t value;

        // Enable the sensor (PON and AEN)
        value = 0x03;
        i2c_write_reg(&handle->i2c_handle, 0x80, &value, 1);

        // Configure integration time (712 ms)
        value = 0xD6;
        i2c_write_reg(&handle->i2c_handle, 0x81, &value, 1);

        // Configure gain (64x)
        value = 0x03;
        i2c_write_reg(&handle->i2c_handle, 0x8F, &value, 1);

        printf("APDS9960 configured successfully!!!\n");
}
