#include <stdio.h>

#include "apds9960_lib.h"

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */


void app_main(void)
{
    APDS9960_t gAPDS9960;

    uint8_t res;
    uint32_t i, times;

    times = 3;
    res = apds9960_basic_init();
    if (res != 0)
    {
        return 1;
    }

    /* 1000 ms */
    apds9960_interface_delay_ms(1000);

    for (i = 0; i < times; i++)
    {
        uint8_t proximity;
        uint16_t red, green, blue, clear;

        /* read rgbc */
        res = apds9960_basic_read_rgbc((uint16_t *)&red, (uint16_t *)&green, (uint16_t *)&blue, (uint16_t *)&clear);
        if (res != 0)
        {
            apds9960_interface_debug_print("apds9960: read rgbc failed.\n");
            (void)apds9960_basic_deinit();

            return 1;
        }

        /* read proximity */
        res = apds9960_basic_read_proximity((uint8_t *)&proximity);
        if (res != 0)
        {
            apds9960_interface_debug_print("apds9960: read proximity failed.\n");
            (void)apds9960_basic_deinit();

            return 1;
        }

        /* output */
        apds9960_interface_debug_print("%d/%d.\n", i + 1, times);
        apds9960_interface_debug_print("apds9960: red is 0x%04X.\n", red);
        apds9960_interface_debug_print("apds9960: green is 0x%04X.\n", green);
        apds9960_interface_debug_print("apds9960: blue is 0x%04X.\n", blue);
        apds9960_interface_debug_print("apds9960: clear is 0x%04X.\n", clear);
        apds9960_interface_debug_print("apds9960: proximity is 0x%02X.\n", proximity);

        /* 1000 ms */
        apds9960_interface_delay_ms(1000);
    }

    /* deinit */
    (void)apds9960_basic_deinit();

    return 0;
}

