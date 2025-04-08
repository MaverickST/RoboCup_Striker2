#include <stdio.h>

#include "apds9960_lib.h"

#define I2C_MASTER_SCL_GPIO 7       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 6       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */


void app_main(void)
{
    printf("APDS9960 RGB Color Sensor Ejemplito\n");
    printf("Initializing...\n");
    APDS9960_t gAPDS9960;
    APDS9960_Init(&gAPDS9960, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO);
    APDS9960_Configure(&gAPDS9960);
    printf("Sensor APDS9960 configurado\n");
    while(1){
        uint16_t red, green, blue, clear;
        APDS9960_Read_Rgbc(&gAPDS9960, &red, &green, &blue, &clear);
        printf("Red: %d, Green: %d, Blue: %d, Clear: %d\n", red, green, blue, clear);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
    
    
    


}

