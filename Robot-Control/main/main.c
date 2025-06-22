/**
 * @file main.c
 * @author Striker2
 * @brief Main file for the project
 * @version 0.1
 * @date 2025-05-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "types.h"
#include "functions.h"
#include "tasks.h"

//Test WiFi
#include "platform_wifi_esp32s3.h"


// --------------------------------------------------------------------------
// ----------------------------- GLOBAL VARIABLES ---------------------------
// --------------------------------------------------------------------------

led_rgb_t gLed;
bldc_pwm_motor_t gMotor;
system_t gSys;
ctrl_senfusion_t gCtrl;
uart_console_t gUc;

AS5600_t gAS5600;
vl53l1x_t gVL53L1X;
BNO055_t gBNO055;

#define WIFI_SSID "Jannis"
#define WIFI_PASS "Krakus3008"
void app_main(void)
{
    if (wifi_sta_init(WIFI_SSID, WIFI_PASS)) {
        ESP_LOGI("APP", "Wi-Fi connected successfully");
    } else {
        ESP_LOGE("APP", "Wi-Fi connection failed");
    }
}
    ///< Initialize the drivers: LED, UART, BLDC
    //init_drivers(); 

    ///< Initialize and setup each sensor
    
    ///< Create the tasks

    ///< Initialize the system
    ///< 'System' refers to more general variables and functions that are used to control the project.


