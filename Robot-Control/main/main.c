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

#define WIFI_SSID "Esp_Test"
#define WIFI_PASS "12345678"
esp_ip4_addr_t gIpAddr;
void app_main(void)
{
    wifi_prepare();
 

    if (wifi_sta_init(WIFI_SSID, WIFI_PASS, &gIpAddr)) {
        

        ESP_LOGI("APP", "Wi-Fi connected successfully");
        ESP_LOGI("APP", "IP Address: " IPSTR, IP2STR(&gIpAddr));
        xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);


    } else {
        ESP_LOGE("APP", "Wi-Fi connection failed");
    }
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}
    ///< Initialize the drivers: LED, UART, BLDC
    //init_drivers(); 

    ///< Initialize and setup each sensor
    
    ///< Create the tasks

    ///< Initialize the system
    ///< 'System' refers to more general variables and functions that are used to control the project.


