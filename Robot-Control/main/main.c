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

// #include "types.h"
#include "functions.h"
#include "tasks.h"




// --------------------------------------------------------------------------
// ----------------------------- GLOBAL VARIABLES ---------------------------
// --------------------------------------------------------------------------

bldc_pwm_motor_t gMotor[3]; ///< Array of BLDC motors
control_t gCtrl[3]; ///< Array of control structures
system_t gSys;

senfusion_t gSenFusion; ///< Sensor fusion structure
uart_console_t gUc;

AS5600_t gAS5600[3]; ///< Array of AS5600 sensors
vl53l1x_t gVL53L1X[3]; ///< Array of VL53L1X sensors
BNO055_t gBNO055;


esp_ip4_addr_t gIpAddr;

void app_main(void)
<<<<<<< HEAD
{

    ///<Create the tasks
    create_tasks();
    
=======
{   
    vTaskDelay(pdMS_TO_TICKS(5000));

    ///< Init wifi
    wifi_prepare();
    if (wifi_sta_init(WIFI_SSID, WIFI_PASS, &gIpAddr)) {
        ESP_LOGI("APP", "Wi-Fi connected successfully");
        ESP_LOGI("APP", "IP Address: " IPSTR, IP2STR(&gIpAddr));
        xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE("APP", "Wi-Fi connection failed");
    }
>>>>>>> 92f2b2870baadd914e93e2cd6f651c451be8b06f
    
    ///< Initialize the drivers: LED, UART, BLDC
    init_drivers(); 

    ///< Kernel objects creation like mutexes, semaphores, and queues
    //if (!create_kernel_objects()){
      //  ESP_LOGI("app_main", "Kernel objects not created");
        //return;
    //}

    ///< Initialize and setup each sensor
    //if (!setup_as5600(100)) { ///< Setup the AS5600 sensor
      //  ESP_LOGI("app_main", "AS5600 sensor is not ready");
        //return;
    //}

    // ///< Perform motor identification for all motors
    // motor_identification_all();

    // while (!setup_bno055(100)) { ///< Setup the BNO055 sensor
    //     ESP_LOGI("app_main", "BNO055 sensor is not ready. Resetting...");
    //     BNO055_Reset(&gBNO055); ///< Reset the BNO055 sensor
    //     vTaskDelay(pdMS_TO_TICKS(1000)); ///< Wait 1 second before retrying
    // }
    // if (!setup_vl53l1x(100)) { ///< Setup the VL53L1X sensor
    //     ESP_LOGI("app_main", "VL53L1X sensor is not ready");
    //     return;
    // }

    // ///< Verify the sensors together
    // if (verify_sensors(100)) { 
    //     ESP_LOGI("app_main", "Sensors are ready");
    // }
    // else {
    //     ESP_LOGI("app_main", "Sensors are not ready");
    //     return;
    // }
    
    // ///< Create the tasks
    // create_tasks(); 

    // ///< Initialize the system
    // ///< 'System' refers to more general variables and functions that are used to control the project.
    // init_system();

}
