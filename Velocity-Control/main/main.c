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

void app_main(void)
{
    ///< Initialize the drivers: LED, UART, BLDC, AS5600, BNO055, VL53L1X
    init_drivers(); 

    ///< Verify the sensors
    if (verify_sensors(100)) { 
        ESP_LOGI("app_main", "Sensors are ready");
    }
    else {
        ESP_LOGI("app_main", "Sensors are not ready");
        return;
    }

    create_tasks(); ///< Create the tasks

    ///< ---------------------- SYSTEM -------------------
    // 'System' refers to more general variables and functions that are used to control the system.
    init_system();

}
