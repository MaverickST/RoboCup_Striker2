

#include "types.h"
#include "functions.h"
#include "tasks.h"


// --------------------------------------------------------------------------
// ----------------------------- PROTOTYPES ---------------------------------
// --------------------------------------------------------------------------
// Define global variables
led_rgb_t gLed;
bldc_pwm_motor_t gMotor; // Assuming gMotor is also a global variable
system_t gSys;
ctrl_senfusion_t gCtrl; // Assuming gCtrl is also a global variable
uart_console_t gUc;     // Assuming gUc is also a global variable
AS5600_t gAS5600;       // Assuming gAS5600 is also a global variable
vl53l1x_t gVL53L1X;     // Assuming gVL53L1X is also a global variable
BNO055_t gBNO055;  


// --------------------------------------------------------------------------
// --------------------------------- MAIN -----------------------------------
// --------------------------------------------------------------------------

void app_main(void)
{
    init_drivers(); ///< Initialize the drivers

    // create_tasks(); ///< Create the tasks

    ///< ---------------------- SYSTEM -------------------
    // 'System' refers to more general variables and functions that are used to control the system, which
    // consists of the BLDC motor, the AS5600 sensor, the LED, and the UART console.
    // init_system();
    
    

}

// --------------------------------------------------------------------------
// ------------------------------- FUNCTIONS --------------------------------
// --------------------------------------------------------------------------

