#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_partition.h"
#include "esp_flash.h"
#include "esp_timer.h"
// #include "nvs_flash.h"
// #include "driver/spi_common.h"
#include "platform_esp32s3.h"

#include "types.h"
#include "led.h"
#include "uart_console.h"
#include "platform_esp32s3.h"
#include "bldc_pwm.h"
#include "as5600_lib.h"
#include "vl53l1x.h"
#include "bno055.h"

// -------------------------------------------------------------------------- 
// ----------------------------- DEFINITIONS --------------------------------
// --------------------------------------------------------------------------

#define I2C_MASTER_SCL_GPIO 8     /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 18       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 0           /*!< I2C port number for master dev */

#define MOTOR_MCPWM_TIMER_RESOLUTION_HZ 1000*1000 // 1MHz, 1 tick = 1us
#define MOTOR_MCPWM_FREQ_HZ             50    // 50Hz PWM
#define MOTOR_MCPWM_DUTY_TICK_MAX       (MOTOR_MCPWM_TIMER_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define MOTOR_MCPWM_GPIO                3
#define MOTOR_REVERSE_GPIO              8
#define MOTOR_PWM_BOTTOM_DUTY           50
#define MOTOR_PWM_TOP_DUTY              120

#define LED_TIME_US     2*1000*1000
#define LED_LSB_GPIO    15

#define UART_NUM        0

#define TIME_SAMPLING_US    10*1000 // 10ms
#define TIME_SAMPLING_S		10		/* 10s sampling data */
#define SAMPLING_RATE_HZ	100 	/* 10ms between each data saved */
#define NUM_SAMPLES			TIME_SAMPLING_S*SAMPLING_RATE_HZ

#define RAD_TO_DEG 57.2957795 // Conversion factor from radians to degrees
#define MAX_BRIGHTNESS 5 // Max brightness of the LED

// --------------------------------------------------------------------------
// ----------------------------- GLOBAL VARIABLES ---------------------------
// --------------------------------------------------------------------------

static const char* TAG_UART_TASK = "uart_task";
static const char* TAG_ADC_TASK = "adc_task";
static const char* TAG_CMD = "cmd";
static const char* TAG_BNO055_TASK = "bno055_task";
static const char* TAG_VL53L1X_TASK = "vl53l1x_task";
static const char* TAG_AS5600_TASK = "as5600_task";
static const char* TAG_CTRL_TASK = "ctrl_task";

volatile flags_t gFlag;
led_rgb_t gLed;
uart_console_t gUc;
bldc_pwm_motor_t gMotor;
AS5600_t gAs5600;
system_t gSys;

esp_timer_handle_t gOneshotTimer;
uint8_t cnt_cali; ///< Counter for the calibration process4

vl53l1x_t gvl53l1x;

BNO055_t bno055; // BNO055 sensor structure
BNO055_CalibProfile_t calib_data; // Calibration data structure


// --------------------------------------------------------------------------
// ----------------------------- PROTOTYPES ---------------------------------
// --------------------------------------------------------------------------

/**
 * @brief Initialize some variables to start the system
 * 
 */
void init_system(void);

/**
 * @brief Callback for the system one-shot timer
 * 
 * @param arg 
 */
void sys_timer_cb(void *arg);

/**
 * @brief Callback for the AS5600 sensor calibration
 * 
 * @param arg 
 */
void sensor_calibration_cb(void *arg);

/**
 * @brief Proccess the command received from the UART console
 * 
 * @param cmd 
 */
void process_cmd(const char *cmd);

/**
 * @brief Task to handle the UART events
 * 
 * @param pvParameters 
 */
void uart_event_task(void *pvParameters);

/**
 * @brief Task to trigger the flow of the tasks
 * 
 * @param pvParameters 
 */
void trigger_task(void *pvParameters);

/**
 * @brief Task to manage the BNO055 sensor
 * 
 * @param pvParameters 
 */
void bno055_task(void *pvParameters);

/**
 * @brief Task to manage the VL53L1X sensor
 * 
 * @param pvParameters 
 */
void vl53l1x_task(void *pvParameters);

/**
 * @brief Task to manage the AS5600 sensor
 * 
 * @param pvParameters 
 */
void as5600_task(void *pvParameters);

/**
 * @brief Task to control the BLDC motor
 * 
 * @param pvParameters 
 */
void control_task(void *pvParameters);

/**
 * @brief Task to save the data in the NVS
 * 
 * @param pvParameters 
 */
void save_nvs_task(void *pvParameters);

// --------------------------------------------------------------------------
// --------------------------------- MAIN -----------------------------------
// --------------------------------------------------------------------------

void app_main(void)
{
    ///< ---------------------- LED ----------------------
    led_init(&gLed, LED_LSB_GPIO, LED_TIME_US, true);
    led_set_blink(&gLed, true, 3);
    led_setup_green(&gLed, LED_TIME_US);

    ///< ---------------------- UART ---------------------
    uconsole_init(&gUc, UART_NUM);
    xTaskCreate(uart_event_task, "uart_event_task", 3*1024, NULL, 1, NULL);

    ///< ---------------------- BLDC ---------------------
    bldc_init(&gMotor, MOTOR_MCPWM_GPIO, MOTOR_REVERSE_GPIO, MOTOR_MCPWM_FREQ_HZ, 0, 
                MOTOR_MCPWM_TIMER_RESOLUTION_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY);
    bldc_enable(&gMotor);
    bldc_set_duty(&gMotor, MOTOR_PWM_BOTTOM_DUTY); 

    ///< ---------------------- BNO055 ------------------
    // BENJAMIN'S CODE
    // Initialize the BNO055 sensor and set the parameters

    // Initialize BNO055 sensor
    int8_t success = 0;
    success = BNO055_Init(&bno055, 17, 18);
    while (success != BNO055_SUCCESS) {
        printf("Error: Failed to initialize BNO055 sensor\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        success = BNO055_Init(&bno055, 17, 18);
    }
    
    //Load Calibration Data
    BNO055_SetOperationMode(&bno055, CONFIGMODE);
    uint8_t calib_offsets[22] = {
        0xF7, 0xFF, 0xCC, 0xFF, 0xC5, 0xFF, 
        0x8A, 0x01, 0x4E, 0x01, 0x5D, 0x00,
        0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
        0xE8, 0x03, 0xAD, 0x01   
    };
    BN055_Write(&bno055, BNO055_ACCEL_OFFSET_X_LSB_ADDR, calib_offsets, 22);
    BNO055_SetOperationMode(&bno055, IMU);

    ///< Create a task to manage the BNO055 sensor
    xTaskCreate(bno055_task, "bno055_task", 2*1024, NULL, 2, &gSys.task_handle_bno055);

    ///< ---------------------- VL53L1X ------------------
    // KEVIN'S CODE
    // direction of the sensor
    // Initialize the VL53L1X sensor and set the parameters
    if (!VL53L1X_init(&gvl53l1x, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO, false)) {
        ESP_LOGE(TAG_VL53L1X, "VL53L1X initialization failed");
        return;
    }

    

    //Flag to check if the sensor is calibrated
    gSys.is_vl53l1x_calibrated = true;

    //Flags true for test
    gSys.is_as5600_calibrated = true;
    gSys.is_bno055_calibrated = true;
    gSys.is_bldc_calibrated = true;
      
    VL53L1X_startContinuous(&gvl53l1x,10);
    // uint16_t distance_mm ;
    // while(1){

    //     // Check if data is ready using VL53L1X_dataReady
    //     if (VL53L1X_dataReady(&gvl53l1x)) {
    //         // If data is ready, read it non-blockingly
    //         distance_mm = VL53L1X_readDistance(&gvl53l1x, false); // false for non-blocking read
            
    //         // VL53L1X_readDistance (when non-blocking and data is ready) should give a valid distance.
    //         // It might still return 0 or an error code if something went wrong during the read itself,
    //         // though the primary purpose of dataReady is to avoid reading when no new data is present.
    //         // The exact return value for "no error" vs "error" in non-blocking mode depends on the
    //         // VL53L1X_readDistance implementation. Assuming it returns measured distance or 0 on error/no data.
    //         if (distance_mm > 0) { // A simple check, adjust if your sensor can legitimately read 0 mm.
    //             // print the distance
    //             ESP_LOGI(TAG_VL53L1X, "Distance: %d mm", distance_mm);
    //             // Save the distance to the system structure
                
    //             } else {
    //             // This might occur if readDistance itself failed after dataReady was true,
    //             // or if 0 is a legitimate but problematic reading.
    //             }
    //     } else {
    //         // Optional: Log if data is not ready, though this might be frequent
    //         // ESP_LOGD(TAG_VL53L1X_MainTest, "Data not ready");
    //     }
        
    //     // Delay to maintain the approximate 10ms loop frequency
    //     vTaskDelay(pdMS_TO_TICKS(10)); 
    
    // }
    

    ///< Create a task to manage the VL53L1X sensor
    xTaskCreate(vl53l1x_task, "vl53l1x_task", 2*1024, NULL, 3, &gSys.task_handle_vl53l1x);

    ///< ---------------------- AS5600 -------------------
    // Initialize the AS5600 sensor
    // AS5600_Init(&gAs5600, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    // Set some configurations to the AS5600
    // AS5600_config_t conf = {
    //     .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
    //     .HYST = AS5600_HYSTERESIS_OFF, ///< Hysteresis off
    //     .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
    //     .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
    //     .SF = AS5600_SLOW_FILTER_16X, ///< Slow filter 16x
    //     .FTH = AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY, ///< Slow filter only
    //     .WD = AS5600_WATCHDOG_ON, ///< Watchdog on
    // };
    // AS5600_SetConf(&gAs5600, conf);

    // Create a one-shot timer to control the sequence
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &sensor_calibration_cb,
        .arg = NULL, ////< argument specified here will be passed to timer callback function
        .name = "as5600_cali-one-shot" ///< name is optional, but may help identify the timer when debugging
    };
    esp_timer_handle_t oneshot_timer;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
    gSys.oneshot_timer2 = oneshot_timer;
    gSys.cnt_cali = 0; ///< Initialize the counter for the calibration process

    // ESP_ERROR_CHECK(esp_timer_start_once(gSys.oneshot_timer2, 500*1000)); ///< Start the timer to calibrate the AS5600 sensor
    ESP_LOGI("app_main", "AS5600 calibration timer started");
    
    // Create a task to manage the AS5600 sensor
    // xTaskCreate(as5600_task, "as5600_task", 2*1024, NULL, 4, &gSys.task_handle_as5600);

    ///< ---------------------- SYSTEM -------------------
    // 'System' refers to more general variables and functions that are used to control the system, which
    // consists of the BLDC motor, the AS5600 sensor, the LED, and the UART console.
    init_system();

}

// --------------------------------------------------------------------------
// ------------------------------- FUNCTIONS --------------------------------
// --------------------------------------------------------------------------

void init_system(void)
{
    ///< Initialize the system variables
    gSys.cnt_sample = 0; ///< Initialize the number of samples readed from all the sensors
    gSys.is_as5600_calibrated = false; ///< Initialize the AS5600 sensor calibration flag
    gSys.is_bno055_calibrated = false; ///< Initialize the BNO055 sensor calibration flag
    gSys.is_vl53l1x_calibrated = false; ///< Initialize the VL53L1X sensor calibration flag
    gSys.is_bldc_calibrated = false; ///< Initialize the BLDC motor calibration flag
    gSys.STATE = INIT_BLDC_STEP_1; ///< Initialize the state machine: initialize the BLDC motor
    gSys.current_bytes_written = 0; ///< Initialize the number of samples readed from the ADC

    // Get the partition table and erase the partition to store new data
    gSys.part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "angle_pos");
    if (gSys.part == NULL) {
        ESP_LOGI("init_system", "Partition not found");
        return;
    }
    ESP_ERROR_CHECK(esp_flash_erase_region(gSys.part->flash_chip, gSys.part->address, gSys.part->size));
    vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait for the erase to finish
    char part_label[] = "Duty\tAngle(deg)\tAcce(m^2)\tDist(m)\n";
    part_label[strlen(part_label)] = '\0'; ///< Add the null terminator to the string
    esp_err_t rest = esp_partition_write(gSys.part, 0, part_label, strlen(part_label));
    gSys.current_bytes_written += strlen(part_label);

    // Print the result of the operation
    if (rest == ESP_OK) {
        ESP_LOGI("init_system", "Text written successfully");
    }
    else {
        ESP_LOGI("init_system", "Error writing text");
    }

    // Create a one-shot timer to control the sequence
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &sys_timer_cb,
        .arg = NULL, ////< argument specified here will be passed to timer callback function
        .name = "sys-one-shot" ///< name is optional, but may help identify the timer when debugging
    };
    esp_timer_handle_t oneshot_timer_s;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer_s));
    gSys.oneshot_timer = oneshot_timer_s;
    ESP_ERROR_CHECK(esp_timer_start_once(gSys.oneshot_timer, NONE_TO_STEPS_US));

    ///< Create the control task
    xTaskCreate(control_task, "control_task", 3*1024, NULL, 5, &gSys.task_handle_ctrl);

    ///< Create the trigger task
    xTaskCreate(trigger_task, "trigger_task", 3*1024, NULL, 1, &gSys.task_handle_trigger);

    ///< Create the save task
    gSys.queue = xQueueCreate(5, sizeof(uint8_t)*20); ///< Create a queue to send the data to the save task
    xTaskCreate(save_nvs_task, "save_nvs_task", 3*1024, NULL, 6, &gSys.task_handle_save);

}

void sys_timer_cb(void *arg)
{
    switch(gSys.STATE)
    {
        case INIT_BLDC_STEP_1:
            ESP_LOGI("sys_timer_cb", "INIT_PWM_BLDC_STEP_1");
            bldc_set_duty(&gMotor, 0); ///< Set the duty to the top position for the ESC

            ///< Use the time for the sensor sampling and control the BLDC motor
            gSys.STATE = CHECK_SENSORS;
            gSys.is_bldc_calibrated = true; ///< Set the flag to true to indicate that the BLDC motor is calibrated
            ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.oneshot_timer, TIME_SAMPLING_US));
            break;

        case CHECK_SENSORS:
            // Check if the sensors are calibrated
            if (gSys.is_as5600_calibrated && gSys.is_bno055_calibrated && gSys.is_vl53l1x_calibrated && gSys.is_bldc_calibrated) {
                gSys.STATE = SYS_SAMPLING; ///< Set the state to sampling
                ESP_LOGI("sys_timer_cb", "Sensors calibrated. Starting the system.");
            }
            break;

        case SYS_SAMPLING:
            BaseType_t mustYield = pdFALSE;
            
            vTaskNotifyGiveFromISR(gSys.task_handle_trigger, &mustYield);

            // portYIELD_FROM_ISR(mustYield); ///< Yield the task to allow the other tasks to run
            gSys.cnt_sample++; ///< Increment the number of samples readed from all the sensors
            if (gSys.cnt_sample >= NUM_SAMPLES) {
                ESP_ERROR_CHECK(esp_timer_stop(gSys.oneshot_timer)); ///< Stop the timer to stop the sampling
                ESP_ERROR_CHECK(esp_timer_delete(gSys.oneshot_timer)); ///< Delete the timer to stop the sampling
                bldc_set_duty(&gMotor, 0); ///< Stop the motor
                ESP_LOGI("sys_timer_cb", "Samples readed from all the sensors: %d", gSys.cnt_sample);
            }

            break;

        default:
            ESP_LOGI("sys_timer_cb", "default");
            break;
    }
}

void sensor_calibration_cb(void *arg)
{
    switch (gSys.cnt_cali) {
        case 0:
            printf("AS5600 calibration step 1. \nAs step 4 in page 22 of the datasheet, move the magnet (or wheel) to the MAX position (5 seconds to move it).\n");

            gSys.cnt_cali++;
            esp_timer_start_once(gSys.oneshot_timer2, 5*1000*1000); ///< Start the timer to calibrate the AS5600 sensor (5s)
            break;
        case 1:
            printf("AS5600 calibration step 2. Setting the max position....\n");

            gSys.raw_angle = 0;
            AS5600_GetRawAngle(&gAs5600, &gSys.raw_angle); ///< Get the raw angle from the AS5600 sensor
            printf("Raw angle readed (max position): 0x%04X\n", gSys.raw_angle);
            AS5600_SetStopPosition(&gAs5600, gSys.raw_angle); ///< Set the stop position to the raw angle readed from the AS5600 sensor
            printf("Max position setted. Wait at least 1ms. \n");

            gSys.cnt_cali++;
            esp_timer_start_once(gSys.oneshot_timer2, 500*1000); ///< Start the timer to calibrate the AS5600 sensor (500ms)
            break;
        case 2:
            printf("AS5600 calibration step 3. \nUse burn commands to permanently write the start and stop positions....\n");

            ///< BURN COMMANDS HERE (warning: this will burn the configuration to the EEPROM of the AS5600 sensor, wait at least 1ms before using the I2C AS5600 again)

            AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver

            /**
             * @brief If the readed voltage is always 0, this indicates an error occurred during calibration.
             * 
             */
            float angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
            printf("angle-> %0.2f\n", angle);

            // Read n times the angle.
            for (int i = 0; i < 1; i++) {
                vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait 1s
                angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
                printf("angle-> %0.2f\n", angle);
            }

            gSys.is_as5600_calibrated = true; ///< Set the flag to true to indicate that the AS5600 sensor is calibrated
            esp_timer_delete(gSys.oneshot_timer2); ///< Delete the timer

            break;
        default:
            printf("AS5600 calibration finished");
            break;
    }
}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    while(true) {
        if(xQueueReceive(gUc.uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
            case UART_DATA: ///< Event of UART receving data
                ESP_LOGI(TAG_UART_TASK, "UART_DATA");
                uconsole_read_data(&gUc);

                char cmd[4];
                strncpy(cmd, (const char *)gUc.data, 3); ///< Get the first 3 characters
                cmd[3] = '\0'; ///< Add the null terminator
                ESP_LOGI(TAG_UART_TASK, "cmd-> %s", cmd);
                process_cmd(cmd);
                break;

            case UART_FIFO_OVF: ///< Event of HW FIFO overflow
                ESP_LOGI(TAG_UART_TASK, "UART_FIFO_OVF");
                uart_flush_input(gUc.uart_num);
                xQueueReset(gUc.uart_queue);
                break;

            case UART_BUFFER_FULL: ///< Event of UART ring buffer full
                ESP_LOGI(TAG_UART_TASK, "UART_BUFFER_FULL");
                uart_flush_input(gUc.uart_num);
                xQueueReset(gUc.uart_queue);
                break;

            default:
                ESP_LOGI(TAG_UART_TASK, "uart event type: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void trigger_task(void *pvParameters)
{
    while (true) {
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Notify the each sensor task to read the data from the sensor
        xTaskNotifyGive(gSys.task_handle_bno055);
        xTaskNotifyGive(gSys.task_handle_vl53l1x);
        xTaskNotifyGive(gSys.task_handle_as5600);
    }
    vTaskDelete(NULL);
}

void bno055_task(void *pvParameters)
{
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;

    while (true) {
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gSys.acceleration = 19.0; ///< Read the acceleration from the BNO055 sensor (dummy value)
      
        ///< Read All data from BNO055 sensor
        BNO055_ReadAll(&bno055);

        ///< Data from the BNO055 sensor

        ///< acceleration m/s^2 in x, y, z axis
        ax = bno055.ax;
        ay = bno055.ay;
        az = bno055.az;

        ///< degree per second in x, y, z axis
        gx = bno055.gx;
        gy = bno055.gy;
        gz = bno055.gz;

        ///< Get Euler angles (like an airplane)
        roll = bno055.roll;     // Do a barrel roll
        pitch = bno055.pitch;   // up, down
        yaw = bno055.yaw;       // rigth, left

        ///< Convert to degrees
        roll *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;
        ///< Invert yaw direction
        yaw = 360.0 - yaw;
    
        ///< Notify the control task to process the data
        xTaskNotifyGiveIndexed(gSys.task_handle_ctrl, 1); ///< Notify the control task to process the data
    }
    vTaskDelete(NULL);
}

void vl53l1x_task(void *pvParameters)
{
    uint16_t distance_mm = 0; ///< Variable to store the distance readed from the VL53L1X sensor
    while (true) {
        
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gSys.distance = 12.0; ///< Read the distance from the VL53L1X sensor (dummy value)
        
        ///< Read the distance from the VL53L1X sensor
        if (VL53L1X_dataReady(&gvl53l1x)) {
            // If data is ready, read it non-blockingly
            distance_mm = VL53L1X_readDistance(&gvl53l1x, false); // false for non-blocking read
        }  
        ///< Notify the control task to process the data
        xTaskNotifyGiveIndexed(gSys.task_handle_ctrl, 2);
    }
    vTaskDelete(NULL);
}

void as5600_task(void *pvParameters)
{
    while (true) {
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ///< Read the angle from the AS5600 sensor and save it in the buffer
        gSys.angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC

        ///< Notify the control task to process the data
        xTaskNotifyGiveIndexed(gSys.task_handle_ctrl, 3);
    }
    vTaskDelete(NULL);
}

void control_task(void *pvParameters)
{
    while (true) {
        ///< Wait for the notification from all task sensors. configTASK_NOTIFICATION_ARRAY_ENTRIES
        ulTaskNotifyTakeIndexed(1, pdFALSE, portMAX_DELAY); ///< Wait for the notification from the BNO055 task
        ulTaskNotifyTakeIndexed(2, pdFALSE, portMAX_DELAY); ///< Wait for the notification from the VL53L1X task
        ulTaskNotifyTakeIndexed(3, pdFALSE, portMAX_DELAY); ///< Wait for the notification from the AS5600 task

        ///< Process the data from the sensors and control the BLDC motor
        if (gSys.cnt_sample < 1*SAMPLING_RATE_HZ) { ///< 1s of sampling
            bldc_set_duty_motor(&gMotor, 65);
            gSys.duty = 65;
        }
        else if (gSys.cnt_sample < 2*SAMPLING_RATE_HZ) { ///< 2s of sampling
            bldc_set_duty_motor(&gMotor, 0);
            gSys.duty = 0;
        }
        else if (gSys.cnt_sample < 4*SAMPLING_RATE_HZ) { ///< 4s of sampling
            bldc_set_duty_motor(&gMotor, -75);
            gSys.duty = -75;
        }
        else if (gSys.cnt_sample < 5*SAMPLING_RATE_HZ) { ///< 5s of sampling
            bldc_set_duty_motor(&gMotor, 0);
            gSys.duty = 0;
        }
        else if (gSys.cnt_sample < 7*SAMPLING_RATE_HZ) { ///< 7s of sampling
            bldc_set_duty_motor(&gMotor, 85); ///< Set the duty cycle to 8.5%
            gSys.duty = 85;
        }
        else if (gSys.cnt_sample < 8*SAMPLING_RATE_HZ) { ///< 8s of sampling
            bldc_set_duty_motor(&gMotor, 0); ///< Set the duty cycle to 0%
            gSys.duty = 0;
        }

        ///< Send the sensor data to the queue
        uint8_t length = snprintf(NULL, 0, "%d\t%d\t%d\t%d\n", (int)gSys.duty, (int)gSys.angle, (int)gSys.acceleration, (int)gSys.distance);
        char str[length + 1];
        snprintf(str, length + 1, "%d\t%d\t%d\t%d\n", (int)gSys.duty, (int)gSys.angle, (int)gSys.acceleration, (int)gSys.distance);
        xQueueSendToBack(gSys.queue, (void *)str, (TickType_t)0); ///< Send the data to the queue to be processed by the save task

    }
    vTaskDelete(NULL);
}


void save_nvs_task(void *pvParameters)
{
    uint8_t data[20]; ///< Buffer to save the data from the queue
    while (true) {
        ///< Wait for the data from the control task
        xQueueReceive(gSys.queue, (void *const)data, portMAX_DELAY); ///< Receive the data from the queue
        uint8_t length = strlen((const char *)data); ///< Get the length of the data

        ///< Save the data in the NVS
        esp_partition_write(gSys.part, gSys.current_bytes_written, data, length); ///< Write the data to the NVS partition
        gSys.current_bytes_written += length; ///< Increment the number of bytes written to the NVS

        if (gSys.cnt_sample >= NUM_SAMPLES) { ///< If the number of samples is greater than the number of samples to save, stop the task
            ESP_LOGI(TAG_CMD, "Save task finished");
            break;
        }

    }
    vTaskDelete(NULL);
}


void process_cmd(const char *cmd)
{
    ///< Check if the data is valid
    uint8_t len_uc_data = strlen((const char *)gUc.data);
    if (len_uc_data < 5) { ///< If the data is less than 5 characters, that means the data is not valid
        ESP_LOGI(TAG_CMD, "Invalid CMD");
        return;
    }
    ///< Command to set the speed of the motor
    if (strcmp(cmd, "pwm") == 0) {
        char str_value[len_uc_data - 4]; ///< 4 is the length of the command "pwm "
        strncpy(str_value, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the value after the command

        uint16_t value = atoi(str_value);
        ESP_LOGI(TAG_CMD, "value-> %d", value);
        if (value != 0) { ///< If value=0, that means data is not a number
            bldc_set_duty(&gMotor, value);
        }
    }
    ///< Command to read the angle from the AS5600 sensor
    else if(strcmp(cmd, "as5") == 0){
        ///< Check the minimum length of the command
        if (len_uc_data <= 8) { ///< 4 for the command "as5 " and 4 for the register
            ESP_LOGI(TAG_CMD, "Invalid AS5600 cmd");
            return;
        }
        ///< From cmd, get if it is read or write: "as5 r" or "as5 w"
        char rw = *(gUc.data + 4);

        ///< Then get the register to read or write: "as5 r zmco", "as5 w zpos", ....
        uint8_t len_reg = 4;
        char reg[len_reg]; ///< 4 is the length of the register, +1 for the null terminator
        strncpy(reg, (const char *)gUc.data + strlen("as5 r "), len_reg); ///< Get the register after the command
        reg[len_reg] = '\0'; ///< Add the null terminator
        AS5600_reg_t addr = AS5600_RegStrToAddr(&gAs5600, reg); ///< Map the str to int address
        if (addr == -1) {
            ESP_LOGI(TAG_CMD, "Invalid register");
            return;
        }

        ///< Read or write the register
        uint16_t data;
        if (rw == 'r') {
            ESP_LOGI(TAG_CMD, "addr-> %02x", (uint8_t)addr);
            AS5600_ReadReg(&gAs5600, addr, &data);
            ESP_LOGI(TAG_CMD, "readed-> %04x", data);
        }
        ///< For write commands, it is necessary to get the value to write
        else if (rw == 'w') {
            ///< Check the minimum length of the command
            if (len_uc_data != 15) { ///< 15 for a command like "as5 w [regi] [0000]"
                ESP_LOGI(TAG_CMD, "Invalid AS5600 cmd");
                return;
            }
            ///< Get the exadecimal value from 3 characters
            uint8_t len_value = 4;
            char str_value[len_value]; ///< value to write in chars
            strncpy(str_value, (const char *)gUc.data + strlen("as5 w regi "), len_value); ///< Get the value after the command
            str_value[len_value] = '\0'; ///< Add the null terminator
            uint16_t value = 0; ///< value to write in int
            for (int i = 0; i < len_value; i++) {
                if (str_value[i] >= '0' && str_value[i] <= '9') {
                    value += (str_value[i] - '0') << (4 * (len_value - i - 1));
                }
                else if (str_value[i] >= 'a' && str_value[i] <= 'f') {
                    value += (str_value[i] - 'a' + 10) << (4 * (len_value - i - 1));
                }
                else {
                    ESP_LOGI(TAG_CMD, "Invalid value");
                    return;
                }
            }
            AS5600_WriteReg(&gAs5600, addr, value);
            ESP_LOGI(TAG_CMD, "addr-> %02x, value-> %04x", (uint8_t)addr, value);
        }
        else if (rw == 'o') {
            printf("as5 o");
        }
        else {
            ESP_LOGI(TAG_CMD, "rw not recognized");
        }

    }
    ///< Command to turn on the LED
    else if(strcmp(cmd, "led") == 0) {
        if (len_uc_data < 7) { ///< 4 for the command "led " and 5 for the color
            ESP_LOGI(TAG_CMD, "Invalid LED cmd");
            return;
        }
        char str_color[len_uc_data - 4]; ///< 4 is the length of the command "led "
        strncpy(str_color, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the color after the command
        str_color[len_uc_data - 4] = '\0'; ///< Add the null terminator
        if (strcmp(str_color, "red") == 0) {
            led_setup_red(&gLed, LED_TIME_US);
        }
        else if (strcmp(str_color, "green") == 0) {
            led_setup_green(&gLed, LED_TIME_US);
        }
        else if (strcmp(str_color, "blue") == 0) {
            led_setup_blue(&gLed, LED_TIME_US);
        }
        else {
            ESP_LOGI(TAG_CMD, "color not recognized");
        }
    }
    else {
        ESP_LOGI(TAG_CMD, "cmd not recognized");
    }
}

