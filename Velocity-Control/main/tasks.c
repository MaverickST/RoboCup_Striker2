#include "tasks.h"

void create_tasks(void)
{
    ///< Create a task to handle the UART events
    xTaskCreate(uart_event_task, "uart_event_task", 3*1024, NULL, 1, NULL);

    ///< Create a task to manage the AS5600 sensor
    xTaskCreate(as5600_task, "as5600_task", 4*1024, NULL, 4, &gSys.task_handle_as5600);

    ///< Create a task to manage the BNO055 sensor
    xTaskCreate(bno055_task, "bno055_task", 4*1024, NULL, 4, &gSys.task_handle_bno055);

    ///< Create a task to manage the VL53L1X sensor
    xTaskCreate(vl53l1x_task, "vl53l1x_task", 4*1024, NULL, 3, &gSys.task_handle_vl53l1x);

    ///< Create the control task
    xTaskCreate(control_task, "control_task", 3*1024, NULL, 5, &gSys.task_handle_ctrl);

    ///< Create the trigger task
    xTaskCreate(trigger_task, "trigger_task", 3*1024, NULL, 1, &gSys.task_handle_trigger);

    ///< Create the save task
    xTaskCreate(save_nvs_task, "save_nvs_task", 3*1024, NULL, 6, &gSys.task_handle_save);

    ///< Crate some kernel objects
    gSys.queue = xQueueCreate(5, sizeof(uint8_t)*30); ///< Create a queue to send the data to the save task
    gSys.mutex = xSemaphoreCreateMutex(); ///< Create a mutex to protect the access to the global variables
    if (gSys.queue == NULL) {
        ESP_LOGI("init_system", "Queue not created");
        return;
    }
    if (gSys.mutex == NULL) {
        ESP_LOGI("init_system", "Mutex not created");
        return;
    }
}


void trigger_task(void *pvParameters)
{
    while (true) {
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Notify the each sensor task to read the data from the sensor
        // xTaskNotifyGive(gSys.task_handle_bno055);
        xTaskNotifyGive(gSys.task_handle_vl53l1x);
        // xTaskNotifyGive(gSys.task_handle_as5600);
    }
    vTaskDelete(NULL);
}

void bno055_task(void *pvParameters)
{
    float ax, ay;
    float acce_prev = 0;

    while (true) {
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        float acceleration = 0; ///< Read the acceleration from the BNO055 sensor (dummy value)
      
        ///< Read All data from BNO055 sensor
        if (BNO055_ReadAll(&gBNO055)) { // error reading data 
            acceleration = acce_prev; ///< Set the acceleration to the previous value  

        }else {
            ///< Data from the BNO055 sensor
            ///< acceleration m/s^2 in x, y, z axis
            ax = gBNO055.ax;
            ay = gBNO055.ay;

            ///< From ax and ay we can calculate the accelation in the plane
            acceleration = sqrt(ax*ax + ay*ay);
            float dir = atan2(ay, ax); ///< Get the direction of the acceleration in radians

            ///< Update
            acce_prev = acceleration;
        }
        xSemaphoreTake(gSys.mutex, portMAX_DELAY); ///< Take the mutex to protect the access to the global variables
        gSys.acceleration = acceleration;
        xSemaphoreGive(gSys.mutex); ///< Give the mutex to protect the access to the global variables

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
        
        ///< Read the distance from the VL53L1X sensor
        if (VL53L1X_dataReady(&gVL53L1X)) {
            // If data is ready, read it non-blockingly
            distance_mm = VL53L1X_readDistance(&gVL53L1X, false); // false for non-blocking read
        }
        xSemaphoreTake(gSys.mutex, portMAX_DELAY); ///< Take the mutex to protect the access to the global variables
        gSys.distance = distance_mm/1000.0f - gSys.dist_origin_offset; ///< Read the distance from the VL53L1X sensor
        xSemaphoreGive(gSys.mutex); ///< Give the mutex to protect the access to the global variables

        ///< Notify the control task to process the data
        xTaskNotifyGiveIndexed(gSys.task_handle_ctrl, 2);
    }
    vTaskDelete(NULL);
}

void as5600_task(void *pvParameters)
{
    float prevAngleDeg = 0;      /**< Last raw angle reading, in degrees [0..360). */
    float unwrappedDeg = 0;      /**< Cumulative unwrapped angle, in degrees. */

    while (true) {
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ///< Read the angle from the AS5600 sensor and save it in the buffer
        float angle = AS5600_ADC_GetAngle(&gAS5600) - gSys.angle_origin_offset; ///< Get the angle from the ADC

        // Compute difference
        float delta = angle - prevAngleDeg;

        // Wrap jumps greater than ±180°
        // if      (delta >  180.0f) delta -= 360.0f;
        // else if (delta < -180.0f) delta += 360.0f;

        // Accumulate
        unwrappedDeg += delta;
        prevAngleDeg = angle;

        // Convert degrees to radians and multiply by radius
        xSemaphoreTake(gSys.mutex, portMAX_DELAY); ///< Take the mutex to protect the access to the global variables
        gSys.dist_enc = RADIUS_M * unwrappedDeg * DEG2RAD;
        gSys.angle = angle;
        xSemaphoreGive(gSys.mutex); ///< Give the mutex to protect the access to the global variables

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
        if (gSys.cnt_sample < 0.5*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 0;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 1*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 7;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 2*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 15;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 2.5*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 0;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 3*SAMPLING_RATE_HZ) { ///<
            gSys.duty = -7;
            bldc_set_duty_motor(&gMotor, gSys.duty); 
        }
        else if (gSys.cnt_sample < 4*SAMPLING_RATE_HZ) { ///<
            gSys.duty = -15;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 5*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 0;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }

        else if (gSys.cnt_sample < 5.5*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 7;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 6.5*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 20;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 7.5*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 0;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 8*SAMPLING_RATE_HZ) { ///<
            gSys.duty = -7;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 9*SAMPLING_RATE_HZ) { ///<
            gSys.duty = -20;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }
        else if (gSys.cnt_sample < 10*SAMPLING_RATE_HZ) { ///<
            gSys.duty = 0;
            bldc_set_duty_motor(&gMotor, gSys.duty);
        }

        ///< Send the sensor data to the queue
        uint8_t length = snprintf(NULL, 0, "%d\t%.2f\t%.3f\t%.2f\n", (int)gSys.duty, gSys.angle, gSys.acceleration, gSys.dist_enc);
        char str[length + 1];
        snprintf(str, length + 1, "%d\t%.2f\t%.3f\t%.2f\n", (int)gSys.duty, gSys.angle, gSys.acceleration, gSys.dist_enc);
        // xQueueSendToBack(gSys.queue, (void *)str, (TickType_t)0); ///< Send the data to the queue to be processed by the save task

        ///< Use Sensor Fusion to get the states of the system: position(m), velocity(m/s)
        ctrl_senfusion_predict(&gCtrl, gSys.duty);
        ctrl_senfusion_update(&gCtrl, gSys.dist_enc, gSys.dist_enc, gSys.acceleration, gSys.cnt_sample);
        float pos = ctrl_senfusion_get_pos(&gCtrl);
        float vel = ctrl_senfusion_get_vel(&gCtrl);
        ESP_LOGI(TAG_CTRL_TASK, "pos-> %.2f m, vel-> %.2f m/s", pos, vel);
        ESP_LOGI(TAG_CTRL_TASK, "angle-> %.2f deg, dist-> %.2f m, acc-> %.2f m/s^2", gSys.angle, gSys.dist_enc, gSys.acceleration);

        ///< Safety check to stop the motor if the distance is less than 0.4m and greater than 0.4m
        if (fabs(gSys.dist_enc) > 0.4) {
            bldc_set_duty_motor(&gMotor, 0); ///< Stop the motor
            esp_timer_stop(gSys.oneshot_timer); ///< Stop the timer to stop the sampling
            gSys.STATE = NONE;
        }

        ///< Calculate the PID control if the system is in the control state
        if (gSys.STATE == SYS_SAMPLING_CONTROL) {
            float error_pos = gSys.setpoint_dist - pos; ///< Calculate the error
            float error_vel = gSys.setpoint_vel - vel; ///< Calculate the error
            float control = ctrl_senfusion_calc_pid(&gCtrl, error_pos); ///< Calculate the control value
            bldc_set_duty_motor(&gMotor, control); ///< Set the duty to the motor
            // if (gSys.setpoint_dist < 0) {
            //     bldc_set_duty_motor(&gMotor, -control); ///< Set the duty to the motor
            // }else {
            //     bldc_set_duty_motor(&gMotor, control); ///< Set the duty to the motor
            // }
            ESP_LOGI(TAG_CTRL_TASK, "control-> %.2f", control);

            if (fabs(gSys.setpoint_dist - pos) < 0.01){
                bldc_set_duty_motor(&gMotor, 0); ///< Stop the motor
                esp_timer_stop(gSys.oneshot_timer); ///< Stop the timer to stop the sampling
                gSys.STATE = NONE;
            }
        }
    }
    vTaskDelete(NULL);
}

void save_nvs_task(void *pvParameters)
{
    uint8_t data[25]; ///< Buffer to save the data from the queue
    while (true) {
        ///< Wait for the data from the control task
        xQueueReceive(gSys.queue, (void *const)data, portMAX_DELAY); ///< Receive the data from the queue
        uint8_t length = strlen((const char *)data); ///< Get the length of the data

        ///< Save the data in the NVS
        // esp_partition_write(gSys.part, gSys.current_bytes_written, data, length); ///< Write the data to the NVS partition
        gSys.current_bytes_written += length; ///< Increment the number of bytes written to the NVS

        if (gSys.cnt_sample >= NUM_SAMPLES) { ///< If the number of samples is greater than the number of samples to save, stop the task
            ESP_LOGI(TAG_CMD, "Save task finished");
            break;
        }

    }
    vTaskDelete(NULL);
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
