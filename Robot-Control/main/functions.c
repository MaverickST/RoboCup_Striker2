#include "functions.h"

void init_drivers(void)
{
    ///< ---------------- CNTROL + SENFUSION ----------------
    pid_block_t config_pid = {
        .Kp = 0.8, ///< Proportional gain
        .Kd = 0.0012, ///< Derivative gain
        .Ki = 137, ///< Integral gain
        .max_output   = 60,
        .min_output   = -60,
        .max_integral = 200,
        .min_integral = -200,
    };

    senfusion_init(&gSenFusion, SAMPLING_PERIOD_S); // 10ms sampling time
    for (int i = 0; i < 3; i++) {
        control_init(&gCtrl[i], SAMPLING_PERIOD_S/10, config_pid); ///< Initialize the control structure
    }

    ///< ---------------------- UART ---------------------
    uconsole_init(&gUc, UART_NUM);

    ///< ---------------------- BLDC ---------------------
    ///< Initialize the BLDC motors
    for (int i = 0; i < 3; i++) {
        bldc_init(&gMotor[i], MOTOR_N_MCPWM_GPIO(i), MOTOR_N_MCPWM_REVERSE_GPIO(i), MOTOR_MCPWM_FREQ_HZ, 1, 
            MOTOR_MCPWM_TIMER_RESOLUTION_HZ, MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY);

        bldc_set_duty(&gMotor[i], MOTOR_PWM_BOTTOM_DUTY); ///< Set the initial duty cycle to 0%
        bldc_set_duty_motor(&gMotor[i], MOTOR_PWM_BOTTOM_DUTY); ///< Set the initial duty cycle to 0% for the motor
        // bldc_calibrate(&gMotor[i], MOTOR_PWM_BOTTOM_DUTY, MOTOR_PWM_TOP_DUTY); ///< Calibrate the BLDC motor
    }
    ESP_LOGI("drivers", "Drivers initialized successfully");

}

bool setup_as5600(uint32_t num_checks)
{
    ///< Initialize the AS5600 sensor
    ESP_LOGI(TAG_AS5600_TASK, "Initializing AS5600 sensors \n");

    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis 2LSB
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 8x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Fast filter threshold 6LSB
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog off
    };
    // for (int i = 0; i < 3; i++) {
    //     AS5600_Init(&gAS5600[i], AS5600_I2C_MASTER_NUM, AS5600_I2C_MASTER_SCL_GPIO, AS5600_I2C_MASTER_SDA_GPIO, AS5600_N_OUT_GPIO(i)); ///< Initialize the AS5600 sensor
    //     AS5600_SetConf(&gAS5600[i], conf); ///< Set the configuration for the AS5600 sensor
    //     AS5600_SetStartPosition(&gAS5600[i], 0x000); ///< Set the start position to 0
    //     AS5600_SetStopPosition(&gAS5600[i], 0xFFF); ///< Set the stop position to 4095

    //     // ///< Burn the angle command and settings to the EEPROM
    //     // AS5600_BurnAngleCommand(&gAS5600[i]);
    //     // AS5600_BurnSettingCommand(&gAS5600[i]);
    //     // vTaskDelay(pdMS_TO_TICKS(10));
        
    //     ///< Read the configuration, start and stop positions to verify
    //     AS5600_config_t read_conf;
    //     uint16_t start_pos, stop_pos;

    //     AS5600_GetConf(&gAS5600[i], &read_conf);
    //     AS5600_GetStartPosition(&gAS5600[i], &start_pos);
    //     AS5600_GetStopPosition(&gAS5600[i], &stop_pos);

    //     printf("Start Position: 0x%03X, Stop Position: 0x%03X\n", start_pos, stop_pos);
    //     if (read_conf.WORD == conf.WORD && start_pos == 0x000 && stop_pos == 0xFFF) {
    //         ESP_LOGI(TAG_AS5600_TASK, "AS5600 sensor %d initialized successfully\n", i);
    //     } else {
    //         ESP_LOGE(TAG_AS5600_TASK, "AS5600 sensor %d initialization failed\n", i);
    //         return false;
    //     }
    //     AS5600_DeinitI2C(&gAS5600[i]); ///< Deinitialize the AS5600 sensor
    // }

    ///< Initialize the ADC for the AS5600 sensors
    adc_oneshot_unit_handle_t handle;
    if (!adc_create_unit(&handle, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE(TAG_AS5600_TASK, "AS5600 ADC initialization failed\n");
        return false;
    }
    for (int i = 0; i < 3; i++) {
        gAS5600[i].conf = conf; ///< Set the configuration for the AS5600 sensor
        gAS5600[i].adc_handle.adc_handle = handle; ///< Assign the ADC handle to the AS5600 sensor
        if (!adc_config_channel(&gAS5600[i].adc_handle, AS5600_N_OUT_GPIO(i), AS5600_ADC_UNIT_ID)) {
            ESP_LOGE(TAG_AS5600_TASK, "AS5600 sensor %d ADC initialization failed\n", i);
            continue;
        }
        gAS5600[i].is_calibrated = true;
        gAS5600[i].angle_offset = AS5600_ADC_GetAngle(&gAS5600[i]);
        wrap_printf("AS5600 sensor %d initialized successfully with angle offset: %.2f\n", i, gAS5600[i].angle_offset);
    }

    return true;
}

bool setup_bno055(uint32_t num_checks)
{
    ///< Initialize BNO055 sensor
    ESP_LOGI(TAG_BNO055_TASK, "Initializing BNO055 sensors \n");
    int8_t success = BNO055_Init(&gBNO055, BNO055_I2C_MASTER_SDA_GPIO, BNO055_I2C_MASTER_SCL_GPIO, BNO055_I2C_MASTER_NUM, BNO055_RST_GPIO);
    while (success != BNO055_SUCCESS) {
        printf("Error: Failed to initialize BNO055 sensor\n");
        BNO055_Reset(&gBNO055);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        success = BNO055_Init(&gBNO055, BNO055_I2C_MASTER_SDA_GPIO, BNO055_I2C_MASTER_SCL_GPIO, BNO055_I2C_MASTER_NUM, BNO055_RST_GPIO);
    }
    
    ///< Load Calibration Data
    BNO055_SetOperationMode(&gBNO055, CONFIGMODE);
    uint8_t calib_offsets[22] = {
        0xF7, 0xFF, 0xCC, 0xFF, 0xC5, 0xFF, 
        0x8A, 0x01, 0x4E, 0x01, 0x5D, 0x00,
        0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
        0xE8, 0x03, 0xAD, 0x01   
    };
    BN055_Write(&gBNO055, BNO055_ACCEL_OFFSET_X_LSB_ADDR, calib_offsets, 22);
    BNO055_SetOperationMode(&gBNO055, IMU);
    gBNO055.is_calibrated = true;

    ///< Individual sensor checks
    float acce = 0, acce_prev = 0;
    uint16_t cnt = 0;
    for (uint32_t i = 0; i < num_checks; i++) {
        BNO055_ReadAll_Lineal(&gBNO055);
        acce = sqrt(gBNO055.ax*gBNO055.ax + gBNO055.ay*gBNO055.ay);
        if (fabs(acce - acce_prev) < 0.0001) {
            cnt++;
        }
        acce_prev = acce;
        // printf("Acce: %.2f m^2\n", acce);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI("setup_bno055", "Verifing BNO055 sensor: err %d - num %d", (int)cnt, (int)num_checks);
    if (cnt > num_checks/2) {
        return false;
    }

    return true;
}

bool setup_vl53l1x(uint32_t num_checks)
{
    ///< Initialize the VL53L1X sensors and set the parameters
    ESP_LOGI(TAG_VL53L1X_TASK, "Initializing VL53L1X sensor\n");

    // // ///< Create I2C bus and add the devices
    // // i2c_init_new_bus(&gVL53L1X[0].i2c_handle, VL53L1X_I2C_MASTER_NUM, VL53L1X_I2C_MASTER_SCL_GPIO, VL53L1X_I2C_MASTER_SDA_GPIO);
    // // gVL53L1X[1].i2c_handle.bus_handle = gVL53L1X[0].i2c_handle.bus_handle;
    // // gVL53L1X[2].i2c_handle.bus_handle = gVL53L1X[0].i2c_handle.bus_handle;
    // // for (int i = 0; i < 3; i++) {
    // //     i2c_init_new_device(&gVL53L1X[i].i2c_handle, VL53L1X_I2C_MASTER_NUM, VL53L1X_SENSOR_ADDR, I2C_MASTER_FREQ_HZ);
    // // }

    for (int i = 0; i < 3; i++) {
        if (!VL53L1X_init(&gVL53L1X[i], VL53L1X_I2C_MASTER_NUM, VL53L1X_I2C_MASTER_SCL_GPIO, VL53L1X_I2C_MASTER_SDA_GPIO, false)) {
            ESP_LOGE(TAG_VL53L1X_TASK, "VL53L1X sensor %d initialization failed\n", i);
            continue;
        }
        // VL53L1X_setAddress(&gVL53L1X[i], VL53L1X_SENSOR_ADDR + i + 1); ///< Set the address of the sensor
        VL53L1X_startContinuous(&gVL53L1X[i], 10); ///< Start continuous mode with a period of 10ms
        gVL53L1X[i].is_calibrated = true; ///< Set the flag to true to indicate that the sensor is calibrated
    }

    return true;
}

bool verify_sensors(uint32_t num_checks)
{
    ///< Count the number of times the sensors are sensing the same value
    int cnt_as5600[3] = {0};
    int cnt_vl53l1x[3] = {0};
    int cnt_bno055 = 0;

    float angle_prev[3] = {0};
    float dist_prev[3] = {0};
    float acce_prev = 0;

    float angle[3] = {0};
    float dist[3] = {0};
    float acce = 0;

    for (uint8_t i = 0; i < num_checks; i++) {
        ///< Get sensor values
        for (int j = 0; j < 3; j++) {
            angle[j] = AS5600_ADC_GetAngle(&gAS5600[j]); ///< Get the angle from the AS5600 sensor
        }

        for (int j = 0; j < 3; j++) {
            if (VL53L1X_dataReady(&gVL53L1X[j])) {            
                dist[j] = VL53L1X_readDistance(&gVL53L1X[j], false); // false for non-blocking read
            } else {
                dist[j] = dist_prev[j]; ///< If the data is not ready, use the previous value
            }
        }

        BNO055_ReadAll_Lineal(&gBNO055);
        acce = sqrt(gBNO055.ax*gBNO055.ax + gBNO055.ay*gBNO055.ay);

        ///< Print the values
        // printf("Angle: %.2f deg, Distance: %.3f m, Acce: %.2f m^2\n", angle, dist/1000.0, acce);

        ///< Check if the values are the same
        for (int j = 0; j < 3; j++) {
            if (fabs(angle[j] - angle_prev[j]) < 0.001) {
                cnt_as5600[j]++;
            }
            if (fabs(dist[j] - dist_prev[j]) < 0.001) {
                cnt_vl53l1x[j]++;
            }
        }
        if (fabs(acce - acce_prev) < 0.001) {
            cnt_bno055++;
        }

        ///< Update the previous values
        for (int j = 0; j < 3; j++) {
            angle_prev[j] = angle[j];
            dist_prev[j] = dist[j];
        }
        acce_prev = acce;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // ESP_LOGI("verify_sensors", "AS5600: %d, VL53L1X: %d, BNO055: %d", cnt_as5600, cnt_vl53l1x, cnt_bno055);

    // if (cnt_as5600 > num_checks/2 && cnt_vl53l1x > num_checks/2 && cnt_bno055 > num_checks/2) {
    //     return false;
    // }

    return true;
}

void init_system(void)
{
    ///< Initialize the system variables
    gSys.cnt_sample = 0; ///< Initialize the number of samples readed from all the sensors
    gSys.STATE = INIT_BLDC_STEP_1; ///< Initialize the state machine: initialize the BLDC motor

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

}

void sys_timer_cb(void *arg)
{
    switch(gSys.STATE)
    {
        case INIT_BLDC_STEP_1:
            ESP_LOGI("sys_timer_cb", "INIT_PWM_BLDC_STEP_1");
            stop_robot();

            ///< Use the time for the sensor sampling and control the BLDC motor
            gSys.STATE = CHECK_SENSORS;
            for (int i = 0; i < 3; i++) {
                gMotor[i].is_calibrated = true;
            }

            ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.oneshot_timer, TIME_SAMPLING_US));
            break;

        case CHECK_SENSORS:
            // Check if the sensors are calibrated
            if (is_drivers_ready()) {
                gSys.STATE = SYS_SAMPLING_EXP; ///< Set the state to sampling
                // gSys.STATE = NONE; ///< Set the state to NONE
                // ESP_ERROR_CHECK(esp_timer_stop(gSys.oneshot_timer)); ///< Stop the timer to stop the sampling
                ESP_LOGI("sys_timer_cb", "Sensors calibrated. Starting the system.");
            }
            break;

        case SYS_SAMPLING_EXP:
            BaseType_t mustYield = pdFALSE;
            vTaskNotifyGiveFromISR(gSys.task_handle_trigger, &mustYield);

            // portYIELD_FROM_ISR(mustYield); ///< Yield the task to allow the other tasks to run
            gSys.cnt_sample++; ///< Increment the number of samples readed from all the sensors
            if (gSys.cnt_sample >= NUM_SAMPLES) {
                ESP_ERROR_CHECK(esp_timer_stop(gSys.oneshot_timer)); ///< Stop the timer to stop the sampling
                // ESP_ERROR_CHECK(esp_timer_delete(gSys.oneshot_timer)); ///< Delete the timer to stop the sampling
                stop_robot();
                ESP_LOGI("sys_timer_cb", "Samples readed from all the sensors: %d", gSys.cnt_sample);
                gSys.STATE = NONE;
            }

            break;

        case SYS_SAMPLING_CONTROL:
            mustYield = pdFALSE;
            vTaskNotifyGiveFromISR(gSys.task_handle_trigger, &mustYield);
            // portYIELD_FROM_ISR(mustYield); ///< Yield the task to allow the other tasks to run

            gSys.cnt_smp_control++; ///< Increment the number of samples readed from all the sensors for control
            gSys.cnt_sample++; ///< Increment the number of samples readed from all the sensors
            if (gSys.cnt_smp_control >= NUM_SAMPLES_CONTROL) {
                ESP_ERROR_CHECK(esp_timer_stop(gSys.oneshot_timer)); ///< Stop the timer to stop the sampling
                // ESP_ERROR_CHECK(esp_timer_delete(gSys.oneshot_timer)); ///< Delete the timer to stop the sampling
                stop_robot();
                ESP_LOGI("sys_timer_cb", "Samples readed from all the sensors: %d", gSys.cnt_smp_control);
                gSys.STATE = NONE; ///< Set the state to NONE to stop the system
            }
            break;

        default:
            ESP_LOGI("sys_timer_cb", "default");
            break;
    }
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

        float value = atof(str_value)/10;
        ESP_LOGI(TAG_CMD, "value-> %.2f", (float)value);
        if (value != 0) { ///< If value=0, that means data is not a number
            for (int i = 0; i < 3; i++) {
                bldc_set_duty_motor(&gMotor[i], value); ///< Set the duty cycle of the motor
            }
        }
    }
    ///< Control ommand to set the setpoint of the motor
    else if (strcmp(cmd, "set") == 0) {
        if (gSys.STATE != NONE) { ///< If the system is not in the NONE state, that means the system is busy
            ESP_LOGI(TAG_CMD, "System is busy");
            return;
        }
        if (len_uc_data < 8) { ///< 4 for the command "set " and 4 for the setpoint
            ESP_LOGI(TAG_CMD, "Invalid SET cmd");
            return;
        }
        char str_value[len_uc_data - 4]; ///< 4 is the length of the command "set "
        strncpy(str_value, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the value after the command
        str_value[len_uc_data - 4] = '\0'; ///< Add the null terminator

        bool dir = false; ///< Direction of the motor
        int dist = 0; ///< Distance to move (cm)
        int vel = 0; ///< Velocity to move (cm/s)

        parse_command_setpoint((const uint8_t *)str_value, &dir, &dist, &vel); ///< Parse the command to get the direction, distance and velocity
        ESP_LOGI(TAG_CMD, "dir-> %d, dist-> %d, vel-> %d", dir, dist, vel);

        gSys.setpoint_dir = dir; ///< Set the direction of the motor
        if (gSys.setpoint_dir) {
            gSys.setpoint_dist = (float)dist/100; ///< Set the distance to move (m)
            gSys.setpoint_vel = (float)vel/100; ///< Set the velocity to move (m/s)
        }
        else {
            gSys.setpoint_dist = -(float)dist/100; ///< Set the distance to move (m)
            gSys.setpoint_vel = -(float)vel/100; ///< Set the velocity to move (m/s)
        }
        gSys.setpoint_dist = gSys.setpoint_dist; ///< Add the distance from the encoder to the setpoint distance
        printf("Setpoint: dir %d, dist %.3f m, vel %.3f m/s\n", gSys.setpoint_dir, gSys.setpoint_dist, gSys.setpoint_vel);

        ///< Init the control experiment
        gSys.STATE = SYS_SAMPLING_CONTROL; ///< Set the state to control the BLDC motor
        gSys.duty = 0; ///< Set the duty to 0
        gSys.cnt_smp_control = 0; ///< Set the number of samples to 0
        ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.oneshot_timer, TIME_SAMPLING_US));
    }
    else {
        ESP_LOGI(TAG_CMD, "cmd not recognized");
    }
}


void parse_command_setpoint(const uint8_t *command, bool *dir, int *dist, int *vel) {
    ///< The first character indicates the direction
    if (command[0] == 'D') {
        *dir = true;  // right
    } else if (command[0] == 'I') {
        *dir = false; // left
    } else {
        printf("Invalid command: unknown direction\n");
        return;
    }

    ///< Find the position of the '_'
    const char *underscore = strchr((const char *)command, '_');
    if (!underscore) {
        printf("Invalid command: '_' not found\n");
        return;
    }

    ///< Extract distance: from command[1] up to the character before '_'
    char distance_str[16] = {0};
    strncpy(distance_str, (const char *)&command[1], underscore - (const char *)&command[1]);
    *dist = atoi(distance_str);

    ///< Extract speed: from the character after '_'
    *vel = atoi(underscore + 1);
}

bool is_drivers_ready(void)
{
    ///< Check if the drivers and sensors are ready to be used
    if (gMotor[0].is_calibrated && gMotor[1].is_calibrated && gMotor[2].is_calibrated &&
        gAS5600[0].is_calibrated && gAS5600[1].is_calibrated && gAS5600[2].is_calibrated &&
        gBNO055.is_calibrated && gVL53L1X[0].is_calibrated && gVL53L1X[1].is_calibrated && gVL53L1X[2].is_calibrated) {
        return true;
    }
    return false;
}

void stop_robot(void)
{
    for (int i = 0; i < 3; i++) {
        bldc_set_duty_motor(&gMotor[i], 0); ///< Stop the motor
    }
}

void wrap_printf(const char *format, ...)
{
    va_list args;

    xSemaphoreTake(gSys.mtx_printf, portMAX_DELAY);

    va_start(args, format);
    vprintf(format, args); ///< Use vprintf to handle formatted output
    va_end(args);
    fflush(stdout); ///< Ensure the output is flushed to the console

    xSemaphoreGive(gSys.mtx_printf); ///< Release the mutex after printing
}

void motor_identification(uint8_t motor_num, uint8_t as5600_num)
{    
    wrap_printf("Starting motor identification...\n");

    ///< Get the motor and AS5600 sensor from the arguments
    bldc_pwm_motor_t *motor = &gMotor[motor_num];
    AS5600_t *as5600 = &gAS5600[as5600_num];

    wrap_printf("Creating timer...\n");

    ///< Create periodic timer for motor identification
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &motor_ident_cb,
        .arg = NULL, ////< argument specified here will be passed to timer callback function
        .name = "bldc-ident" ///< name is optional, but may help identify the timer when debugging
    };
    esp_timer_handle_t oneshot_timer_s;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer_s));
    gSys.timer_bldc = oneshot_timer_s;
    ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.timer_bldc, TIME_SAMP_MOTOR_US));

    ///< Experiment
    char (*data)[15] = calloc(TIME_SAMP_MOTOR_S * SAMP_RATE_MOTOR_HZ + 1, sizeof(*data));
    int cnt = 0; ///< Counter to store the number of samples

    while (cnt < TIME_SAMP_MOTOR_S * SAMP_RATE_MOTOR_HZ) {
        xSemaphoreTake(gSys.smph_bldc, portMAX_DELAY); ///< Wait for the semaphore to be given by the timer callback

        ///< Get the angle from the AS5600 sensor
        float angle = AS5600_ADC_GetAngle(as5600) - as5600->angle_offset;

        ///< Set a duty cycle to the motor
        int duty;
        if (cnt < 0.5 * SAMP_RATE_MOTOR_HZ) {
            duty = 7;
        } else if (cnt < 1.5 * SAMP_RATE_MOTOR_HZ) {
            duty = 20;
        } else if (cnt < 2.5 * SAMP_RATE_MOTOR_HZ) {
            duty = 30;
        } else if (cnt < 3 * SAMP_RATE_MOTOR_HZ) {
            duty = 0;
        } else if (cnt < 3.5 * SAMP_RATE_MOTOR_HZ) {
            duty = -7;
        } else if (cnt < 4.5 * SAMP_RATE_MOTOR_HZ) {
            duty = -20;
        } else if (cnt < 5 * SAMP_RATE_MOTOR_HZ) {
            duty = -30;
        } else {
            ESP_ERROR_CHECK(esp_timer_stop(gSys.timer_bldc)); ///< Stop the timer to stop the experiment
            break; ///< Stop the experiment after 5 seconds
        }
        bldc_set_duty_motor(motor, (float)duty);

        ///< Store the duty cycle and angle in the data buffer
        snprintf(data[cnt], sizeof(data[cnt]), "%d, %.2f", (int)duty, angle); ///< Store the data in the buffer
        cnt++; ///< Increment the counter
    }

    ///< Print the data to the console
    wrap_printf("Duty Cycle (0-100): Angle (rad)\n");
    for (uint32_t i = 0; i <= cnt; i++) {
        wrap_printf("%s\n", data[i]);
        if (i % 100 == 0) {
            vTaskDelay(pdMS_TO_TICKS(100)); ///< Delay to avoid flooding the console
        }
    }
    if (data != NULL) {
        free(data);
        data = NULL;  // avoid using dangling pointers
    }
}

void motor_identification_all(void)
{
    wrap_printf("Starting motor identification of all motors one by one...\n");

    const int num_motors = 3;
    const int num_samples = TIME_SAMP_MOTOR_S * SAMP_RATE_MOTOR_HZ;
    
    ///< Estructura: fila = muestra, columnas = duty, ángulos de motor 1-2-3
    typedef struct {
        int duty;
        float angle[3];  // M1, M2, M3
    } motor_data_t;

    motor_data_t* data = calloc(num_samples, sizeof(motor_data_t));
    if (data == NULL) {
        wrap_printf("Error: no se pudo asignar memoria.\n");
        return;
    }

    ///< Crear el timer de forma general, se usará para todos los motores
    const esp_timer_create_args_t timer_args = {
        .callback = &motor_ident_cb,
        .arg = NULL,
        .name = "bldc-ident"
    };
    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    gSys.timer_bldc = timer_handle;

    for (int motor_index = 0; motor_index < num_motors; motor_index++) {
        wrap_printf("\n⚙️  Identificando motor %d...\n", motor_index + 1);

        bldc_pwm_motor_t *motor = &gMotor[motor_index];
        AS5600_t *as5600 = &gAS5600[motor_index];
        bldc_set_duty_motor(motor, 0);

        ///< Iniciar timer periódico
        ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.timer_bldc, TIME_SAMP_MOTOR_US));

        for (int cnt = 0; cnt < num_samples; cnt++) {
            xSemaphoreTake(gSys.smph_bldc, portMAX_DELAY);

            int duty;
            if (cnt < 0.5 * SAMP_RATE_MOTOR_HZ) {
                duty = 7;
            } else if (cnt < 1.5 * SAMP_RATE_MOTOR_HZ) {
                duty = 20;
            } else if (cnt < 2.5 * SAMP_RATE_MOTOR_HZ) {
                duty = 30;
            } else if (cnt < 3 * SAMP_RATE_MOTOR_HZ) {
                duty = 0;
            } else if (cnt < 3.5 * SAMP_RATE_MOTOR_HZ) {
                duty = -7;
            } else if (cnt < 4.5 * SAMP_RATE_MOTOR_HZ) {
                duty = -20;
            } else if (cnt < 5 * SAMP_RATE_MOTOR_HZ) {
                duty = -30;
            } else {
                break;
            }

            bldc_set_duty_motor(motor, (float)duty);

            if (motor_index == 0) {
                data[cnt].duty = duty;  // Solo se asigna una vez en la primera ronda
            }

            float angle = AS5600_ADC_GetAngle(as5600) - as5600->angle_offset;
            data[cnt].angle[motor_index] = angle;
        }

        ESP_ERROR_CHECK(esp_timer_stop(gSys.timer_bldc));
        bldc_set_duty_motor(motor, 0);  // Seguridad: detener el motor al terminar
    }

    ///< Imprimir tabla
    wrap_printf("\n Resultados:\n");
    wrap_printf("Duty(%%)\tAngle_M1\tAngle_M2\tAngle_M3\n");
    for (int i = 0; i < num_samples; i++) {
        wrap_printf("%d\t\t%.2f\t\t%.2f\t\t%.2f\n",
                    data[i].duty,
                    data[i].angle[0],
                    data[i].angle[1],
                    data[i].angle[2]);

        if (i % 100 == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    free(data);
}


void motor_ident_cb(void *arg)
{
    BaseType_t mustYield = pdFALSE;
    xSemaphoreGiveFromISR(gSys.smph_bldc, &mustYield); 
}
