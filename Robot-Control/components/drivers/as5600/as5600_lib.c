#include "as5600_lib.h"

void AS5600_Init(AS5600_t *as5600, uint8_t i2c_num, uint8_t scl, uint8_t sda, uint8_t out)
{
    as5600->out = out; // Set the GPIO pin connected to the OUT pin of the AS5600 sensor
    as5600->i2c_handle.i2c_num = i2c_num; // Set the I2C port number
    as5600->i2c_handle.gpio_scl = scl; // Set the SCL GPIO pin
    as5600->i2c_handle.gpio_sda = sda; // Set the SDA GPIO pin
}

void AS5600_Deinit(AS5600_t *as5600)
{
    i2c_deinit(&as5600->i2c_handle);
    adc_deinit(&as5600->adc_handle);
    gpio_deinit(&as5600->gpio_handle);
}

void AS5600_InitI2C(AS5600_t *as5600, uint8_t i2c_num, uint8_t scl, uint8_t sda)
{
    //  I2C master configuration 
    if (!i2c_init(&as5600->i2c_handle, i2c_num, scl, sda, I2C_MASTER_FREQ_HZ, AS5600_SENSOR_ADDR)) {
        printf("AS5600: I2C initialization failed\n");
        return;
    }

    // i2c_init_new_bus(&as5600->i2c_handle, i2c_num, scl, sda);
    // i2c_init_new_device(&as5600->i2c_handle, i2c_num, AS5600_SENSOR_ADDR, I2C_MASTER_FREQ_HZ);
}

void AS5600_DeinitI2C(AS5600_t *as5600)
{
    i2c_deinit(&as5600->i2c_handle);
}

bool AS5600_Calibrate(AS5600_t *as5600, AS5600_config_t conf, uint16_t start_position, uint16_t stop_position)
{

    ///< Initialize the I2C driver
    AS5600_InitI2C(as5600, as5600->i2c_handle.i2c_num, as5600->i2c_handle.gpio_scl, as5600->i2c_handle.gpio_sda);

    ///< Set configuration, start and stop positions
    // AS5600_SetConf(as5600, conf);
    // AS5600_SetStartPosition(as5600, start_position); 
    // AS5600_SetStopPosition(as5600, stop_position);

    ///< Read the configuration, start and stop positions to verify
    AS5600_config_t read_conf;
    uint16_t start_pos, stop_pos;
    AS5600_GetConf(as5600, &read_conf);
    AS5600_GetStartPosition(as5600, &start_pos);
    AS5600_GetStopPosition(as5600, &stop_pos);
    
    printf("Config: 0x%04x Start Position: 0x%03X, Stop Position: 0x%03X\n", read_conf.WORD, start_pos, stop_pos);

    assert(read_conf.WORD == conf.WORD); ///< Assert that the configuration is correct
    assert(start_pos == start_position); ///< Assert that the start position is 0
    assert(stop_pos == stop_position); ///< Assert that the stop position is 4095

    // ///< Burn the angle command and settings to the EEPROM
    // AS5600_BurnAngleCommand(as5600);
    // AS5600_BurnSettingCommand(as5600);

    if (read_conf.WORD == conf.WORD && start_pos == start_position && stop_pos == stop_position) {
        as5600->is_calibrated = true; ///< Set the flag to true if the configuration is correct
        printf("AS5600 sensor calibrated successfully\n");
    } else {
        as5600->is_calibrated = false; ///< Set the flag to false if the configuration is not correct
    }
    AS5600_DeinitI2C(as5600); ///< Deinitialize the AS5600 sensor

    return true;
}

float AS5600_ADC_GetAngle(AS5600_t *as5600)
{
    float angle;
    if (as5600->adc_handle.is_calibrated && as5600->conf.OUTS == AS5600_OUTPUT_STAGE_ANALOG_RR) {
        uint16_t voltage;
        adc_read_mvolt(&as5600->adc_handle, &voltage);
        voltage = LIMIT(voltage, VCC_3V3_MIN_RR_MV, VCC_3V3_MAX_RR_MV); // The OUT pin of the AS5600 sensor has a range of 10%-90% of VCC
        angle = MAP((float)voltage, VCC_3V3_MIN_RR_MV, VCC_3V3_MAX_RR_MV, 0, 6.2831853); // Map the voltage to the angle in radians
    }
    else {
        return -1;
    }
    as5600->angle = angle;
    return angle; // Subtract the angle offset
}

void AS5600_BurnAngleCommand(AS5600_t *as5600)
{
    uint8_t data = AS5600_BURN_MODE_BURN_ANGLE;
    i2c_write_reg(&as5600->i2c_handle, AS5600_REG_BURN, (uint8_t *)&data, 1);
}

void AS5600_BurnSettingCommand(AS5600_t *as5600)
{
    uint8_t data = AS5600_BURN_MODE_BURN_SETTING;
    i2c_write_reg(&as5600->i2c_handle, AS5600_REG_BURN, (uint8_t *)&data, 1);
}

AS5600_reg_t AS5600_RegStrToAddr(AS5600_t *as5600, const char *reg_str)
{
    if (strcmp(reg_str, "zmco") == 0) {
        as5600->reg = AS5600_REG_ZMCO;
    }
    else if (strcmp(reg_str, "zpos") == 0) {
        as5600->reg = AS5600_REG_ZPOS_H;
    }
    else if (strcmp(reg_str, "mpos") == 0) {
        as5600->reg = AS5600_REG_MPOS_H;
    }
    else if (strcmp(reg_str, "mang") == 0) {
        as5600->reg = AS5600_REG_MANG_H;
    }
    else if (strcmp(reg_str, "conf") == 0) {
        as5600->reg = AS5600_REG_CONF_H;
    }
    else if (strcmp(reg_str, "stat") == 0) {
        as5600->reg = AS5600_REG_STATUS;
    }
    else if (strcmp(reg_str, "rang") == 0) {
        as5600->reg = AS5600_REG_RAW_ANGLE_H;
    }
    else if (strcmp(reg_str, "angl") == 0) {
        as5600->reg = AS5600_REG_ANGLE_H;
    }
    else if (strcmp(reg_str, "agco") == 0) {
        as5600->reg = AS5600_REG_AGC;
    }
    else if (strcmp(reg_str, "magn") == 0) {
        as5600->reg = AS5600_REG_MAGNITUDE_H;
    }
    else if (strcmp(reg_str, "burn") == 0) {
        as5600->reg = AS5600_REG_BURN;
    }
    else {
        return -1;
    }
    return as5600->reg;
}

void AS5600_ReadReg(AS5600_t *as5600, AS5600_reg_t reg, uint16_t *data)
{
    if (!AS5600_IsValidReadReg(as5600, reg)) {
        printf("Invalid register");
        return;
    }

    ///< Read 1 byte for ZMCO, STATUS, AGC
    if (reg == AS5600_REG_ZMCO || reg == AS5600_REG_STATUS || reg == AS5600_REG_AGC) {
        i2c_read_reg(&as5600->i2c_handle, reg, (uint8_t *)data, 1);
    }
    ///< Read 2 bytes for the rest of the readeable registers
    else {
        i2c_read_reg(&as5600->i2c_handle, reg, (uint8_t *)data, 2);
        *data = (*data << 8) | (*data >> 8);
    }
}

void AS5600_WriteReg(AS5600_t *as5600, AS5600_reg_t reg, uint16_t data)
{
    if (!AS5600_IsValidWriteReg(as5600, reg)) {
        printf("Invalid register");
        return;
    }
    ///< Write 1 byte for BURN
    if (reg == AS5600_REG_BURN) {
        i2c_write_reg(&as5600->i2c_handle, reg, (uint8_t *)&data, 1);
    }
    ///< Write 2 bytes for the rest of the writeable registers
    else {
        uint8_t write_buffer[] = {data >> 8, data};
        i2c_write_reg(&as5600->i2c_handle, reg, write_buffer, 2);
    }
}

bool AS5600_IsValidReadReg(AS5600_t *as5600, AS5600_reg_t reg)
{
    if (reg == AS5600_REG_ZMCO || reg == AS5600_REG_ZPOS_H || reg == AS5600_REG_ZPOS_L || 
        reg == AS5600_REG_MPOS_H || reg == AS5600_REG_MPOS_L || reg == AS5600_REG_MANG_H || 
        reg == AS5600_REG_MANG_L || reg == AS5600_REG_CONF_H || reg == AS5600_REG_CONF_L || 
        reg == AS5600_REG_STATUS || reg == AS5600_REG_RAW_ANGLE_H || reg == AS5600_REG_RAW_ANGLE_L || 
        reg == AS5600_REG_ANGLE_H || reg == AS5600_REG_ANGLE_L || reg == AS5600_REG_AGC || 
        reg == AS5600_REG_MAGNITUDE_H || reg == AS5600_REG_MAGNITUDE_L)
    {
        return true;
    }
    return false;
}

bool AS5600_IsValidWriteReg(AS5600_t *as5600, AS5600_reg_t reg)
{
    if (reg == AS5600_REG_ZPOS_H || reg == AS5600_REG_ZPOS_L || reg == AS5600_REG_MPOS_H || 
        reg == AS5600_REG_MPOS_L || reg == AS5600_REG_MANG_H || reg == AS5600_REG_MANG_L || 
        reg == AS5600_REG_CONF_H || reg == AS5600_REG_CONF_L || reg == AS5600_REG_BURN)
    {
        return true;
    }
    return false;
}

// -------------------------------------------------------------
// ------------------ GPIO and ADC FUNCTIONS -------------------
bool AS5600_InitADC(AS5600_t *as5600)
{
    // ADC pin OUT configuration 
    if (!adc_init(&as5600->adc_handle, as5600->out)) {
        printf("ADC initialization failed");
        return false;
    }
    return true;
}

void AS5600_DeinitADC(AS5600_t *as5600)
{
    adc_deinit(&as5600->adc_handle);
}

void AS5600_InitGPIO(AS5600_t *as5600)
{
    // GPIO pin OUT configuration 
    if (!gpio_init_basic(&as5600->gpio_handle, as5600->out, 2, false, false)) {
        printf("GPIO initialization failed");
        return;
    }
    gpio_set_low(&as5600->gpio_handle); // Set the GPIO to low (calibration process)
}

void AS5600_DeinitGPIO(AS5600_t *as5600)
{
    gpio_deinit(&as5600->gpio_handle);
}

void AS5600_SetGPIO(AS5600_t *as5600, uint8_t value)
{
    if (value) {
        gpio_set_high(&as5600->gpio_handle);
    }
    else {
        gpio_set_low(&as5600->gpio_handle);
    }
}

// -------------------------------------------------------------
// ---------------------- CONFIG REGISTERS ---------------------
// -------------------------------------------------------------

void AS5600_SetStartPosition(AS5600_t *as5600, uint16_t start_position)
{
    uint8_t write_buffer[] = {AS5600_REG_ZPOS_H, start_position >> 8, start_position};
    i2c_write(&as5600->i2c_handle, write_buffer, 3);
}

void AS5600_GetStartPosition(AS5600_t *as5600, uint16_t *start_position)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_ZPOS_H, (uint8_t *)start_position, 2);
    *start_position = (*start_position << 8) | (*start_position >> 8);
}

void AS5600_SetStopPosition(AS5600_t *as5600, uint16_t stop_position)
{
    uint8_t write_buffer[] = {AS5600_REG_MPOS_H, stop_position >> 8, stop_position };
    i2c_write(&as5600->i2c_handle, write_buffer, 3);
}

void AS5600_GetStopPosition(AS5600_t *as5600, uint16_t *stop_position)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_MPOS_H, (uint8_t *)stop_position, 2);
    *stop_position = (*stop_position << 8) | (*stop_position >> 8);
}

void AS5600_SetMaxAngle(AS5600_t *as5600, uint16_t max_angle)
{
    uint8_t write_buffer[] = {AS5600_REG_MANG_H, max_angle >> 8, max_angle};
    i2c_write(&as5600->i2c_handle, write_buffer, 3);
}

void AS5600_GetMaxAngle(AS5600_t *as5600, uint16_t *max_angle)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_MANG_H, (uint8_t *)max_angle, 2);
    *max_angle = (*max_angle << 8) | (*max_angle >> 8);
}

void AS5600_SetConf(AS5600_t *as5600, AS5600_config_t conf)
{
    as5600->conf = conf;
    uint8_t write_buffer[] = {AS5600_REG_CONF_H, conf.WORD >> 8, conf.WORD};
    i2c_write(&as5600->i2c_handle, write_buffer, 3);
}

void AS5600_GetConf(AS5600_t *as5600, AS5600_config_t *conf)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_CONF_H, (uint8_t *)&conf->WORD, 2);    
    conf->WORD = (conf->WORD << 8) | (conf->WORD >> 8);
}

// -------------------------------------------------------------
// ---------------------- OUTPUT REGISTERS ---------------------
// -------------------------------------------------------------

void AS5600_GetRawAngle(AS5600_t *as5600, uint16_t *raw_angle)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_RAW_ANGLE_H, (uint8_t *)raw_angle, 2);
    *raw_angle = (*raw_angle << 8) | (*raw_angle >> 8);
}

void AS5600_GetAngle(AS5600_t *as5600, uint16_t *angle)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_ANGLE_H, (uint8_t *)angle, 2);
    *angle = (*angle << 8) | (*angle >> 8);
}

// -------------------------------------------------------------
// ---------------------- STATUS REGISTERS ---------------------
// -------------------------------------------------------------

void AS5600_GetStatus(AS5600_t *as5600, uint8_t *status)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_STATUS, status, 1);
}

void AS5600_GetAgc(AS5600_t *as5600, uint8_t *agc)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_AGC, agc, 1);
}

void AS5600_GetMagnitude(AS5600_t *as5600, uint16_t *magnitude)
{
    i2c_read_reg(&as5600->i2c_handle, AS5600_REG_MAGNITUDE_H, (uint8_t *)magnitude, 2);
    *magnitude = (*magnitude << 8) | (*magnitude >> 8);
}