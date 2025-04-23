/**
 * \file        apds9960_lib.h
 * \brief       APDS9960 library
 * \details     Apds9960 basic driver functions to use Color and Als
 * 
 * 
 * \author      Striker2
 * \version     0.0.1
 * \date        02/04/20245
 * \copyright   Unlicensed
 */

 #ifndef __apds9960_H__
 #define __apds9960_H__
 
 #include <stdint.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdio.h>
 
 #include "apds9960_defs.h"
 #include "platform_esp32s3.h"
 
 #ifdef __cplusplus
 extern "C"{
 #endif
 
 /**
   * @brief apds9960 handle structure definition
   */
  typedef struct apds9960_handle_s
  {
      i2c_t i2c_handle;   ///< I2C handle for the AS5600 sensor
      APDS9960_reg_t reg;
  }APDS9960_t;
 
  /**
   * @brief apds9960 information structure definition
   */
  typedef struct apds9960_info_s
  {
      char chip_name[32];                /**< chip name */
      char manufacturer_name[32];        /**< manufacturer name */
      char interface[8];                 /**< chip interface name */
      float supply_voltage_min_v;        /**< chip min supply voltage */
      float supply_voltage_max_v;        /**< chip max supply voltage */
      float max_current_ma;              /**< chip max current */
      float temperature_min;             /**< chip min operating temperature */
      float temperature_max;             /**< chip max operating temperature */
      uint32_t driver_version;           /**< driver version */
  } APDS9960_info_t;
 
  /**
   * @brief      get chip's information
   * @param[out] *info pointer to an apds9960 info structure
   * @note       none
   */
  void APDS9960_Info(APDS9960_info_t *info);
 
  /**
   * @brief     initialize the chip
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] i2c_num i2c port number
   * @param[in] gpio_scl gpio number for scl
   * @param[in] gpio_sda gpio number for sda
   * @note      none
   */
  void APDS9960_Init(APDS9960_t *handle, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda);
 
  /**
   * @brief     close the chip
   * @param[in] *handle pointer to an apds9960 handle structure
   * @note      none
   */
  void APDS9960_Deinit(APDS9960_t *handle);
 
 /**
   * @brief      read the rgbc data
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *red pointer to a red buffer
   * @param[out] *green pointer to a green buffer
   * @param[out] *blue pointer to a blue buffer
   * @param[out] *clear pointer to a clear buffer
   * @note       none
   */
  void APDS9960_Configure(APDS9960_t *handle);
  
 /**
  * @brief 
  * 
  * @param[in]  *handle pointer to an apds9960 handle structure
  * @param[in] *red pointer to a red buffer
  * @param[in] *green pointer to a green buffer
  * @param[in] *blue pointer to a blue buffer
  * @param[in] *clear pointer to a clear buffer
  */

  void APDS9960_Read_Rgbc(APDS9960_t *handle, uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear);
 
 #ifdef __cplusplus
  }
 #endif
 
 #endif // __APDS9960_H__
 