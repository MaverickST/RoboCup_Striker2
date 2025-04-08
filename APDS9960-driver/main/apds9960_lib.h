/**
 * \file        apds9960_lib.h
 * \brief       APDS9960 library
 * \details
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
      i2c_t i2c_handle;
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
   * @defgroup apds9960_basic_driver apds9960 basic driver function
   * @brief    apds9960 basic driver modules
   * @ingroup  apds9960_driver
   * @{
   */
  
  /**
   * @brief      get chip's information
   * @param[out] *info pointer to an apds9960 info structure
   * @return     status code
   *             - 0 success
   *             - 2 handle is NULL
   * @note       none
   */
  void APDS9960_Info(APDS9960_info_t *info);
 
 /**
   * @brief     irq handler
   * @param[in] *handle pointer to an apds9960 handle structure
   * @return    status code
   *            - 0 success
   *            - 1 run failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Irq_Handler(APDS9960_t *handle);
 
  /**
   * @brief     initialize the chip
   * @param[in] *handle pointer to an apds9960 handle structure
   * @return    status code
   *            - 0 success
   *            - 1 i2c initialization failed
   *            - 2 handle is NULL
   *            - 3 linked functions is NULL
   *            - 4 read id failed
   *            - 5 id is invalid
   * @note      none
   */
  void APDS9960_Init(APDS9960_t *handle, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda);
 
  /**
   * @brief     close the chip
   * @param[in] *handle pointer to an apds9960 handle structure
   * @return    status code
   *            - 0 success
   *            - 1 iic deinit failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   *            - 4 power down failed
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
   * @return     status code
   *             - 0 success
   *             - 1 read rgbc failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Configure(APDS9960_t *handle);
  
  void APDS9960_Read_Rgbc(APDS9960_t *handle, uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear);
 
 /**
   * @brief     set the configuration
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] conf chip configuration
   * @param[in] enable bool value
   * @return    status code
   *            - 0 success
   *            - 1 set conf failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Conf(APDS9960_t *handle, APDS9960_conf_t conf, APDS9960_bool_t enable);
  
 /**
   * @brief      get the configuration
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[in]  conf chip configuration
   * @param[out] *enable pointer to a bool value buffer
   * @return     status code
   *             - 0 success
   *             - 1 get conf failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Conf(APDS9960_t *handle, APDS9960_conf_t conf, APDS9960_bool_t *enable);
  
 /**
   * @brief     set the adc integration time
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] integration_time adc integration time
   * @return    status code
   *            - 0 success
   *            - 1 set adc integration time failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Adc_Integration_Time(APDS9960_t *handle, uint8_t integration_time);
  
 /**
   * @brief      get the adc integration time
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *integration_time pointer to an adc integration time buffer
   * @return     status code
   *             - 0 success
   *             - 1 get adc integration time failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Adc_Integration_Time(APDS9960_t *handle, uint8_t integration_time);
  
 /**
   * @brief      convert the adc integration time to the register raw data
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[in]  ms integration time
   * @param[out] *reg pointer to a register raw buffer
   * @return     status code
   *             - 0 success
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Adc_Integration_Time_Convert_To_Register(APDS9960_t *handle, float ms, uint8_t *reg);
 
  /**
   * @brief      convert the register raw data to the integration time
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[in]  reg register raw data
   * @param[out] *ms pointer to an adc integration time buffer
   * @return     status code
   *             - 0 success
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Adc_Integration_Time_Convert_To_Data(APDS9960_t *handle, uint8_t reg, float *ms);
  
 /**
   * @brief     set the wait time
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] wait_time wait time
   * @return    status code
   *            - 0 success
   *            - 1 set wait time failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Wait_Time(APDS9960_t *handle, uint8_t wait_time);
  
 /**
   * @brief      get the wait time
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *wait_time pointer to a wait time buffer
   * @return     status code
   *             - 0 success
   *             - 1 get wait time failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Wait_Time(APDS9960_t *handle, uint8_t wait_time);
 
 /**
   * @brief      convert the wait time to the register raw data
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[in]  ms wait time
   * @param[out] *reg pointer to a register raw buffer
   * @return     status code
   *             - 0 success
   *             - 1 get configuration register 1 failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Wait_Time_Convert_To_Register(APDS9960_t *handle, float ms, uint8_t *reg);
 
 /**
   * @brief      convert the register raw data to the wait time
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[in]  reg register raw data
   * @param[out] *ms pointer to a wait time buffer
   * @return     status code
   *             - 0 success
   *             - 1 get configuration register 1 failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Wait_Time_Convert_To_Data(APDS9960_t *handle, uint8_t reg, float *ms);
 
 /**
   * @brief     set the als interrupt low threshold
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] threshold low threshold
   * @return    status code
   *            - 0 success
   *            - 1 set als interrupt low threshold failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Als_Interrupt_Low_Threshold(APDS9960_t *handle, uint16_t threshold);
 
 /**
   * @brief      get the als interrupt low threshold
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *threshold pointer to a low threshold buffer
   * @return     status code
   *             - 0 success
   *             - 1 get als interrupt low threshold failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Als_Interrupt_Low_Threshold(APDS9960_t *handle, uint16_t *threshold);
 
 /**
   * @brief     set the als interrupt high threshold
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] threshold high threshold
   * @return    status code
   *            - 0 success
   *            - 1 set als interrupt high threshold failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Als_Interrupt_High_Threshold(APDS9960_t *handle, uint16_t threshold);
  
  /**
   * @brief      get the als interrupt high threshold
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *threshold pointer to a high threshold buffer
   * @return     status code
   *             - 0 success
   *             - 1 get als interrupt high threshold failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Als_Interrupt_High_Threshold(APDS9960_t *handle, uint16_t *threshold);
 
 /**
   * @brief     set the als interrupt cycle
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] cycle als interrupt cycle
   * @return    status code
   *            - 0 success
   *            - 1 set als interrupt cycle failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Als_Interrupt_Cycle(APDS9960_t *handle, APDS9960_als_interrupt_cycle_t cycle);
  
 /**
   * @brief      get the als interrupt cycle
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *cycle pointer to an als interrupt cycle buffer
   * @return     status code
   *             - 0 success
   *             - 1 get als interrupt cycle failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Als_Interrupt_Cycle(APDS9960_t *handle, APDS9960_als_interrupt_cycle_t *cycle);
 
 /**
   * @brief     enable or disable the wait long
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] enable bool value
   * @return    status code
   *            - 0 success
   *            - 1 set wait long failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Wait_Long(APDS9960_t *handle, APDS9960_bool_t enable);
 
 /**
   * @brief      get the wait long status
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *enable pointer to a bool value buffer
   * @return     status code
   *             - 0 success
   *             - 1 get wait long failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Wait_Long(APDS9960_t *handle, APDS9960_bool_t *enable);
 
 /**
   * @brief     set the led current
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] current led current
   * @return    status code
   *            - 0 success
   *            - 1 set led current failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Led_Current(APDS9960_t *handle, APDS9960_led_current_t current);
 
 /**
   * @brief      get the led current
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *current pointer to a led current buffer
   * @return     status code
   *             - 0 success
   *             - 1 get led current failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Led_Current(APDS9960_t *handle, APDS9960_led_current_t *current);
 
 /**
   * @brief     set the als color gain
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] gain als color gain
   * @return    status code
   *            - 0 success
   *            - 1 set als color gain failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Als_Color_Gain(APDS9960_t *handle, APDS9960_als_color_gain_t gain);
 
 /**
   * @brief      get the als color gain
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *gain pointer to an als color gain buffer
   * @return     status code
   *             - 0 success
   *             - 1 get als color gain failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Als_Color_Gain(APDS9960_t *handle, APDS9960_als_color_gain_t *gain);
  
 /**
   * @brief     set the saturation interrupt
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] saturation saturation interrupt type
   * @param[in] enable bool value
   * @return    status code
   *            - 0 success
   *            - 1 set saturation interrupt failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Saturation_Interrupt(APDS9960_t *handle, APDS9960_saturation_interrupt_t saturation, APDS9960_bool_t enable);
 
 /**
   * @brief      get the saturation interrupt
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[in]  saturation saturation interrupt type
   * @param[out] *enable pointer to a bool value buffer
   * @return     status code
   *             - 0 success
   *             - 1 get saturation interrupt failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Saturation_Interrupt(APDS9960_t *handle, APDS9960_saturation_interrupt_t saturation, APDS9960_bool_t *enable);
  
 /**
   * @brief      get the status
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *status pointer to a status buffer
   * @return     status code
   *             - 0 success
   *             - 1 get status failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Status(APDS9960_t *handle, void *status);
  
 /**
   * @brief     enable or disable sleeping after interrupt
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] enable bool value
   * @return    status code
   *            - 0 success
   *            - 1 set sleep after interrupt failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Sleep_After_Interrupt(APDS9960_t *handle, APDS9960_bool_t enable);
 
 /**
   * @brief      get the sleeping after interrupt status
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[out] *enable pointer to a bool value buffer
   * @return     status code
   *             - 0 success
   *             - 1 get sleep after interrupt failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Sleep_After_Interrupt(APDS9960_t *handle, APDS9960_bool_t *enable);
  
 /**
   * @brief     force an interrupt
   * @param[in] *handle pointer to an apds9960 handle structure
   * @return    status code
   *            - 0 success
   *            - 1 force interrupt failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_force_interrupt(APDS9960_t *handle);
  
 /**
   * @brief     clear the als interrupt
   * @param[in] *handle pointer to an apds9960 handle structure
   * @return    status code
   *            - 0 success
   *            - 1 als interrupt clear failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Als_Interrupt_Clear(APDS9960_t *handle);
 
 /**
   * @brief     clear the all not gesture interrupt
   * @param[in] *handle pointer to an apds9960 handle structure
   * @return    status code
   *            - 0 success
   *            - 1 all non gesture interrupt clear failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_All_Non_Gesture_Interrupt_Clear(APDS9960_t *handle);
  
 /**
   * @brief     set the chip register
   * @param[in] *handle pointer to an apds9960 handle structure
   * @param[in] reg iic register address
   * @param[in] *buf pointer to a data buffer
   * @param[in] len data buffer length
   * @return    status code
   *            - 0 success
   *            - 1 write failed
   *            - 2 handle is NULL
   *            - 3 handle is not initialized
   * @note      none
   */
  void APDS9960_Set_Reg(APDS9960_t *handle, uint8_t reg, uint8_t *buf, uint16_t len);
 
 /**
   * @brief      get the chip register
   * @param[in]  *handle pointer to an apds9960 handle structure
   * @param[in]  reg iic register address
   * @param[out] *buf pointer to a data buffer
   * @param[in]  len data buffer length
   * @return     status code
   *             - 0 success
   *             - 1 read failed
   *             - 2 handle is NULL
   *             - 3 handle is not initialized
   * @note       none
   */
  void APDS9960_Get_Reg(APDS9960_t *handle, uint8_t reg, uint8_t *buf, uint16_t len);
  
 
 
 
 
 
 // typedef struct {
     
 // } APDS9960_t;
 
 #ifdef __cplusplus
  }
 #endif
 
 #endif // __APDS9960_H__
 