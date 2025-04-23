#include "apds9960_lib.h"
#include <stdlib.h>

/**
  * @brief      read bytes
  * @param[in]  *handle pointer to an apds9960 handle structure
  * @param[in]  reg iic register address
  * @param[out] *data pointer to a data buffer
  * @param[in]  len data length
  * @return     status code
  *             - 0 success
  *             - 1 read failed
  * @note       none
  */

void APDS9960_ReadReg(APDS9960_t *handle, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (!APDS9960_IsValidReadReg(handle, reg)) {
        printf("Invalid register");
        return;
    }

    i2c_read_reg(&handle->i2c_handle, reg, *data, 1);

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

bool APDS9960_IsValidReadReg(APDS9960_t *handle, uint8_t reg)
{
    if (reg == APDS9960_REG_ENABLE || reg == APDS9960_REG_AIHTH || reg == APDS9960_REG_STATUS || 
        reg == APDS9960_REG_ATIME || reg == APDS9960_REG_PERS || reg == APDS9960_REG_CDATAL || 
        reg == APDS9960_REG_WTIME || reg == APDS9960_REG_CONFIG1 || reg == APDS9960_REG_CDATAH || 
        reg == APDS9960_REG_AILTL || reg == APDS9960_REG_CONTROL || reg == APDS9960_REG_RDATAL || 
        reg == APDS9960_REG_AILTH || reg == APDS9960_REG_CONFIG2 || reg == APDS9960_REG_RDATAH || 
        reg == APDS9960_REG_AIHTL || reg == APDS9960_REG_ID || reg == APDS9960_REG_GDATAL || 
        reg == APDS9960_REG_GDATAH || reg == APDS9960_REG_BDATAL || reg == APDS9960_REG_BDATAH || 
        reg == APDS9960_REG_CONFIG3 || reg == APDS9960_REG_IFORCE || reg == APDS9960_REG_CICLEAR || 
        reg == APDS9960_REG_AICLEAR)
    {
        return true;
    }
    return false;
}

 void APDS9960_WriteReg(APDS9960_t *handle, uint8_t reg, uint8_t *data)
 {
     if (!APDS9960_IsValidWriteReg(handle, reg)) {
         printf("Invalid register");
         return;
     }
     i2c_write_reg(&handle->i2c_handle, reg, (uint8_t *)data, 1);

    //  ///< Write 1 byte for BURN
    //  if (reg == AS5600_REG_BURN) {
    //      i2c_write_reg(&as5600->i2c_handle, reg, (uint8_t *)&data, 1);
    //  }
    //  ///< Write 2 bytes for the rest of the writeable registers
    //  else {
    //      uint8_t write_buffer[] = {data >> 8, data};
    //      i2c_write_reg(&as5600->i2c_handle, reg, write_buffer, 2);
    //  }
 }

 bool APDS9960_IsValidWriteReg(APDS9960_t *handle, uint8_t reg)
 {
     if (reg == APDS9960_REG_ENABLE || reg == APDS9960_REG_AIHTH || reg == APDS9960_REG_STATUS || 
        reg == APDS9960_REG_ATIME || reg == APDS9960_REG_PERS || reg == APDS9960_REG_CDATAL || 
        reg == APDS9960_REG_WTIME || reg == APDS9960_REG_CONFIG1 || reg == APDS9960_REG_CDATAH || 
        reg == APDS9960_REG_AILTL || reg == APDS9960_REG_CONTROL || reg == APDS9960_REG_RDATAL || 
        reg == APDS9960_REG_AILTH || reg == APDS9960_REG_CONFIG2 || reg == APDS9960_REG_RDATAH || 
        reg == APDS9960_REG_AIHTL || reg == APDS9960_REG_ID || reg == APDS9960_REG_GDATAL || 
        reg == APDS9960_REG_GDATAH || reg == APDS9960_REG_BDATAL || reg == APDS9960_REG_BDATAH || 
        reg == APDS9960_REG_CONFIG3 || reg == APDS9960_REG_IFORCE || reg == APDS9960_REG_CICLEAR || 
        reg == APDS9960_REG_AICLEAR)
     {
         return true;
     }
     return false;
 }

 uint8_t APDS9960_Init(APDS9960_t *handle)
 {
     uint8_t id;
     
     if (handle == NULL)                                                      /* check handle */
     {
         return 2;                                                            /* return error */
     }
     if (handle->debug_print == NULL)                                         /* check debug_print */
     {
         return 3;                                                            /* return error */
     }

     //I2C??

     if (handle->delay_ms == NULL)                                            /* check delay_ms */
     {
         handle->debug_print("apds9960: delay_ms is null.\n");                /* delay_ms is null */
         
         return 3;                                                            /* return error */
     }
     if (handle->receive_callback == NULL)                                    /* check receive_callback */
     {
         handle->debug_print("apds9960: receive_callback is null.\n");        /* receive_callback is null */
         
         return 3;                                                            /* return error */
     }
     APDS9960_ReadReg(handle, APDS9960_REG_ID, (uint8_t *)&id);
     if (id != 0xAB)                                                          /* check id */
     {
         handle->debug_print("apds9960: id is invalid.\n");                   /* id is invalid */
         i2c_deinit(&handle->i2c_handle);                                          /* iic deinit */
         
         return 5;                                                            /* return error */
     }
     handle->inited = 1;                                                      /* flag inited */
     
     return 0;                                                                /* success return 0 */
 }

 uint8_t APDS9960_Deinit(APDS9960_t *handle)
 {
     uint8_t res, prev;
     
     if (handle == NULL)                                                                 /* check handle */
     {
         return 2;                                                                       /* return error */
     }
     if (handle->inited != 1)                                                            /* check handle initialization */
     {
         return 3;                                                                       /* return error */
     }
     
     APDS9960_ReadReg(handle, APDS9960_REG_ENABLE, (uint8_t *)&prev, 1);        /* read enable register */
     if (res != 0)                                                                       /* check the result */
     {
         handle->debug_print("apds9960: read enable register failed.\n");                /* read enable register failed */
         
         return 4;                                                                       /* return error */
     }
     prev &= ~(1 << 0);                                                                  /* set power down */
     res = a_apds9960_iic_write(handle, APDS9960_REG_ENABLE, (uint8_t *)&prev, 1);       /* write enable register */
     if (res != 0)                                                                       /* check the result */
     {
         handle->debug_print("apds9960: write enable register failed.\n");               /* write enable register failed */
         
         return 4;                                                                       /* return error */
     }
     res = handle->iic_deinit();                                                         /* iic deinit */
     if (res != 0)                                                                       /* check the result */
     {
         handle->debug_print("apds9960: iic deinit failed.\n");                          /* iic deinit failed */
         
         return 1;                                                                       /* return error */
     }
     else
     {
         handle->inited = 0;                                                             /* flag closed */
         
         return 0;                                                                       /* success return 0 */
     }
 }
