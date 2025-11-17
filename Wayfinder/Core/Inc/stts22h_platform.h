/*
 * stts22h_platform.h
 *
 *  Created on: Nov 17, 2025
 *      Author: jacob
 */

#ifndef INC_STTS22H_PLATFORM_H_
#define INC_STTS22H_PLATFORM_H_

#include <stm32l0xx_hal.h>   // or your MCU series header
#include <stts22h.h>         // ST driver header

#define STTS22H_I2C_HANDLE     hi2c2

#define STTS22H_I2C_ADDR	   STTS22H_I2C_ADD_L

int32_t STTS22H_BusIO_Register_I2C(STTS22H_Object_t *pObj);

/* I2C interface */
int32_t STTS22H_I2C_Init(void);
int32_t STTS22H_I2C_DeInit(void);

int32_t STTS22H_I2C_ReadReg (uint16_t devAddr, uint16_t reg,
                             uint8_t *pData, uint16_t length);
int32_t STTS22H_I2C_WriteReg(uint16_t devAddr, uint16_t reg,
                             uint8_t *pData, uint16_t length);

/* Timing helpers */
int32_t STTS22H_GetTick_Platform(void);
void    STTS22H_Delay_Platform(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_STTS22H_PLATFORM_H_ */
