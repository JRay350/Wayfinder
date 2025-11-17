/*
 * lps22hh_platform.h
 *
 *  Created on: Nov 16, 2025
 *      Author: jacob
 */

#ifndef LPS22HH_PLATFORM_H
#define LPS22HH_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"   // or your MCU series header
#include "lps22hh.h"         // ST driver header

#define LPS22HH_SPI_HANDLE     hspi1     // <-- change to SPI2 or SPI3 if needed

#define LPS22HH_CS_PORT        GPIOA     // <-- change to your CS pin port
#define LPS22HH_CS_PIN         GPIO_PIN_4 // <-- change to your CS pin number


int32_t LPS22HH_BusIO_Register_SPI(LPS22HH_Object_t *pObj);

/* SPI interface */
int32_t LPS22HH_SPI_Init(void);
int32_t LPS22HH_SPI_DeInit(void);

int32_t LPS22HH_SPI_ReadReg (uint16_t devAddr, uint16_t reg,
                             uint8_t *pData, uint16_t length);
int32_t LPS22HH_SPI_WriteReg(uint16_t devAddr, uint16_t reg,
                             uint8_t *pData, uint16_t length);

/* Timing helpers */
int32_t LPS22HH_GetTick(void);
void    LPS22HH_Delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // LPS22HH_PLATFORM_H
