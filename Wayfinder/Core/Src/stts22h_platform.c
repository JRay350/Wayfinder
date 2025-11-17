/*
 * stts22h_platform.c
 *
 *  Created on: Nov 17, 2025
 *      Author: jacob
 */

#include "stts22h_platform.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

/* I2C interface */
int32_t STTS22H_I2C_Init(void) {
	// Handled by MX_I2C2_Init()
	return STTS22H_OK;
}

int32_t STTS22H_I2C_DeInit(void) {
	HAL_I2C_DeInit(&STTS22H_I2C_HANDLE);
	return STTS22H_OK;
}

int32_t STTS22H_I2C_ReadReg (uint16_t devAddr, uint16_t reg,
                             uint8_t *pData, uint16_t length) {
	uint16_t addr = (uint16_t)(devAddr);

	if (HAL_I2C_Mem_Read(&STTS22H_I2C_HANDLE,
						 addr,					// 8-bit device addr
						 (uint16_t) reg,		// register address
						 I2C_MEMADD_SIZE_8BIT,	// 8-bit sub-address
						 pData,
						 length,
						 HAL_MAX_DELAY) != HAL_OK)
		{
			return STTS22H_ERROR;
		}

		return STTS22H_OK;
}

int32_t STTS22H_I2C_WriteReg(uint16_t devAddr, uint16_t reg,
                             uint8_t *pData, uint16_t length)
{
    uint16_t addr = (uint16_t)(devAddr);

    if (HAL_I2C_Mem_Write(&STTS22H_I2C_HANDLE,
                          addr,                   // 8-bit device addr
                          (uint16_t)reg,          // register address
                          I2C_MEMADD_SIZE_8BIT,   // 8-bit sub-address
                          pData,
                          length,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return STTS22H_ERROR;
    }

    return STTS22H_OK;
}

/* Timing helpers */
int32_t STTS22H_GetTick_Platform(void) {
	return (int32_t)HAL_GetTick();
}

void    STTS22H_Delay_Platform(uint32_t ms) {
	return HAL_Delay(ms);
}

int32_t STTS22H_BusIO_Register_I2C(STTS22H_Object_t *pObj)
{
    STTS22H_IO_t io_ctx;

    io_ctx.Init     = STTS22H_I2C_Init;
    io_ctx.DeInit   = STTS22H_I2C_DeInit;
    io_ctx.BusType  = STTS22H_I2C_BUS;
    io_ctx.Address  = STTS22H_I2C_ADDR;
    io_ctx.WriteReg = STTS22H_I2C_WriteReg;
    io_ctx.ReadReg  = STTS22H_I2C_ReadReg;
    io_ctx.GetTick  = STTS22H_GetTick_Platform;
    io_ctx.Delay    = STTS22H_Delay_Platform;

    return STTS22H_RegisterBusIO(pObj, &io_ctx);
}
