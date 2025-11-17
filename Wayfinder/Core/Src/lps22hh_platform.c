#include "lps22hh_platform.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

// Chip Select pin for LPS22HH
#define LPS22HH_CS_PORT   GPIOA
#define LPS22HH_CS_PIN    GPIO_PIN_4


int32_t LPS22HH_SPI_Init(void)
{
    // SPI is already initialized by MX_SPI1_Init()
    // Just make sure CS is high (inactive)
    HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_SET);
    return LPS22HH_OK;
}

int32_t LPS22HH_SPI_DeInit(void)
{
	HAL_SPI_DeInit(&hspi1);
    return LPS22HH_OK;
}

int32_t LPS22HH_GetTick_Platform(void)
{
    return (int32_t)HAL_GetTick();
}

void LPS22HH_Delay_Platform(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * SPI write: devAddr is unused in SPI mode, but kept for API compatibility.
 */
int32_t LPS22HH_SPI_WriteReg(uint16_t devAddr, uint16_t reg,
                             uint8_t *pData, uint16_t length)
{
    (void)devAddr;

    // For SPI write: bit7 = 0 (write), address in bits 6:0
    uint8_t addr = (uint8_t)reg;   // DO NOT OR 0x40 HERE

    HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&LPS22HH_SPI_HANDLE, &addr, 1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_SET);
        return LPS22HH_ERROR;
    }

    if (HAL_SPI_Transmit(&LPS22HH_SPI_HANDLE, pData, length, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_SET);
        return LPS22HH_ERROR;
    }

    HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_SET);
    return LPS22HH_OK;
}


/**
 * SPI read: devAddr is unused in SPI mode.
 */
int32_t LPS22HH_SPI_ReadReg(uint16_t devAddr, uint16_t reg,
                            uint8_t *pData, uint16_t length)
{
    (void)devAddr;

    // For SPI read: bit7 = 1, no extra auto-increment bit
    uint8_t addr = (uint8_t)reg | 0x80;   // READ bit only

    HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&LPS22HH_SPI_HANDLE, &addr, 1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_SET);
        return LPS22HH_ERROR;
    }

    if (HAL_SPI_Receive(&LPS22HH_SPI_HANDLE, pData, length, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_SET);
        return LPS22HH_ERROR;
    }

    HAL_GPIO_WritePin(LPS22HH_CS_PORT, LPS22HH_CS_PIN, GPIO_PIN_SET);
    return LPS22HH_OK;
}


int32_t LPS22HH_BusIO_Register_SPI(LPS22HH_Object_t *pObj)
{
    LPS22HH_IO_t io_ctx;

    io_ctx.Init     = LPS22HH_SPI_Init;
    io_ctx.DeInit   = LPS22HH_SPI_DeInit;
    io_ctx.BusType  = LPS22HH_SPI_4WIRES_BUS;
    io_ctx.Address  = 0; // unused in SPI, but must be set
    io_ctx.WriteReg = LPS22HH_SPI_WriteReg;
    io_ctx.ReadReg  = LPS22HH_SPI_ReadReg;
    io_ctx.GetTick  = LPS22HH_GetTick_Platform;
    io_ctx.Delay    = LPS22HH_Delay_Platform;

    return LPS22HH_RegisterBusIO(pObj, &io_ctx);
}
