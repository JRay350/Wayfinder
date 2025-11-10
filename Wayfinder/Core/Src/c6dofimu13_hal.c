#include "c6dofimu13_hal.h"

#define C6DOFIMU13_I2C_TIMEOUT   100U  // ms

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Calculate two's complement for a given bit length.
 */
static void C6DOFIMU13_GetTwosComp(int16_t *raw, uint8_t length)
{
    if (*raw & ((uint16_t)1 << (length - 1)))
    {
        *raw -= (uint16_t)1 << length;
    }
}

/* -------------------------------------------------------------------------- */
/* Basic helpers                                                              */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef C6DOFIMU13_Init(
    C6DOFIMU13_HandleTypeDef *himu,
    I2C_HandleTypeDef         *hi2c,
    uint8_t                    accel_address,
    uint8_t                    mag_address
)
{
    if (himu == NULL || hi2c == NULL)
        return HAL_ERROR;

    himu->hi2c          = hi2c;
    himu->accel_address = accel_address;   // 7-bit address
    himu->mag_address   = mag_address;     // 7-bit address

    himu->accel_int_port = NULL;
    himu->accel_int_pin  = 0;
    himu->mag_int_port   = NULL;
    himu->mag_int_pin    = 0;

    himu->accel_coef     = 1.0f;

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_WriteReg(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t                    dev_addr,
    uint8_t                    reg,
    uint8_t                   *pData,
    uint16_t                   size
)
{
    if (himu == NULL || himu->hi2c == NULL)
        return HAL_ERROR;

    /* HAL expects 8-bit address (7-bit << 1) */
    return HAL_I2C_Mem_Write(
        himu->hi2c,
        (uint16_t)(dev_addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        pData,
        size,
        C6DOFIMU13_I2C_TIMEOUT
    );
}

HAL_StatusTypeDef C6DOFIMU13_ReadReg(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t                    dev_addr,
    uint8_t                    reg,
    uint8_t                   *pData,
    uint16_t                   size
)
{
    if (himu == NULL || himu->hi2c == NULL)
        return HAL_ERROR;

    return HAL_I2C_Mem_Read(
        himu->hi2c,
        (uint16_t)(dev_addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        pData,
        size,
        C6DOFIMU13_I2C_TIMEOUT
    );
}

/* -------------------------------------------------------------------------- */
/* Magnetometer                                                               */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef C6DOFIMU13_Mag_Init(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t res,
    uint8_t op_mode,
    uint8_t temp_meas
)
{
    HAL_StatusTypeDef status;
    uint8_t cmd_byte;
    uint8_t range_byte;
    uint8_t force_state;

    if (himu == NULL)
        return HAL_ERROR;

    /* Reset */
    cmd_byte = C6DOFIMU13_MAG_CTL_3_SRST_SR;
    status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                 C6DOFIMU13_MAG_CTL_3, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    /* Temperature measure enable/disable bit */
    if (temp_meas == 1)
        force_state = C6DOFIMU13_MAG_CTL_3_TCS_EN;
    else
        force_state = C6DOFIMU13_MAG_CTL_3_TCS_NOP;

    /* Resolution settings */
    if (res == 1)
        range_byte = 0x80 | C6DOFIMU13_MAG_CTL_4_RS_S_15;
    else
        range_byte = 0x80 | C6DOFIMU13_MAG_CTL_4_RS_S_14;

    switch (op_mode)
    {
        case C6DOFIMU13_MAG_OP_MODE_S_SHOT:
        {
            cmd_byte = C6DOFIMU13_MAG_CTL_1_PC_NORM |
                       C6DOFIMU13_MAG_CTL_1_ODR_100 |
                       C6DOFIMU13_MAG_CTL_1_FS_DEF;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_3, &force_state, 1);
            if (status != HAL_OK) return status;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_4, &range_byte, 1);
            if (status != HAL_OK) return status;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_1, &cmd_byte, 1);
            if (status != HAL_OK) return status;
            break;
        }

        case C6DOFIMU13_MAG_OP_MODE_CONT:
        {
            force_state |= C6DOFIMU13_MAG_CTL_3_FRC_SM;
            cmd_byte = C6DOFIMU13_MAG_CTL_1_PC_NORM |
                       C6DOFIMU13_MAG_CTL_1_ODR_100;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_4, &range_byte, 1);
            if (status != HAL_OK) return status;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_1, &cmd_byte, 1);
            if (status != HAL_OK) return status;
            break;
        }

        default:
        {
            cmd_byte = C6DOFIMU13_MAG_CTL_1_PC_NORM |
                       C6DOFIMU13_MAG_CTL_1_ODR_100 |
                       C6DOFIMU13_MAG_CTL_1_FS_DEF;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_3, &force_state, 1);
            if (status != HAL_OK) return status;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_4, &range_byte, 1);
            if (status != HAL_OK) return status;

            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_CTL_1, &cmd_byte, 1);
            if (status != HAL_OK) return status;
            break;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef C6DOFIMU13_Mag_GetAxis(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t                    reg_lsb,
    float                     *result
)
{
    HAL_StatusTypeDef status;
    int16_t adc_val;
    uint8_t range;
    uint8_t rx_buf[2];

    if (himu == NULL || result == NULL)
        return HAL_ERROR;

    /* Read resolution setting */
    status = C6DOFIMU13_ReadReg(himu, himu->mag_address,
                                C6DOFIMU13_MAG_CTL_4, &range, 1);
    if (status != HAL_OK) return status;

    /* Read axis data */
    status = C6DOFIMU13_ReadReg(himu, himu->mag_address,
                                reg_lsb, rx_buf, 2);
    if (status != HAL_OK) return status;

    adc_val = rx_buf[1];
    adc_val <<= 8;
    adc_val |= rx_buf[0];

    if (range & C6DOFIMU13_MAG_CTL_4_RS_S_15)
    {
        adc_val &= 0x7FFF;
        if (adc_val & 0x4000)
        {
            C6DOFIMU13_GetTwosComp(&adc_val, 15);
        }
    }
    else
    {
        adc_val &= 0x3FFF;
        if (adc_val & 0x2000)
        {
            C6DOFIMU13_GetTwosComp(&adc_val, 14);
        }
    }

    *result = (float)adc_val * C6DOFIMU13_MAG_SENS;

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_Mag_GetX(C6DOFIMU13_HandleTypeDef *himu, float *x_mg)
{
    return C6DOFIMU13_Mag_GetAxis(himu, C6DOFIMU13_MAG_OUT_X_LSB, x_mg);
}

HAL_StatusTypeDef C6DOFIMU13_Mag_GetY(C6DOFIMU13_HandleTypeDef *himu, float *y_mg)
{
    return C6DOFIMU13_Mag_GetAxis(himu, C6DOFIMU13_MAG_OUT_Y_LSB, y_mg);
}

HAL_StatusTypeDef C6DOFIMU13_Mag_GetZ(C6DOFIMU13_HandleTypeDef *himu, float *z_mg)
{
    return C6DOFIMU13_Mag_GetAxis(himu, C6DOFIMU13_MAG_OUT_Z_LSB, z_mg);
}

HAL_StatusTypeDef C6DOFIMU13_Mag_GetXYZ(
    C6DOFIMU13_HandleTypeDef *himu,
    float *x_mg,
    float *y_mg,
    float *z_mg
)
{
    HAL_StatusTypeDef status;

    status = C6DOFIMU13_Mag_GetX(himu, x_mg);
    if (status != HAL_OK) return status;

    status = C6DOFIMU13_Mag_GetY(himu, y_mg);
    if (status != HAL_OK) return status;

    status = C6DOFIMU13_Mag_GetZ(himu, z_mg);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_Mag_SetOffset(
    C6DOFIMU13_HandleTypeDef *himu,
    int16_t                    offset,
    uint8_t                    axis
)
{
    HAL_StatusTypeDef status;
    uint8_t cmd_byte;
    uint8_t tx_buf[2];

    if (himu == NULL)
        return HAL_ERROR;

    /* Enable offset cancellation */
    cmd_byte = C6DOFIMU13_MAG_CTL_1_FS_DEF;
    status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                 C6DOFIMU13_MAG_CTL_3, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    cmd_byte = C6DOFIMU13_MAG_CTL_3_OCL_EN;
    status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                 C6DOFIMU13_MAG_CTL_3, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    tx_buf[0] = (uint8_t)(offset & 0xFF);
    tx_buf[1] = (uint8_t)((offset >> 8) & 0xFF);

    switch (axis)
    {
        case C6DOFIMU13_MAG_AXIS_X:
            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_OFFS_X_LSB, tx_buf, 2);
            break;

        case C6DOFIMU13_MAG_AXIS_Y:
            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_OFFS_Y_LSB, tx_buf, 2);
            break;

        case C6DOFIMU13_MAG_AXIS_Z:
            status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                         C6DOFIMU13_MAG_OFFS_Z_LSB, tx_buf, 2);
            break;

        default:
            return HAL_ERROR;
    }

    if (status != HAL_OK) return status;

    /* Disable offset cancellation */
    cmd_byte = 0x00;
    status = C6DOFIMU13_WriteReg(himu, himu->mag_address,
                                 C6DOFIMU13_MAG_CTL_3, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Accelerometer                                                              */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef C6DOFIMU13_Accel_Init(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t samp_rate,
    uint8_t samp_range,
    uint8_t samp_res
)
{
    HAL_StatusTypeDef status;
    uint8_t cmd_byte;
    float res_val;
    float range_val;

    if (himu == NULL)
        return HAL_ERROR;

    /* Put into standby */
    cmd_byte = C6DOFIMU13_ACCEL_MODE_STDBY;
    status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                 C6DOFIMU13_ACCEL_MODE, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    /* Sample rate */
    cmd_byte = samp_rate & 0x0F;
    status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                 C6DOFIMU13_ACCEL_SRTFR, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    /* Range + resolution */
    cmd_byte = (samp_range & 0x70) | (samp_res & 0x07);
    status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                 C6DOFIMU13_ACCEL_OUTCFG, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    /* Resolution factor */
    switch (samp_res)
    {
        case C6DOFIMU13_ACCEL_OUTCFG_RES_6:  res_val = 32.0f;    break;
        case C6DOFIMU13_ACCEL_OUTCFG_RES_7:  res_val = 64.0f;    break;
        case C6DOFIMU13_ACCEL_OUTCFG_RES_8:  res_val = 128.0f;   break;
        case C6DOFIMU13_ACCEL_OUTCFG_RES_10: res_val = 512.0f;   break;
        case C6DOFIMU13_ACCEL_OUTCFG_RES_12: res_val = 2048.0f;  break;
        case C6DOFIMU13_ACCEL_OUTCFG_RES_14: res_val = 8192.0f;  break;
        default:                             res_val = 32.0f;    break;
    }

    /* Range factor */
    switch (samp_range)
    {
        case C6DOFIMU13_ACCEL_OUTCFG_RANGE_2:  range_val = 2.0f;   break;
        case C6DOFIMU13_ACCEL_OUTCFG_RANGE_4:  range_val = 4.0f;   break;
        case C6DOFIMU13_ACCEL_OUTCFG_RANGE_8:  range_val = 8.0f;   break;
        case C6DOFIMU13_ACCEL_OUTCFG_RANGE_16: range_val = 16.0f;  break;
        default:                               range_val = 2.0f;   break;
    }

    /* Store scaling coefficient (same math as original driver) */
    himu->accel_coef = res_val / range_val;

    /* Wake up */
    cmd_byte = C6DOFIMU13_ACCEL_MODE_WAKE;
    status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                 C6DOFIMU13_ACCEL_MODE, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

static HAL_StatusTypeDef C6DOFIMU13_Accel_GetRawAxis(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t                    reg_lsb,
    int16_t                   *result
)
{
    HAL_StatusTypeDef status;
    uint8_t rx_buf[2];
    uint8_t res;

    if (himu == NULL || result == NULL)
        return HAL_ERROR;

    /* Read resolution bits */
    status = C6DOFIMU13_ReadReg(himu, himu->accel_address,
                                C6DOFIMU13_ACCEL_OUTCFG, &res, 1);
    if (status != HAL_OK) return status;

    res &= 0x07;

    if (res < C6DOFIMU13_ACCEL_OUTCFG_RES_10)
    {
        status = C6DOFIMU13_ReadReg(himu, himu->accel_address,
                                    reg_lsb, rx_buf, 1);
        if (status != HAL_OK) return status;

        *result = rx_buf[0];

        switch (res)
        {
            case C6DOFIMU13_ACCEL_OUTCFG_RES_6:
                *result &= 0x003F;
                if (*result > 0x001F)
                    C6DOFIMU13_GetTwosComp(result, 6);
                break;

            case C6DOFIMU13_ACCEL_OUTCFG_RES_7:
                *result &= 0x007F;
                if (*result > 0x003F)
                    C6DOFIMU13_GetTwosComp(result, 7);
                break;

            case C6DOFIMU13_ACCEL_OUTCFG_RES_8:
                *result &= 0x00FF;
                if (*result > 0x007F)
                    C6DOFIMU13_GetTwosComp(result, 8);
                break;

            default:
                *result &= 0x003F;
                if (*result > 0x001F)
                    C6DOFIMU13_GetTwosComp(result, 6);
                break;
        }
    }
    else
    {
        status = C6DOFIMU13_ReadReg(himu, himu->accel_address,
                                    reg_lsb, rx_buf, 2);
        if (status != HAL_OK) return status;

        *result = rx_buf[1];
        *result <<= 8;
        *result |= rx_buf[0];

        switch (res)
        {
            case C6DOFIMU13_ACCEL_OUTCFG_RES_10:
                *result &= 0x03FF;
                if (*result > 0x01FF)
                    C6DOFIMU13_GetTwosComp(result, 10);
                break;

            case C6DOFIMU13_ACCEL_OUTCFG_RES_12:
                *result &= 0x0FFF;
                if (*result > 0x07FF)
                    C6DOFIMU13_GetTwosComp(result, 12);
                break;

            case C6DOFIMU13_ACCEL_OUTCFG_RES_14:
                *result &= 0x3FFF;
                if (*result > 0x1FFF)
                    C6DOFIMU13_GetTwosComp(result, 14);
                break;

            default:
                *result &= 0x03FF;
                if (*result > 0x01FF)
                    C6DOFIMU13_GetTwosComp(result, 10);
                break;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_Accel_GetRawX(
    C6DOFIMU13_HandleTypeDef *himu,
    int16_t                   *raw
)
{
    return C6DOFIMU13_Accel_GetRawAxis(himu, C6DOFIMU13_ACCEL_XOUT_EX_L, raw);
}

HAL_StatusTypeDef C6DOFIMU13_Accel_GetRawY(
    C6DOFIMU13_HandleTypeDef *himu,
    int16_t                   *raw
)
{
    return C6DOFIMU13_Accel_GetRawAxis(himu, C6DOFIMU13_ACCEL_YOUT_EX_L, raw);
}

HAL_StatusTypeDef C6DOFIMU13_Accel_GetRawZ(
    C6DOFIMU13_HandleTypeDef *himu,
    int16_t                   *raw
)
{
    return C6DOFIMU13_Accel_GetRawAxis(himu, C6DOFIMU13_ACCEL_ZOUT_EX_L, raw);
}

HAL_StatusTypeDef C6DOFIMU13_Accel_GetX(
    C6DOFIMU13_HandleTypeDef *himu,
    float                    *x_g
)
{
    HAL_StatusTypeDef status;
    int16_t adc_val;

    if (x_g == NULL)
        return HAL_ERROR;

    status = C6DOFIMU13_Accel_GetRawX(himu, &adc_val);
    if (status != HAL_OK) return status;

    *x_g = (float)adc_val / himu->accel_coef;

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_Accel_GetY(
    C6DOFIMU13_HandleTypeDef *himu,
    float                    *y_g
)
{
    HAL_StatusTypeDef status;
    int16_t adc_val;

    if (y_g == NULL)
        return HAL_ERROR;

    status = C6DOFIMU13_Accel_GetRawY(himu, &adc_val);
    if (status != HAL_OK) return status;

    *y_g = (float)adc_val / himu->accel_coef;

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_Accel_GetZ(
    C6DOFIMU13_HandleTypeDef *himu,
    float                    *z_g
)
{
    HAL_StatusTypeDef status;
    int16_t adc_val;

    if (z_g == NULL)
        return HAL_ERROR;

    status = C6DOFIMU13_Accel_GetRawZ(himu, &adc_val);
    if (status != HAL_OK) return status;

    *z_g = (float)adc_val / himu->accel_coef;

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_Accel_GetXYZ(
    C6DOFIMU13_HandleTypeDef *himu,
    float *x_g,
    float *y_g,
    float *z_g
)
{
    HAL_StatusTypeDef status;

    status = C6DOFIMU13_Accel_GetX(himu, x_g);
    if (status != HAL_OK) return status;

    status = C6DOFIMU13_Accel_GetY(himu, y_g);
    if (status != HAL_OK) return status;

    status = C6DOFIMU13_Accel_GetZ(himu, z_g);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef C6DOFIMU13_Accel_SetOffset(
    C6DOFIMU13_HandleTypeDef *himu,
    int16_t                    offset,
    uint8_t                    axis
)
{
    HAL_StatusTypeDef status;
    uint8_t cmd_byte;
    uint8_t tx_buf[2];

    if (himu == NULL)
        return HAL_ERROR;

    /* Standby */
    cmd_byte = C6DOFIMU13_ACCEL_MODE_STDBY;
    status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                 C6DOFIMU13_ACCEL_MODE, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    tx_buf[0] = (uint8_t)(offset & 0xFF);
    tx_buf[1] = (uint8_t)((offset >> 8) & 0xFF);

    switch (axis)
    {
        case C6DOFIMU13_ACCEL_AXIS_X:
            status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                         C6DOFIMU13_ACCEL_XOFFL, tx_buf, 2);
            break;

        case C6DOFIMU13_ACCEL_AXIS_Y:
            status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                         C6DOFIMU13_ACCEL_YOFFL, tx_buf, 2);
            break;

        case C6DOFIMU13_ACCEL_AXIS_Z:
            status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                         C6DOFIMU13_ACCEL_ZOFFL, tx_buf, 2);
            break;

        default:
            return HAL_ERROR;
    }

    if (status != HAL_OK) return status;

    /* Wake */
    cmd_byte = C6DOFIMU13_ACCEL_MODE_WAKE;
    status = C6DOFIMU13_WriteReg(himu, himu->accel_address,
                                 C6DOFIMU13_ACCEL_MODE, &cmd_byte, 1);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Interrupt helpers                                                          */
/* -------------------------------------------------------------------------- */

GPIO_PinState C6DOFIMU13_GetAccelIntPin(C6DOFIMU13_HandleTypeDef *himu)
{
    if (himu == NULL || himu->accel_int_port == NULL)
        return GPIO_PIN_RESET;

    return HAL_GPIO_ReadPin(himu->accel_int_port, himu->accel_int_pin);
}

GPIO_PinState C6DOFIMU13_GetMagIntPin(C6DOFIMU13_HandleTypeDef *himu)
{
    if (himu == NULL || himu->mag_int_port == NULL)
        return GPIO_PIN_RESET;

    return HAL_GPIO_ReadPin(himu->mag_int_port, himu->mag_int_pin);
}
