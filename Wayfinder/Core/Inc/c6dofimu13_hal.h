#ifndef C6DOFIMU13_HAL_H
#define C6DOFIMU13_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"

/* ============================================================================
 * REGISTER MAP (unchanged from original MikroE header)
 * ============================================================================
 */

/* Magnetometer registers */
#define C6DOFIMU13_MAG_SELFTEST_RESP     0x0C
#define C6DOFIMU13_MAG_MORE_INFO_VER     0x0D
#define C6DOFIMU13_MAG_MORE_INFO         0x0E
#define C6DOFIMU13_MAG_WHO_I_AM          0x0F
#define C6DOFIMU13_MAG_OUT_X_LSB         0x10
#define C6DOFIMU13_MAG_OUT_X_MSB         0x11
#define C6DOFIMU13_MAG_OUT_Y_LSB         0x12
#define C6DOFIMU13_MAG_OUT_Y_MSB         0x13
#define C6DOFIMU13_MAG_OUT_Z_LSB         0x14
#define C6DOFIMU13_MAG_OUT_Z_MSB         0x15
#define C6DOFIMU13_MAG_STAT              0x18
#define C6DOFIMU13_MAG_CTL_1             0x1B
#define C6DOFIMU13_MAG_CTL_2             0x1C
#define C6DOFIMU13_MAG_CTL_3             0x1D
#define C6DOFIMU13_MAG_CTL_4             0x1E
#define C6DOFIMU13_MAG_OFFS_X_LSB        0x20
#define C6DOFIMU13_MAG_OFFS_X_MSB        0x21
#define C6DOFIMU13_MAG_OFFS_Y_LSB        0x22
#define C6DOFIMU13_MAG_OFFS_Y_MSB        0x23
#define C6DOFIMU13_MAG_OFFS_Z_LSB        0x24
#define C6DOFIMU13_MAG_OFFS_Z_MSB        0x25
#define C6DOFIMU13_MAG_ITHR_L            0x26
#define C6DOFIMU13_MAG_ITHR_H            0x27
#define C6DOFIMU13_MAG_TEMP_VAL          0x31

/* Accelerometer registers */
#define C6DOFIMU13_ACCEL_SR              0x03
#define C6DOFIMU13_ACCEL_OPSTAT          0x04
#define C6DOFIMU13_ACCEL_INTEN           0x06
#define C6DOFIMU13_ACCEL_MODE            0x07
#define C6DOFIMU13_ACCEL_SRTFR           0x08
#define C6DOFIMU13_ACCEL_TAPEN           0x09
#define C6DOFIMU13_ACCEL_TTTRX           0x0A
#define C6DOFIMU13_ACCEL_TTTRY           0x0B
#define C6DOFIMU13_ACCEL_TTTRZ           0x0C
#define C6DOFIMU13_ACCEL_XOUT_EX_L       0x0D
#define C6DOFIMU13_ACCEL_XOUT_EX_H       0x0E
#define C6DOFIMU13_ACCEL_YOUT_EX_L       0x0F
#define C6DOFIMU13_ACCEL_YOUT_EX_H       0x10
#define C6DOFIMU13_ACCEL_ZOUT_EX_L       0x11
#define C6DOFIMU13_ACCEL_ZOUT_EX_H       0x12
#define C6DOFIMU13_ACCEL_OUTCFG          0x20
#define C6DOFIMU13_ACCEL_XOFFL           0x21
#define C6DOFIMU13_ACCEL_XOFFH           0x22
#define C6DOFIMU13_ACCEL_YOFFL           0x23
#define C6DOFIMU13_ACCEL_YOFFH           0x24
#define C6DOFIMU13_ACCEL_ZOFFL           0x25
#define C6DOFIMU13_ACCEL_ZOFFH           0x26
#define C6DOFIMU13_ACCEL_XGAIN           0x27
#define C6DOFIMU13_ACCEL_YGAIN           0x28
#define C6DOFIMU13_ACCEL_ZGAIN           0x29
#define C6DOFIMU13_ACCEL_PCODE           0x3B

/* Status / control bits etc. (unchanged) */
#define C6DOFIMU13_MAG_STAT_DRDY         0x40
#define C6DOFIMU13_MAG_STAT_DOR          0x20

#define C6DOFIMU13_MAG_CTL_1_PC_NORM     0x80
#define C6DOFIMU13_MAG_CTL_1_PC_STBY     0x00
#define C6DOFIMU13_MAG_CTL_1_ODR_05      0x00
#define C6DOFIMU13_MAG_CTL_1_ODR_10      0x08
#define C6DOFIMU13_MAG_CTL_1_ODR_20      0x10
#define C6DOFIMU13_MAG_CTL_1_ODR_100     0x18
#define C6DOFIMU13_MAG_CTL_1_FS_DEF      0x02
#define C6DOFIMU13_MAG_CTL_1_FS_NORM     0x00

#define C6DOFIMU13_MAG_CTL_3_SRST_SR     0x80
#define C6DOFIMU13_MAG_CTL_3_SRST_NOP    0x00
#define C6DOFIMU13_MAG_CTL_3_FRC_SM      0x40
#define C6DOFIMU13_MAG_CTL_3_FRC_NOP     0x00
#define C6DOFIMU13_MAG_CTL_3_STC_EN      0x10
#define C6DOFIMU13_MAG_CTL_3_STC_NOP     0x00
#define C6DOFIMU13_MAG_CTL_3_TCS_EN      0x02
#define C6DOFIMU13_MAG_CTL_3_TCS_NOP     0x00
#define C6DOFIMU13_MAG_CTL_3_OCL_EN      0x01
#define C6DOFIMU13_MAG_CTL_3_OCL_NOP     0x00

#define C6DOFIMU13_MAG_CTL_4_RS_S_14     0x00
#define C6DOFIMU13_MAG_CTL_4_RS_S_15     0x10

#define C6DOFIMU13_MAG_SENS              0.15f
#define C6DOFIMU13_MAG_ORIENTATION_COEF  -1.0f

#define C6DOFIMU13_MAG_RES_14_BIT        0x00
#define C6DOFIMU13_MAG_RES_15_BIT        0x01
#define C6DOFIMU13_MAG_OP_MODE_S_SHOT    0x00
#define C6DOFIMU13_MAG_OP_MODE_CONT      0x01
#define C6DOFIMU13_MAG_TEMP_MEAS_OFF     0x00
#define C6DOFIMU13_MAG_TEMP_MEAS_ON      0x01

#define C6DOFIMU13_MAG_AXIS_X            0x00
#define C6DOFIMU13_MAG_AXIS_Y            0x01
#define C6DOFIMU13_MAG_AXIS_Z            0x02

/* Accelerometer flags */
#define C6DOFIMU13_ACCEL_SR_TAP_XP       0x01
#define C6DOFIMU13_ACCEL_SR_TAP_XN       0x02
#define C6DOFIMU13_ACCEL_SR_TAP_YP       0x04
#define C6DOFIMU13_ACCEL_SR_TAP_YN       0x08
#define C6DOFIMU13_ACCEL_SR_TAP_ZP       0x10
#define C6DOFIMU13_ACCEL_SR_TAP_ZN       0x20
#define C6DOFIMU13_ACCEL_SR_AC_INT       0x80

#define C6DOFIMU13_ACCEL_OPSTAT_WAKE     0x01
#define C6DOFIMU13_ACCEL_OPSTAT_WDT      0x10
#define C6DOFIMU13_ACCEL_OPSTAT_OTPA     0x80

#define C6DOFIMU13_ACCEL_INTEN_TIXPEN    0x01
#define C6DOFIMU13_ACCEL_INTEN_TIXNEN    0x02
#define C6DOFIMU13_ACCEL_INTEN_TIYPEN    0x04
#define C6DOFIMU13_ACCEL_INTEN_TIYNEN    0x08
#define C6DOFIMU13_ACCEL_INTEN_TIZPEN    0x10
#define C6DOFIMU13_ACCEL_INTEN_TIZNEN    0x20
#define C6DOFIMU13_ACCEL_INTEN_AC_INT_EN 0x80

#define C6DOFIMU13_ACCEL_MODE_STDBY      0x00
#define C6DOFIMU13_ACCEL_MODE_WAKE       0x01
#define C6DOFIMU13_ACCEL_MODE_WDT_NEG    0x10
#define C6DOFIMU13_ACCEL_MODE_WDT_POS    0x20
#define C6DOFIMU13_ACCEL_MODE_IPP        0x40
#define C6DOFIMU13_ACCEL_MODE_IAH        0x80

#define C6DOFIMU13_ACCEL_SRTFR_RATE_32   0x00
#define C6DOFIMU13_ACCEL_SRTFR_RATE_16   0x01
#define C6DOFIMU13_ACCEL_SRTFR_RATE_8    0x02
#define C6DOFIMU13_ACCEL_SRTFR_RATE_4    0x03
#define C6DOFIMU13_ACCEL_SRTFR_RATE_2    0x04
#define C6DOFIMU13_ACCEL_SRTFR_RATE_1    0x05
#define C6DOFIMU13_ACCEL_SRTFR_RATE_0_5  0x06
#define C6DOFIMU13_ACCEL_SRTFR_RATE_0_25 0x07
#define C6DOFIMU13_ACCEL_SRTFR_RATE_64   0x08
#define C6DOFIMU13_ACCEL_SRTFR_RATE_128  0x09
#define C6DOFIMU13_ACCEL_SRTFR_RATE_256  0x0A
#define C6DOFIMU13_ACCEL_SRTFR_FLIP_TAPX 0x10
#define C6DOFIMU13_ACCEL_SRTFR_FLIP_TAPY 0x20
#define C6DOFIMU13_ACCEL_SRTFR_FLIP_TAPZ 0x40
#define C6DOFIMU13_ACCEL_SRTFR_TAP_LATCH 0x80

#define C6DOFIMU13_ACCEL_TAPEN_TAPXPEN   0x01
#define C6DOFIMU13_ACCEL_TAPEN_TAPXNEN   0x02
#define C6DOFIMU13_ACCEL_TAPEN_TAPYPEN   0x04
#define C6DOFIMU13_ACCEL_TAPEN_TAPYNEN   0x08
#define C6DOFIMU13_ACCEL_TAPEN_TAPZPEN   0x10
#define C6DOFIMU13_ACCEL_TAPEN_TAPZNEN   0x20
#define C6DOFIMU13_ACCEL_TAPEN_THRDUR    0x40
#define C6DOFIMU13_ACCEL_TAPEN_TAP_EN    0x80

#define C6DOFIMU13_ACCEL_OUTCFG_RANGE_2  0x00
#define C6DOFIMU13_ACCEL_OUTCFG_RANGE_4  0x10
#define C6DOFIMU13_ACCEL_OUTCFG_RANGE_8  0x20
#define C6DOFIMU13_ACCEL_OUTCFG_RANGE_16 0x30
#define C6DOFIMU13_ACCEL_OUTCFG_RES_6    0x00
#define C6DOFIMU13_ACCEL_OUTCFG_RES_7    0x01
#define C6DOFIMU13_ACCEL_OUTCFG_RES_8    0x02
#define C6DOFIMU13_ACCEL_OUTCFG_RES_10   0x03
#define C6DOFIMU13_ACCEL_OUTCFG_RES_12   0x04
#define C6DOFIMU13_ACCEL_OUTCFG_RES_14   0x05

#define C6DOFIMU13_ACCEL_AXIS_X          0x00
#define C6DOFIMU13_ACCEL_AXIS_Y          0x01
#define C6DOFIMU13_ACCEL_AXIS_Z          0x02

/* I2C addresses (7-bit) */
#define C6DOFIMU13_DEV_ADDRESS_ACCEL_GND 0x4C
#define C6DOFIMU13_DEV_ADDRESS_ACCEL_VCC 0x6C
#define C6DOFIMU13_DEV_ADDRESS_MAG       0x0C

/**
 * @brief 6DOF IMU 13 handle structure for STM32 HAL.
 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;      /**< Pointer to configured HAL I2C handle */

    uint8_t accel_address;        /**< Accelerometer I2C address (7-bit) */
    uint8_t mag_address;          /**< Magnetometer I2C address (7-bit) */

    GPIO_TypeDef *accel_int_port; /**< Accelerometer interrupt GPIO port */
    uint16_t      accel_int_pin;  /**< Accelerometer interrupt GPIO pin */

    GPIO_TypeDef *mag_int_port;   /**< Magnetometer interrupt GPIO port */
    uint16_t      mag_int_pin;    /**< Magnetometer interrupt GPIO pin */

    float         accel_coef;     /**< Accelerometer scaling coefficient (e.g. LSB->g) */

} C6DOFIMU13_HandleTypeDef;

/* ----------------- Basic helpers ----------------- */

HAL_StatusTypeDef C6DOFIMU13_Init(
    C6DOFIMU13_HandleTypeDef *himu,
    I2C_HandleTypeDef         *hi2c,
    uint8_t                    accel_address,
    uint8_t                    mag_address
);

HAL_StatusTypeDef C6DOFIMU13_WriteReg(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t                    dev_addr,
    uint8_t                    reg,
    uint8_t                   *pData,
    uint16_t                   size
);

HAL_StatusTypeDef C6DOFIMU13_ReadReg(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t                    dev_addr,
    uint8_t                    reg,
    uint8_t                   *pData,
    uint16_t                   size
);

/* ----------------- High-level config ----------------- */

HAL_StatusTypeDef C6DOFIMU13_Mag_Init(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t res,
    uint8_t op_mode,
    uint8_t temp_meas
);

HAL_StatusTypeDef C6DOFIMU13_Accel_Init(
    C6DOFIMU13_HandleTypeDef *himu,
    uint8_t samp_rate,
    uint8_t samp_range,
    uint8_t samp_res
);

/* ----------------- Magnetometer read functions ----------------- */

HAL_StatusTypeDef C6DOFIMU13_Mag_GetX(C6DOFIMU13_HandleTypeDef *himu, float *x_mg);
HAL_StatusTypeDef C6DOFIMU13_Mag_GetY(C6DOFIMU13_HandleTypeDef *himu, float *y_mg);
HAL_StatusTypeDef C6DOFIMU13_Mag_GetZ(C6DOFIMU13_HandleTypeDef *himu, float *z_mg);
HAL_StatusTypeDef C6DOFIMU13_Mag_GetXYZ(C6DOFIMU13_HandleTypeDef *himu,
                                        float *x_mg, float *y_mg, float *z_mg);

HAL_StatusTypeDef C6DOFIMU13_Mag_SetOffset(C6DOFIMU13_HandleTypeDef *himu,
                                           int16_t offset, uint8_t axis);

/* ----------------- Accelerometer read functions ----------------- */

HAL_StatusTypeDef C6DOFIMU13_Accel_GetRawX(C6DOFIMU13_HandleTypeDef *himu, int16_t *raw);
HAL_StatusTypeDef C6DOFIMU13_Accel_GetRawY(C6DOFIMU13_HandleTypeDef *himu, int16_t *raw);
HAL_StatusTypeDef C6DOFIMU13_Accel_GetRawZ(C6DOFIMU13_HandleTypeDef *himu, int16_t *raw);

HAL_StatusTypeDef C6DOFIMU13_Accel_GetX(C6DOFIMU13_HandleTypeDef *himu, float *x_g);
HAL_StatusTypeDef C6DOFIMU13_Accel_GetY(C6DOFIMU13_HandleTypeDef *himu, float *y_g);
HAL_StatusTypeDef C6DOFIMU13_Accel_GetZ(C6DOFIMU13_HandleTypeDef *himu, float *z_g);
HAL_StatusTypeDef C6DOFIMU13_Accel_GetXYZ(C6DOFIMU13_HandleTypeDef *himu,
                                          float *x_g, float *y_g, float *z_g);

HAL_StatusTypeDef C6DOFIMU13_Accel_SetOffset(C6DOFIMU13_HandleTypeDef *himu,
                                             int16_t offset, uint8_t axis);

/* ----------------- Interrupt helpers ----------------- */

GPIO_PinState C6DOFIMU13_GetAccelIntPin(C6DOFIMU13_HandleTypeDef *himu);
GPIO_PinState C6DOFIMU13_GetMagIntPin(C6DOFIMU13_HandleTypeDef *himu);

#ifdef __cplusplus
}
#endif

#endif /* C6DOFIMU13_HAL_H */
