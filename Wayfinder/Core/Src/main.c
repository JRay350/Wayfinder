/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;

	uint8_t day;
	uint8_t month;
	uint16_t year;
} DateTime_t;

typedef enum {
	SET_TIME,
	CALIBRATION,
	TIME,
	COMPASS,
	INCLINE,
	PRESSURE,
	TEMPERATURE,
} Interface_State_t;

typedef enum {
	EDIT_MONTH,
	EDIT_DAY,
	EDIT_YEAR,
    EDIT_HOUR,
    EDIT_MINUTE,
    EDIT_SECOND,
} TimeEditField_t;

typedef enum {
	TEMPERATURE_FIELD,
	MAGNETOMETER_FIELD,
	ACCELEROMETER_FIELD,
	PRESSURE_FIELD,
} CalibrationEditField_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
C6DOFIMU13_HandleTypeDef h6dof;

LPS22HH_Object_t lps22hh;
STTS22H_Object_t stts22h;
bool isDisplayOn;

volatile bool rtc_tick_flag;
volatile bool power_button_flag;

Interface_State_t prev_state = SET_TIME;
Interface_State_t interface_state = SET_TIME;

volatile TimeEditField_t time_edit_field = EDIT_MONTH;
DateTime_t edit_time;
float_t edit_temp;
volatile bool ui_dirty = true;
volatile bool edit_time_dirty = false;
volatile bool blink = false;

volatile CalibrationEditField_t calibration_field = TEMPERATURE_FIELD;

float_t temperature_offset = -3.60;
float_t magnetometer_offset = 0.0;
float_t accelerometer_offset = 0.0;
float_t pressure_offset = 0.0;

static float_t press_hist[SPARK_W];
static uint8_t press_head = 0;   // next write index
static uint8_t press_count = 0;  // how many valid samples (<= SPARK_W)

static float_t temp_hist[SPARK_W];
static uint8_t temp_head = 0;
static uint8_t temp_count = 0;

static float_t incline_hist[SPARK_W];
static uint8_t incline_head = 0;
static uint8_t incline_count = 0;


static int16_t press_scale_min = 260;
static int16_t press_scale_max = 1260;

static int16_t temp_scale_min = 24;
static int16_t temp_scale_max = 100;

static int16_t incline_scale_min = 0;
static int16_t incline_scale_max = 360;

extern uint8_t displayBuffer[1024];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static bool is_leap_year(uint16_t year);
static uint8_t days_in_month(uint8_t month, uint16_t year);
static void clamp_day_to_month(DateTime_t *t);
static void IMU_Init(void);
static void Spark_Push(float *hist, uint8_t *head, uint8_t *count, float v);
static void Spark_DrawLine(uint8_t x, uint8_t y, uint8_t w, uint8_t h, Interface_State_t state, const float *hist, uint8_t head, uint8_t count, uint8_t draw_box);
void EnterSetTimeMode(void);
void RTC_GetDateTime(DateTime_t *dt);
HAL_StatusTypeDef RTC_CommitDateTime(const DateTime_t *dt);
void RTC_DisplayDateTime(DateTime_t *dt);
void RTC_DisplayEditDateTime(void);
void RTC_DisplayCalibrate(void);
void NextTimeField(void);
void IncrementTime(void);
void DecrementTime(void);
void AdjustOffset(float_t offset_delta);
void NextCalibrationField(void);
void Draw_Compass(float heading_deg);
void ftoa(char* buf, float value, int decimals);
float Calculate_Altitude(float pressure_hpa);
float Celsius_To_Fahrenheit(float celsius_temperature);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool is_leap_year(uint16_t year)
{
    return ((year % 4u) == 0u && (year % 100u) != 0u) || ((year % 400u) == 0u);
}

static uint8_t days_in_month(uint8_t month, uint16_t year)
{
    static const uint8_t dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month < 1 || month > 12) return 31;

    if (month == 2 && is_leap_year(year)) return 29;
    return dim[month - 1];
}

static void clamp_day_to_month(DateTime_t *t)
{
    uint8_t maxd = days_in_month(t->month, t->year);
    if (t->day < 1) t->day = 1;
    if (t->day > maxd) t->day = maxd;
}

static void IMU_Init(void) {
	C6DOFIMU13_Init(&h6dof, &hi2c1, C6DOFIMU13_DEV_ADDRESS_ACCEL_GND, C6DOFIMU13_DEV_ADDRESS_MAG);

	C6DOFIMU13_Accel_Init(&h6dof,
	                          C6DOFIMU13_ACCEL_SRTFR_RATE_32,
	                          C6DOFIMU13_ACCEL_OUTCFG_RANGE_16,
	                          C6DOFIMU13_ACCEL_OUTCFG_RES_14);

	C6DOFIMU13_Mag_Init(&h6dof,
	                        C6DOFIMU13_MAG_RES_15_BIT,
	                        C6DOFIMU13_MAG_OP_MODE_CONT,
	                        C6DOFIMU13_MAG_TEMP_MEAS_ON);
}

static void Spark_Push(float *hist, uint8_t *head, uint8_t *count, float v)
{
    hist[*head] = v;
    *head = (uint8_t)((*head + 1u) % SPARK_W);
    if (*count < SPARK_W) (*count)++;
}

static void Spark_DrawLine(
    uint8_t x, uint8_t y, uint8_t w, uint8_t h, Interface_State_t state,
    const float *hist, uint8_t head, uint8_t count,
    uint8_t draw_box
)
{
    if (w == 0 || h == 0) return;

    ST7565_fillrect(x, y, w, h, WHITE);
    if (draw_box) ST7565_drawrect(x, y, w, h, BLACK);
    if (count < 2) return;

    // 1) Compute actual data min/max (for expand-only logic)
    float data_min =  1e30f;
    float data_max = -1e30f;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (uint8_t)((head + SPARK_W - count + i) % SPARK_W);
        float v = hist[idx];
        if (v < data_min) data_min = v;
        if (v > data_max) data_max = v;
    }

    // 2) Pick the right standard range and persistent scale vars
    int16_t std_min_i16, std_max_i16;
    int16_t *scale_min_i16, *scale_max_i16;

    // Expand margin and relax step are in *your units*
    // (keep small; you can tune per-state)
    float expand_margin = 0.0f;
    int16_t relax_step_i16 = 0; // 0 = don't relax toward standard

    switch (state) {
        case PRESSURE:
            std_min_i16   = 260;
            std_max_i16   = 1260;
            scale_min_i16 = &press_scale_min;
            scale_max_i16 = &press_scale_max;

            expand_margin = 5.0f;       // e.g. 5 units of whatever your pressure units are
            relax_step_i16 = 0;         // set to 1..5 if you want it to drift back
            break;

        case TEMPERATURE:
            std_min_i16   = -1;
            std_max_i16   = 140;
            scale_min_i16 = &temp_scale_min;
            scale_max_i16 = &temp_scale_max;

            expand_margin = 1.0f;       // 1 degree margin
            relax_step_i16 = 0;
            break;

        default: // INCLINE
            std_min_i16   = 0;
            std_max_i16   = 360;
            scale_min_i16 = &incline_scale_min;
            scale_max_i16 = &incline_scale_max;

            expand_margin = 2.0f;       // a couple degrees
            relax_step_i16 = 0;
            break;
    }

    // 3) Optional relax back toward standard range
    // (Only matters if you allow expansion AND later want to shrink back)
    if (relax_step_i16 > 0) {
        if (*scale_min_i16 < std_min_i16) {
            int16_t next = (int16_t)(*scale_min_i16 + relax_step_i16);
            *scale_min_i16 = (next > std_min_i16) ? std_min_i16 : next;
        } else if (*scale_min_i16 > std_min_i16) {
            int16_t next = (int16_t)(*scale_min_i16 - relax_step_i16);
            *scale_min_i16 = (next < std_min_i16) ? std_min_i16 : next;
        }

        if (*scale_max_i16 < std_max_i16) {
            int16_t next = (int16_t)(*scale_max_i16 + relax_step_i16);
            *scale_max_i16 = (next > std_max_i16) ? std_max_i16 : next;
        } else if (*scale_max_i16 > std_max_i16) {
            int16_t next = (int16_t)(*scale_max_i16 - relax_step_i16);
            *scale_max_i16 = (next < std_max_i16) ? std_max_i16 : next;
        }
    } else {
        // If you want strictly fixed range, just force:
        // *scale_min_i16 = std_min_i16;
        // *scale_max_i16 = std_max_i16;
        //
        // If you want "fixed-unless-exceeded", leave them as-is.
    }

    // 4) Expand-only if exceeded (using data_min/data_max)
    // NOTE: only expand outward; never shrink here.
    /*
    if (data_min < (float)(*scale_min_i16)) {
        float new_min = data_min - expand_margin;
        if (new_min < -32768.0f) new_min = -32768.0f;
        *scale_min_i16 = (int16_t)new_min;
    }
    if (data_max > (float)(*scale_max_i16)) {
        float new_max = data_max + expand_margin;
        if (new_max > 32767.0f) new_max = 32767.0f;
        *scale_max_i16 = (int16_t)new_max;
    }
*/
    // 5) Use persistent scales for plotting (convert to float)
    float vmin = (float)(*scale_min_i16);
    float vmax = (float)(*scale_max_i16);

    float span = vmax - vmin;
    if (span < 1e-6f) span = 1e-6f;

    uint8_t prev_px = x;
    uint8_t prev_py = (uint8_t)(y + (h - 1));

    uint8_t n = count;
    if (n > w) n = w;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t src_i = (uint8_t)(count - n + i);
        uint8_t idx = (uint8_t)((head + SPARK_W - count + src_i) % SPARK_W);
        float v = hist[idx];

        // Clamp to range so it always stays inside the box
        if (v < vmin) v = vmin;
        if (v > vmax) v = vmax;

        float t = (v - vmin) / span; // 0..1

        int16_t yy = (int16_t)((float)y + t * (float)(h - 1));

        uint8_t px = (uint8_t)(x + i);
        uint8_t py = (yy < y) ? y : (yy >= (y + h) ? (uint8_t)(y + h - 1) : (uint8_t)yy);

        if (i > 0) ST7565_drawline(prev_px, prev_py, px, py, BLACK, 1);

        prev_px = px;
        prev_py = py;
    }
}

void EnterSetTimeMode(void) {
	RTC_GetDateTime((DateTime_t *)&edit_time);
	time_edit_field = EDIT_MONTH;
	edit_time_dirty = false;
}

void RTC_GetDateTime(DateTime_t *dt) {
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef rtcDate;

	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

    dt->hours   = rtcTime.Hours;
    dt->minutes = rtcTime.Minutes;
    dt->seconds = rtcTime.Seconds;

    dt->day   = rtcDate.Date;
    dt->month = rtcDate.Month;
    dt->year  = 2000 + rtcDate.Year;
}

HAL_StatusTypeDef RTC_CommitDateTime(const DateTime_t *dt) {
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};

    time.Hours = dt->hours;
    time.Minutes = dt->minutes;
    time.Seconds = dt->seconds;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_RESET;

    date.Year  = (uint8_t)(dt->year - 2000u);  // HAL expects 0..99
    date.Month = dt->month;
    date.Date  = dt->day;
    date.WeekDay = RTC_WEEKDAY_SUNDAY; // Placeholder

    if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
        return HAL_ERROR;

    if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

void RTC_DisplayDateTime(DateTime_t *dt)
{
    char time_str[16];
    char date_str[16];

    snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u",
             (unsigned)(dt->hours   % 24),
             (unsigned)(dt->minutes % 60),
             (unsigned)(dt->seconds % 60));

    snprintf(date_str, sizeof(date_str), "%02u/%02u/%04u",
             (unsigned)(dt->month % 13),
             (unsigned)(dt->day   % 32),
             (unsigned)dt->year);

    memset(displayBuffer, 0, sizeof(displayBuffer));

    /* ---------- Centering using 6x10 step ---------- */
    uint16_t time_w = (uint16_t)strlen(time_str) * FONT8X13_STEP;
    uint16_t date_w = (uint16_t)strlen(date_str) * FONT8X13_STEP;

    uint8_t time_x = (time_w < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - time_w) / 2) : 0;
    uint8_t date_x = (date_w < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - date_w) / 2) : 0;

    uint8_t gap = 8;
    uint8_t block_h = 2 * FONT8X13_H + gap;
    uint8_t top_y   = (LCD_HEIGHT > block_h) ? (uint8_t)((LCD_HEIGHT - block_h) / 2) : 0;

    uint8_t time_y = top_y;
    uint8_t date_y = (uint8_t)(top_y + FONT6X10_H + gap);

    ST7565_drawstring_anywhere_8x13(time_x, time_y, time_str);
    ST7565_drawstring_anywhere_8x13(date_x, date_y, date_str);

    // if you want immediate refresh here:
    // updateDisplay();
}

void RTC_DisplayEditDateTime(void)
{
    char time_str[16];
    char date_str[16];

    snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u",
             (unsigned)(edit_time.hours   % 24),
             (unsigned)(edit_time.minutes % 60),
             (unsigned)(edit_time.seconds % 60));

    snprintf(date_str, sizeof(date_str), "%02u/%02u/%04u",
             (unsigned)edit_time.month,
             (unsigned)edit_time.day,
             (unsigned)edit_time.year);

    const char *prompt = "Set Date/Time";

    memset(displayBuffer, 0, sizeof(displayBuffer));

    /* ---------- Centering (7x12 for ALL text) ---------- */
    uint16_t prompt_w = (uint16_t)strlen(prompt)   * FONT7X12_STEP;
    uint16_t time_w   = (uint16_t)strlen(time_str) * FONT7X12_STEP;
    uint16_t date_w   = (uint16_t)strlen(date_str) * FONT7X12_STEP;

    uint8_t prompt_x = (prompt_w < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - prompt_w) / 2) : 0;
    uint8_t time_x   = (time_w   < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - time_w)   / 2) : 0;
    uint8_t date_x   = (date_w   < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - date_w)   / 2) : 0;

    /* ---------- Spacing control ---------- */
    uint8_t prompt_gap = 10;  // visual gap between title and time
    uint8_t line_gap   = 6;   // gap between time and date

    uint8_t block_h = (uint8_t)(
        FONT7X12_H + prompt_gap +
        FONT7X12_H + line_gap +
        FONT7X12_H
    );

    /* ---------- Inverted Y layout ---------- */
    // Smaller y = visually LOWER
    uint8_t bottom_y = (LCD_HEIGHT > block_h)
                         ? (uint8_t)((LCD_HEIGHT - block_h) / 2)
                         : 0;

    uint8_t time_y   = bottom_y;
    uint8_t date_y   = (uint8_t)(time_y + FONT7X12_H + line_gap);
    uint8_t prompt_y = (uint8_t)(date_y + FONT7X12_H + prompt_gap);

    /* ---------- Draw ---------- */
    ST7565_drawstring_anywhere_7x12(prompt_x, prompt_y, prompt);
    ST7565_drawstring_anywhere_7x12(time_x,   time_y,   time_str);
    ST7565_drawstring_anywhere_7x12(date_x,   date_y,   date_str);

    /* ---------- Underline active field ---------- */
    if (blink) {
        uint8_t ul_x = 0, ul_y = 0, ul_w = 0;

        // Keep original behavior: visually DOWN = SMALLER y
        const uint8_t UL_BELOW_BASELINE = 1;

        uint8_t underline_y_time =
            (time_y > UL_BELOW_BASELINE) ? (uint8_t)(time_y - UL_BELOW_BASELINE) : time_y;

        uint8_t underline_y_date =
            (date_y > UL_BELOW_BASELINE) ? (uint8_t)(date_y - UL_BELOW_BASELINE) : date_y;

        switch (time_edit_field) {
        case EDIT_MONTH:
            ul_x = (uint8_t)(date_x + 0 * FONT7X12_STEP);
            ul_y = underline_y_date;
            ul_w = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_DAY:
            ul_x = (uint8_t)(date_x + 3 * FONT7X12_STEP);
            ul_y = underline_y_date;
            ul_w = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_YEAR:
            ul_x = (uint8_t)(date_x + 6 * FONT7X12_STEP);
            ul_y = underline_y_date;
            ul_w = (uint8_t)(4 * FONT7X12_STEP);
            break;

        case EDIT_HOUR:
            ul_x = (uint8_t)(time_x + 0 * FONT7X12_STEP);
            ul_y = underline_y_time;
            ul_w = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_MINUTE:
            ul_x = (uint8_t)(time_x + 3 * FONT7X12_STEP);
            ul_y = underline_y_time;
            ul_w = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_SECOND:
            ul_x = (uint8_t)(time_x + 6 * FONT7X12_STEP);
            ul_y = underline_y_time;
            ul_w = (uint8_t)(2 * FONT7X12_STEP);
            break;

        default:
            break;
        }

        if (ul_w > 0) {
            ST7565_drawline(
                ul_x,
                ul_y,
                (uint8_t)(ul_x + ul_w - 1),
                ul_y,
                BLACK,
                1
            );
        }
    }

    // updateDisplay(); // call if needed
}



void RTC_DisplayCalibrate(void)
{
    const char *title = "Calibration";

    const char *labels[4] = {
        "Temp.",
        "Compass",
        "Incline",
        "Press."
    };

    char vals[4][20]; // right-side strings (NO UNITS)

    // Build right-side strings
    {
        char tmp[20];

        ftoa(tmp, temperature_offset, 2);
        snprintf(vals[0], sizeof(vals[0]), "%s%cF", tmp, (char)DEGREE_CHAR);

        ftoa(tmp, magnetometer_offset, 2);
        snprintf(vals[1], sizeof(vals[1]), "%s%c ", tmp, (char)DEGREE_CHAR);

        ftoa(tmp, accelerometer_offset, 2);
        snprintf(vals[2], sizeof(vals[2]), "%s%c ", tmp, (char)DEGREE_CHAR);

        ftoa(tmp, pressure_offset, 2);
        snprintf(vals[3], sizeof(vals[3]), "%shPa", tmp);
    }

    memset(displayBuffer, 0, sizeof(displayBuffer));

    /* ---------- Layout ---------- */
    // Title SAME size as menu rows
    const uint8_t TITLE_H   = FONT7X12_H;
    const uint8_t TITLE_STEP= FONT7X12_STEP;

    const uint8_t ROW_H     = FONT7X12_H;
    const uint8_t ROW_STEP  = FONT7X12_STEP;

    const uint8_t LEFT_MARGIN  = 0;
    const uint8_t RIGHT_MARGIN = 30;

    const uint8_t split_x = 72; // prevent value/label collision

    // Title centered (7x12)
    uint16_t title_w = (uint16_t)strlen(title) * (uint16_t)TITLE_STEP;
    uint8_t  title_x = (title_w < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - title_w) / 2) : 0;

    // --------- IMPORTANT: your Y is visually inverted ---------
    // Bigger y appears higher on the screen.
    const uint8_t top_gap = 2;

    // Choose a row gap that fits
    uint8_t row_gap = 2;
    {
        uint8_t needed = (uint8_t)(TITLE_H + top_gap + 4 * ROW_H);
        if (LCD_HEIGHT > needed) {
            uint8_t leftover = (uint8_t)(LCD_HEIGHT - needed);
            row_gap = (uint8_t)(leftover / 3);
            if (row_gap > 10) row_gap = 10;
        }
    }

    // Title at top visually (large y in your coords)
    int16_t title_y_i16 = (int16_t)LCD_HEIGHT - (int16_t)TITLE_H;
    if (title_y_i16 < 0) title_y_i16 = 0;
    uint8_t title_y = (uint8_t)title_y_i16;

    // First menu row just below title visually => smaller y
    int16_t row0_y_i16 = (int16_t)title_y - (int16_t)top_gap - (int16_t)ROW_H;
    if (row0_y_i16 < 0) row0_y_i16 = 0;

    // Rows go downward visually => keep subtracting (smaller y)
    uint8_t row_y[4];
    for (uint8_t i = 0; i < 4; i++) {
        int16_t yi = row0_y_i16 - (int16_t)i * (int16_t)(ROW_H + row_gap);
        if (yi < 0) yi = 0;
        row_y[i] = (uint8_t)yi;
    }

    /* ---------- Draw ---------- */
    ST7565_drawstring_anywhere_7x12(title_x, title_y, title);

    for (uint8_t i = 0; i < 4; i++) {
        ST7565_drawstring_anywhere_7x12(LEFT_MARGIN, row_y[i], labels[i]);

        uint16_t vw = (uint16_t)strlen(vals[i]) * (uint16_t)ROW_STEP;
        int16_t vx  = (int16_t)LCD_WIDTH - (int16_t)vw - RIGHT_MARGIN;
        if (vx < split_x) vx = split_x;

        ST7565_drawstring_anywhere_7x12((uint8_t)vx, row_y[i], vals[i]);
    }

    /* ---------- Underline selected VALUE (your original behavior) ---------- */
    if (blink) {
        uint8_t idx = (calibration_field < 4) ? calibration_field : 0;

        const uint8_t UL_BELOW_BASELINE = 1;
        uint8_t ul_y = (row_y[idx] > UL_BELOW_BASELINE)
                         ? (uint8_t)(row_y[idx] - UL_BELOW_BASELINE)
                         : row_y[idx];

        uint16_t vw = (uint16_t)strlen(vals[idx]) * (uint16_t)ROW_STEP;
        int16_t vx  = (int16_t)LCD_WIDTH - (int16_t)vw - RIGHT_MARGIN;
        if (vx < split_x) vx = split_x;

        if (vw > 0 && vw < 255) {
            ST7565_drawline(
                (uint8_t)vx,
                ul_y,
                (uint8_t)((uint8_t)vx + (uint8_t)vw - 1),
                ul_y,
                BLACK,
                1
            );
        }
    }
}



void NextTimeField(void) {
    if (time_edit_field == EDIT_SECOND)
        time_edit_field = EDIT_MONTH;
    else
        time_edit_field++;

    ui_dirty = true;
}

void IncrementTime(void) {
    switch (time_edit_field) {

    case EDIT_MONTH:
        edit_time.month = (edit_time.month >= 12) ? 1 : (edit_time.month + 1);
        clamp_day_to_month((DateTime_t *)&edit_time);
        break;

    case EDIT_DAY: {
        uint8_t maxd = days_in_month(edit_time.month, edit_time.year);
        edit_time.day = (edit_time.day >= maxd) ? 1 : (edit_time.day + 1);
        break;
    }

    case EDIT_YEAR:
        edit_time.year = (edit_time.year >= YEAR_MAX) ? YEAR_MIN : (edit_time.year + 1);
        clamp_day_to_month((DateTime_t *)&edit_time);
        break;

    case EDIT_HOUR:
        edit_time.hours = (edit_time.hours + 1) % 24;
        break;

    case EDIT_MINUTE:
        edit_time.minutes = (edit_time.minutes + 1) % 60;
        break;

    case EDIT_SECOND:
        edit_time.seconds = (edit_time.seconds + 1) % 60;
        break;
    }
    edit_time_dirty = true;
    ui_dirty = true;
}

void DecrementTime(void) {
	switch (time_edit_field) {

    case EDIT_MONTH:
        edit_time.month = (edit_time.month <= 1) ? 12 : (edit_time.month - 1);
        clamp_day_to_month((DateTime_t *)&edit_time);
        break;

    case EDIT_DAY: {
        uint8_t maxd = days_in_month(edit_time.month, edit_time.year);
        edit_time.day = (edit_time.day <= 1) ? maxd : (edit_time.day - 1);
        break;
    }

    case EDIT_YEAR:
        edit_time.year = (edit_time.year <= YEAR_MIN) ? YEAR_MAX : (edit_time.year - 1);
        clamp_day_to_month((DateTime_t *)&edit_time);
        break;

	case EDIT_HOUR:
	    edit_time.hours = (edit_time.hours == 0) ? 23 : edit_time.hours - 1;
	    break;

	case EDIT_MINUTE:
	    edit_time.minutes = (edit_time.minutes == 0) ? 59 : edit_time.minutes - 1;
	    break;

	case EDIT_SECOND:
	    edit_time.seconds = (edit_time.seconds == 0) ? 59 : edit_time.seconds - 1;
	    break;
    }

    edit_time_dirty = true;
    ui_dirty = true;
}

void AdjustOffset(float_t offset_delta) {
	switch (calibration_field) {

	case TEMPERATURE_FIELD:
		temperature_offset += offset_delta;
		break;

	case MAGNETOMETER_FIELD:
		magnetometer_offset += offset_delta;
		break;

	case ACCELEROMETER_FIELD:
		accelerometer_offset += offset_delta;
		break;

	case PRESSURE_FIELD:
		pressure_offset += offset_delta;
		break;
	}
}

void NextCalibrationField(void) {
	calibration_field++;
	if (calibration_field >= 4) calibration_field = TEMPERATURE_FIELD;
}

void Draw_Compass(float heading_deg)
{
    const uint8_t cx = 64;
    const uint8_t cy = 32;
    const uint8_t r  = 20;

    ST7565_drawcircle(cx, cy, r, BLACK);

    // Y-UP coordinate system:
    // North is ABOVE center => larger y
    ST7565_drawchar_anywhere(cx - 2,  cy + r + 2,  'N');  // top
    ST7565_drawchar_anywhere(cx + r + 4, cy - 3,   'E');  // right
    ST7565_drawchar_anywhere(cx - 2,  cy - r - 10, 'S');  // bottom
    ST7565_drawchar_anywhere(cx - r - 10, cy - 3,  'W');  // left

    float offset = 0.0f;
    float angle = (heading_deg + offset) * (3.14159265f / 180.0f);

    // Keep 0° pointing to North:
    float fx = cx + r * sinf(angle);
    float fy = cy + r * cosf(angle);

    uint8_t x1 = (uint8_t)(fx + 0.5f);
    uint8_t y1 = (uint8_t)(fy + 0.5f);

    ST7565_drawline(cx, cy, x1, y1, BLACK, 2);
}

// Convert float to string with fixed number of decimals
// Example: ftoa(buf, 3.14159f, 3) → "3.142"
void ftoa(char *buf, float value, int decimals)
{
    // Handle negative numbers
    if (value < 0) {
        *buf++ = '-';
        value = -value;
    }

    // Extract integer part
    int int_part = (int)value;

    // Extract fractional part
    float remainder = value - (float)int_part;

    // Scale fractional part
    int scale = 1;
    for (int i = 0; i < decimals; i++)
        scale *= 10;

    int frac_part = (int)(remainder * scale + 0.5f);  // round correctly

    // Convert integer part
    sprintf(buf, "%d", int_part);   // uses ONLY %d → no float printf

    // Move buffer to end
    while (*buf != '\0') buf++;

    if (decimals > 0) {
        *buf++ = '.';

        // Zero padding for fractional part
        int pad = scale / 10;
        while (pad > 1 && frac_part < pad) {
            *buf++ = '0';
            pad /= 10;
        }

        // Convert fractional part
        sprintf(buf, "%d", frac_part);  // still only %d
    }
}

float Calculate_Altitude(float pressure_hpa) {
	float base = pressure_hpa / 1013.25;
	float exp = 0.190284;
	return (1 - pow(base, exp)) * 145366.45;
}

float Celsius_To_Fahrenheit(float celsius_temperature) {
	return celsius_temperature * 1.8 + 32;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // MC6470 Init
  IMU_Init();


  // LPS22HH Init
  const float_t lps22h_odr = 1.0f;
  int32_t lps22hh_status = LPS22HH_OK;

  do {
	  if (LPS22HH_BusIO_Register_SPI(&lps22hh) != LPS22HH_OK) break;
	  if ((lps22hh_status = LPS22HH_Init(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_TEMP_Disable(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_PRESS_Enable(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_PRESS_SetOutputDataRate(&lps22hh, lps22h_odr)) != LPS22HH_OK) break;
  } while (0);

  // STTS22H Init
  const float_t stts22h_odr = 1.0f;
  int32_t stts22h_status = STTS22H_OK;

  do {
      if ((stts22h_status = STTS22H_BusIO_Register_I2C(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_Init(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_TEMP_Enable(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_TEMP_SetOutputDataRate(&stts22h, stts22h_odr)) != STTS22H_OK) break;
  } while (0);

  // LCD Init
  ST7565_init();
  isDisplayOn = true;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* ---------- Power button handling ---------- */
      if (power_button_flag) {
          power_button_flag = 0;

          uint8_t status = isDisplayOn ? ST7565_off() : ST7565_on();
          if (status == HAL_OK) {
              isDisplayOn = !isDisplayOn;
          }

          ui_dirty = true;  // refresh screen after display toggle
      }

      /* ---------- 1 Hz tick handling (NO drawing here) ---------- */
      if (rtc_tick_flag) {
          rtc_tick_flag = 0;

          blink = !blink;

          // Refresh screens that need periodic updates
          if (interface_state == TIME) {
              ui_dirty = true;               // update time once per second
          } else if (interface_state == SET_TIME || interface_state == CALIBRATION) {
              ui_dirty = true;               // blink underline/cursor
          }
          else if (interface_state == PRESSURE || interface_state == TEMPERATURE) ui_dirty = true;
          else if (interface_state == COMPASS) ui_dirty = true;
      }

      /* ---------- Draw only when dirty ---------- */
      if (ui_dirty) {
          ui_dirty = false;

          switch (interface_state) {

          case SET_TIME:
              // RTC_DisplayEditDateTime() should only draw to displayBuffer.
              // If you want blink control, have it use the global 'blink' flag.
              RTC_DisplayEditDateTime();
              updateDisplay();
              break;

          case TIME: {
              DateTime_t now;
              RTC_GetDateTime(&now);

              // RTC_DisplayDateTime() should only draw to displayBuffer.
              RTC_DisplayDateTime(&now);
              updateDisplay();
              break;
          }

          case PRESSURE: {
        	  char pressure_display_string[37];
        	  float_t pressure;

        	  if (LPS22HH_PRESS_GetPressure(&lps22hh, &pressure) == LPS22HH_OK) {
        	      Spark_Push(press_hist, &press_head, &press_count, pressure + pressure_offset);
        	      char pressure_string[20];
        	      ftoa(pressure_string, pressure + pressure_offset, 2); // e.g. "1013.25"

        	      snprintf(pressure_display_string, sizeof(pressure_display_string),
        	               "%s HPa", pressure_string);
        	  } else {
        	      snprintf(pressure_display_string, sizeof(pressure_display_string),
        	               "Pressure Failure");
        	  }

              memset(displayBuffer, 0, sizeof(displayBuffer));

              ST7565_drawstring_anywhere_7x12(
                  (LCD_WIDTH / 2) - ((strlen(pressure_display_string) / 2) * 6),
                  27,
                  pressure_display_string
              );

              Spark_DrawLine(
                  24,                     // x
                  5,                     // y (below numeric readout)
                  80,                     // width
                  32,                     // height
                  PRESSURE,               // state selects press_scale_min/max
                  press_hist,
                  press_head,
                  press_count,
                  1                       // draw box
              );

              updateDisplay();
              break;
          }

          case CALIBRATION: {
        	  RTC_DisplayCalibrate();
        	  updateDisplay();
        	  break;
          }

          case TEMPERATURE: {
        	  char temperature_display_string[38];
        	  float temperature;

        	  if (STTS22H_TEMP_GetTemperature(&stts22h, &temperature) == STTS22H_OK) {
        		  temperature = Celsius_To_Fahrenheit(temperature);
        	      Spark_Push(temp_hist, &temp_head, &temp_count, temperature + temperature_offset);
        	      char temperature_string[20];
        	      ftoa(temperature_string, temperature + temperature_offset, 2);   // e.g. "23.45"

        	      snprintf(temperature_display_string, sizeof(temperature_display_string),
        	               "%s%cF", temperature_string, (char)DEGREE_CHAR);
        	  } else {
        	      snprintf(temperature_display_string, sizeof(temperature_display_string),
        	               "Temperature Failure");
        	  }


              memset(displayBuffer, 0, sizeof(displayBuffer));
              ST7565_drawstring_anywhere_7x12(
                  (LCD_WIDTH - strlen(temperature_display_string) * 7) / 2,
                  LCD_HEIGHT - 16,
                  temperature_display_string
              );

              Spark_DrawLine(
                  0,                     // x
                  0,                     // y
                  128,                     // width
                  40,                     // height
                  TEMPERATURE,               //
                  temp_hist,
                  temp_head,
                  temp_count,
                  1                       // draw box
              );

              updateDisplay();
              break;
          }

          case COMPASS: {
              float ax, ay, az;
              float mx, my, mz;

              // ---- Hard-iron offset calibration (min/max box) ----
              // Keep these static so they persist across frames while you rotate the device.
              static float mx_min =  1e9f, mx_max = -1e9f;
              static float my_min =  1e9f, my_max = -1e9f;

              // Set this to 0 when you want to freeze calibration (e.g., after one full rotation).
              static uint8_t mag_cal_active = 1;

              memset(displayBuffer, 0, sizeof(displayBuffer));

              if (C6DOFIMU13_Accel_GetXYZ(&h6dof, &ax, &ay, &az) == HAL_OK &&
                  C6DOFIMU13_Mag_GetXYZ(&h6dof, &mx, &my, &mz) == HAL_OK)
              {
                  // Update min/max while calibrating
                  if (mag_cal_active)
                  {
                      if (mx < mx_min) mx_min = mx;
                      if (mx > mx_max) mx_max = mx;

                      if (my < my_min) my_min = my;
                      if (my > my_max) my_max = my;
                  }

                  // Compute hard-iron offsets (center)
                  float mx_off = 0.5f * (mx_max + mx_min);
                  float my_off = 0.5f * (my_max + my_min);

                  // Center the horizontal field vector
                  float mx_c = mx - mx_off;
                  float my_c = my - my_off;

                  // Heading (using your chosen convention atan2(my, -mx))
                  float heading_rad = atan2f(my_c, mx_c);
                  float heading_deg = heading_rad * (180.0f / 3.14159265f) + magnetometer_offset;

                  if (heading_deg < 0.0f) heading_deg += 360.0f;
                  if (heading_deg >= 360.0f) heading_deg -= 360.0f;


                  // Draw compass into displayBuffer only
                  Draw_Compass(heading_deg);
              }
              else
              {
                  const char *IMU_error = "IMU Failure";
                  ST7565_drawstring_anywhere(
                      (LCD_WIDTH / 2) - ((strlen(IMU_error) / 2) * 6),
                      27,
                      (char*)IMU_error
                  );
              }

              updateDisplay();
              break;
          }
          default:
              break;
          }
      }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV16;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000103;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000103;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	rtc_tick_flag = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) { // PA0
		if (interface_state == SET_TIME) {
			RTC_CommitDateTime(&edit_time);
			edit_time_dirty = false;
			interface_state = TIME;
			ui_dirty = true;
		} else if (interface_state == CALIBRATION) {
			interface_state = prev_state;
		}
		else power_button_flag = true;
	} else if (GPIO_Pin == GPIO_PIN_9) { // PB9
        if (interface_state == SET_TIME) {
            IncrementTime();
        } else if (interface_state == COMPASS) {
			interface_state = CALIBRATION;
			prev_state = COMPASS;
		} else if (interface_state == CALIBRATION) {
			AdjustOffset(0.1);
		} else {
			interface_state = COMPASS;
		}
	} else if (GPIO_Pin == GPIO_PIN_8) { // PB8
		if (interface_state == SET_TIME) {
			DecrementTime();
		} else if (interface_state == CALIBRATION) {
			AdjustOffset(-0.1);
		}
		else if (interface_state == TEMPERATURE) {
			interface_state = CALIBRATION;
			prev_state = TEMPERATURE;
		} else {
			interface_state = TEMPERATURE;
		}
	} else if (GPIO_Pin == GPIO_PIN_3) { // PB3
		switch (interface_state) {
			case SET_TIME:
				NextTimeField();
				break;
			case COMPASS:
				interface_state = TIME;
				break;
			case PRESSURE:
				interface_state = TIME;
				break;
			case TIME:
				interface_state = SET_TIME;
				break;
			case CALIBRATION:
				NextCalibrationField();
				break;
			default:
				break;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
