#include "app_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "freertos_app.h"
#include "i2c.h"
#include "iwdg.h"
#include "system.h"

#define OLED_ADDR              (0x3CU << 1)
#define OLED_WIDTH             128U
#define OLED_HEIGHT            64U
#define OLED_PAGES             (OLED_HEIGHT / 8U)
#define OLED_REFRESH_MS        250U

#define BUTTON_ACTIVE          GPIO_PIN_RESET
#define BUTTON_LONG_MS         800U
#define BUTTON_DEBOUNCE_MS     40U

#define ADC_SAMPLE_MS          100U
#define CONTROL_TIMEOUT_MS     250U

#define BATTERY_DIVIDER_X100   200U
#define BATTERY_WARN_MV        6400U
#define BATTERY_CUTOFF_MV      6000U
#define SPEED_LIMIT_MIN_CMPS   10U
#define SPEED_LIMIT_MAX_CMPS   60U

extern UART_HandleTypeDef huart5;

static uint8_t oled_buf[OLED_WIDTH * OLED_PAGES];
static uint8_t oled_present = 0;
static uint8_t oled_dirty = 0;
static uint8_t oled_flush_page = 0;

static AppDisplayPage display_page = APP_PAGE_MAIN;
static bool telemetry_enabled = true;
static volatile bool emergency_stop = false;
static bool low_battery_latched = false;
static const char *safety_reason = "OK";
static uint16_t battery_mv = 0;
static uint16_t speed_limit_cmps = SPEED_LIMIT_MAX_CMPS;
static uint32_t last_control_tick = 0;
static uint32_t last_oled_render_tick = 0;
static uint32_t last_adc_tick = 0;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState stable_state;
    GPIO_PinState last_raw_state;
    uint32_t last_change_ms;
    uint32_t pressed_ms;
    uint8_t long_sent;
} ButtonState;

static ButtonState button_user = {B1_GPIO_Port, B1_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0, 0, 0};
static ButtonState button_aux = {GPIOB, GPIO_PIN_2, GPIO_PIN_SET, GPIO_PIN_SET, 0, 0, 0};

static void oled_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00U, cmd};
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, data, sizeof(data), 5) != HAL_OK) {
        oled_present = 0;
    }
}

static void oled_init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR, 2, 20) != HAL_OK) {
        oled_present = 0;
        return;
    }

    oled_present = 1;
    oled_cmd(0xAE); oled_cmd(0x20); oled_cmd(0x00); oled_cmd(0xB0);
    oled_cmd(0xC8); oled_cmd(0x00); oled_cmd(0x10); oled_cmd(0x40);
    oled_cmd(0x81); oled_cmd(0x7F); oled_cmd(0xA1); oled_cmd(0xA6);
    oled_cmd(0xA8); oled_cmd(0x3F); oled_cmd(0xA4); oled_cmd(0xD3);
    oled_cmd(0x00); oled_cmd(0xD5); oled_cmd(0x80); oled_cmd(0xD9);
    oled_cmd(0xF1); oled_cmd(0xDA); oled_cmd(0x12); oled_cmd(0xDB);
    oled_cmd(0x40); oled_cmd(0x8D); oled_cmd(0x14); oled_cmd(0xAF);

    memset(oled_buf, 0, sizeof(oled_buf));
    oled_dirty = 1;
}

static uint8_t glyph_col(char c, uint8_t col)
{
    static const uint8_t digits[10][3] = {
        {0x1F,0x11,0x1F},{0x00,0x1F,0x00},{0x1D,0x15,0x17},{0x15,0x15,0x1F},{0x07,0x04,0x1F},
        {0x17,0x15,0x1D},{0x1F,0x15,0x1D},{0x01,0x01,0x1F},{0x1F,0x15,0x1F},{0x17,0x15,0x1F}
    };
    static const uint8_t letters[26][3] = {
        {0x1E,0x05,0x1E},{0x1F,0x15,0x0A},{0x0E,0x11,0x11},{0x1F,0x11,0x0E},{0x1F,0x15,0x15},
        {0x1F,0x05,0x05},{0x0E,0x11,0x1D},{0x1F,0x04,0x1F},{0x11,0x1F,0x11},{0x08,0x10,0x0F},
        {0x1F,0x04,0x1B},{0x1F,0x10,0x10},{0x1F,0x02,0x1F},{0x1F,0x06,0x1F},{0x0E,0x11,0x0E},
        {0x1F,0x05,0x02},{0x0E,0x19,0x1E},{0x1F,0x0D,0x12},{0x12,0x15,0x09},{0x01,0x1F,0x01},
        {0x0F,0x10,0x0F},{0x07,0x18,0x07},{0x1F,0x08,0x1F},{0x1B,0x04,0x1B},{0x03,0x1C,0x03},
        {0x19,0x15,0x13}
    };
    if (col >= 3U) return 0;
    if (c >= '0' && c <= '9') return digits[c - '0'][col];
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'][col];
    switch (c) {
        case '.': return (col == 1U) ? 0x10U : 0U;
        case ',': return (col == 1U) ? 0x18U : 0U;
        case ':': return (col == 1U) ? 0x0AU : 0U;
        case '-': return (col == 1U) ? 0x04U : 0U;
        case '/': return (col == 0U) ? 0x18U : (col == 1U ? 0x04U : 0x03U);
        case '+': return (col == 1U) ? 0x0EU : 0x04U;
        case '!': return (col == 1U) ? 0x17U : 0U;
        default: return 0;
    }
}

static void oled_put_char(uint8_t x, uint8_t y, char c)
{
    if (y >= OLED_PAGES || x >= OLED_WIDTH) return;
    uint16_t idx = (uint16_t)y * OLED_WIDTH + x;
    for (uint8_t col = 0; col < 3U && (x + col) < OLED_WIDTH; col++) {
        oled_buf[idx + col] = glyph_col(c, col);
    }
    if ((x + 3U) < OLED_WIDTH) {
        oled_buf[idx + 3U] = 0;
    }
}

static void oled_puts(uint8_t x, uint8_t y, const char *text)
{
    while (*text != '\0' && x < (OLED_WIDTH - 3U)) {
        oled_put_char(x, y, *text++);
        x = (uint8_t)(x + 4U);
    }
}

static void render_oled(void)
{
    char line[32];
    SlamPose2D_t pose;
    ControlDebugSnapshot_t control;
    FreertosRuntimeStats_t runtime;

    Odometry_GetPoseSnapshot(&pose);
    Control_GetDebugSnapshot(&control);
    Freertos_GetRuntimeStatsSnapshot(&runtime);
    memset(oled_buf, 0, sizeof(oled_buf));

    if (display_page == APP_PAGE_MAIN) {
        snprintf(line, sizeof(line), "MODE:%s", g_control_mode == CONTROL_MODE_POSITION ? "AUTO" : "MAN");
        oled_puts(0, 0, line);
        snprintf(line, sizeof(line), "SAFE:%s", App_MotorsAllowed() ? "OK" : safety_reason);
        oled_puts(0, 1, line);
        snprintf(line, sizeof(line), "BAT:%umV", battery_mv);
        oled_puts(0, 2, line);
        snprintf(line, sizeof(line), "SPD:%ucm/s", speed_limit_cmps);
        oled_puts(0, 3, line);
        snprintf(line, sizeof(line), "LIDAR:%u", runtime.last_scan_raw_point_count);
        oled_puts(0, 4, line);
        snprintf(line, sizeof(line), "PWM:%u/%u", control.left_pwm, control.right_pwm);
        oled_puts(0, 5, line);
    } else if (display_page == APP_PAGE_POSE) {
        snprintf(line, sizeof(line), "X:%ldmm", (long)(pose.x_m * 1000.0f));
        oled_puts(0, 0, line);
        snprintf(line, sizeof(line), "Y:%ldmm", (long)(pose.y_m * 1000.0f));
        oled_puts(0, 1, line);
        snprintf(line, sizeof(line), "TH:%lddeg", (long)pose.theta_deg);
        oled_puts(0, 2, line);
        snprintf(line, sizeof(line), "STATE:%d", (int)g_relative_move_state);
        oled_puts(0, 3, line);
        snprintf(line, sizeof(line), "RET:PB2 LONG");
        oled_puts(0, 5, line);
    } else {
        snprintf(line, sizeof(line), "VL:%ld/%ld", (long)(g_left_speed * 1000.0f), (long)(g_right_speed * 1000.0f));
        oled_puts(0, 0, line);
        snprintf(line, sizeof(line), "ANG:%ld", (long)control.angle_error_deg);
        oled_puts(0, 1, line);
        snprintf(line, sizeof(line), "BASE:%ld", (long)(base_car_speed * 100.0f));
        oled_puts(0, 2, line);
        snprintf(line, sizeof(line), "BT:%s", telemetry_enabled ? "ON" : "OFF");
        oled_puts(0, 3, line);
        snprintf(line, sizeof(line), "OVF:%lu", (unsigned long)overflow_count);
        oled_puts(0, 4, line);
    }

    oled_dirty = 1;
}

static void oled_flush_step(void)
{
    uint8_t tx[OLED_WIDTH + 1U];
    if (!oled_present || !oled_dirty) return;

    oled_cmd((uint8_t)(0xB0U + oled_flush_page));
    oled_cmd(0x00);
    oled_cmd(0x10);
    tx[0] = 0x40U;
    memcpy(&tx[1], &oled_buf[oled_flush_page * OLED_WIDTH], OLED_WIDTH);
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, tx, sizeof(tx), 8) != HAL_OK) {
        oled_present = 0;
        return;
    }

    oled_flush_page++;
    if (oled_flush_page >= OLED_PAGES) {
        oled_flush_page = 0;
        oled_dirty = 0;
    }
}

static uint16_t read_adc_mv(void)
{
    uint32_t raw = 0;
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return battery_mv;
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
        raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return (uint16_t)((raw * 3300UL * BATTERY_DIVIDER_X100) / (4095UL * 100UL));
}

static AppButtonEvent button_update(ButtonState *button, uint32_t now)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(button->port, button->pin);
    if (raw != button->last_raw_state) {
        button->last_raw_state = raw;
        button->last_change_ms = now;
    }

    if ((now - button->last_change_ms) < BUTTON_DEBOUNCE_MS) {
        return APP_BUTTON_NONE;
    }

    if (raw != button->stable_state) {
        button->stable_state = raw;
        if (raw == BUTTON_ACTIVE) {
            button->pressed_ms = now;
            button->long_sent = 0;
        } else if (!button->long_sent && button->pressed_ms != 0U) {
            button->pressed_ms = 0;
            return APP_BUTTON_SHORT;
        }
    }

    if (button->stable_state == BUTTON_ACTIVE && !button->long_sent &&
        (now - button->pressed_ms) >= BUTTON_LONG_MS) {
        button->long_sent = 1;
        return APP_BUTTON_LONG;
    }

    return APP_BUTTON_NONE;
}

static void update_battery_and_limit(uint32_t now)
{
    if ((now - last_adc_tick) < ADC_SAMPLE_MS) return;
    last_adc_tick = now;

    battery_mv = read_adc_mv();
    speed_limit_cmps = SPEED_LIMIT_MIN_CMPS +
        (uint16_t)(((uint32_t)battery_mv % 3300UL) *
                   (SPEED_LIMIT_MAX_CMPS - SPEED_LIMIT_MIN_CMPS) / 3299UL);

    if (battery_mv > 0U && battery_mv < BATTERY_WARN_MV && !low_battery_latched) {
        low_battery_latched = true;
        App_StartReturnHome();
        safety_reason = "LOWBAT";
    }
    if (battery_mv > 0U && battery_mv < BATTERY_CUTOFF_MV) {
        App_RequestEmergencyStop("LOWBAT");
    }
}

void App_Init(void)
{
    last_control_tick = HAL_GetTick();
    battery_mv = read_adc_mv();
    oled_init();
    render_oled();
}

void App_Task10ms(float dt)
{
    uint32_t now = HAL_GetTick();
    AppButtonEvent user_event;
    AppButtonEvent aux_event;
    (void)dt;

    last_control_tick = now;
    update_battery_and_limit(now);

    user_event = button_update(&button_user, now);
    aux_event = button_update(&button_aux, now);

    if (user_event == APP_BUTTON_SHORT) {
        if (emergency_stop) {
            App_ClearEmergencyStop();
        } else {
            App_RequestEmergencyStop("STOP");
        }
    } else if (user_event == APP_BUTTON_LONG) {
        App_RequestEmergencyStop("EMG");
    }

    if (aux_event == APP_BUTTON_SHORT) {
        display_page = (AppDisplayPage)((display_page + 1U) % APP_PAGE_COUNT);
        render_oled();
    } else if (aux_event == APP_BUTTON_LONG) {
        App_StartReturnHome();
    }

    App_ApplySpeedLimit();
    if ((now - last_oled_render_tick) >= OLED_REFRESH_MS) {
        last_oled_render_tick = now;
        render_oled();
    }

    oled_flush_step();
    HAL_IWDG_Refresh(&hiwdg);
}

void App_BackgroundTask(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - last_control_tick) > CONTROL_TIMEOUT_MS) {
        App_RequestEmergencyStop("WDT");
    }
}

bool App_MotorsAllowed(void)
{
    return !emergency_stop;
}

void App_ApplySpeedLimit(void)
{
    float limit = (float)speed_limit_cmps / 100.0f;
    float limited_speed = base_car_speed;

    if (limited_speed > limit) limited_speed = limit;
    if (limited_speed < -limit) limited_speed = -limit;
    if (limited_speed != base_car_speed) {
        Control_SetBaseSpeed(limited_speed);
    }
}

void App_RequestEmergencyStop(const char *reason)
{
    emergency_stop = true;
    safety_reason = reason;
    Control_SetBaseSpeed(0.0f);
    Motor_StopAll();
}

void App_ClearEmergencyStop(void)
{
    emergency_stop = false;
    safety_reason = low_battery_latched ? "LOWBAT" : "OK";
}

void App_StartReturnHome(void)
{
    SlamPose2D_t pose;

    if (g_odomMutex != NULL) {
        (void)osMutexAcquire(g_odomMutex, osWaitForever);
    }
    Odometry_GetPoseSnapshot(&pose);
    if (g_odomMutex != NULL) {
        (void)osMutexRelease(g_odomMutex);
    }

    if (emergency_stop) return;
    Start_Relative_Move(-pose.x_m, -pose.y_m);
}

void App_SetTelemetryEnabled(bool enabled)
{
    telemetry_enabled = enabled;
}

bool App_GetTelemetryEnabled(void)
{
    return telemetry_enabled;
}

uint16_t App_GetBatteryMillivolts(void)
{
    return battery_mv;
}

uint16_t App_GetSpeedLimitCmps(void)
{
    return speed_limit_cmps;
}

const char *App_GetSafetyReason(void)
{
    return safety_reason;
}
