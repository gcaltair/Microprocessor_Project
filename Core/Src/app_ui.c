#include "app_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "freertos_app.h"
#include "i2c.h"
#include "iwdg.h"
#include "mapping_task.h"
#include "navigation_task.h"
#include "system.h"

#define OLED_ADDR                    (0x3CU << 1)
#define OLED_WIDTH                   128U
#define OLED_HEIGHT                  64U
#define OLED_PAGES                   (OLED_HEIGHT / 8U)
#define OLED_REFRESH_MS              250U

#define BUTTON_ACTIVE                GPIO_PIN_RESET
#define BUTTON_LONG_MS               800U
#define BUTTON_DEBOUNCE_MS           40U

#define ADC_SAMPLE_MS                100U
#define CONTROL_TIMEOUT_MS           250U
#define LIDAR_STALE_MS               2000U
#define MOTOR_STALL_MS               1000U
#define MOTOR_STALL_PWM_MIN          2500U
#define MOTOR_STALL_SPEED_MAX_MPS    0.008f
#define MOTOR_STALL_COMMAND_MIN_MPS  0.05f

#define BATTERY_ADC_CHANNEL          ADC_CHANNEL_4
#define SPEED_KNOB_ADC_CHANNEL       ADC_CHANNEL_10
#define BATTERY_DIVIDER_X100         200U
#define BATTERY_WARN_MV              6400U
#define BATTERY_CUTOFF_MV            6000U
#define SPEED_LIMIT_MIN_CMPS         10U
#define SPEED_LIMIT_MAX_CMPS         40U

static uint8_t oled_buf[OLED_WIDTH * OLED_PAGES];
static uint8_t oled_present = 0U;
static uint8_t oled_dirty = 0U;
static uint8_t oled_flush_page = 0U;

static AppDisplayPage display_page = APP_PAGE_MAIN;
static volatile bool telemetry_enabled = true;
static volatile bool emergency_stop = false;
static volatile AppSafetyCode safety_code = APP_SAFETY_OK;
static volatile uint8_t stop_cleanup_pending = 0U;
static volatile uint8_t clear_stop_on_short_press = 0U;
static bool low_battery_latched = false;
static const char *volatile safety_reason = "OK";
static uint16_t battery_mv = 0U;
static uint16_t speed_limit_cmps = SPEED_LIMIT_MAX_CMPS;
static volatile uint32_t last_control_tick = 0U;
static uint32_t last_oled_render_tick = 0U;
static uint32_t last_adc_tick = 0U;
static uint32_t last_lidar_scan_tick = 0U;
static uint32_t last_lidar_scan_count = 0U;
static uint32_t lidar_recovery_count = 0U;
static uint32_t motor_stall_started_tick = 0U;

static AppBenchmarkState benchmark_state = APP_BENCHMARK_IDLE;
static uint32_t benchmark_start_tick = 0U;
static uint32_t benchmark_exit_tick = 0U;
static uint32_t benchmark_end_tick = 0U;
static uint32_t benchmark_exit_time_ms = 0U;
static uint32_t benchmark_return_time_ms = 0U;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState stable_state;
    GPIO_PinState last_raw_state;
    uint32_t last_change_ms;
    uint32_t pressed_ms;
    uint8_t long_sent;
} ButtonState;

static ButtonState button_user = {B1_GPIO_Port, B1_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0U, 0U, 0U};
static ButtonState button_aux = {GPIOB, GPIO_PIN_2, GPIO_PIN_SET, GPIO_PIN_SET, 0U, 0U, 0U};

static void set_safety_status(AppSafetyCode code, const char *reason)
{
    safety_code = code;
    safety_reason = reason;
}

static void oled_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00U, cmd};

    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, data, sizeof(data), 5U) != HAL_OK) {
        oled_present = 0U;
    }
}

static void oled_init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR, 2U, 20U) != HAL_OK) {
        oled_present = 0U;
        return;
    }

    oled_present = 1U;
    oled_cmd(0xAEU); oled_cmd(0x20U); oled_cmd(0x00U); oled_cmd(0xB0U);
    oled_cmd(0xC8U); oled_cmd(0x00U); oled_cmd(0x10U); oled_cmd(0x40U);
    oled_cmd(0x81U); oled_cmd(0x7FU); oled_cmd(0xA1U); oled_cmd(0xA6U);
    oled_cmd(0xA8U); oled_cmd(0x3FU); oled_cmd(0xA4U); oled_cmd(0xD3U);
    oled_cmd(0x00U); oled_cmd(0xD5U); oled_cmd(0x80U); oled_cmd(0xD9U);
    oled_cmd(0xF1U); oled_cmd(0xDAU); oled_cmd(0x12U); oled_cmd(0xDBU);
    oled_cmd(0x40U); oled_cmd(0x8DU); oled_cmd(0x14U); oled_cmd(0xAFU);

    (void)memset(oled_buf, 0, sizeof(oled_buf));
    oled_dirty = 1U;
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

    if (col >= 3U) return 0U;
    if ((c >= '0') && (c <= '9')) return digits[c - '0'][col];
    if ((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
    if ((c >= 'A') && (c <= 'Z')) return letters[c - 'A'][col];
    switch (c) {
        case '.': return (col == 1U) ? 0x10U : 0U;
        case ',': return (col == 1U) ? 0x18U : 0U;
        case ':': return (col == 1U) ? 0x0AU : 0U;
        case '-': return (col == 1U) ? 0x04U : 0U;
        case '/': return (col == 0U) ? 0x18U : ((col == 1U) ? 0x04U : 0x03U);
        case '+': return (col == 1U) ? 0x0EU : 0x04U;
        case '!': return (col == 1U) ? 0x17U : 0U;
        default: return 0U;
    }
}

static void oled_put_char(uint8_t x, uint8_t y, char c)
{
    uint16_t idx;
    uint8_t col;

    if ((y >= OLED_PAGES) || (x >= OLED_WIDTH)) return;
    idx = (uint16_t)y * OLED_WIDTH + x;
    for (col = 0U; (col < 3U) && ((x + col) < OLED_WIDTH); ++col) {
        oled_buf[idx + col] = glyph_col(c, col);
    }
    if ((x + 3U) < OLED_WIDTH) {
        oled_buf[idx + 3U] = 0U;
    }
}

static void oled_puts(uint8_t x, uint8_t y, const char *text)
{
    while ((*text != '\0') && (x < (OLED_WIDTH - 3U))) {
        oled_put_char(x, y, *text++);
        x = (uint8_t)(x + 4U);
    }
}

static const char *benchmark_label(void)
{
    switch (benchmark_state) {
        case APP_BENCHMARK_OUTBOUND: return "OUT";
        case APP_BENCHMARK_RETURNING: return "HOME";
        case APP_BENCHMARK_COMPLETE: return "DONE";
        default: return "IDLE";
    }
}

static void render_oled(void)
{
    char line[32];
    SlamPose2D_t pose;
    ControlDebugSnapshot_t control;
    FreertosRuntimeStats_t runtime;
    NavigationTaskStats_t navigation;

    Odometry_GetPoseSnapshot(&pose);
    Control_GetDebugSnapshot(&control);
    Freertos_GetRuntimeStatsSnapshot(&runtime);
    NavigationTask_GetStatsSnapshot(&navigation);
    (void)memset(oled_buf, 0, sizeof(oled_buf));

    if (display_page == APP_PAGE_MAIN) {
        (void)snprintf(line, sizeof(line), "MODE:%s", g_control_mode == CONTROL_MODE_POSITION ? "AUTO" : "MAN");
        oled_puts(0U, 0U, line);
        (void)snprintf(line, sizeof(line), "SAFE:%s", (const char *)safety_reason);
        oled_puts(0U, 1U, line);
        (void)snprintf(line, sizeof(line), "BAT:%umV", battery_mv);
        oled_puts(0U, 2U, line);
        (void)snprintf(line, sizeof(line), "LIM:%ucm/s", speed_limit_cmps);
        oled_puts(0U, 3U, line);
        (void)snprintf(line, sizeof(line), "MAP:%lu", (unsigned long)runtime.lidar_scan_complete_count);
        oled_puts(0U, 4U, line);
        (void)snprintf(line, sizeof(line), "TASK:%s", benchmark_label());
        oled_puts(0U, 5U, line);
    } else if (display_page == APP_PAGE_POSE) {
        (void)snprintf(line, sizeof(line), "X:%ldmm", (long)(pose.x_m * 1000.0f));
        oled_puts(0U, 0U, line);
        (void)snprintf(line, sizeof(line), "Y:%ldmm", (long)(pose.y_m * 1000.0f));
        oled_puts(0U, 1U, line);
        (void)snprintf(line, sizeof(line), "TH:%lddeg", (long)pose.theta_deg);
        oled_puts(0U, 2U, line);
        (void)snprintf(line, sizeof(line), "GX:%ld", (long)(navigation.goal_pose.x_m * 1000.0f));
        oled_puts(0U, 3U, line);
        (void)snprintf(line, sizeof(line), "GY:%ld", (long)(navigation.goal_pose.y_m * 1000.0f));
        oled_puts(0U, 4U, line);
        (void)snprintf(line, sizeof(line), "DST:%ld", (long)(navigation.distance_to_goal_m * 1000.0f));
        oled_puts(0U, 5U, line);
    } else {
        (void)snprintf(line, sizeof(line), "PWM:%u/%u", control.left_pwm, control.right_pwm);
        oled_puts(0U, 0U, line);
        (void)snprintf(line, sizeof(line), "VL:%ld/%ld", (long)(g_left_speed * 1000.0f), (long)(g_right_speed * 1000.0f));
        oled_puts(0U, 1U, line);
        (void)snprintf(line, sizeof(line), "LREC:%lu", (unsigned long)lidar_recovery_count);
        oled_puts(0U, 2U, line);
        (void)snprintf(line, sizeof(line), "EXIT:%lus", (unsigned long)(benchmark_exit_time_ms / 1000U));
        oled_puts(0U, 3U, line);
        (void)snprintf(line, sizeof(line), "RET:%lus", (unsigned long)(benchmark_return_time_ms / 1000U));
        oled_puts(0U, 4U, line);
        (void)snprintf(line, sizeof(line), "BT:%s", telemetry_enabled ? "ON" : "OFF");
        oled_puts(0U, 5U, line);
    }

    oled_dirty = 1U;
}

static void oled_flush_step(void)
{
    uint8_t tx[OLED_WIDTH + 1U];

    if ((oled_present == 0U) || (oled_dirty == 0U)) return;
    oled_cmd((uint8_t)(0xB0U + oled_flush_page));
    oled_cmd(0x00U);
    oled_cmd(0x10U);
    tx[0] = 0x40U;
    (void)memcpy(&tx[1], &oled_buf[oled_flush_page * OLED_WIDTH], OLED_WIDTH);
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, tx, sizeof(tx), 8U) != HAL_OK) {
        oled_present = 0U;
        return;
    }

    oled_flush_page++;
    if (oled_flush_page >= OLED_PAGES) {
        oled_flush_page = 0U;
        oled_dirty = 0U;
    }
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
            button->long_sent = 0U;
        } else if ((button->long_sent == 0U) && (button->pressed_ms != 0U)) {
            button->pressed_ms = 0U;
            return APP_BUTTON_SHORT;
        }
    }
    if ((button->stable_state == BUTTON_ACTIVE) && (button->long_sent == 0U) &&
        ((now - button->pressed_ms) >= BUTTON_LONG_MS)) {
        button->long_sent = 1U;
        return APP_BUTTON_LONG;
    }
    return APP_BUTTON_NONE;
}

static void update_adc_inputs(uint32_t now)
{
    uint16_t knob_raw;

    if ((now - last_adc_tick) < ADC_SAMPLE_MS) return;
    last_adc_tick = now;
    battery_mv = (uint16_t)(((uint32_t)ADC_ReadRawChannel(BATTERY_ADC_CHANNEL) *
                             3300UL * BATTERY_DIVIDER_X100) / (4095UL * 100UL));
    knob_raw = ADC_ReadRawChannel(SPEED_KNOB_ADC_CHANNEL);
    speed_limit_cmps = SPEED_LIMIT_MIN_CMPS +
        (uint16_t)(((uint32_t)knob_raw * (SPEED_LIMIT_MAX_CMPS - SPEED_LIMIT_MIN_CMPS)) / 4095UL);
    Control_SetSpeedLimit((float)speed_limit_cmps / 100.0f);

    if ((battery_mv >= BATTERY_WARN_MV) && low_battery_latched) {
        low_battery_latched = false;
        if ((!emergency_stop) && (safety_code == APP_SAFETY_LOW_BATTERY)) {
            set_safety_status(APP_SAFETY_OK, "OK");
        }
    } else if ((battery_mv < BATTERY_WARN_MV) && !low_battery_latched) {
        low_battery_latched = true;
        set_safety_status(APP_SAFETY_LOW_BATTERY, "LOWBAT");
        App_StartReturnHome();
    }
    if (battery_mv < BATTERY_CUTOFF_MV) {
        App_RequestEmergencyStop("LOWBAT");
    }
}

static void monitor_lidar(uint32_t now)
{
    FreertosRuntimeStats_t runtime;

    Freertos_GetRuntimeStatsSnapshot(&runtime);
    if (runtime.lidar_scan_complete_count != last_lidar_scan_count) {
        last_lidar_scan_count = runtime.lidar_scan_complete_count;
        last_lidar_scan_tick = now;
        if ((!emergency_stop) && (safety_code == APP_SAFETY_LIDAR_RECOVERY)) {
            if (low_battery_latched) {
                set_safety_status(APP_SAFETY_LOW_BATTERY, "LOWBAT");
            } else {
                set_safety_status(APP_SAFETY_OK, "OK");
            }
        }
        return;
    }
    if ((lidar_raw_stream_active != 0U) && ((now - last_lidar_scan_tick) >= LIDAR_STALE_MS)) {
        RPLIDAR_StopRaw();
        RPLIDAR_StartRaw();
        lidar_recovery_count++;
        last_lidar_scan_tick = now;
        if ((!emergency_stop) && (!low_battery_latched)) {
            set_safety_status(APP_SAFETY_LIDAR_RECOVERY, "LIDAR");
        }
    }
}

static void monitor_motor_stall(uint32_t now)
{
    ControlDebugSnapshot_t control;
    uint8_t drive_requested;
    uint8_t wheels_stalled;

    if (emergency_stop) {
        motor_stall_started_tick = 0U;
        return;
    }
    Control_GetDebugSnapshot(&control);
    drive_requested = ((fabsf(control.base_speed_mps) >= MOTOR_STALL_COMMAND_MIN_MPS) &&
                      ((control.left_pwm >= MOTOR_STALL_PWM_MIN) ||
                       (control.right_pwm >= MOTOR_STALL_PWM_MIN))) ? 1U : 0U;
    wheels_stalled = ((fabsf(control.left_speed_feedback_mps) <= MOTOR_STALL_SPEED_MAX_MPS) &&
                      (fabsf(control.right_speed_feedback_mps) <= MOTOR_STALL_SPEED_MAX_MPS)) ? 1U : 0U;
    if ((drive_requested != 0U) && (wheels_stalled != 0U)) {
        if (motor_stall_started_tick == 0U) {
            motor_stall_started_tick = now;
        } else if ((now - motor_stall_started_tick) >= MOTOR_STALL_MS) {
            App_RequestEmergencyStop("MOTOR");
        }
    } else {
        motor_stall_started_tick = 0U;
    }
}

void App_Init(void)
{
    uint32_t now = HAL_GetTick();

    last_control_tick = now;
    last_lidar_scan_tick = now;
    last_adc_tick = now - ADC_SAMPLE_MS;
    update_adc_inputs(now);
    oled_init();
    render_oled();
}

void App_RecordControlTick(void)
{
    last_control_tick = HAL_GetTick();
}

void App_ServiceTask(void)
{
    uint32_t now = HAL_GetTick();
    AppButtonEvent user_event;
    AppButtonEvent aux_event;

    if (stop_cleanup_pending != 0U) {
        stop_cleanup_pending = 0U;
        NavigationTask_ClearGoal();
        Cancel_Relative_Move();
        Control_SetBaseSpeed(0.0f);
    }

    update_adc_inputs(now);
    user_event = button_update(&button_user, now);
    aux_event = button_update(&button_aux, now);
    if (user_event == APP_BUTTON_SHORT) {
        if (clear_stop_on_short_press != 0U) {
            App_StartExploration();
        }
        clear_stop_on_short_press = 0U;
    } else if (user_event == APP_BUTTON_LONG) {
        clear_stop_on_short_press = 0U;
        App_RequestEmergencyStop("STOP");
    }
    if (aux_event == APP_BUTTON_SHORT) {
        display_page = (AppDisplayPage)((display_page + 1U) % APP_PAGE_COUNT);
        render_oled();
    } else if (aux_event == APP_BUTTON_LONG) {
        App_StartReturnHome();
    }

    monitor_lidar(now);
    monitor_motor_stall(now);
    if ((now - last_oled_render_tick) >= OLED_REFRESH_MS) {
        last_oled_render_tick = now;
        render_oled();
    }
    oled_flush_step();

    if ((now - last_control_tick) > CONTROL_TIMEOUT_MS) {
        App_RequestEmergencyStop("WDT");
        return;
    }
    (void)HAL_IWDG_Refresh(&hiwdg);
}

bool App_MotorsAllowed(void)
{
    return !emergency_stop;
}

void App_RequestEmergencyStop(const char *reason)
{
    AppSafetyCode code = APP_SAFETY_EMERGENCY;

    if ((reason != NULL) && (strcmp(reason, "LOWBAT") == 0)) {
        code = APP_SAFETY_LOW_BATTERY;
    } else if ((reason != NULL) && (strcmp(reason, "WDT") == 0)) {
        code = APP_SAFETY_WATCHDOG;
    } else if ((reason != NULL) && (strcmp(reason, "MOTOR") == 0)) {
        code = APP_SAFETY_MOTOR_STALL;
    } else if ((reason != NULL) && (strcmp(reason, "STOP") == 0)) {
        code = APP_SAFETY_STOP;
    }

    emergency_stop = true;
    clear_stop_on_short_press = 0U;
    set_safety_status(code, (reason != NULL) ? reason : "EMG");
    NavigationTask_ClearGoal();
    Cancel_Relative_Move();
    Control_SetBaseSpeed(0.0f);
    Motor_StopAll();
}

void App_RequestEmergencyStopFromIsr(void)
{
    uint8_t was_stopped = emergency_stop ? 1U : 0U;

    emergency_stop = true;
    clear_stop_on_short_press = was_stopped;
    stop_cleanup_pending = 1U;
    safety_code = APP_SAFETY_EMERGENCY;
    safety_reason = "EMG";
    base_car_speed = 0.0f;
    g_pid_speed_left.setpoint = 0.0f;
    g_pid_speed_right.setpoint = 0.0f;
    Motor_StopAll();
}

void App_ClearEmergencyStop(void)
{
    if (battery_mv < BATTERY_CUTOFF_MV) {
        return;
    }
    emergency_stop = false;
    if (low_battery_latched) {
        set_safety_status(APP_SAFETY_LOW_BATTERY, "LOWBAT");
    } else {
        set_safety_status(APP_SAFETY_OK, "OK");
    }
}

void App_StartExploration(void)
{
    App_ClearEmergencyStop();
    if (!App_MotorsAllowed()) return;
    if (low_battery_latched) {
        App_StartReturnHome();
        return;
    }
    NavigationTask_ClearGoal();
    Cancel_Relative_Move();
    Control_SetBaseSpeed(0.0f);
    MappingTask_ResetGrid();
    RPLIDAR_StartRaw();
}

void App_StartReturnHome(void)
{
    if (emergency_stop) return;
    Cancel_Relative_Move();
    Control_SetBaseSpeed(0.0f);
    NavigationTask_RequestReturnHome();
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
    return (const char *)safety_reason;
}

void App_BeginBenchmark(void)
{
    benchmark_start_tick = HAL_GetTick();
    benchmark_exit_tick = 0U;
    benchmark_end_tick = 0U;
    benchmark_exit_time_ms = 0U;
    benchmark_return_time_ms = 0U;
    benchmark_state = APP_BENCHMARK_OUTBOUND;
}

void App_RecordExitReached(void)
{
    if (benchmark_state != APP_BENCHMARK_OUTBOUND) return;
    benchmark_exit_tick = HAL_GetTick();
    benchmark_exit_time_ms = benchmark_exit_tick - benchmark_start_tick;
    benchmark_state = APP_BENCHMARK_RETURNING;
}

void App_RecordReturnReached(void)
{
    if (benchmark_state != APP_BENCHMARK_RETURNING) return;
    benchmark_end_tick = HAL_GetTick();
    benchmark_return_time_ms = benchmark_end_tick - benchmark_exit_tick;
    benchmark_state = APP_BENCHMARK_COMPLETE;
}

void App_GetStatusSnapshot(AppStatusSnapshot_t *status)
{
    uint32_t now;

    if (status == NULL) return;
    now = HAL_GetTick();
    status->battery_mv = battery_mv;
    status->speed_limit_cmps = speed_limit_cmps;
    status->safety_code = (uint8_t)safety_code;
    status->emergency_stop = emergency_stop ? 1U : 0U;
    status->lidar_stream_active = lidar_raw_stream_active;
    status->benchmark_state = (uint8_t)benchmark_state;
    status->lidar_recovery_count = lidar_recovery_count;
    status->control_age_ms = now - last_control_tick;
    if (benchmark_state == APP_BENCHMARK_COMPLETE) {
        status->mission_elapsed_ms = benchmark_end_tick - benchmark_start_tick;
    } else if (benchmark_state != APP_BENCHMARK_IDLE) {
        status->mission_elapsed_ms = now - benchmark_start_tick;
    } else {
        status->mission_elapsed_ms = 0U;
    }
    status->exit_time_ms = benchmark_exit_time_ms;
    status->return_time_ms = benchmark_return_time_ms;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {
        App_RequestEmergencyStopFromIsr();
    }
}
