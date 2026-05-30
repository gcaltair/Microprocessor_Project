#include <math.h>

#include "../Inc/control_logic.h"

#define CONTROL_LOGIC_BASE_SPEED_ACTIVE_THRESHOLD_MPS   0.001f
#define CONTROL_LOGIC_TURN_WHEEL_SPEED_THRESHOLD_MPS    0.02f
#define CONTROL_LOGIC_TURN_HEADING_ERROR_THRESHOLD_DEG  3.0f
#define CONTROL_LOGIC_MOVE_STATE_TURNING                1U
#define CONTROL_LOGIC_MOVE_STATE_IDLE                   0U

float ControlLogic_WrapAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

float ControlLogic_ResolveAbsoluteSetpointFromCurrentHeading(float current_heading_deg, float delta_angle_deg)
{
    return ControlLogic_WrapAngleDeg(current_heading_deg + delta_angle_deg);
}

static uint8_t control_logic_has_manual_turn_intent(float current_heading_deg,
                                                    float heading_setpoint_deg,
                                                    float base_speed_mps)
{
    float heading_error_deg;

    if (fabsf(base_speed_mps) > CONTROL_LOGIC_BASE_SPEED_ACTIVE_THRESHOLD_MPS) {
        return 0U;
    }

    heading_error_deg = fabsf(ControlLogic_WrapAngleDeg(heading_setpoint_deg - current_heading_deg));
    return (heading_error_deg >= CONTROL_LOGIC_TURN_HEADING_ERROR_THRESHOLD_DEG) ? 1U : 0U;
}

static uint8_t control_logic_wheels_show_in_place_turn(float base_speed_mps,
                                                       float left_speed_mps,
                                                       float right_speed_mps)
{
    if (fabsf(base_speed_mps) > CONTROL_LOGIC_BASE_SPEED_ACTIVE_THRESHOLD_MPS) {
        return 0U;
    }

    if ((fabsf(left_speed_mps) < CONTROL_LOGIC_TURN_WHEEL_SPEED_THRESHOLD_MPS) ||
        (fabsf(right_speed_mps) < CONTROL_LOGIC_TURN_WHEEL_SPEED_THRESHOLD_MPS)) {
        return 0U;
    }

    return ((left_speed_mps > 0.0f) && (right_speed_mps < 0.0f)) ||
           ((left_speed_mps < 0.0f) && (right_speed_mps > 0.0f));
}

uint8_t ControlLogic_ShouldPauseMappingForTurn(float current_heading_deg,
                                               float heading_setpoint_deg,
                                               float base_speed_mps,
                                               float left_speed_mps,
                                               float right_speed_mps,
                                               uint8_t move_state)
{
    //第一层：状态机判断
    if (move_state == CONTROL_LOGIC_MOVE_STATE_TURNING) {
        return 1U;
    }

    if (control_logic_has_manual_turn_intent(current_heading_deg,
                                             heading_setpoint_deg,
                                             base_speed_mps) != 0U) {
        return 1U;
    }

    return control_logic_wheels_show_in_place_turn(base_speed_mps,
                                                   left_speed_mps,
                                                   right_speed_mps);
}

