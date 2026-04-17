#ifndef CONTROL_LOGIC_H
#define CONTROL_LOGIC_H

#include <stdint.h>

float ControlLogic_WrapAngleDeg(float angle_deg);
float ControlLogic_ResolveAbsoluteSetpointFromCurrentHeading(float current_heading_deg, float delta_angle_deg);
uint8_t ControlLogic_ShouldPauseMappingForTurn(float current_heading_deg,
                                               float heading_setpoint_deg,
                                               float base_speed_mps,
                                               float left_speed_mps,
                                               float right_speed_mps,
                                               uint8_t move_state);
uint8_t ControlLogic_ShouldFuseCorrectedControlPose(float current_heading_deg,
                                                    float heading_setpoint_deg,
                                                    float base_speed_mps,
                                                    float left_speed_mps,
                                                    float right_speed_mps,
                                                    uint8_t move_state);

#endif /* CONTROL_LOGIC_H */