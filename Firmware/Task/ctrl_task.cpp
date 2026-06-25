#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include "iwdg.h"
#include "Component/control_params.hpp"
#include <cmath>
#include <optional>

namespace {

enum WheelCommandMode : uint8_t {
    kWheelCommandVelocity = 0,
    kWheelCommandMIT = 1,
};

static constexpr WheelCommandMode kWheelCommandMode = kWheelCommandMIT;

struct MITCommandConfig {
    float kp;
    float kd;
    float torque_ff;
};

static constexpr MITCommandConfig kMITRunConfig{
    control_config::kMITRunKp,
    control_config::kMITRunKd,
    control_config::kMITRunTorqueFf,
};
static constexpr MITCommandConfig kMITSafeConfig{
    control_config::kMITSafeKp,
    control_config::kMITSafeKd,
    control_config::kMITSafeTorqueFf,
};
static constexpr float kRadPerRpm = 3.1415926535f / 30.0f;
static constexpr float kDegToRad = 3.1415926535f / 180.0f;
static constexpr uint32_t kCanTxBudgetUs = 400;
static constexpr float kMITKdRampTimeSec = control_config::kMITKdRampTimeSec;
static constexpr float kMITKdRampStep =
    (kMITKdRampTimeSec > 1e-6f) ? (control_config::kControlDtSec / kMITKdRampTimeSec) : 1.0f;

float g_kd_ramp_alpha[4] = {0.0f, 0.0f, 0.0f, 0.0f};
bool g_wheel_prev_enabled[4] = {false, false, false, false};

WheelMotorBase::Mode active_motor_mode() {
    return (kWheelCommandMode == kWheelCommandMIT)
               ? WheelMotorBase::kModeMITControl
               : WheelMotorBase::kModeVelocityControl;
}

bool build_wheel_command(Robot& robot, uint8_t index, bool safe_output, can_Message_t& out_msg) {
    if (index >= 4) {
        return false;
    }

    WheelMotorBase* motor = robot.wheel_motors[index];
    const bool motor_enabled = motor->is_enabled();

    if (!motor_enabled) {
        g_wheel_prev_enabled[index] = false;
        g_kd_ramp_alpha[index] = 0.0f;
    } else if (!g_wheel_prev_enabled[index]) {
        // Rising edge of enable: restart Kd ramp from zero.
        g_wheel_prev_enabled[index] = true;
        g_kd_ramp_alpha[index] = 0.0f;
    }

    const WheelMotorBase::Mode required_mode = active_motor_mode();
    if (motor->get_mode() != required_mode) {
        motor->reset_wheel_speed_pid();
        motor->build_set_mode_msg(required_mode, out_msg);
        return true;
    }

    out_msg.id = motor->command_can_id();
    out_msg.isExt = false;
    out_msg.rtr = false;

    const std::optional<float> vel_cmd_rpm_opt = motor->velocity_cmd_input_port()->any();
    const float vel_cmd_rpm = vel_cmd_rpm_opt.has_value() ? *vel_cmd_rpm_opt : robot.motor_vel[index];

    if (kWheelCommandMode == kWheelCommandMIT) {
        uint8_t tx_data[8] = {0};
        const std::optional<float> torque_ff_opt = motor->torque_ff_cmd_input_port()->any();
        const float torque_ff = torque_ff_opt.has_value() ? *torque_ff_opt : 0.0f;
        const float velocity_ref = safe_output ? 0.0f : (vel_cmd_rpm * kRadPerRpm);
        const float position_ref = motor->get_angle() * kDegToRad;
        const MITCommandConfig cfg = safe_output ? kMITSafeConfig : kMITRunConfig;
        float kd_cmd = cfg.kd;

        if (!safe_output && motor_enabled) {
            kd_cmd = cfg.kd * g_kd_ramp_alpha[index];
            g_kd_ramp_alpha[index] += kMITKdRampStep;
            if (g_kd_ramp_alpha[index] > 1.0f) {
                g_kd_ramp_alpha[index] = 1.0f;
            }
        }

        if (safe_output) {
            motor->reset_wheel_speed_pid();
        }
        motor->pack_mit_data(position_ref, velocity_ref, cfg.kp, kd_cmd, (safe_output ? cfg.torque_ff : torque_ff), tx_data);
        out_msg.len = 8;
        memcpy(out_msg.buf, tx_data, out_msg.len);
        return true;
    }

    if (kWheelCommandMode == kWheelCommandVelocity) {
        motor->reset_wheel_speed_pid();
        uint8_t tx_data[4] = {0};
        const float velocity_ref = safe_output ? 0.0f : (vel_cmd_rpm * kRadPerRpm);
        motor->pack_velocity_data(velocity_ref, tx_data);
        out_msg.len = 4;
        memcpy(out_msg.buf, tx_data, out_msg.len);
        return true;
    }

    return false;
}


} // namespace

// Dribbler CAN debug (defined in dribbler_zfoc.cpp)
extern volatile uint32_t dribbler_can_cmd_id_debug;
extern volatile float dribbler_can_torque_debug;
extern volatile float dribbler_can_velocity_debug;

// Debug variables
float debug_pose, debug_vel;
float debug_K, debug_D, debug_M, debug_angle_ref;
bool debug_enable;
volatile uint32_t wait_us_debug = 0;
volatile uint32_t exec_time_us = 0;
volatile uint32_t tx_cmd_sent_count = 0;
volatile uint32_t tx_cmd_drop_count = 0;
volatile uint32_t ctrl_dt_us_debug = 0;

float wheelInput[4];
float debug_motor_vel[4];
float debug_motor_acc[4];

void motor_init() {
    volatile uint8_t i;
    for (i = 0; i < 4; ++i) {
        can_Message_t msg;
        robot.wheel_motors[i]->build_set_mode_msg(active_motor_mode(), msg);
        can2_bus.send_message(msg);
        osDelay(20);
        robot.wheel_motors[i]->build_set_acc_msg(MotorDMH3510::Parameter_t().acc, msg);
        can2_bus.send_message(msg);
        osDelay(20);
        robot.wheel_motors[i]->build_set_dec_msg(MotorDMH3510::Parameter_t().dec, msg);
        can2_bus.send_message(msg);
        osDelay(20);
        robot.wheel_motors[i]->build_set_pmax_msg(MotorDMH3510::Parameter_t().pmax, msg);
        can2_bus.send_message(msg);
        osDelay(20);
        if (kWheelCommandMode == kWheelCommandVelocity) {
            robot.wheel_motors[i]->build_set_velocity_kp_msg(MotorDMH3510::Parameter_t().kp_asr, msg);
            can2_bus.send_message(msg);
            osDelay(20);
            robot.wheel_motors[i]->build_set_velocity_ki_msg(MotorDMH3510::Parameter_t().ki_asr, msg);
            can2_bus.send_message(msg);
            osDelay(20);
        }
        // HAL_IWDG_Refresh(&hiwdg);
        // osDelay(400);
        // HAL_IWDG_Refresh(&hiwdg);
        robot.wheel_motors[i]->build_enable_msg(msg);
        can2_bus.send_message(msg);
        osDelay(20);
        can_Message_t wheel_msg;
        if (build_wheel_command(robot, i, true, wheel_msg)) {
            can2_bus.send_message(wheel_msg);
        }
        osDelay(20);
    }
}

enum MotorRecoverState : uint8_t {
    kMotorNormal = 0,
    kMotorDisabledWait,
    kMotorClearErrorWait
};

extern "C" {
    
void StartCrtlTask(void *argument) {
    // Wait for system initialization
    osDelay(500);

    robot.bind_estimator_imu_ports(imu);
    robot.bind_estimator_optflow_ports(opt_flow);

    // Connect OptFlow IMU input ports (cross-task Port-based data flow)
    opt_flow.imu_acc_x_input()->connect_to(imu.acc_x_port());
    opt_flow.imu_acc_y_input()->connect_to(imu.acc_y_port());
    opt_flow.imu_acc_z_input()->connect_to(imu.acc_z_port());
    opt_flow.imu_gyro_x_input()->connect_to(imu.omega_x_port());
    opt_flow.imu_gyro_y_input()->connect_to(imu.omega_y_port());
    opt_flow.imu_gyro_z_input()->connect_to(imu.omega_z_port());

    // Initialize motors (set control mode, PID gains, enable)
    motor_init();
    
    uint32_t ctrl_start_tick = 0;
    uint32_t ctrl_end_tick = 0;
    uint32_t ctrl_last_time = 0;
    bool ctrl_first = true;
    uint8_t tx_rr_start = 0;

    MotorRecoverState motor_recover_state[4] = {kMotorNormal, kMotorNormal, kMotorNormal, kMotorNormal};
    uint32_t motor_recover_tick[4] = {0};

    // Dribbler runtime state tracking
    uint32_t last_heartbeat_count = 0;
    uint8_t last_dribbler_mode = 0;
    bool last_active_torque_mode = false;
    (void)last_active_torque_mode;  // silence unused warning when torque mode disabled
    uint8_t hybrid_switch_delay = 0;   // skip cmd for 1 frame after mode switch

    for(;;) {
        if (osSemaphoreAcquire(sem_ctrl_triggerHandle, osWaitForever) == osOK) {
            ctrl_start_tick = TIM2->CNT;

            const uint32_t ctrl_now = TIM13->CNT;
            const uint32_t ctrl_dt_us = ctrl_first ? TIM2_PERIOD_CLOCKS
                : ((ctrl_now >= ctrl_last_time) ? (ctrl_now - ctrl_last_time)
                                                : (ctrl_now + 65536u - ctrl_last_time));
            ctrl_last_time = ctrl_now;
            ctrl_first = false;

            ctrl_dt_us_debug = ctrl_dt_us;
            
            // Acquire robot state mutex
            if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                imu.reset_ports();
                for (uint8_t i = 0; i < 4; i++) {
                    robot.wheel_motors[i]->reset_ports();
                }
                
                bool any_disabled = false;
                for (uint8_t i = 0; i < 4; i++) {
                    if (!robot.wheel_motors[i]->is_enabled()) {
                        any_disabled = true;
                        break;
                    }
                }
                
                if (any_disabled) {
                    robot.robot_vel[0] = 0.0f;
                    robot.robot_vel[1] = 0.0f;
                    robot.robot_vel[2] = 0.0f;
                }

                // ============ Dribbler: Stage A – Sensor Input ============
                robot.infra_voltage = robot.dribbler.infra_voltage_raw;
                robot.dribbler.filter_voltage();  // asymmetric LPF

                // ============ Stage B – Mode Dispatch & ZFOC State Machine ============
                // B1: determine ZFOC torque/velocity mode, run startup state machine
                const bool need_torque_mode =
                    (robot.dribbler_mode == control_config::kDribblerModeTorque) ||
                    (robot.dribbler_mode == control_config::kDribblerModeHybrid &&
                     robot.dribbler_hybrid_phase == Robot::kDribblerHybridTorquePhase);
                robot.dribbler.process_state_machine(need_torque_mode);

                // B2: CM4 mode change → queue hot switch (safe-append, never clears pending)
                if (robot.dribbler_mode != 0 && robot.dribbler_mode != last_dribbler_mode) {
                    if (robot.dribbler.current_state == DribblerZfoc::kAxisStateClosedLoopControl) {
                        if (robot.dribbler_mode == control_config::kDribblerModeTorque) {
                            robot.dribbler.queue_controller_mode_switch(true);
                        } else if (robot.dribbler_mode == control_config::kDribblerModeSpeed) {
                            robot.dribbler.queue_controller_mode_switch(false);
                        } else if (robot.dribbler_mode == control_config::kDribblerModeHybrid) {
                            robot.dribbler.queue_controller_mode_switch(true);  // enter hybrid: start torque
                        }
                    }
                }
                last_dribbler_mode = robot.dribbler_mode;

                // ============ Stage C – Hybrid (30) Anti-Bounce State Machine ============
                if (robot.dribbler_mode == control_config::kDribblerModeHybrid) {
                    // Only count on new heartbeat frames (200 Hz, every 5 ms)
                    if (robot.dribbler.heartbeat_count != last_heartbeat_count) {
                        last_heartbeat_count = robot.dribbler.heartbeat_count;
                        const bool has_ball = (robot.dribbler.infra_voltage_filt > INFRARED_THRESHOLD);

                        if (robot.dribbler_hybrid_phase == Robot::kDribblerHybridTorquePhase) {
                            // ---- Torque phase: count consecutive ball-hold frames ----
                            if (has_ball) {
                                robot.dribbler_ball_hold_count++;
                                if (robot.dribbler_ball_hold_count >= control_config::kDribblerHybridBallHoldFrames) {
                                    // Confirmed ball hold → switch to speed phase
                                    robot.dribbler_hybrid_phase = Robot::kDribblerHybridSpeedPhase;
                                    robot.dribbler_ball_hold_count = 0;  // reuse for lost-ball counting
                                    if (robot.dribbler.current_state == DribblerZfoc::kAxisStateClosedLoopControl) {
                                        robot.dribbler.queue_controller_mode_switch(false);
                                    }
                                }
                            } else {
                                // Anti-bounce: fast-decrement, never reset-to-zero on a single frame
                                if (robot.dribbler_ball_hold_count > control_config::kDribblerHybridDebounceStep) {
                                    robot.dribbler_ball_hold_count -= control_config::kDribblerHybridDebounceStep;
                                } else {
                                    robot.dribbler_ball_hold_count = 0;
                                }
                            }
                        } else {
                            // ---- Speed phase: count consecutive lost-ball frames for fallback ----
                            if (!has_ball) {
                                robot.dribbler_ball_hold_count++;
                                if (robot.dribbler_ball_hold_count >= control_config::kDribblerHybridBallLostFrames) {
                                    // Confirmed ball lost → fall back to torque phase
                                    robot.dribbler_hybrid_phase = Robot::kDribblerHybridTorquePhase;
                                    robot.dribbler_ball_hold_count = 0;
                                    if (robot.dribbler.current_state == DribblerZfoc::kAxisStateClosedLoopControl) {
                                        robot.dribbler.queue_controller_mode_switch(true);
                                    }
                                }
                            } else {
                                // Ball reappeared → reduce lost-ball suspicion
                                if (robot.dribbler_ball_hold_count > 0) {
                                    robot.dribbler_ball_hold_count--;
                                }
                            }
                        }
                    }
                }

                // ============ Stage D – Build CAN Command (Explicit 4-Branch Dispatch) ============
                const bool dribbler_enabled = (robot.dribble_power != 0);
                const float chassis_vx = robot.robot_real_vel[0];  // backward is negative
                can_Message_t dribbler_cmd_msg;
                can_Message_t dribbler_stop_msg;

                if (robot.dribbler_mode == control_config::kDribblerModeTorque) {
                    // ── Branch 1: Pure Torque (10) ──
                    if (dribbler_enabled) {
                        last_active_torque_mode = true;
                        robot.dribbler.build_torque_msg(dribbler_cmd_msg, -0.05f, 0.0f);
                        dribbler_can_cmd_id_debug = DribblerZfoc::kCanIdSetInputTorque;
                        dribbler_can_torque_debug = robot.dribble_torque_ff;
                        dribbler_can_velocity_debug = 0.0f;
                    }
                    robot.dribbler.build_torque_msg(dribbler_stop_msg, 0.0f, 0.0f);

                } else if (robot.dribbler_mode == control_config::kDribblerModeSpeed) {
                    // ── Branch 2: Pure Speed (20) ──
                    if (dribbler_enabled) {
                        last_active_torque_mode = false;
                        const float torque_limit_abs = fabsf(robot.dribble_torque_ff);
                        robot.dribbler.build_velocity_msg(dribbler_cmd_msg, robot.dribble_velocity, torque_limit_abs);
                        dribbler_can_cmd_id_debug = DribblerZfoc::kCanIdSetInputVelocity;
                        dribbler_can_torque_debug = torque_limit_abs;
                        dribbler_can_velocity_debug = robot.dribble_velocity;
                    }
                    robot.dribbler.build_velocity_msg(dribbler_stop_msg, 0.0f, 0.0f);

                } else if (robot.dribbler_mode == control_config::kDribblerModeHybrid) {
                    // ── Branch 3: Hybrid (30) ──
                    if (robot.dribbler_hybrid_phase == Robot::kDribblerHybridTorquePhase) {
                        // Torque phase: fixed torque -0.07 Nm
                        if (dribbler_enabled) {
                            last_active_torque_mode = true;
                            robot.dribbler.build_torque_msg(dribbler_cmd_msg,
                                control_config::kDribblerHybridTorqueNm, 0.0f);
                            dribbler_can_cmd_id_debug = DribblerZfoc::kCanIdSetInputTorque;
                            dribbler_can_torque_debug = control_config::kDribblerHybridTorqueNm;
                            dribbler_can_velocity_debug = 0.0f;
                        }
                        robot.dribbler.build_torque_msg(dribbler_stop_msg, 0.0f, 0.0f);
                    } else {
                        // Speed phase: fixed speed -100 rps, no chassis compensation
                        if (dribbler_enabled) {
                            last_active_torque_mode = false;
                            robot.dribbler.build_velocity_msg(dribbler_cmd_msg,
                                control_config::kDribblerHybridSpeedRps,
                                control_config::kDribblerHybridTorqueLimitNm);
                            dribbler_can_cmd_id_debug = DribblerZfoc::kCanIdSetInputVelocity;
                            dribbler_can_torque_debug = control_config::kDribblerHybridTorqueLimitNm;
                            dribbler_can_velocity_debug = control_config::kDribblerHybridSpeedRps;
                        }
                        robot.dribbler.build_velocity_msg(dribbler_stop_msg, 0.0f, 0.0f);
                    }

                } else {
                    // ── Branch 4: Off or Unknown ──
                    // Do not update last_active_torque_mode; keep last format for stop message
                    robot.dribbler.build_torque_msg(dribbler_stop_msg, 0.0f, 0.0f);
                    robot.dribbler.build_velocity_msg(dribbler_stop_msg, 0.0f, 0.0f);
                }

                // Kick trigger
                if (control_config::kTestKick) {
                    // Test mode: rising-edge on kick_discharge_time (0 → non-zero)
                    static uint16_t prev_discharge_time = 0;
                    if (robot.kick_discharge_time > 0 && prev_discharge_time == 0) {
                        robot.request_kick_from_spi();
                    }
                    prev_discharge_time = robot.kick_discharge_time;
                } else {
                    // Normal mode: require infrared ball detection
                    if (robot.dribbler.infra_voltage_filt > INFRARED_THRESHOLD) {
                        robot.infrare_flag = true;
                        robot.request_kick_from_spi();
                    } else {
                        robot.infrare_flag = false;
                    }
                }

                // Motion planning: compute acceleration from velocity setpoints
                const float dt_s = static_cast<float>(ctrl_dt_us / 1000000.0);
                robot.motion_planner(ctrl_dt_us);

                // Yaw angle control: S-curve planned omega (not raw P controller)
                robot.prepare_yaw_control(dt_s);

                // Inverse kinematics: compute wheel velocities
                robot.ik_solve();

                // Observer-based control law: compute wheel torque feedforward
                robot.update_torque_feedforward(ctrl_dt_us);

                // Stationary hold: blend wheel-speed PID authority based on
                // commanded chassis velocity and angular velocity.
                // Authority = 1.0 only when both |v_cmd|→0 and |ω_ref|→0.
                const float v_cmd_norm = sqrtf(robot.robot_real_vel[0] * robot.robot_real_vel[0] +
                                               robot.robot_real_vel[1] * robot.robot_real_vel[1]);
                const float alpha_v = 1.0f - std::clamp(v_cmd_norm / control_config::kStationaryHoldSpeedThreshold,
                                                        0.0f, 1.0f);
                const float omega_cmd_norm = fabsf(robot.chassis_controller.omega_ref());
                const float alpha_w = 1.0f - std::clamp(omega_cmd_norm / control_config::kStationaryHoldOmegaThreshold,
                                                        0.0f, 1.0f);
                const float hold_limit = control_config::kStationaryHoldPidOutputLimitNm * alpha_v * alpha_w;

                can_Message_t wheel_msgs[4];
                can_Message_t extra_can_msgs[4];
                uint8_t extra_msg_count = 0;
                uint32_t current_tick = HAL_GetTick();

                for (uint8_t i = 0; i < 4; i++) {
                    const bool safe_output = !robot.wheel_motors[i]->is_enabled() || !robot.watchdog_check();
                    robot.wheel_motors[i]->set_stationary_hold_limit(safe_output ? 0.0f : hold_limit);
                    build_wheel_command(robot, i, safe_output, wheel_msgs[i]);

                    if (!robot.wheel_motors[i]->is_enabled()) {
                        if (motor_recover_state[i] == kMotorNormal) {
                            motor_recover_state[i] = kMotorDisabledWait;
                            motor_recover_tick[i] = current_tick;
                        } else if (motor_recover_state[i] == kMotorDisabledWait) {
                            if (current_tick - motor_recover_tick[i] >= control_config::kMotorRecoverDelayMs) {
                                robot.wheel_motors[i]->build_clear_error_msg(extra_can_msgs[extra_msg_count++]);
                                motor_recover_state[i] = kMotorClearErrorWait;
                                motor_recover_tick[i] = current_tick;
                            }
                        } else if (motor_recover_state[i] == kMotorClearErrorWait) {
                            if (current_tick - motor_recover_tick[i] >= control_config::kMotorClearErrorToEnableDelayMs) {
                                robot.wheel_motors[i]->build_enable_msg(extra_can_msgs[extra_msg_count++]);
                                motor_recover_state[i] = kMotorNormal;
                            }
                        }
                    } else {
                        motor_recover_state[i] = kMotorNormal;
                    }
                }
            
                if (osSemaphoreAcquire(sem_can_txHandle, 10) == osOK) {
                    // Send dribbler pending messages (state transitions, only when enabled)
                    if (dribbler_enabled) {
                        uint8_t keep = 0;
                        for (uint8_t i = 0; i < robot.dribbler.pending_count; i++) {
                            if (can1_bus.send_message(robot.dribbler.pending_msgs[i])) {
                                // sent OK
                            } else {
                                // failed — shift down for retry next iteration
                                if (keep < i) {
                                    robot.dribbler.pending_msgs[keep] = robot.dribbler.pending_msgs[i];
                                }
                                keep++;
                            }
                        }
                        robot.dribbler.pending_count = keep;  // only clear messages that were sent
                    }
                    // Send dribbler command only after queued mode-switch messages have been sent.
                    // This avoids one velocity-control frame with a stale zero velocity target.
                    const bool dribbler_mode_update_pending =
                        dribbler_enabled && (robot.dribbler.pending_count > 0);
                    if (robot.dribbler.current_state == DribblerZfoc::kAxisStateClosedLoopControl) {
                        if (!dribbler_mode_update_pending) {
                            can1_bus.send_message(dribbler_enabled ? dribbler_cmd_msg : dribbler_stop_msg);
                        }
                    }

                    bool sent[4] = {false, false, false, false};
                    uint8_t sent_count = 0;
                    uint8_t extra_sent_count = 0;
                    const uint32_t tx_start_tick = TIM2->CNT;

                    while (sent_count < 4 || extra_sent_count < extra_msg_count) {
                        for (uint8_t k = 0; k < 4; ++k) {
                            const uint8_t idx = (tx_rr_start + k) & 0x03;
                            if (sent[idx]) continue;
                            if (can2_bus.send_message(wheel_msgs[idx])) {
                                sent[idx] = true;
                                sent_count++;
                                tx_cmd_sent_count++;
                            }
                        }
                        
                        while (extra_sent_count < extra_msg_count) {
                            if (can2_bus.send_message(extra_can_msgs[extra_sent_count])) {
                                extra_sent_count++;
                            } else {
                                break;
                            }
                        }

                        uint32_t wait_us = TIM2->CNT - tx_start_tick;
                        wait_us_debug = wait_us;
                        if ((sent_count == 4 && extra_sent_count == extra_msg_count) || wait_us >= kCanTxBudgetUs) {
                            break;
                        }
                    }

                    tx_cmd_drop_count += (4 - sent_count) + (extra_msg_count - extra_sent_count);
                    tx_rr_start = (tx_rr_start + 1) & 0x03;
                    osSemaphoreRelease(sem_can_txHandle);
                }
                
                // Release mutex
                osMutexRelease(mtx_robot_stateHandle);
            }
            
            ctrl_end_tick = TIM2->CNT;
            // Control loop execution time can be monitored here
            exec_time_us = (ctrl_end_tick >= ctrl_start_tick) ? (ctrl_end_tick - ctrl_start_tick) : (0xFFFFFFFF - ctrl_start_tick + ctrl_end_tick);
        }
    }
}

} // extern "C"
