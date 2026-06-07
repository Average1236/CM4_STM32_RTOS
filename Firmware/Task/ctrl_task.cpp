#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include "iwdg.h"
#include "Component/control_params.hpp"
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

                // Dribbler: sync raw voltage from heartbeat
                robot.infra_voltage = robot.dribbler.infra_voltage_raw;
                // Dribbler: filter infra voltage (asymmetric LPF)
                robot.dribbler.filter_voltage();
                // Dribbler: state machine (builds pending CAN messages)
                robot.dribbler.process_state_machine();
                // Dribbler: build torque command (only when in closed-loop control)
                can_Message_t dribbler_torque_msg;
                robot.dribbler.build_torque_msg(dribbler_torque_msg, robot.dribble_power);

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
                        robot.request_kick_from_spi();
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
                
                can_Message_t wheel_msgs[4];
                can_Message_t extra_can_msgs[4];
                uint8_t extra_msg_count = 0;
                uint32_t current_tick = HAL_GetTick();

                for (uint8_t i = 0; i < 4; i++) {
                    const bool safe_output = !robot.wheel_motors[i]->is_enabled() || !robot.watchdog_check();
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
                    // Send dribbler pending messages (state transitions)
                    for (uint8_t i = 0; i < robot.dribbler.pending_count; i++) {
                        can1_bus.send_message(robot.dribbler.pending_msgs[i]);
                    }
                    // Send dribbler torque command (only when in closed-loop control)
                    if (robot.dribbler.current_state == DribblerZfoc::kAxisStateClosedLoopControl) {
                        can1_bus.send_message(dribbler_torque_msg);
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
