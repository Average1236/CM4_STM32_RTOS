#include "robot.hpp"
#include "control_params.hpp"
#include "tim.h"
#include <cstring>
#include <cmath>
#include <algorithm>

// Debug
volatile float target_vx_debug = 0;
volatile float target_vy_debug = 0;
volatile float target_vw_debug = 0;
volatile float robot_ay_debug = 0;
volatile float wheel_vel_debug = 0;
volatile float robot_real_vx_debug = 0;
volatile uint16_t kick_pulse_debug = 0;
volatile float dribble_power_debug = 0;
volatile int use_imu_debug = 0;
volatile float dribble_velocity_debug = 0;
volatile float dribble_torque_ff_debug = 0;
volatile float yaw_vel_max_debug = 0;
volatile float yaw_acc_max_debug = 0;
volatile float yaw_jerk_max_debug = 0;

// Wheel geometry
static constexpr float WHEEL_ANGLE_FORWARD = control_config::kWheelAlphaRad;
static constexpr float WHEEL_ANGLE_BACKWARD = control_config::kWheelBetaRad;

// Pre-computed trigonometric values
static const float SIN_WHEEL_ANGLE_FWD = sinf(WHEEL_ANGLE_FORWARD);
static const float COS_WHEEL_ANGLE_FWD = cosf(WHEEL_ANGLE_FORWARD);
static const float SIN_WHEEL_ANGLE_BWD = sinf(WHEEL_ANGLE_BACKWARD);
static const float COS_WHEEL_ANGLE_BWD = cosf(WHEEL_ANGLE_BACKWARD);

// Robot dynamics parameters
static constexpr float ROBOT_RADIUS = control_config::kWheelCenterDistanceM;
static constexpr float WHEEL_RADIUS = control_config::kWheelRadiusM;

namespace {

struct AxisMotionPlan {
    float v_start = 0.0f;
    float v_target = 0.0f;
    float sign = 1.0f;
    float t_jerk = 0.0f;
    float t_const_acc = 0.0f;
    float t_total = 0.0f;
    float elapsed = 0.0f;
    bool has_const_acc = false;
    bool active = false;
};

AxisMotionPlan g_axis_plans[3];

constexpr float kPlannerVelEps = 1e-5f;
constexpr float kPlannerReplanEps = 1e-4f;
constexpr uint16_t kKickPulseMaxUs = 15000;
constexpr uint32_t kKickIntervalMs = 300;

inline float signf_nonzero(const float x) {
    return (x >= 0.0f) ? 1.0f : -1.0f;
}

int16_t encode_i16(const float value) {
    if (!std::isfinite(value)) {
        return 0;
    }

    return static_cast<int16_t>(std::lroundf(std::clamp(value, -32768.0f, 32767.0f)));
}

int16_t encode_velocity_mm_s(const float velocity_m_s) {
    return encode_i16(velocity_m_s * 1000.0f);
}

int16_t encode_accel_mm_s2(const float accel_m_s2) {
    return encode_i16(accel_m_s2 * 1000.0f);
}

int16_t encode_linear_jerk_deci_m_s3(const float jerk_m_s3) {
    return encode_i16(jerk_m_s3 * 10.0f);
}

int16_t encode_angular_centi(const float value) {
    return encode_i16(value * 100.0f);
}

float decomposed_axis_velocity_limit_m_s(const float wheel_axis_coeff[4], const float wheel_vel_limit_rpm) {
    float max_coeff = 0.0f;
    for (uint8_t i = 0; i < 4; ++i) {
        max_coeff = fmaxf(max_coeff, fabsf(wheel_axis_coeff[i]));
    }

    if (max_coeff <= 1e-6f) {
        return 0.0f;
    }

    const float wheel_vel_limit_rad_s = wheel_vel_limit_rpm * control_config::kPi / 30.0f;
    return wheel_vel_limit_rad_s * WHEEL_RADIUS / max_coeff;
}

void init_axis_plan(
    AxisMotionPlan& plan,
    const float v_start,
    const float v_target,
    const float a_max,
    const float j_max
) {
    plan.v_start = v_start;
    plan.v_target = v_target;
    plan.elapsed = 0.0f;
    plan.active = false;
    plan.has_const_acc = false;
    plan.t_jerk = 0.0f;
    plan.t_const_acc = 0.0f;
    plan.t_total = 0.0f;

    if (a_max <= 1e-8f || j_max <= 1e-8f) {
        return;
    }

    const float delta_v = v_target - v_start;
    const float delta = fabsf(delta_v);
    if (delta < kPlannerVelEps) {
        return;
    }

    plan.sign = signf_nonzero(delta_v);

    const float threshold = (a_max * a_max) / j_max;
    if (delta >= threshold) {
        plan.has_const_acc = true;
        plan.t_jerk = a_max / j_max;
        plan.t_const_acc = (delta / a_max) - plan.t_jerk;
        plan.t_total = 2.0f * plan.t_jerk + plan.t_const_acc;
    } else {
        plan.has_const_acc = false;
        plan.t_jerk = sqrtf(delta / j_max);
        plan.t_const_acc = 0.0f;
        plan.t_total = 2.0f * plan.t_jerk;
    }

    plan.active = plan.t_total > 1e-8f;
}

void sample_axis_plan(
    const AxisMotionPlan& plan,
    const float j_max,
    const float a_max,
    const float t,
    float& a_ref,
    float& v_ref
) {
    if (!plan.active) {
        a_ref = 0.0f;
        v_ref = plan.v_target;
        return;
    }

    const float tc = std::clamp(t, 0.0f, plan.t_total);
    const float s = plan.sign;

    if (plan.has_const_acc) {
        if (tc < plan.t_jerk) {
            a_ref = s * j_max * tc;
            v_ref = plan.v_start + 0.5f * s * j_max * tc * tc;
            return;
        }

        if (tc < (plan.t_jerk + plan.t_const_acc)) {
            a_ref = s * a_max;
            v_ref = plan.v_start + s * a_max * (tc - 0.5f * plan.t_jerk);
            return;
        }

        const float rem = plan.t_total - tc;
        const float td = tc - plan.t_jerk - plan.t_const_acc;
        a_ref = s * (a_max - j_max * td);
        v_ref = plan.v_target - 0.5f * s * j_max * rem * rem;
        return;
    }

    if (tc < plan.t_jerk) {
        a_ref = s * j_max * tc;
        v_ref = plan.v_start + 0.5f * s * j_max * tc * tc;
        return;
    }

    const float rem = plan.t_total - tc;
    a_ref = s * j_max * rem;
    v_ref = plan.v_target - 0.5f * s * j_max * rem * rem;
}

} // namespace

// Wheel velocity angle matrices
static const float WHEEL_VX_ANGLE[4] = {
    COS_WHEEL_ANGLE_FWD, -COS_WHEEL_ANGLE_FWD, 
    -COS_WHEEL_ANGLE_BWD, COS_WHEEL_ANGLE_BWD
};

static const float WHEEL_VY_ANGLE[4] = {
    -SIN_WHEEL_ANGLE_FWD, -SIN_WHEEL_ANGLE_FWD, 
    SIN_WHEEL_ANGLE_BWD, SIN_WHEEL_ANGLE_BWD
};

// Motor parameters for wheel motors
static const WheelMotorBase::Config_t WHEEL_MOTOR_PARAMS[4] = {
    {.control_id = 1, .feedback_id = 1, .direction = 1.0f, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = false, .limit_vel_curr_proportion = 0.8f},
    {.control_id = 2, .feedback_id = 2, .direction = 1.0f, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = false, .limit_vel_curr_proportion = 0.8f},
    {.control_id = 3, .feedback_id = 3, .direction = 1.0f, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = false, .limit_vel_curr_proportion = 0.8f},
    {.control_id = 4, .feedback_id = 4, .direction = 1.0f, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = false, .limit_vel_curr_proportion = 0.8f}
};

Robot::Robot() {
    yaw_s_curve_.set_config({yaw_max_vel, yaw_max_acc,
                             control_config::kYawAngleLinearFallbackMinTimeSec});
    planner_start_vx_filter_.set_parameter({control_config::kPlannerStartVelocityLpfCutoffHz,
                                             control_config::kControlDtSec});
    planner_start_vy_filter_.set_parameter({control_config::kPlannerStartVelocityLpfCutoffHz,
                                             control_config::kControlDtSec});

    // Initialize wheel motors
    for (int i = 0; i < 4; i++) {
        wheel_motors[i] = new MotorDMH3510(WHEEL_MOTOR_PARAMS[i]);

        wheel_motors[i]->velocity_cmd_input_port()->connect_to(&motor_vel[i]);
        chassis_estimator.wheel_velocity_input_port(i)->connect_to(wheel_motors[i]->velocity_output_port());
        wheel_motors[i]->torque_ff_cmd_input_port()->connect_to(chassis_controller.wheel_torque_ff_output_port(i));
        chassis_controller.wheel_sent_torque_input_port(i)->connect_to(wheel_motors[i]->torque_cmd_output_port());
    }

    chassis_controller.chassis_vx_input_port()->connect_to(chassis_estimator.chassis_vx_output_port());
    chassis_controller.chassis_vy_input_port()->connect_to(chassis_estimator.chassis_vy_output_port());
    chassis_controller.chassis_omega_z_input_port()->connect_to(chassis_estimator.chassis_omega_z_output_port());
    chassis_controller.chassis_yaw_input_port()->connect_to(chassis_estimator.chassis_yaw_output_port());
    chassis_estimator.vision_vx_input_port()->connect_to(&raw_vision_vel_mm_s[0]);
    chassis_estimator.vision_vy_input_port()->connect_to(&raw_vision_vel_mm_s[1]);
    chassis_estimator.vision_source_input_port()->connect_to(&vision_source);
}

Robot::~Robot() {
    for (int i = 0; i < 4; i++) {
        delete wheel_motors[i];
    }
}

void Robot::pi_decode_spi() {
    memcpy(&SpiRx, spi_rx_data, sizeof(SpiRx));

    // Decode robot velocity commands from Pi (if implemented)
    for (uint8_t i = 0; i < 2; i++) {
        robot_vel[i] = SpiRx.vel[i] / 1000.0f;
    }
    robot_vel[2] = SpiRx.vel[2] / 100.0f;

    dribble_power = (SpiRx.drib_power != 0) ? 1.0f : 0.0f;
    dribble_velocity = SpiRx.drib_velocity / 100.0f;
    dribble_torque_ff = SpiRx.drib_torque_ff / 1000.0f;

    for (uint8_t i = 0; i < 2; ++i) {
        const float acc_cmd = SpiRx.xy_max_acc[i] / 1000.0f;
        const float jerk_cmd = SpiRx.xy_max_jerk[i] / 10.0f;
        const float dec_cmd = SpiRx.xy_max_dec[i] / 1000.0f;
        if (acc_cmd > 0.0f) xy_max_acc[i] = acc_cmd;
        if (jerk_cmd > 0.0f) xy_max_jerk[i] = jerk_cmd;
        if (dec_cmd > 0.0f) xy_max_dec[i] = dec_cmd;
    }
    const float yaw_vel_cmd = SpiRx.yaw_max_vel / 100.0f;
    const float yaw_acc_cmd = SpiRx.yaw_max_acc / 100.0f;
    const float yaw_jerk_cmd = SpiRx.yaw_max_jerk / 100.0f;
    if (yaw_vel_cmd > 0.0f) yaw_max_vel = yaw_vel_cmd;
    if (yaw_acc_cmd > 0.0f) yaw_max_acc = yaw_acc_cmd;
    if (yaw_jerk_cmd > 0.0f) yaw_max_jerk = yaw_jerk_cmd;
    yaw_vel_max_debug = yaw_max_vel;
    yaw_acc_max_debug = yaw_max_acc;
    yaw_jerk_max_debug = yaw_max_jerk;
    raw_vision_vel_mm_s[0] = static_cast<float>(SpiRx.raw_vision_vel[0]);
    raw_vision_vel_mm_s[1] = static_cast<float>(SpiRx.raw_vision_vel[1]);
    vision_source = static_cast<float>(SpiRx.vision_source);
    yaw_s_curve_.set_config({yaw_max_vel, yaw_max_acc,
                             control_config::kYawAngleLinearFallbackMinTimeSec});
    robot_vel[2] = std::clamp(robot_vel[2], -yaw_max_vel, yaw_max_vel);

    dribble_power = SpiRx.drib_power / 50.0f * -1.0f;

    kick_mode = SpiRx.kick_mode ? false : true;
    kick_discharge_time = SpiRx.kick_discharge_time;

    use_imu = SpiRx.use_imu;
    // use_imu = true;
    use_imu_debug = use_imu ? 1 : 0;

    if (use_imu) {
        const float new_target = wrap_to_pi(robot_vel[2]);
        if (fabsf(wrap_to_pi(new_target - yaw_target_rad)) > 1e-6f) {
            yaw_target_rad = new_target;
            const auto yaw = chassis_estimator.chassis_yaw_output_port()->any();
            const auto omega_z = chassis_estimator.chassis_omega_z_output_port()->any();
            yaw_s_curve_.set_target(yaw_target_rad,
                                    yaw.has_value() ? *yaw : 0.0f,
                                    omega_z.has_value() ? *omega_z : robot_real_vel[2]);
        }
    }

    // Debug
    target_vx_debug = robot_vel[0];
    target_vy_debug = robot_vel[1];
    target_vw_debug = robot_vel[2];
    dribble_power_debug = dribble_power;
    dribble_velocity_debug = dribble_velocity;
    dribble_torque_ff_debug = dribble_torque_ff;
}

void Robot::request_kick_from_spi() {
    // Debug
    kick_pulse_debug = kick_pulse_us_;

    if (kick_discharge_time == 0) {
        return;
    }

    // Busy policy: ignore new kick request while pulse is active.
    if (kick_active_) {
        return;
    }

    // Minimum interval between kicks: 300ms
    if (HAL_GetTick() - last_kick_tick_ < kKickIntervalMs) {
        return;
    }

    last_kick_tick_ = HAL_GetTick();
    const uint16_t pulse_us = static_cast<uint16_t>(std::min<uint32_t>(kick_discharge_time, kKickPulseMaxUs));
    start_kick_pulse(kick_mode, pulse_us);
}

void Robot::start_kick_pulse(const bool chip_mode, const uint16_t pulse_us) {
    if (pulse_us == 0) {
        return;
    }

    // Ensure exclusive output: clear both lines before selecting one.
    HAL_GPIO_WritePin(CHIP_GPIO_Port, CHIP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHOOT_GPIO_Port, SHOOT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(chip_mode ? CHIP_GPIO_Port : SHOOT_GPIO_Port,
                      chip_mode ? CHIP_Pin : SHOOT_Pin,
                      GPIO_PIN_SET);

    kick_active_ = true;
    kick_pulse_us_ = pulse_us;

    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim6, pulse_us - 1u);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        HAL_GPIO_WritePin(CHIP_GPIO_Port, CHIP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SHOOT_GPIO_Port, SHOOT_Pin, GPIO_PIN_RESET);
        kick_active_ = false;
        kick_pulse_us_ = 0;
    }
}

void Robot::on_kick_timeout_irq() {
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_GPIO_WritePin(CHIP_GPIO_Port, CHIP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHOOT_GPIO_Port, SHOOT_Pin, GPIO_PIN_RESET);
    kick_active_ = false;
    kick_pulse_us_ = 0;
}

void Robot::pi_encode_spi() {
    float imu_data[9] = {0.0f};
    imu.get_data(imu_data);

    // SpiTx.infrare_flag = (infra_ADC1_val > 0.5f) ? 1 : 0;
    SpiTx.getBall = false;
    SpiTx.imu_online = true;
    SpiTx.battery_vol = static_cast<int16_t>(bat_ADC2_val * 5);
    SpiTx.cap_vol = static_cast<int16_t>(cap_ADC3_val * 100);

    // Encode wheel motor velocities
    for (uint8_t i = 0; i < 4; i++) {
        SpiTx.wheel[i] = static_cast<int16_t>(wheel_motors[i]->get_velocity() * 10);
        // SpiTx.wheel_ref[i] = static_cast<int16_t>(motor_vel[i] * 10);
    }

    const auto chassis_vx = chassis_estimator.chassis_vx_output_port()->any();
    const auto chassis_vy = chassis_estimator.chassis_vy_output_port()->any();
    SpiTx.odom_vel[0] = encode_velocity_mm_s(chassis_vx.has_value() ? *chassis_vx : 0.0f);
    SpiTx.odom_vel[1] = encode_velocity_mm_s(chassis_vy.has_value() ? *chassis_vy : 0.0f);

    const auto optflow_body_vx = (optflow_body_vx_port_ != nullptr) ? optflow_body_vx_port_->any() : std::optional<float>{};
    const auto optflow_body_vy = (optflow_body_vy_port_ != nullptr) ? optflow_body_vy_port_->any() : std::optional<float>{};
    const auto optflow_kf_vx = (optflow_kf_vx_port_ != nullptr) ? optflow_kf_vx_port_->any() : std::optional<float>{};
    const auto optflow_kf_vy = (optflow_kf_vy_port_ != nullptr) ? optflow_kf_vy_port_->any() : std::optional<float>{};
    const auto wheel_chassis_vx = chassis_estimator.wheel_chassis_vx_output_port()->any();
    const auto wheel_chassis_vy = chassis_estimator.wheel_chassis_vy_output_port()->any();
    const auto fused_chassis_vx = chassis_estimator.fused_chassis_vx_output_port()->any();
    const auto fused_chassis_vy = chassis_estimator.fused_chassis_vy_output_port()->any();

    SpiTx.optflow_body_vel[0] = encode_i16(optflow_body_vx.has_value() ? *optflow_body_vx : 0.0f);
    SpiTx.optflow_body_vel[1] = encode_i16(optflow_body_vy.has_value() ? *optflow_body_vy : 0.0f);
    SpiTx.optflow_kf_vel[0] = encode_i16(optflow_kf_vx.has_value() ? *optflow_kf_vx : 0.0f);
    SpiTx.optflow_kf_vel[1] = encode_i16(optflow_kf_vy.has_value() ? *optflow_kf_vy : 0.0f);
    SpiTx.wheel_chassis_vel[0] = encode_velocity_mm_s(wheel_chassis_vx.has_value() ? *wheel_chassis_vx : 0.0f);
    SpiTx.wheel_chassis_vel[1] = encode_velocity_mm_s(wheel_chassis_vy.has_value() ? *wheel_chassis_vy : 0.0f);
    SpiTx.fused_chassis_vel[0] = encode_velocity_mm_s(fused_chassis_vx.has_value() ? *fused_chassis_vx : 0.0f);
    SpiTx.fused_chassis_vel[1] = encode_velocity_mm_s(fused_chassis_vy.has_value() ? *fused_chassis_vy : 0.0f);

    SpiTx.xy_max_vel[0] = encode_velocity_mm_s(
        decomposed_axis_velocity_limit_m_s(WHEEL_VX_ANGLE, wheel_vel_limit));
    SpiTx.xy_max_vel[1] = encode_velocity_mm_s(
        decomposed_axis_velocity_limit_m_s(WHEEL_VY_ANGLE, wheel_vel_limit));

    SpiTx.xy_max_acc[0] = encode_accel_mm_s2(xy_max_acc[0]);
    SpiTx.xy_max_acc[1] = encode_accel_mm_s2(xy_max_acc[1]);
    SpiTx.xy_max_jerk[0] = encode_linear_jerk_deci_m_s3(xy_max_jerk[0]);
    SpiTx.xy_max_jerk[1] = encode_linear_jerk_deci_m_s3(xy_max_jerk[1]);
    SpiTx.xy_max_dec[0] = encode_accel_mm_s2(xy_max_dec[0]);
    SpiTx.xy_max_dec[1] = encode_accel_mm_s2(xy_max_dec[1]);

    SpiTx.yaw_max_vel = encode_angular_centi(yaw_max_vel);
    SpiTx.yaw_max_acc = encode_angular_centi(yaw_max_acc);
    SpiTx.yaw_max_jerk = encode_angular_centi(yaw_max_jerk);
    SpiTx.planned_vel[0] = encode_velocity_mm_s(robot_real_vel[0]);
    SpiTx.planned_vel[1] = encode_velocity_mm_s(robot_real_vel[1]);
    SpiTx.reserved = 0;

    // Encode IMU data
    for (uint8_t i = 0; i < 9; i++) {
        SpiTx.imu_data[i] = static_cast<int16_t>(imu_data[i] * 100);
    }

    memcpy(spi_tx_data, &SpiTx, sizeof(SpiTx));
}

void Robot::prepare_yaw_control(float dt_s) {
    if (!use_imu) {
        return;
    }

    // Step the S-curve yaw planner → use its velocity as omega reference
    yaw_s_curve_.step(dt_s);
    robot_real_vel[2] = yaw_s_curve_.velocity();
    robot_acc[2] = 0.0f;
}

void Robot::ik_solve() {
    // Calculate motor velocities
    for (uint8_t i = 0; i < 4; i++) {
        motor_vel[i] = ((robot_real_vel[0] * WHEEL_VX_ANGLE[i] + 
                        robot_real_vel[1] * WHEEL_VY_ANGLE[i] - 
                        robot_real_vel[2] * ROBOT_RADIUS) / WHEEL_RADIUS) 
                       * 30.0f / control_config::kPi;
        // Limit wheel velocity
        motor_vel[i] = std::clamp(motor_vel[i], -wheel_vel_limit, wheel_vel_limit);
    }
    wheel_vel_debug = motor_vel[0];
}

void Robot::motion_planner(const double _dt) {
    const float dt_s = static_cast<float>(_dt / 1000000.0);  // Convert microseconds to seconds
    if (dt_s <= 1e-9) {
        return;
    }

    const auto measured_chassis_vx = chassis_estimator.chassis_vx_output_port()->any();
    const auto measured_chassis_vy = chassis_estimator.chassis_vy_output_port()->any();
    const float measured_vel[2] = {
        measured_chassis_vx.has_value() ? *measured_chassis_vx : last_robot_real_vel[0],
        measured_chassis_vy.has_value() ? *measured_chassis_vy : last_robot_real_vel[1],
    };
    const float planner_start_vel[2] = {
        planner_start_vx_filter_.filter(measured_vel[0]),
        planner_start_vy_filter_.filter(measured_vel[1]),
    };

    for (uint8_t i = 0; i < 3; i++) {
        if (use_imu && i == 2) continue;  // yaw angle control bypasses planner

        AxisMotionPlan& plan = g_axis_plans[i];
        const bool linear_axis = (i < 2);
        const float v_target = robot_vel[i];
        // S-curve uses the larger limit for timing; directional clamp enforces per-phase limits.
        // This handles zero-crossing correctly: AccMax limits the speeding-up phase,
        // DecMax limits the slowing-down phase, regardless of sign.
        const float a_acc = (i == 0) ? xy_max_acc[0] : (i == 1) ? xy_max_acc[1] : yaw_max_acc;
        const float a_dec = (i == 0) ? xy_max_dec[0] : (i == 1) ? xy_max_dec[1] : yaw_max_acc;
        const float a_max = fmaxf(a_acc, a_dec);
        const float j_max = (i == 0) ? xy_max_jerk[0] : (i == 1) ? xy_max_jerk[1] : yaw_max_jerk;

        const bool target_changed = fabsf(v_target - plan.v_target) > kPlannerReplanEps;
        const bool should_replan = target_changed;
        const float v_plan_start = (linear_axis && control_config::kPlannerStartFromMeasuredVelocity)
            ? planner_start_vel[i]
            : last_robot_real_vel[i];
        if (should_replan) {
            init_axis_plan(plan, v_plan_start, v_target, a_max, j_max);
            if (linear_axis) {
                last_robot_real_vel[i] = v_plan_start;
            }
        }
        const float v_now = last_robot_real_vel[i];

        float a_profile = 0.0f;
        float v_profile = v_target;
        if (plan.active) {
            plan.elapsed = std::min(plan.elapsed + dt_s, plan.t_total);
            sample_axis_plan(plan, j_max, a_max, plan.elapsed, a_profile, v_profile);
        }

        // Track planned acceleration while enforcing per-step jerk and acceleration limits.
        const float max_da = j_max * dt_s;
        const float da = std::clamp(a_profile - robot_acc[i], -max_da, max_da);
        // Direction-aware clamp: AccMax limits speed-up, DecMax limits slow-down
        const float a_hi = (v_now > kPlannerVelEps) ? a_acc
                         : (v_now < -kPlannerVelEps) ? a_dec
                         : fmaxf(a_acc, a_dec);
        const float a_lo = (v_now > kPlannerVelEps) ? -a_dec
                         : (v_now < -kPlannerVelEps) ? -a_acc
                         : -fmaxf(a_acc, a_dec);
        robot_acc[i] = std::clamp(robot_acc[i] + da, a_lo, a_hi);

        robot_real_vel[i] = v_now + robot_acc[i] * dt_s;

        // Keep integration close to profile when jerk limit is inactive.
        const float profile_err = v_profile - robot_real_vel[i];
        if (fabsf(profile_err) < kPlannerReplanEps) {
            robot_real_vel[i] = v_profile;
        }

        const float before = v_target - v_now;
        const float after = v_target - robot_real_vel[i];
        if (before * after < 0.0f || fabsf(after) < kPlannerVelEps) {
            robot_real_vel[i] = v_target;
            robot_acc[i] = 0.0f;
            plan.active = false;
        }

        last_robot_real_vel[i] = robot_real_vel[i];
    }
    robot_ay_debug = robot_acc[1];

    robot_real_vx_debug = robot_real_vel[0];
}

void Robot::bind_estimator_imu_ports(IMU& imu_ref) {
    chassis_estimator.imu_yaw_input_port()->connect_to(imu_ref.yaw_port());
    chassis_estimator.imu_omega_z_input_port()->connect_to(imu_ref.omega_z_port());
    chassis_estimator.imu_gyro_x_input_port()->connect_to(imu_ref.omega_x_port());
    chassis_estimator.imu_gyro_y_input_port()->connect_to(imu_ref.omega_y_port());
}

void Robot::bind_estimator_optflow_ports(OptFlow& optflow_ref) {
    optflow_body_vx_port_ = optflow_ref.body_vx_output();
    optflow_body_vy_port_ = optflow_ref.body_vy_output();
    optflow_kf_vx_port_ = optflow_ref.kf_vx_output();
    optflow_kf_vy_port_ = optflow_ref.kf_vy_output();

    chassis_estimator.optflow_vx_input_port()->connect_to(optflow_kf_vx_port_);
    chassis_estimator.optflow_vy_input_port()->connect_to(optflow_kf_vy_port_);
}

void Robot::update_torque_feedforward(const double _dt) {
    const float dt_s = static_cast<float>(_dt / 1000000.0);
    if (dt_s <= 1e-9f) {
        return;
    }

    chassis_controller.set_reference(robot_real_vel, robot_acc);
    chassis_controller.set_use_imu_yaw(use_imu);
    if (use_imu) {
        chassis_controller.set_yaw_target(yaw_s_curve_.position(), yaw_s_curve_.velocity());
    }
    chassis_estimator.set_reference_accel(robot_acc);
    chassis_estimator.step(dt_s);
    chassis_controller.step(dt_s);

    for (uint8_t i = 0; i < 4; ++i) {
        const std::optional<float> tau_ff = chassis_controller.wheel_torque_ff_output_port(i)->any();
        motor_Ff[i] = tau_ff.has_value() ? *tau_ff : 0.0f;
    }
}

void Robot::watchdog_feed() {
    watchdog_current_value_ = watchdog_timeout_;
}

bool Robot::watchdog_check() {
    if (watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } else {
        return false;
    }
}
