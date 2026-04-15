#include "robot.hpp"
#include "control_params.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>

// Debug
volatile float target_vx_debug = 0;
volatile float target_vy_debug = 0;
volatile float target_vw_debug = 0;
volatile float wheel_vel_debug = 0;
volatile float robot_real_vx_debug = 0;

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
static constexpr float ACC_THRESHOLD[3] = {
    control_config::kAccThresholdX,
    control_config::kAccThresholdY,
    control_config::kAccThresholdYaw,
};

static constexpr float JERK_LIMIT[3] = {
    control_config::kJerkLimitX,
    control_config::kJerkLimitY,
    control_config::kJerkLimitYaw,
};

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

inline float signf_nonzero(const float x) {
    return (x >= 0.0f) ? 1.0f : -1.0f;
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

static const DibbleMotorBase::Config_t DRIBBLER_MOTOR_PARAMS = {
    .control_id = 5, .feedback_id = 5, .direction = -1.0f, .ex_reduce_rate = 20, 
    .remove_built_in_reducer = false
};

// PID parameters for dribbler
static const PID::Parameter_t DRIBBLER_PID_PARAMS = {
    .kp = 0.001f, .ki = 0.005f, .kd = 0,
    .output_limit = motor_info_M2006.broad_current_limit,
    .integ_limit = motor_info_M2006.broad_current_limit, .dt = control_config::kControlDtSec
};

// TD (tracking differentiator) parameters for dribbler
static const TD::Parameter_t DRIBBLER_TD_PARAMS = {
    .r = 2000, .h = 0.01f, .dt = control_config::kControlDtSec,
    .is_cycle = false, .cycle_low = -180.0f, .cycle_high = 180.0f
};

Robot::Robot() {
    // Initialize wheel motors
    for (int i = 0; i < 4; i++) {
        wheel_motors[i] = new MotorDMH3510(WHEEL_MOTOR_PARAMS[i]);

        wheel_motors[i]->velocity_cmd_input_port()->connect_to(&motor_vel[i]);
        chassis_estimator.wheel_velocity_input_port(i)->connect_to(wheel_motors[i]->velocity_output_port());
        wheel_motors[i]->torque_ff_cmd_input_port()->connect_to(chassis_controller.wheel_torque_ff_output_port(i));
    }

    chassis_controller.chassis_vx_input_port()->connect_to(chassis_estimator.chassis_vx_output_port());
    chassis_controller.chassis_vy_input_port()->connect_to(chassis_estimator.chassis_vy_output_port());
    chassis_controller.chassis_omega_z_input_port()->connect_to(chassis_estimator.chassis_omega_z_output_port());

    // Initialize dribbler
    dribbler = new MotorM2006(DRIBBLER_MOTOR_PARAMS);
    dribbler_PID_controller = new PID(DRIBBLER_PID_PARAMS);
    dribbler_filter = new TD(DRIBBLER_TD_PARAMS, 0);
}

Robot::~Robot() {
    for (int i = 0; i < 4; i++) {
        delete wheel_motors[i];
    }
    delete dribbler;
    delete dribbler_PID_controller;
    delete dribbler_filter;
}

void Robot::pi_decode_spi() {
    memcpy(&SpiRx, spi_rx_data, sizeof(SpiRx));

    // Decode robot velocity commands from Pi (if implemented)
    for (uint8_t i = 0; i < 2; i++) {
        robot_vel[i] = SpiRx.vel[i] / 1000.0f;
    }
    robot_vel[2] = SpiRx.vel[2] / 100.0f;

    // Debug
    target_vx_debug = robot_vel[0];
    target_vy_debug = robot_vel[1];
    target_vw_debug = robot_vel[2];
}

void Robot::pi_encode_spi() {
    float imu_data[9] = {0.0f};
    imu.get_data(imu_data);

    SpiTx.infrare_flag = (infra_ADC1_val > 0.5f) ? 1 : 0;
    SpiTx.getBall = false;
    SpiTx.imu_online = true;
    SpiTx.battery_vol = static_cast<int16_t>(bat_ADC2_val * 5);
    SpiTx.cap_vol = static_cast<int16_t>(cap_ADC3_val * 100);

    // Encode wheel motor velocities
    for (uint8_t i = 0; i < 4; i++) {
        SpiTx.wheel[i] = static_cast<int16_t>(wheel_motors[i]->get_velocity() * 10);
        // SpiTx.wheel_ref[i] = static_cast<int16_t>(motor_vel[i] * 10);
    }

    // Encode IMU data
    for (uint8_t i = 0; i < 9; i++) {
        SpiTx.imu_data[i] = static_cast<int16_t>(imu_data[i] * 100);
    }

    memcpy(spi_tx_data, &SpiTx, sizeof(SpiTx));
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

    for (uint8_t i = 0; i < 3; i++) {
        AxisMotionPlan& plan = g_axis_plans[i];
        const float v_now = last_robot_real_vel[i];
        const float v_target = robot_vel[i];
        const float a_max = ACC_THRESHOLD[i];
        const float j_max = JERK_LIMIT[i];

        const bool target_changed = fabsf(v_target - plan.v_target) > kPlannerReplanEps;
        const bool plan_finished = plan.active && (plan.elapsed >= plan.t_total - 1e-8f);
        if (!plan.active || target_changed || plan_finished) {
            init_axis_plan(plan, v_now, v_target, a_max, j_max);
        }

        float a_profile = 0.0f;
        float v_profile = v_target;
        if (plan.active) {
            plan.elapsed = std::min(plan.elapsed + dt_s, plan.t_total);
            sample_axis_plan(plan, j_max, a_max, plan.elapsed, a_profile, v_profile);
        }

        // Track planned acceleration while enforcing per-step jerk and acceleration limits.
        const float max_da = j_max * dt_s;
        const float da = std::clamp(a_profile - robot_acc[i], -max_da, max_da);
        robot_acc[i] = std::clamp(robot_acc[i] + da, -a_max, a_max);

        robot_real_vel[i] = last_robot_real_vel[i] + robot_acc[i] * dt_s;

        // Keep integration close to profile when jerk limit is inactive.
        const float profile_err = v_profile - robot_real_vel[i];
        if (fabsf(profile_err) < kPlannerReplanEps) {
            robot_real_vel[i] = v_profile;
        }

        const float before = v_target - last_robot_real_vel[i];
        const float after = v_target - robot_real_vel[i];
        if (before * after < 0.0f || fabsf(after) < kPlannerVelEps) {
            robot_real_vel[i] = v_target;
            robot_acc[i] = 0.0f;
            plan.active = false;
        }

        last_robot_real_vel[i] = robot_real_vel[i];
    }

    yaw_ref_rel_rad_ += robot_real_vel[2] * dt_s;
    robot_real_vx_debug = robot_real_vel[0];
}

void Robot::bind_estimator_imu_ports(IMU& imu_ref) {
    chassis_estimator.imu_yaw_input_port()->connect_to(imu_ref.yaw_port());
    chassis_estimator.imu_omega_z_input_port()->connect_to(imu_ref.omega_z_port());
}

void Robot::update_torque_feedforward(const double _dt) {
    const float dt_s = static_cast<float>(_dt / 1000000.0);
    if (dt_s <= 1e-9f) {
        return;
    }

    chassis_controller.set_reference(robot_real_vel, robot_acc, yaw_ref_rel_rad_);
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