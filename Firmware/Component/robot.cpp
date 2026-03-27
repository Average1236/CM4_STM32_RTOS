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
volatile float robot_fused_vx_debug = 0;
volatile float robot_fused_vy_debug = 0;

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

static constexpr float VEL_ERR_GAIN[3] = {
    control_config::kVelErrGainX,
    control_config::kVelErrGainY,
    control_config::kVelErrGainYaw,
};

static constexpr float JERK_LIMIT[3] = {
    control_config::kJerkLimitX,
    control_config::kJerkLimitY,
    control_config::kJerkLimitYaw,
};

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

// PID parameters for wheel velocity control
static const PID::Parameter_t WHEEL_VEL_PID_PARAMS[4] = {
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = control_config::kControlDtSec},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = control_config::kControlDtSec},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = control_config::kControlDtSec},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = control_config::kControlDtSec}
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

// TD parameters for wheel motors
static const TD::Parameter_t WHEEL_TD_PARAMS[4] = {
    {.r = 200000, .h = 0.01f, .dt = control_config::kControlDtSec, .is_cycle = false,
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = control_config::kControlDtSec, .is_cycle = false,
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = control_config::kControlDtSec, .is_cycle = false,
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = control_config::kControlDtSec, .is_cycle = false,
     .cycle_low = -180.0f, .cycle_high = 180.0f}
};

// IMU data buffer for transmission
static float imu_data_buffer[9] = {0};

Robot::Robot() {
    // Initialize wheel motors
    for (int i = 0; i < 4; i++) {
        wheel_motors[i] = new MotorDMH3510(WHEEL_MOTOR_PARAMS[i]);
        wheel_vel_PID_controllers[i] = new PID(WHEEL_VEL_PID_PARAMS[i]);
        wheel_filter[i] = new TD(WHEEL_TD_PARAMS[i], 0);

        wheel_motors[i]->velocity_cmd_input_port()->connect_to(&motor_vel[i]);
        chassis_controller.wheel_velocity_input_port(i)->connect_to(wheel_motors[i]->velocity_output_port());
        velocity_estimator.wheel_velocity_input_port(i)->connect_to(wheel_motors[i]->velocity_output_port());
        wheel_motors[i]->torque_ff_cmd_input_port()->connect_to(chassis_controller.wheel_torque_ff_output_port(i));
    }

    // Initialize dribbler
    dribbler = new MotorM2006(DRIBBLER_MOTOR_PARAMS);
    dribbler_PID_controller = new PID(DRIBBLER_PID_PARAMS);
    dribbler_filter = new TD(DRIBBLER_TD_PARAMS, 0);
}

Robot::~Robot() {
    for (int i = 0; i < 4; i++) {
        delete wheel_motors[i];
        delete wheel_PID_controllers[i];
        delete wheel_vel_PID_controllers[i];
        delete wheel_filter[i];
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
        SpiTx.imu_data[i] = static_cast<int16_t>(imu_data_buffer[i] * 100);
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
    double dt_s = _dt / 1000000.0;  // Convert microseconds to seconds
    if (dt_s <= 1e-9) {
        return;
    }

    for (uint8_t i = 0; i < 3; i++) {
        const float vel_err = robot_vel[i] - last_robot_real_vel[i];
        const float desired_acc = std::clamp(
            VEL_ERR_GAIN[i] * vel_err,
            -ACC_THRESHOLD[i],
            ACC_THRESHOLD[i]
        );

        const float jerk_cmd = std::clamp(
            static_cast<float>((desired_acc - robot_acc[i]) / dt_s),
            -JERK_LIMIT[i],
            JERK_LIMIT[i]
        );

        robot_acc[i] += jerk_cmd * static_cast<float>(dt_s);

        robot_acc[i] = std::clamp(robot_acc[i], -ACC_THRESHOLD[i], ACC_THRESHOLD[i]);

        robot_real_vel[i] = last_robot_real_vel[i] + robot_acc[i] * static_cast<float>(dt_s);

        const float before = robot_vel[i] - last_robot_real_vel[i];
        const float after = robot_vel[i] - robot_real_vel[i];
        if (before * after < 0.0f) {
            robot_real_vel[i] = robot_vel[i];
            robot_acc[i] = 0.0f;
        }

        last_robot_real_vel[i] = robot_real_vel[i];
    }
    robot_real_vx_debug = robot_real_vel[0];
}

void Robot::bind_imu_ports(IMU& imu_ref) {
    velocity_estimator.imu_acc_x_input_port()->connect_to(imu_ref.acc_x_port());
    velocity_estimator.imu_acc_y_input_port()->connect_to(imu_ref.acc_y_port());
    velocity_estimator.imu_yaw_input_port()->connect_to(imu_ref.yaw_port());
    velocity_estimator.imu_omega_z_input_port()->connect_to(imu_ref.omega_z_port());

    chassis_controller.imu_yaw_input_port()->connect_to(imu_ref.yaw_port());
    chassis_controller.imu_omega_z_input_port()->connect_to(imu_ref.omega_z_port());
}

void Robot::update_velocity_estimate(const double _dt) {
    const float dt_s = static_cast<float>(_dt / 1000000.0);
    if (dt_s <= 1e-9f) {
        return;
    }

    velocity_estimator.step(dt_s);

    const std::optional<float> vx = velocity_estimator.vx_output_port()->any();
    const std::optional<float> vy = velocity_estimator.vy_output_port()->any();
    const std::optional<float> omega = velocity_estimator.omega_z_output_port()->any();
    const std::optional<float> speed = velocity_estimator.speed_output_port()->any();

    robot_fused_vel[0] = vx.has_value() ? *vx : robot_fused_vel[0];
    robot_fused_vel[1] = vy.has_value() ? *vy : robot_fused_vel[1];
    robot_fused_vel[2] = omega.has_value() ? *omega : robot_fused_vel[2];
    robot_fused_speed = speed.has_value() ? *speed : robot_fused_speed;
    robot_fused_vx_debug = robot_fused_vel[0];
    robot_fused_vy_debug = robot_fused_vel[1];
}

void Robot::update_torque_feedforward(const double _dt) {
    const float dt_s = static_cast<float>(_dt / 1000000.0);
    if (dt_s <= 1e-9f) {
        return;
    }

    chassis_controller.set_reference(robot_real_vel, robot_acc);
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
