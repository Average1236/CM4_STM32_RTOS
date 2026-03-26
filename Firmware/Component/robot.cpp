#include "robot.hpp"
#include <cstring>
#include <cmath>

// Debug
volatile float target_vx_debug = 0;
volatile float target_vy_debug = 0;
volatile float target_vw_debug = 0;
volatile float wheel_vel_debug = 0;
volatile float robot_real_vx_debug = 0;

// Constants
static constexpr float DT = 1.0f / 1000.0f;

// Wheel geometry
static constexpr float WHEEL_ANGLE_FORWARD = 60.0f / 180.0f * PI;
static constexpr float WHEEL_ANGLE_BACKWARD = 45.0f / 180.0f * PI;

// Pre-computed trigonometric values
static const float SIN_WHEEL_ANGLE_FWD = sinf(WHEEL_ANGLE_FORWARD);
static const float COS_WHEEL_ANGLE_FWD = cosf(WHEEL_ANGLE_FORWARD);
static const float SIN_WHEEL_ANGLE_BWD = sinf(WHEEL_ANGLE_BACKWARD);
static const float COS_WHEEL_ANGLE_BWD = cosf(WHEEL_ANGLE_BACKWARD);

// Robot dynamics parameters
static constexpr float ROBOT_MASS = 2.19692f;           // kg
static constexpr float ROBOT_INERTIA = 2.29587357e-6f; // kg*m^2
static constexpr float ROBOT_RADIUS = 0.073f;           // m (avg of 85mm and 61mm)
static constexpr float WHEEL_INERTIA = 8.38e-5f;        // kg*m^2
static constexpr float WHEEL_RADIUS = 0.0285f;          // m
static constexpr float WHEEL_PARALLEL_RESISTANCE = 0.016f;
static constexpr float VERTICAL_RESISTANCE = 0.0f;
static constexpr float ACC_THRESHOLD[3] = {8.0f, 8.0f, 40.0f};

// Wheel velocity angle matrices
static const float WHEEL_VX_ANGLE[4] = {
    SIN_WHEEL_ANGLE_FWD, -SIN_WHEEL_ANGLE_FWD, 
    -SIN_WHEEL_ANGLE_BWD, SIN_WHEEL_ANGLE_BWD
};

static const float WHEEL_VY_ANGLE[4] = {
    -COS_WHEEL_ANGLE_FWD, -COS_WHEEL_ANGLE_FWD, 
    COS_WHEEL_ANGLE_BWD, COS_WHEEL_ANGLE_BWD
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
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT}
};

// PID parameters for dribbler
static const PID::Parameter_t DRIBBLER_PID_PARAMS = {
    .kp = 0.001f, .ki = 0.005f, .kd = 0,
    .output_limit = motor_info_M2006.broad_current_limit,
    .integ_limit = motor_info_M2006.broad_current_limit, .dt = DT
};

// TD (tracking differentiator) parameters for dribbler
static const TD::Parameter_t DRIBBLER_TD_PARAMS = {
    .r = 2000, .h = 0.01f, .dt = DT,
    .is_cycle = false, .cycle_low = -180.0f, .cycle_high = 180.0f
};

// TD parameters for wheel motors
static const TD::Parameter_t WHEEL_TD_PARAMS[4] = {
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
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
                       * 30.0f / PI;
        // Limit wheel velocity
        motor_vel[i] = std::clamp(motor_vel[i], -wheel_vel_limit, wheel_vel_limit);
    }
    wheel_vel_debug = motor_vel[0];
}

void Robot::motion_planner(const double _dt) {
    double dt_s = _dt / 1000000.0;  // Convert microseconds to seconds

    for (uint8_t i = 0; i < 3; i++) {
        // Calculate acceleration
        robot_acc[i] = (robot_vel[i] - last_robot_real_vel[i]) / dt_s;

        // Limit acceleration
        if (robot_acc[i] > ACC_THRESHOLD[i]) {
            robot_acc[i] = ACC_THRESHOLD[i];
        } else if (robot_acc[i] < -ACC_THRESHOLD[i]) {
            robot_acc[i] = -ACC_THRESHOLD[i];
        }

        // Update velocity
        robot_real_vel[i] = last_robot_real_vel[i] + robot_acc[i] * dt_s;
        last_robot_real_vel[i] = robot_real_vel[i];
    }
    robot_real_vx_debug = robot_real_vel[0];
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