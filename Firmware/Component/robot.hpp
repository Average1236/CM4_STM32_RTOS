#ifndef __ROBOT_HPP
#define __ROBOT_HPP

#include "Task/z_main.h"
#include "chassis_controller.hpp"
#include "chassis_estimator.hpp"
#include "dribbler_zfoc.hpp"
#include "yaw_s_curve.hpp"
#include "circular_lpf.hpp"

struct __attribute__((packed)) CM4_to_stm32_spi
{
    uint8_t drib_power;
    uint8_t drib_mode;     // 1: torque, 2: speed, 3: hybrid
    int16_t vel[3];
    int16_t angle_pid[3];
    int16_t wheel_pid[3];
    bool use_imu;
    bool kick_mode; // chip:1  shoot:0
    uint16_t kick_discharge_time;
    int16_t drib_velocity;  // turns/s * 100
    int16_t drib_torque_ff; // Nm * 1000
    int16_t xy_max_acc[2];   // vx, vy acceleration limits in mm/s^2
    int16_t xy_max_jerk[2];  // vx, vy jerk limits in (m/s^3) * 10
    int16_t xy_max_dec[2];   // vx, vy deceleration limits in mm/s^2
    int16_t yaw_max_vel;     // rad/s * 100
    int16_t yaw_max_acc;     // rad/s^2 * 100
    int16_t yaw_max_jerk;    // rad/s^3 * 100
    int16_t raw_vision_vel[2]; // vx, vy in mm/s
    uint8_t vision_source;     // 3: optflow + vision, 4: wheel + vision
    int16_t chassis_vel_pid[2][3]; // x/y Kp, Ki, Kd * 100
    int16_t acceleration_ff[2]; // robot-local vx/vy feedforward in mm/s^2
    int16_t yaw_angle_pid[3]; // yaw angle Kp, Ki, Kd * 1000
};

static_assert(sizeof(CM4_to_stm32_spi) <= SPI_LENGTH, "CM4_to_stm32_spi exceeds SPI_LENGTH");

struct __attribute__((packed)) stm32_to_CM4_spi
{
    int16_t imu_data[9];
    bool infrare_flag;
    bool getBall;
    bool imu_online;
    int8_t battery_vol;
    int16_t cap_vol;
    int16_t wheel[4];
    int16_t odom_vel[2]; // vx, vy in mm/s
    int16_t optflow_body_vel[2]; // raw body vx, vy before optflow Kalman in mm/s
    int16_t optflow_kf_vel[2];   // optflow Kalman vx, vy in mm/s
    int16_t wheel_chassis_vel[2]; // wheel-only chassis vx, vy in mm/s
    int16_t fused_chassis_vel[2]; // wheel + optflow Kalman fused vx, vy in mm/s
    int16_t xy_max_vel[2];   // vx, vy axis limits in mm/s
    int16_t xy_max_acc[2];   // vx, vy acceleration limits in mm/s^2
    int16_t xy_max_jerk[2];  // vx, vy jerk limits in (m/s^3) * 10
    int16_t xy_max_dec[2];   // vx, vy deceleration limits in mm/s^2
    int16_t yaw_max_vel;     // rad/s * 100
    int16_t yaw_max_acc;     // rad/s^2 * 100
    int16_t yaw_max_jerk;    // rad/s^3 * 100
    int16_t planned_vel[2];  // planned robot_real_vel vx, vy in mm/s
    int16_t chassis_yaw_rad;       // rad * 1000
    int16_t chassis_omega_z;       // rad/s * 100
    int16_t controller_omega_ref;  // rad/s * 100
    int16_t controller_f_task[3];  // Fx, Fy, Fpsi * 100
    int16_t chassis_vel_pid[2][3]; // applied x/y Kp, Ki, Kd * 100
    int16_t wheel1_target;  // wheel 1 target RPM * 10
    int16_t wheel3_target;  // wheel 3 target RPM * 10
    int16_t yaw_angle_pid[3]; // applied yaw angle Kp, Ki, Kd * 1000
};

static_assert(sizeof(CM4_to_stm32_spi) <= SPI_LENGTH, "CM4_to_stm32_spi exceeds SPI buffer length");
static_assert(sizeof(stm32_to_CM4_spi) <= SPI_LENGTH, "stm32_to_CM4_spi exceeds SPI buffer length");

class Robot: public RobotIntf {
public:
    Robot();
    ~Robot();

    void pi_encode_spi();
    void pi_decode_spi();
    void request_kick_from_spi();
    void on_kick_timeout_irq();

    void ik_solve();
    void motion_planner(const double _dt);
    void prepare_yaw_control(float dt_s);
    void bind_estimator_imu_ports(IMU& imu_ref);
    void bind_estimator_optflow_ports(OptFlow& optflow_ref);
    void update_torque_feedforward(const double _dt);

    void watchdog_feed();
    bool watchdog_check();

    float infra_voltage = 0;
    bool infrare_flag = false;
    float dribble_power = 0;
    float dribble_velocity = -50.0f;
    float dribble_torque_ff = -0.15f;

    uint8_t dribbler_mode = 0;
    enum DribblerHybridPhase : uint8_t {
        kDribblerHybridTorquePhase = 0,
        kDribblerHybridSpeedPhase = 1,
    };
    DribblerHybridPhase dribbler_hybrid_phase = kDribblerHybridTorquePhase;
    uint32_t dribbler_ball_hold_count = 0;

    DribblerZfoc dribbler;

    float bat_ADC2_val = 0;
    float cap_ADC3_val = 0;

    MotorDMH3510* wheel_motors[4];

    float motor_vel[4] = {0};
    float last_motor_vel[4] = {0};
    float motor_torq[4] = {0};
    float motor_acc[4] = {0};
    float motor_acc_t[4] = {0};
    float motor_F[4] = {0};
    float motor_Ff[4] = {0};

    uint8_t pi_uart_rx_data[PI_UART_RX_DATA_LENGTH] = {0};
    uint8_t pi_uart_tx_data[PI_UART_TX_DATA_LENGTH] = {0};
    uint8_t spi_rx_data[SPI_LENGTH] = {0};
    uint8_t spi_tx_data[SPI_LENGTH] = {0};

    CM4_to_stm32_spi SpiRx;
    stm32_to_CM4_spi SpiTx;

    bool kick_mode = false; // chip:1  shoot:0
    uint16_t kick_discharge_time = 0;

    float wheel_PID[3] = {0.000, 0.005, 0};
    float wheel_vel_PID[3] = {0.5, 0.1, 0};
    float wheel_vel_limit = 2000; // rpm

    bool use_imu = false;
    float yaw_target_rad = 0.0f;
    bool yaw_target_initialized = false;
    YawSCurve yaw_s_curve_{{0.0f, 0.0f, 0.0f, 0.0f}};
    CircularLowPassFilter yaw_target_lpf_;
    // vx/vy: Butterworth LPFs (heap-allocated, replace TD-based planner)
    ButterworthLowPass2* vx_ref_lpf_ = nullptr;
    ButterworthLowPass2* vy_ref_lpf_ = nullptr;
    ButterworthLowPass2* yaw_fallback_omega_z_lpf_ = nullptr;
    // Yaw TD fallback (non-IMU mode only)
    TD* td_yaw_fallback_ = nullptr;
    float xy_max_acc[2] = {7.0f, 7.0f};
    float xy_max_jerk[2] = {1000.0f, 1000.0f};
    float xy_max_dec[2] = {10.0f, 10.0f};
    float yaw_max_vel = 25.0f;
    float yaw_max_acc = 40.0f;
    float yaw_max_jerk = 200.0f;
    float chassis_vel_pid[2][3] = {
        {30.0f, 300.0f, 0.0f},
        {30.0f, 300.0f, 0.0f},
    };
    float yaw_angle_pid[3] = {15.0f, 2.0f, 0.0f};

    // m/s
    float robot_vel[3] = {0};

    float robot_real_vel[3] = {0};
    float last_robot_real_vel[3] = {0};
    float robot_acc[3] = {0};
    float acceleration_ff[2] = {0.0f, 0.0f};
    float ik_solve_basis[3] = {0, 1, 2};
    float ik_solve_inv_b[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float raw_vision_vel_mm_s[2] = {0.0f, 0.0f};
    float vision_source = 0.0f;

    ChassisEstimator chassis_estimator;
    ChassisController chassis_controller;

    uint32_t spi_error_count = 0;

    // Watchdog
    uint32_t watchdog_current_value_ = 0;
    uint32_t watchdog_timeout_ = 200 * 4;

private:
    void start_kick_pulse(bool chip_mode, uint16_t pulse_us);

    OutputPort<float>* optflow_body_vx_port_ = nullptr;
    OutputPort<float>* optflow_body_vy_port_ = nullptr;
    OutputPort<float>* optflow_kf_vx_port_ = nullptr;
    OutputPort<float>* optflow_kf_vy_port_ = nullptr;

    bool kick_active_ = false;
    uint32_t last_kick_tick_ = 0;
    uint16_t kick_pulse_us_ = 0;
    float yaw_fallback_pid_integ_ = 0.0f;
};

#endif // __ROBOT_HPP
