#ifndef __ROBOT_HPP
#define __ROBOT_HPP

#include "Task/z_main.h"
#include "chassis_controller.hpp"
#include "chassis_estimator.hpp"
#include "dribbler_zfoc.hpp"
#include "yaw_s_curve.hpp"

struct __attribute__((packed)) CM4_to_stm32_spi
{
    uint8_t drib_power;
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
    int16_t reserved;
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
    float dribble_power = 0;
    float dribble_velocity = -50.0f;
    float dribble_torque_ff = -0.01f;

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
    float xy_max_acc[2] = {7.0f, 7.0f};
    float xy_max_jerk[2] = {1000.0f, 1000.0f};
    float xy_max_dec[2] = {10.0f, 10.0f};
    float yaw_max_vel = 25.0f;
    float yaw_max_acc = 40.0f;
    float yaw_max_jerk = 200.0f;

    // m/s
    float robot_vel[3] = {0};

    float robot_real_vel[3] = {0};
    float last_robot_real_vel[3] = {0};
    float robot_acc[3] = {0};
    float ik_solve_basis[3] = {0, 1, 2};
    float ik_solve_inv_b[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float raw_vision_vel_mm_s[2] = {0.0f, 0.0f};
    float vision_source = 0.0f;

    ChassisEstimator chassis_estimator;
    ChassisController chassis_controller;
    ButterworthLowPass2 planner_start_vx_filter_{{0.0f, 0.0f}};
    ButterworthLowPass2 planner_start_vy_filter_{{0.0f, 0.0f}};

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
};

#endif // __ROBOT_HPP
