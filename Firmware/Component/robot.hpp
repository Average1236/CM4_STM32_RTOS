#ifndef __ROBOT_HPP
#define __ROBOT_HPP

#include "Task/z_main.h"
#include "chassis_controller.hpp"
#include "chassis_estimator.hpp"
#include "dribbler_zfoc.hpp"

struct __attribute__((packed)) CM4_to_stm32_spi
{
    uint8_t drib_power;
    int16_t vel[3];
    int16_t angle_pid[3];
    int16_t wheel_pid[3];
    bool use_imu;
    bool kick_mode; // chip:1  shoot:0
    uint16_t kick_discharge_time;
};

struct __attribute__((packed)) stm32_to_CM4_spi
{
    int16_t imu_data[9];
    bool infrare_flag;
    bool getBall;
    bool imu_online;
    int8_t battery_vol;
    int16_t cap_vol;
    int16_t wheel[4];
};

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
    void bind_estimator_imu_ports(IMU& imu_ref);
    void update_torque_feedforward(const double _dt);

    void watchdog_feed();
    bool watchdog_check();

    float infra_voltage = 0;
    float dribble_power = 0;

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

    // m/s
    float robot_vel[3] = {0};

    float robot_real_vel[3] = {0};
    float last_robot_real_vel[3] = {0};
    float robot_acc[3] = {0};
    float yaw_ref_rel_rad_ = 0.0f;
    float ik_solve_basis[3] = {0, 1, 2};
    float ik_solve_inv_b[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    ChassisEstimator chassis_estimator;
    MixedLesoChassisController chassis_controller;

    uint32_t spi_error_count = 0;

    // Watchdog
    uint32_t watchdog_current_value_ = 0;
    uint32_t watchdog_timeout_ = 200 * 4;

private:
    void start_kick_pulse(bool chip_mode, uint16_t pulse_us);

    bool kick_active_ = false;
    uint32_t last_kick_tick_ = 0;
    uint16_t kick_pulse_us_ = 0;
};

#endif // __ROBOT_HPP