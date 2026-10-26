#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "Task/z_main.h"
#include "dribbler_zfoc.hpp"
#include "motion_engine.hpp"
#include <atomic>

class Robot {
public:
    Robot();
    ~Robot();
    void process_spi_v2(motion::TimeUs now_us);
    void ensure_motion_engine(motion::TimeUs now_us);
    void update_motion_v2(motion::TimeUs now_us);
    // Single producer (IMU task), single consumer (control task).
    void queue_raw_imu(const motion::RawImuSample& sample);
    void drain_raw_imu(motion::TimeUs now_us);
    void request_kick_from_spi();
    void on_kick_timeout_irq();

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
    stm32_motion::MotionEngine motion_engine;
    motion::ControllerOutput motion_control_;
    motion::OptFlowSample latest_flow_;
    static constexpr std::size_t kImuQueueSize = 16;
    motion::RawImuSample imu_queue_[kImuQueueSize];
    std::atomic<std::uint32_t> imu_queue_head_{0};
    std::atomic<std::uint32_t> imu_queue_tail_{0};
    OutputPort<float> motion_torque_ports_[4] = {
        OutputPort<float>(0), OutputPort<float>(0),
        OutputPort<float>(0), OutputPort<float>(0)};
    uint8_t spi_rx_data[SPI_LENGTH] = {};
    uint8_t spi_tx_data[SPI_LENGTH] = {};
    bool kick_mode = false;
    uint16_t kick_discharge_time = 0;

private:
    void start_kick_pulse(bool chip_mode, uint16_t pulse_us);
    bool motion_engine_ready_ = false;
    bool kick_active_ = false;
    uint32_t last_kick_tick_ = 0;
};
#endif
