#include "robot.hpp"
#include "control_params.hpp"
#include "tim.h"
#include <algorithm>

namespace {
constexpr uint16_t kKickPulseMaxUs = 15000;
constexpr uint32_t kKickIntervalMs = 300;
}

static const MotorDMH3510::Config_t WHEEL_MOTOR_PARAMS[4] = {
    {.control_id = 1, .feedback_id = 1, .direction = 1.0f},
    {.control_id = 2, .feedback_id = 2, .direction = 1.0f},
    {.control_id = 3, .feedback_id = 3, .direction = 1.0f},
    {.control_id = 4, .feedback_id = 4, .direction = 1.0f}
};

Robot::Robot() {
    for (int i = 0; i < 4; ++i) {
        wheel_motors[i] = new MotorDMH3510(WHEEL_MOTOR_PARAMS[i]);
        wheel_motors[i]->torque_ff_cmd_input_port()->connect_to(&motion_torque_ports_[i]);
    }
}

Robot::~Robot() {
    for (auto* motor : wheel_motors) delete motor;
}

void Robot::ensure_motion_engine(motion::TimeUs now_us) {
    if (motion_engine_ready_) return;

    motion::MotionConfigV2 config;
    config.robot = motion::defaultRobotModel();
    config.robot.mass_kg = control_config::kRobotMassKg;
    config.robot.inertia_kg_m2 = control_config::kRobotInertiaKgM2;
    config.robot.wheel_radius_m = control_config::kWheelRadiusM;
    config.robot.wheel_center_distance_m = control_config::kWheelCenterDistanceM;
    config.robot.wheel_inertia_kg_m2 = control_config::kWheelInertiaKgM2;
    config.robot.wheel_viscous_nm_per_rad_s =
        control_config::kWheelViscousDampingNmPerRadPS;
    config.robot.efficiency = 0.90f;
    config.robot.max_wheel_torque_nm = 0.70f;
    config.robot.max_wheel_speed_rad_s = 3.14159265f * 2000.0f / 30.0f;
    config.robot.max_linear_speed_m_s = 3.0f;
    config.robot.max_linear_accel_m_s2 = 7.0f;
    config.robot.max_linear_decel_m_s2 = 10.0f;
    config.robot.max_yaw_speed_rad_s = 25.0f;
    config.robot.max_yaw_accel_rad_s2 = 40.0f;

    config.limits.max_linear_speed_m_s = 3.0f;
    config.limits.max_linear_accel_m_s2 = 7.0f;
    config.limits.max_linear_decel_m_s2 = 10.0f;
    config.limits.max_yaw_speed_rad_s = 25.0f;
    config.limits.max_yaw_accel_rad_s2 = 40.0f;
    config.limits.max_wheel_torque_nm = 0.70f;
    config.limits.max_torque_rate_nm_per_s = 200.0f;
    config.limits.position_correction_gain = 4.0f;
    config.limits.position_correction_limit_m_s = 0.5f;
    config.limits.yaw_correction_gain = 6.0f;
    config.limits.yaw_rate_gain = 0.3f;
    config.limits.wheel_speed_kp = 0.08f;
    config.limits.wheel_speed_ki = 1.0f;
    config.limits.wheel_speed_backcalc = 0.4f;
    config.command_timeout_s = 0.100f;
    config.spi_timeout_s = 0.050f;
    config.vision_degrade_s = 0.100f;
    config.vision_invalid_s = 0.250f;
    config.config_version = 1;

    motion::ImuPreprocessorConfig imu_config;
    motion_engine.configure(config, imu_config, true, now_us);
    motion_engine_ready_ = true;
}

void Robot::queue_raw_imu(const motion::RawImuSample& sample) {
    const std::uint32_t head = imu_queue_head_.load(std::memory_order_relaxed);
    const std::uint32_t next = (head + 1u) % kImuQueueSize;
    if (next == imu_queue_tail_.load(std::memory_order_acquire)) {
        return;  // consumer is late; never overwrite the oldest snapshot
    }
    imu_queue_[head] = sample;
    imu_queue_head_.store(next, std::memory_order_release);
}

void Robot::drain_raw_imu(motion::TimeUs now_us) {
    std::uint32_t tail = imu_queue_tail_.load(std::memory_order_relaxed);
    const std::uint32_t head = imu_queue_head_.load(std::memory_order_acquire);
    while (tail != head) {
        motion_engine.submitRawImu(imu_queue_[tail], now_us);
        tail = (tail + 1u) % kImuQueueSize;
        imu_queue_tail_.store(tail, std::memory_order_release);
    }
}

void Robot::update_motion_v2(motion::TimeUs now_us) {
    if (!motion_engine_ready_) ensure_motion_engine(now_us);
    drain_raw_imu(now_us);

    motion::WheelSample wheels;
    wheels.sample_time_us = now_us;
    wheels.sequence = static_cast<std::uint32_t>(now_us);
    wheels.valid = true;
    for (uint8_t i = 0; i < 4; ++i) {
        const auto time = wheel_motors[i]->feedback_time_us();
        wheels.valid = wheels.valid && wheel_motors[i]->is_enabled() &&
            time != 0 && time <= now_us && now_us - time <= 15000;
        if (time < wheels.sample_time_us) wheels.sample_time_us = time;
        wheels.wheel_speed_rad_s[i] =
            wheel_motors[i]->get_velocity() * control_config::kPi / 30.0f;
    }
    motion_engine.submitWheels(wheels);
    if (latest_flow_.valid) motion_engine.submitOptFlow(latest_flow_);
    motion_engine.step(now_us, &motion_control_);

    for (uint8_t i = 0; i < 4; ++i) {
        motion_torque_ports_[i] = motion_control_.applied_torque.nm[i];
    }
}

void Robot::process_spi_v2(motion::TimeUs now_us) {
    if (!motion_engine_ready_) ensure_motion_engine(now_us);
    std::int16_t encoders[4];
    for (uint8_t i = 0; i < 4; ++i) {
        encoders[i] = static_cast<std::int16_t>(
            std::clamp(wheel_motors[i]->get_velocity() * 10.0f, -32768.0f, 32767.0f));
    }
    motion_engine.setAuxSnapshot(bat_ADC2_val * 5.0f, cap_ADC3_val * 100.0f,
                                 infrare_flag ? 1.0f : -1.0f, true, encoders);
    motion_engine.onSpiFrame(spi_rx_data, spi_tx_data, now_us);

    const auto& command = motion_engine.core().activeCommand();
    if (motion_engine.core().commandActive() &&
        motion_engine.core().safety() == motion::SafetyState::kReady) {
        dribble_power = command.dribble_power;
        dribbler_mode = command.dribble_mode;
        dribble_velocity = command.dribble_velocity_rps;
        dribble_torque_ff = command.dribble_torque_ff_nm;
        kick_mode = command.chip_mode;
        kick_discharge_time = command.kick_discharge_time_us;
    } else {
        dribble_power = 0.0f;
        kick_discharge_time = 0;
    }
}

void Robot::request_kick_from_spi() {
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

    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim6, pulse_us - 1u);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        HAL_GPIO_WritePin(CHIP_GPIO_Port, CHIP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SHOOT_GPIO_Port, SHOOT_Pin, GPIO_PIN_RESET);
        kick_active_ = false;
    }
}

void Robot::on_kick_timeout_irq() {
    HAL_TIM_Base_Stop_IT(&htim6);
    HAL_GPIO_WritePin(CHIP_GPIO_Port, CHIP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHOOT_GPIO_Port, SHOOT_Pin, GPIO_PIN_RESET);
    kick_active_ = false;
}
