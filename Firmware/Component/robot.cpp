#include "robot.hpp"
#include "control_params.hpp"
#include "tim.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <cstddef>

// Debug
volatile float target_vx_debug = 0;
volatile float target_vy_debug = 0;
volatile float target_vw_debug = 0;
volatile float a_des_x_debug = 0;
volatile float a_des_y_debug = 0;
volatile float wheel_vel_debug = 0;
volatile float v_des_x_debug = 0;
volatile float v_des_y_debug = 0;
volatile float kick_discharge_time_debug = 0;
volatile uint16_t kick_pulse_debug = 0;
volatile float dribble_power_debug = 0;
volatile uint8_t dribbler_mode_debug = 0;
volatile int use_imu_debug = 0;
volatile float dribble_velocity_debug = 0;
volatile float dribble_torque_ff_debug = 0;
volatile float yaw_vel_max_debug = 0;
volatile float yaw_acc_max_debug = 0;
volatile float yaw_jerk_max_debug = 0;
volatile float yaw_target_debug = 0;
volatile float yaw_ref_debug = 0;
volatile float yaw_ref_vel_debug = 0;
volatile float yaw_ref_acc_debug = 0;
volatile float fallback_yaw_integ_debug = 0;
volatile float fallback_yaw_diff_debug = 0;

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

constexpr uint16_t kKickPulseMaxUs = 15000;
constexpr uint32_t kKickIntervalMs = 300;
constexpr std::size_t kSpiCrcLength = 2;
constexpr std::size_t kSpiCrcPayloadLength = SPI_LENGTH - kSpiCrcLength;
constexpr uint32_t kSpiMotionFailsafeBadFrames = 50;
constexpr uint32_t kSpiDribbleHoldBadFrames = 500;

uint16_t spi_crc16_ccitt(const uint8_t* data, std::size_t length) {
    uint16_t crc = 0xffffu;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000u) != 0u) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

uint16_t read_spi_frame_crc(const uint8_t* frame) {
    return static_cast<uint16_t>(frame[kSpiCrcPayloadLength]) |
           (static_cast<uint16_t>(frame[kSpiCrcPayloadLength + 1]) << 8);
}

void write_spi_frame_crc(uint8_t* frame) {
    const uint16_t crc = spi_crc16_ccitt(frame, kSpiCrcPayloadLength);
    frame[kSpiCrcPayloadLength] = static_cast<uint8_t>(crc & 0xffu);
    frame[kSpiCrcPayloadLength + 1] = static_cast<uint8_t>(crc >> 8);
}

bool spi_frame_crc_valid(const uint8_t* frame) {
    return read_spi_frame_crc(frame) == spi_crc16_ccitt(frame, kSpiCrcPayloadLength);
}

void apply_spi_failsafe_command(Robot& robot_ref) {
    robot_ref.robot_vel[0] = 0.0f;
    robot_ref.robot_vel[1] = 0.0f;
    robot_ref.robot_vel[2] = robot_ref.use_imu ? robot_ref.yaw_target_rad : 0.0f;
    robot_ref.acceleration_ff[0] = 0.0f;
    robot_ref.acceleration_ff[1] = 0.0f;
    robot_ref.raw_vision_vel_mm_s[0] = 0.0f;
    robot_ref.raw_vision_vel_mm_s[1] = 0.0f;
    robot_ref.vision_source = 0.0f;
    robot_ref.kick_discharge_time = 0;

    if (robot_ref.spi_crc_bad_streak > kSpiDribbleHoldBadFrames) {
        robot_ref.dribble_power = 0.0f;
        robot_ref.dribble_velocity = 0.0f;
        robot_ref.dribble_torque_ff = 0.0f;
    }

    target_vx_debug = robot_ref.robot_vel[0];
    target_vy_debug = robot_ref.robot_vel[1];
    target_vw_debug = robot_ref.robot_vel[2];
    yaw_target_debug = robot_ref.yaw_target_rad;
    kick_discharge_time_debug = 0.0f;
    dribble_power_debug = robot_ref.dribble_power;
    dribble_velocity_debug = robot_ref.dribble_velocity;
    dribble_torque_ff_debug = robot_ref.dribble_torque_ff;
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

int16_t encode_angular_milli(const float value) {
    return encode_i16(value * 1000.0f);
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
    // Init yaw fallback TD (non-IMU mode only)
    // vx/vy no longer use TDs — Butterworth LPFs are initialized in-class (robot.hpp)
    TD::Parameter_t td_p{};
    td_p.r  = 100.0f;
    td_p.h  = 0.05f;
    td_p.dt = control_config::kControlDtSec;
    td_p.is_cycle = false;
    td_yaw_fallback_ = new TD(td_p, 0.0f);

    // Init vx/vy reference Butterworth LPFs
    vx_ref_lpf_ = new ButterworthLowPass2(
        {control_config::kChassisVxRefButterworthCutoffHz, control_config::kControlDtSec}, 0.0f);
    vy_ref_lpf_ = new ButterworthLowPass2(
        {control_config::kChassisVyRefButterworthCutoffHz, control_config::kControlDtSec}, 0.0f);
    yaw_fallback_omega_z_lpf_ = new ButterworthLowPass2(
        {control_config::kYawFallbackOmegaZFilterCutoffHz, control_config::kControlDtSec}, 0.0f);

    // Initialize wheel motors
    for (int i = 0; i < 4; i++) {
        wheel_motors[i] = new MotorDMH3510(WHEEL_MOTOR_PARAMS[i]);

        wheel_motors[i]->velocity_cmd_input_port()->connect_to(&motor_vel[i]);
        chassis_estimator.wheel_velocity_input_port(i)->connect_to(wheel_motors[i]->velocity_output_port());
        wheel_motors[i]->torque_ff_cmd_input_port()->connect_to(&motion_torque_ports_[i]);
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
    delete vx_ref_lpf_;
    delete vy_ref_lpf_;
    delete yaw_fallback_omega_z_lpf_;
    delete td_yaw_fallback_;
}


bool Robot::select_motion_backend(bool use_v2, motion::TimeUs now_us) {
    if (use_v2 == motion_v2_enabled_) return true;
    if (motion_engine_ready_ && !motion_engine.core().isStopped()) return false;
    motion_v2_enabled_ = use_v2;
    for (uint8_t i = 0; i < 4; ++i) {
        wheel_motors[i]->set_external_torque_only(use_v2);
    }
    if (use_v2) {
        ensure_motion_engine(now_us);
    } else {
        motion_engine_ready_ = false;
    }
    return true;
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
    for (uint8_t i = 0; i < 4; ++i) {
        wheel_motors[i]->set_external_torque_only(true);
    }
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
    motion_engine.step(now_us, &motion_control_);

    for (uint8_t i = 0; i < 4; ++i) {
        const float target_rpm =
            motion_control_.wheel_speed_target.rad_s[i] * 30.0f / control_config::kPi;
        motor_vel[i] = target_rpm;
        motion_torque_ports_[i] = motion_control_.applied_torque.nm[i];
    }
    const motion::EstimatorOutput& estimate = motion_engine.core().estimate();
    robot_real_vel[0] = estimate.velocity.linear.x;
    robot_real_vel[1] = estimate.velocity.linear.y;
    robot_real_vel[2] = estimate.velocity.angular;
}

void Robot::push_legacy_torque_to_motion_ports() {
    for (uint8_t i = 0; i < 4; ++i) {
        const auto torque = chassis_controller.wheel_torque_ff_output_port(i)->any();
        motion_torque_ports_[i] = torque.has_value() ? *torque : 0.0f;
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
        // The unified safety state gates dribbler and kicker together with the
        // four wheels; an expired command must not leave any actuator armed.
        dribble_power = 0.0f;
        kick_discharge_time = 0;
    }
}

void Robot::pi_decode_spi() {
    if (!spi_frame_crc_valid(spi_rx_data)) {
        spi_error_count++;
        if (spi_crc_bad_streak < 0xffffffffu) {
            spi_crc_bad_streak++;
        }
        if (spi_crc_bad_streak > kSpiMotionFailsafeBadFrames) {
            apply_spi_failsafe_command(*this);
        }
        return;
    }

    spi_crc_ok_count++;
    spi_crc_bad_streak = 0;
    memcpy(&SpiRx, spi_rx_data, sizeof(SpiRx));

    // Decode robot velocity commands from Pi (if implemented)
    for (uint8_t i = 0; i < 2; i++) {
        robot_vel[i] = SpiRx.vel[i] / 1000.0f;
    }
    robot_vel[2] = SpiRx.vel[2] / 100.0f;

    const uint8_t prev_dribbler_mode = dribbler_mode;
    dribble_power = SpiRx.drib_power;

    dribbler_mode = SpiRx.drib_mode;
    if (dribbler_mode < control_config::kDribblerModeTorque ||
        dribbler_mode > control_config::kDribblerModeHybrid) {
        dribbler_mode = control_config::kDribblerModeHybrid;
    }

    if (dribbler_mode == control_config::kDribblerModeHybrid &&
        prev_dribbler_mode != control_config::kDribblerModeHybrid) {
        dribbler_hybrid_phase = kDribblerHybridTorquePhase;
        dribbler_ball_hold_count = 0;
    }

    dribble_velocity = SpiRx.drib_velocity / 100.0f;
    dribble_torque_ff = SpiRx.drib_torque_ff / 1000.0f;
    if (dribbler_mode == control_config::kDribblerModeSpeed) {
        if (SpiRx.drib_power == 10) {
            dribble_velocity = -10.0f;
            dribble_torque_ff = 0.15f;
        } else if (SpiRx.drib_power == 20) {
            dribble_velocity = -50.0f;
            dribble_torque_ff = 0.15f;
        } else if (SpiRx.drib_power == 30) {
            dribble_velocity = -120.0f;
            dribble_torque_ff = 0.15f;
        }
    }

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

    // // FIXME: temporary hardcode
    // yaw_max_vel = 10.0f;

    yaw_vel_max_debug = yaw_max_vel;
    yaw_acc_max_debug = yaw_max_acc;
    yaw_jerk_max_debug = yaw_max_jerk;
    raw_vision_vel_mm_s[0] = static_cast<float>(SpiRx.raw_vision_vel[0]);
    raw_vision_vel_mm_s[1] = static_cast<float>(SpiRx.raw_vision_vel[1]);
    vision_source = static_cast<float>(SpiRx.vision_source);
    for (uint8_t axis = 0; axis < 2; ++axis) {
        for (uint8_t gain = 0; gain < 3; ++gain) {
            if (SpiRx.chassis_vel_pid[axis][gain] >= 0) {
                chassis_vel_pid[axis][gain] = SpiRx.chassis_vel_pid[axis][gain] / 100.0f;
            }
        }
        acceleration_ff[axis] = SpiRx.acceleration_ff[axis] / 1000.0f;
    }
    chassis_controller.set_velocity_pid_gains(chassis_vel_pid[0], chassis_vel_pid[1]);
    // In IMU mode robot_vel[2] is a yaw angle, not a velocity — do not clamp to
    // velocity limits here.  Velocity clamping happens in the yaw reference tracker.
    if (!use_imu) {
        robot_vel[2] = std::clamp(robot_vel[2], -yaw_max_vel, yaw_max_vel);
    }

    kick_mode = SpiRx.kick_mode ? false : true;
    kick_discharge_time = SpiRx.kick_discharge_time;
    kick_discharge_time_debug = kick_discharge_time;

    use_imu = SpiRx.use_imu;
    // use_imu = true;
    use_imu_debug = use_imu ? 1 : 0;

    if (use_imu) {
        yaw_target_rad = wrap_to_pi(robot_vel[2]);
    } else {
        yaw_target_initialized = false;
    }

    // Debug
    target_vx_debug = robot_vel[0];
    target_vy_debug = robot_vel[1];
    target_vw_debug = robot_vel[2];
    yaw_target_debug = yaw_target_rad;
    dribble_power_debug = dribble_power;
    dribbler_mode_debug = dribbler_mode;
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

    SpiTx.infrare_flag = infrare_flag;
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
    const auto chassis_yaw = chassis_estimator.chassis_yaw_output_port()->any();
    const auto chassis_omega_z = chassis_estimator.chassis_omega_z_output_port()->any();
    SpiTx.chassis_yaw_rad = encode_angular_milli(chassis_yaw.has_value() ? *chassis_yaw : 0.0f);
    SpiTx.chassis_omega_z = encode_angular_centi(chassis_omega_z.has_value() ? *chassis_omega_z : 0.0f);
    SpiTx.controller_omega_ref = encode_angular_centi(chassis_controller.omega_ref());
    for (uint8_t i = 0; i < 3; ++i) {
        SpiTx.controller_f_task[i] = encode_angular_centi(chassis_controller.f_task(i));
    }
    for (uint8_t axis = 0; axis < 2; ++axis) {
        for (uint8_t gain = 0; gain < 3; ++gain) {
            SpiTx.chassis_vel_pid[axis][gain] = encode_angular_centi(chassis_vel_pid[axis][gain]);
        }
    }
    SpiTx.reserved = 0;

    // Encode IMU data
    for (uint8_t i = 0; i < 9; i++) {
        SpiTx.imu_data[i] = static_cast<int16_t>(imu_data[i] * 100);
    }

    memset(spi_tx_data, 0, SPI_LENGTH);
    memcpy(spi_tx_data, &SpiTx, sizeof(SpiTx));
    write_spi_frame_crc(spi_tx_data);
}

void Robot::prepare_yaw_control(float dt_s) {
    if (!use_imu) {
        yaw_target_initialized = false;
        yaw_fallback_pid_integ_ = 0.0f;
        if (yaw_fallback_omega_z_lpf_ != nullptr) {
            yaw_fallback_omega_z_lpf_->reset(0.0f);
        }
        return;
    }

    if (dt_s <= 1e-6f) {
        return;
    }

    // Circular LPF on wrapped target
    const float cutoff = control_config::kYawTargetLowPassCutoffHz;
    const float alpha = (cutoff > 0.0f)
        ? (6.283185307f * cutoff * dt_s) / (6.283185307f * cutoff * dt_s + 1.0f)
        : 1.0f;
    yaw_target_lpf_.step(yaw_target_rad, alpha);

    // One-time init
    if (!yaw_target_initialized) {
        yaw_target_lpf_.reset(0.0f);
        yaw_target_initialized = true;
    }

    const float yaw_ref = yaw_target_lpf_.state();
    // Pass filtered target and runtime limits to the yaw trapezoid planner.
    chassis_controller.set_yaw_angle_target(yaw_ref, yaw_max_vel, yaw_max_acc);

    // robot_real_vel[2] / robot_acc[2] no longer used for yaw in IMU mode;
    // yaw trapezoid planner -> inner rate LADRC runs inside ChassisController::step().
    float omega_ref = 0.0f;
    if constexpr (control_config::kUseWheelSpeedPidFallback) {
        const auto yaw_opt = chassis_estimator.chassis_yaw_output_port()->any();
        const auto omega_z_opt = chassis_estimator.chassis_omega_z_output_port()->any();
        const float yaw_rad = yaw_opt.has_value() ? *yaw_opt : 0.0f;
        const float omega_z_raw_rad_s = omega_z_opt.has_value() ? *omega_z_opt : 0.0f;
        const float omega_z_rad_s = (yaw_fallback_omega_z_lpf_ != nullptr)
            ? yaw_fallback_omega_z_lpf_->filter(omega_z_raw_rad_s)
            : omega_z_raw_rad_s;
        const float err_angle = wrap_to_pi(yaw_ref - yaw_rad);

        yaw_fallback_pid_integ_ += control_config::kYawFallbackAnglePidKi * err_angle * dt_s;
        yaw_fallback_pid_integ_ = std::clamp(yaw_fallback_pid_integ_, -yaw_max_vel, yaw_max_vel);

        const float p_out = control_config::kYawFallbackAnglePidKp * err_angle;
        const float i_out = yaw_fallback_pid_integ_;
        const float d_out = -control_config::kYawFallbackAnglePidKd * omega_z_rad_s;
        omega_ref = std::clamp(p_out + i_out + d_out, -yaw_max_vel, yaw_max_vel);
        robot_real_vel[2] = omega_ref;

        fallback_yaw_integ_debug = i_out;
        fallback_yaw_diff_debug = d_out;
    } else {
        yaw_fallback_pid_integ_ = 0.0f;
        if (yaw_fallback_omega_z_lpf_ != nullptr) {
            yaw_fallback_omega_z_lpf_->reset(0.0f);
        }
        robot_real_vel[2] = 0.0f;
    }
    robot_acc[2] = 0.0f;

    // Debug
    yaw_ref_debug     = yaw_ref;
    yaw_ref_vel_debug = omega_ref;
    yaw_ref_acc_debug = 0.0f;
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
    const float dt_s = static_cast<float>(_dt / 1000000.0);
    if (dt_s <= 1e-9f) return;

    // Filter velocity targets and preserve host-planned acceleration feedforward.
    robot_real_vel[0] = vx_ref_lpf_->filter(robot_vel[0]);
    robot_real_vel[1] = vy_ref_lpf_->filter(robot_vel[1]);
    robot_acc[0] = acceleration_ff[0];
    robot_acc[1] = acceleration_ff[1];

    // ── Yaw: TD-based planner (non-IMU fallback only) ──
    if (!use_imu && td_yaw_fallback_ != nullptr) {
        const uint8_t i = 2;
        const float v_target = robot_vel[i];

        td_yaw_fallback_->calc(v_target);
        const float target_vel_est = td_yaw_fallback_->get_data();
        const float target_acc_est = td_yaw_fallback_->get_diff();

        const float e = target_vel_est - last_robot_real_vel[i];
        float a_des = target_acc_est + e / dt_s;
        a_des = std::clamp(a_des, -yaw_max_acc, yaw_max_acc);

        robot_acc[i] = a_des;
        robot_real_vel[i] = last_robot_real_vel[i] + a_des * dt_s;
        last_robot_real_vel[i] = robot_real_vel[i];
    }

    a_des_x_debug = robot_acc[0];
    a_des_y_debug = robot_acc[1];
    v_des_x_debug = robot_real_vel[0];
    v_des_y_debug = robot_real_vel[1];
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

    chassis_controller.set_vxvy_acc_limits(xy_max_acc[0], xy_max_acc[1]);
    chassis_controller.set_reference(robot_real_vel, robot_acc);
    chassis_controller.set_use_imu_yaw(use_imu && !control_config::kUseWheelSpeedPidFallback);
    // set_yaw_target no longer called: outer angle loop runs in prepare_yaw_control(),
    // inner velocity loop tracks vel_ref_[2] (set via set_reference) directly.
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
