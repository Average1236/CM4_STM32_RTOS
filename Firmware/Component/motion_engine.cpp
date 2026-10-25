#include "motion_engine.hpp"
#include "motion/validation.hpp"

namespace stm32_motion {

void MotionEngine::configure(const motion::MotionConfigV2& config,
                             const motion::ImuPreprocessorConfig& imu_config,
                             bool enable_spi_watchdog, motion::TimeUs now_us) {
    config_ = config;
    imu_config_ = imu_config;
    spi_watchdog_enabled_ = enable_spi_watchdog;
    core_.configure(config_, imu_config_);
    core_.setSpiWatchdogEnabled(spi_watchdog_enabled_);
    core_.reset(motion::Pose2{}, now_us);
    control_ = motion::ControllerOutput{};
    command_time_map_.reset();
    last_kick_sequence_ = 0;
    last_kick_session_ = 0;
    kick_seen_ = false;
    kick_consumed_ = false;
    kick_edge_state_ = false;
    last_spi_frame_us_ = now_us;
    has_spi_frame_ = false;
    crc_errors_ = 0;
    frames_received_ = 0;
    config_frames_ = 0;
}

void MotionEngine::reset(motion::TimeUs now_us) {
    core_.reset(motion::Pose2{}, now_us);
    control_ = motion::ControllerOutput{};
    last_spi_frame_us_ = now_us;
    has_spi_frame_ = false;
}

void MotionEngine::resetToVision(const motion::VisionObservationV2& vision,
                                 motion::TimeUs now_us) {
    core_.resetToVision(vision, now_us);
}

void MotionEngine::mapCommandTimes(motion::MotionCommandV2* command,
                                   motion::VisionObservationV2* vision,
                                   bool* has_vision, motion::TimeUs now_us) {
    if (command == nullptr) return;
    if (command->generated_time_us != 0) {
        command_time_map_.observe(command->generated_time_us, now_us);
        command->generated_time_us =
            command_time_map_.toLocal(command->generated_time_us);
        if (command->generated_time_us > now_us) command->generated_time_us = now_us;
    } else {
        command->generated_time_us = now_us;
    }
    if (vision != nullptr && has_vision != nullptr && *has_vision) {
        if (vision->capture_time_us != 0) {
            vision->capture_time_us =
                command_time_map_.toLocal(vision->capture_time_us);
            if (vision->capture_time_us > now_us) vision->capture_time_us = now_us;
        } else {
            vision->capture_time_us = now_us;
        }
        vision->receive_time_us = now_us;
    }
}

bool MotionEngine::handleCommandFrame(const std::uint8_t* frame,
                                      motion::TimeUs now_us) {
    motion::MotionCommandV2 command;
    motion::VisionObservationV2 vision;
    bool has_vision = false;
    if (!motion::decodeCommandFrame(frame, &command, &vision, &has_vision)) {
        ++crc_errors_;
        return false;
    }
    mapCommandTimes(&command, &vision, &has_vision, now_us);
    command.received_time_us = now_us;

    if (has_vision && vision.valid) {
        submitVision(vision, now_us);
    }
    if (core_.submitCommand(command, now_us)) {
        const bool new_event = command.kick_requested &&
            (!kick_seen_ || command.session_id != last_kick_session_ ||
             (command.kick_sequence != 0 && command.kick_sequence != last_kick_sequence_) ||
             (command.kick_sequence == 0 && !kick_edge_state_));
        if (new_event) {
            last_kick_session_ = command.session_id;
            last_kick_sequence_ = command.kick_sequence;
            kick_event_time_us_ = command.generated_time_us;
            kick_seen_ = true;
            kick_consumed_ = false;
        }
        kick_edge_state_ = command.kick_requested;
    }
    core_.noteSpiFrame(now_us);
    last_spi_frame_us_ = now_us;
    has_spi_frame_ = true;
    ++frames_received_;
    return true;
}

bool MotionEngine::onSpiFrame(const std::uint8_t* rx_frame, std::uint8_t* tx_frame,
                              motion::TimeUs now_us) {
    bool ok = false;
    if (rx_frame != nullptr) {
        motion::FrameHeader header;
        if (motion::decodeFrameHeader(rx_frame, &header) &&
            motion::frameIsValid(rx_frame, header.type)) {
            if (header.type == motion::FrameType::kConfig) {
                motion::MotionConfigV2 config;
                if (motion::decodeConfigFrame(rx_frame, &config) &&
                    config_sequences_.accepts(header.session_id, header.sequence)) {
                    // A configuration change is only legal at rest.
                    if (core_.isStopped()) {
                        config_sequences_.accept(header.session_id, header.sequence);
                        configure(config, imu_config_, spi_watchdog_enabled_, now_us);
                        // Configuration carries a fresh bridge timestamp.
                        // Seed the mapping before a potentially old command
                        // arrives, so its original age survives boot offsets.
                        command_time_map_.observe(header.generated_time_us, now_us);
                        core_.noteSpiFrame(now_us);
                        last_spi_frame_us_ = now_us;
                        has_spi_frame_ = true;
                        ok = true;
                    }
                }
                ++config_frames_;
            } else if (header.type == motion::FrameType::kCommand) {
                ok = handleCommandFrame(rx_frame, now_us);
            } else {
                ++crc_errors_;
            }
        } else {
            ++crc_errors_;
        }
    }
    encodeFeedback(tx_frame, now_us);
    return ok;
}

bool MotionEngine::applyConfigFrame(const std::uint8_t* frame,
                                    motion::TimeUs now_us) {
    motion::MotionConfigV2 config;
    if (!motion::decodeConfigFrame(frame, &config)) return false;
    if (!core_.isStopped()) return false;
    configure(config, imu_config_, spi_watchdog_enabled_, now_us);
    return true;
}

void MotionEngine::setAuxSnapshot(float battery_voltage, float capacitance_voltage,
                                  float infrared_age_ms, bool have_imu,
                                  const std::int16_t wheel_encoder[4]) {
    battery_voltage_ = battery_voltage;
    capacitance_voltage_ = capacitance_voltage;
    infrared_age_ms_ = infrared_age_ms;
    have_imu_ = have_imu;
    if (wheel_encoder != nullptr) {
        for (int i = 0; i < 4; ++i) wheel_encoder_[i] = wheel_encoder[i];
    }
}

void MotionEngine::encodeFeedback(std::uint8_t* frame, motion::TimeUs now_us) const {
    if (frame == nullptr) return;
    motion::MotionFeedbackV2 feedback = core_.feedback();
    feedback.feedback_time_us = now_us;
    feedback.session_id = core_.activeCommand().session_id;
    feedback.executed_sequence = core_.lastCommandSequence();
    feedback.trajectory_epoch = core_.activeEpoch();
    feedback.safety = core_.safety();
    feedback.battery_voltage = battery_voltage_;
    feedback.capacitance_voltage = capacitance_voltage_;
    feedback.infrared_age_ms = infrared_age_ms_;
    feedback.have_imu = have_imu_;
    for (int i = 0; i < 4; ++i) feedback.wheel_encoder[i] = wheel_encoder_[i];
    motion::encodeFeedbackFrame(feedback, frame);
}

void MotionEngine::step(motion::TimeUs now_us, motion::ControllerOutput* control) {
    core_.step(now_us, &control_);
    if (core_.safety() != motion::SafetyState::kReady) kick_consumed_ = true;
    if (control != nullptr) *control = control_;
}

bool MotionEngine::submitRawImu(const motion::RawImuSample& raw,
                                motion::TimeUs now_us) {
    return core_.submitRawImu(raw, now_us);
}

void MotionEngine::submitWheels(const motion::WheelSample& sample) {
    core_.submitWheels(sample);
}

void MotionEngine::submitOptFlow(const motion::OptFlowSample& sample) {
    core_.submitOptFlow(sample);
}

void MotionEngine::submitVision(const motion::VisionObservationV2& vision,
                                motion::TimeUs now_us) {
    if (!motion::validVision(vision) || vision.capture_time_us > now_us ||
        now_us - vision.capture_time_us > motion::secondsToUs(config_.vision_degrade_s)) return;
    core_.submitVision(vision, now_us);
    if (!core_.estimate().vision_fused) core_.resetToVision(vision, now_us);
}

bool MotionEngine::motorsEnabled() const {
    return core_.safety() == motion::SafetyState::kReady ||
           core_.safety() == motion::SafetyState::kCommandTimeout ||
           core_.safety() == motion::SafetyState::kLocalizationBrake;
}

bool MotionEngine::dribbleAllowed() const {
    return core_.safety() == motion::SafetyState::kReady && core_.commandActive();
}

bool MotionEngine::kickAllowed(motion::TimeUs now_us) {
    if (core_.safety() != motion::SafetyState::kReady || !core_.commandActive()) {
        kick_consumed_ = true;
        return false;
    }
    const motion::MotionCommandV2& command = core_.activeCommand();
    const motion::TimeUs valid_for = command.kick_valid_for_us != 0
        ? command.kick_valid_for_us : motion::secondsToUs(0.1f);
    if (!command.kick_requested || !kick_seen_ || kick_consumed_ ||
        !command.isValidAt(now_us) || now_us < kick_event_time_us_ ||
        now_us - kick_event_time_us_ > valid_for) return false;
    kick_consumed_ = true;
    return true;
}

bool MotionEngine::spiFresh(motion::TimeUs now_us) const {
    if (!has_spi_frame_) return false;
    return now_us >= last_spi_frame_us_ &&
           now_us - last_spi_frame_us_ <=
               motion::secondsToUs(config_.spi_timeout_s);
}

}  // namespace stm32_motion
