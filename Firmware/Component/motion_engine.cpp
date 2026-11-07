#include "motion_engine.hpp"
#include "motion/validation.hpp"

namespace stm32_motion {

void MotionEngine::configure(const motion::MotionConfigV2& config,
                             const motion::ImuPreprocessorConfig& imu_config,
                             bool enable_spi_watchdog, motion::TimeUs now_us) {
    // A configuration frame starts a new CM4 session.  Keep the explicitly
    // enabled match mode, but require a fresh capability handshake and drop
    // unacknowledged events of the previous session (audit only).
    const bool match_enabled = match_execution_.enabled();
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
    kick_dedupe_counted_ = false;
    last_spi_frame_us_ = now_us;
    has_spi_frame_ = false;
    crc_errors_ = 0;
    frames_received_ = 0;
    config_frames_ = 0;
    clearKickExecutions();
    match_execution_.configure(match_enabled, now_us);
}

void MotionEngine::reset(motion::TimeUs now_us) {
    core_.reset(motion::Pose2{}, now_us);
    control_ = motion::ControllerOutput{};
    last_spi_frame_us_ = now_us;
    has_spi_frame_ = false;
    clearKickExecutions();
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
    motion::MatchExecutionExtension extension;
    const bool match_enabled = match_execution_.enabled();
    if (match_enabled) {
        // Match mode: the trailing extension is part of the frame; a malformed
        // tail rejects the whole frame instead of being partially applied.
        if (!motion::decodeCommandFrame(frame, &command, &vision, &has_vision,
                                        &extension)) {
            ++crc_errors_;
            return false;
        }
    } else if (!motion::decodeCommandFrame(frame, &command, &vision, &has_vision)) {
        ++crc_errors_;
        return false;
    }
    mapCommandTimes(&command, &vision, &has_vision, now_us);
    command.received_time_us = now_us;

    if (has_vision && vision.valid) {
        submitVision(vision, now_us);
    }
    // ACK clears exactly the matching unacknowledged event; it carries no new
    // command semantics and never extends command age or the watchdog.
    if (match_enabled && extension.ack.present) {
        acknowledgeKickExecution(extension.ack);
    }
    // Capability declaration of the CM4 side (retransmitted until observed).
    if (match_enabled && extension.capability.present) {
        match_execution_.observePeerCapabilities(extension.capability.capability_bits,
                                                 now_us);
    }
    if (core_.submitCommand(command, now_us)) {
        const bool same_identity = kick_seen_ &&
            command.session_id == last_kick_session_ &&
            command.kick_sequence == last_kick_sequence_;
        const bool new_event = command.kick_requested && !same_identity;
        if (new_event) {
            last_kick_session_ = command.session_id;
            last_kick_sequence_ = command.kick_sequence;
            kick_event_time_us_ = command.generated_time_us;
            kick_seen_ = true;
            kick_consumed_ = false;
            kick_dedupe_counted_ = false;
        } else if (command.kick_requested && same_identity && !kick_dedupe_counted_) {
            // A retransmitted command for an already handled event is an audit
            // duplicate: count it once, never create a second execution event.
            kick_dedupe_counted_ = true;
            ++kick_execution_deduplicated_;
        }
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
    // Capability timeout: a pending startup handshake that never observes the
    // peer declaration reports match_execution_not_supported instead of
    // silently degrading to v3 guesswork.
    match_execution_.poll(now_us);
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

void MotionEngine::encodeFeedback(std::uint8_t* frame, motion::TimeUs now_us) {
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

    motion::MatchExecutionExtension extension;
    if (match_execution_.enabled()) {
        // One retransmission per feedback frame until the event is ACKed; the
        // capability declaration uses the remaining frames so the handshake
        // always completes even if early frames are lost.
        motion::KickExecutionFeedback event = nextRetransmission();
        if (event.present) {
            extension.feedback = event;
        } else if (has_audit_kick_) {
            extension.feedback = audit_kick_;
            has_audit_kick_ = false;
        } else if (match_execution_.needsCapabilityTransmission()) {
            extension.capability =
                match_execution_.declaration(world_session_id_);
        }
        feedback.execution_feedback = extension.feedback;
        feedback.match_capability = extension.capability;
        if (extension.feedback.present) {
            last_kick_execution_feedback_ = extension.feedback;
        }
    }
    motion::encodeFeedbackFrame(feedback, frame);
}

void MotionEngine::setMatchExecutionEnabled(bool enabled) {
    if (enabled == match_execution_.enabled()) return;
    clearKickExecutions();
    match_execution_.configure(enabled, last_spi_frame_us_);
}

void MotionEngine::setMatchExecutionContext(std::uint32_t world_session_id,
                                            std::uint32_t placement_generation) {
    world_session_id_ = world_session_id;
    placement_generation_ = placement_generation;
}

void MotionEngine::clearKickExecutions() {
    for (auto& event : kick_executions_) event = PendingKick{};
    kick_execution_count_ = 0;
    kick_execution_send_ = 0;
    kick_feedback_sequence_ = 0;
    audit_kick_ = motion::KickExecutionFeedback{};
    has_audit_kick_ = false;
    kick_execution_events_ = 0;
    kick_execution_overflows_ = 0;
    kick_execution_acks_ = 0;
    kick_execution_stale_acks_ = 0;
    kick_execution_deduplicated_ = 0;
    kick_execution_not_ready_ = 0;
    kick_execution_executed_ = 0;
    kick_execution_rejected_ = 0;
    last_kick_execution_feedback_ = motion::KickExecutionFeedback{};
    kick_dedupe_counted_ = false;
}

void MotionEngine::beginKickExecution(motion::TimeUs now_us) {
    const motion::MotionCommandV2& command = core_.activeCommand();
    PendingKick pending;
    auto& event = pending.feedback;
    event.present = true;
    event.feedback_sequence = ++kick_feedback_sequence_;
    event.world_session_id = world_session_id_;
    event.placement_generation = placement_generation_;
    event.producer_session_id = command.session_id;
    event.kick_sequence = command.kick_sequence;
    event.event_type = command.chip_mode ? motion::KickEventType::kChip
                                         : motion::KickEventType::kFlat;
    const std::uint16_t requested = command.kick_discharge_time_us;
    event.discharge_us = requested > motion::kMatchMaxKickDischargeUs
                             ? motion::kMatchMaxKickDischargeUs
                             : requested;
    event.executed_time_us = now_us;
    event.state = motion::KickExecutionState::kExecuted;
    event.queue_depth = static_cast<std::uint8_t>(kick_execution_count_ + 1);
    pending.result_pending = true;
    kick_executions_[kick_execution_count_++] = pending;
    ++kick_execution_events_;
}

void MotionEngine::commitKickExecution(motion::KickExecutionState state,
                                       std::uint16_t discharge_us,
                                       motion::TimeUs executed_time_us) {
    // Resolve the most recent approved event; approved events are queued
    // optimistically as executed so multiple approvals in one SPI period are
    // never lost.
    for (std::size_t i = kick_execution_count_; i > 0; --i) {
        PendingKick& event = kick_executions_[i - 1];
        if (!event.result_pending) continue;
        event.feedback.state = state;
        if (discharge_us != 0) event.feedback.discharge_us = discharge_us;
        if (executed_time_us != 0) event.feedback.executed_time_us = executed_time_us;
        if (state != motion::KickExecutionState::kExecuted) {
            event.feedback.event_type = motion::KickEventType::kNone;
            event.feedback.discharge_us = 0;
            ++kick_execution_rejected_;
        } else {
            ++kick_execution_executed_;
        }
        event.result_pending = false;
        return;
    }
}

void MotionEngine::noteKickExecution(motion::TimeUs now_us, bool executed,
                                     std::uint16_t discharge_us,
                                     motion::TimeUs executed_time_us) {
    if (!match_execution_.enabled()) return;
    commitKickExecution(executed ? motion::KickExecutionState::kExecuted
                                 : motion::KickExecutionState::kRejected,
                        discharge_us, executed_time_us != 0 ? executed_time_us : now_us);
}

void MotionEngine::queueAuditKick(motion::KickExecutionState state,
                                  motion::TimeUs now_us,
                                  std::uint16_t discharge_us) {
    if (has_audit_kick_) return;  // one pending audit record is enough
    const motion::MotionCommandV2& command = core_.activeCommand();
    motion::KickExecutionFeedback event;
    event.present = true;
    event.feedback_sequence = ++kick_feedback_sequence_;
    event.world_session_id = world_session_id_;
    event.placement_generation = placement_generation_;
    event.producer_session_id = command.session_id;
    event.kick_sequence = command.kick_sequence;
    event.event_type = state == motion::KickExecutionState::kExecuted
                           ? (command.chip_mode ? motion::KickEventType::kChip
                                                : motion::KickEventType::kFlat)
                           : motion::KickEventType::kNone;
    event.discharge_us = discharge_us;
    event.executed_time_us = now_us;
    event.state = state;
    event.queue_depth = static_cast<std::uint8_t>(kick_execution_count_);
    audit_kick_ = event;
    has_audit_kick_ = true;
    ++kick_execution_rejected_;
}

motion::KickExecutionFeedback MotionEngine::nextRetransmission() {
    if (kick_execution_count_ == 0) return motion::KickExecutionFeedback{};
    // The optimistic "executed" state stands when no explicit outcome arrived
    // before the frame; that is the unchanged hardware path.
    for (auto& event : kick_executions_) {
        if (!event.result_pending) continue;
        event.result_pending = false;
        ++kick_execution_executed_;
    }
    const std::size_t index = kick_execution_send_ % kick_execution_count_;
    ++kick_execution_send_;
    motion::KickExecutionFeedback feedback = kick_executions_[index].feedback;
    feedback.present = true;
    feedback.queue_depth = static_cast<std::uint8_t>(kick_execution_count_);
    return feedback;
}

bool MotionEngine::dequeueKickExecution(const motion::KickExecutionAck& ack) {
    for (std::size_t i = 0; i < kick_execution_count_; ++i) {
        const auto& event = kick_executions_[i].feedback;
        if (event.world_session_id != ack.world_session_id ||
            event.producer_session_id != ack.producer_session_id ||
            event.kick_sequence != ack.kick_sequence) {
            continue;
        }
        for (std::size_t j = i + 1; j < kick_execution_count_; ++j) {
            kick_executions_[j - 1] = kick_executions_[j];
        }
        --kick_execution_count_;
        kick_executions_[kick_execution_count_] = PendingKick{};
        if (kick_execution_count_ == 0) kick_execution_send_ = 0;
        return true;
    }
    return false;
}

bool MotionEngine::acknowledgeKickExecution(const motion::KickExecutionAck& ack) {
    if (!match_execution_.enabled() || !ack.present) return false;
    if (dequeueKickExecution(ack)) {
        ++kick_execution_acks_;
        return true;
    }
    // ACKs for already cleared events are idempotent (lost-ACK retransmission);
    // ACKs of an old placement/session only complete the audit trail.
    ++kick_execution_stale_acks_;
    return false;
}

void MotionEngine::step(motion::TimeUs now_us, motion::ControllerOutput* control) {
    core_.step(now_us, &control_);
    if (core_.safety() != motion::SafetyState::kReady ||
        core_.estimate().quality != motion::LocalizationQuality::kGood) kick_consumed_ = true;
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
    return core_.safety() == motion::SafetyState::kReady && core_.commandActive() &&
           core_.estimate().quality == motion::LocalizationQuality::kGood;
}

bool MotionEngine::kickAllowed(motion::TimeUs now_us) {
    if (core_.safety() != motion::SafetyState::kReady || !core_.commandActive() ||
        core_.estimate().quality != motion::LocalizationQuality::kGood) {
        kick_consumed_ = true;
        return false;
    }
    const motion::MotionCommandV2& command = core_.activeCommand();
    const motion::TimeUs valid_for = command.kick_valid_for_us != 0
        ? command.kick_valid_for_us : motion::secondsToUs(0.1f);
    if (!command.kick_requested || !kick_seen_ || kick_consumed_ ||
        !command.isValidAt(now_us) || now_us < kick_event_time_us_ ||
        now_us - kick_event_time_us_ > valid_for) return false;
    if (match_execution_.enabled()) {
        if (!match_execution_.ready()) {
            // Match play may only start after the capability handshake; the
            // request is terminally rejected, never silently executed as v3.
            kick_consumed_ = true;
            ++kick_execution_not_ready_;
            queueAuditKick(motion::KickExecutionState::kRejected, now_us, 0);
            return false;
        }
        // Bounded unacknowledged queue: reject the NEW kick instead of
        // overwriting an older unconfirmed event.
        if (kick_execution_count_ >= motion::kMatchMaxPendingKickExecutions) {
            kick_consumed_ = true;
            ++kick_execution_overflows_;
            queueAuditKick(motion::KickExecutionState::kRejected, now_us, 0);
            return false;
        }
    }
    kick_consumed_ = true;
    if (match_execution_.enabled()) beginKickExecution(now_us);
    return true;
}

bool MotionEngine::spiFresh(motion::TimeUs now_us) const {
    if (!has_spi_frame_) return false;
    return now_us >= last_spi_frame_us_ &&
           now_us - last_spi_frame_us_ <=
               motion::secondsToUs(config_.spi_timeout_s);
}

}  // namespace stm32_motion
