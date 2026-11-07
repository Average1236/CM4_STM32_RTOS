#ifndef MOTION_ENGINE_HPP
#define MOTION_ENGINE_HPP

// STM32 production motion boundary.  This file is hardware independent and is
// the firmware only supplies clock, sensor snapshots and the SPI buffers.

#include "motion/motion_core.hpp"
#include "motion/protocol.hpp"
#include "motion/time_mapper.hpp"

#include <array>
#include <cstdint>

namespace stm32_motion {

// Kick execution feedback state machine (docs/MUJOCO_MATCH_EXECUTION.md).
//
// One event is created when the production kick gate approves a request; its
// actual outcome is committed by noteKickExecution() (board/host) or, when no
// explicit result arrives, reported as executed at the next feedback frame.
// Committed events stay in a bounded per-robot unacknowledged queue and are
// retransmitted (one per feedback frame) until a matching execution ACK
// arrives.  Duplicate feedback never extends a status pulse or the watchdog.
class MotionEngine {
public:
    void configure(const motion::MotionConfigV2& config,
                   const motion::ImuPreprocessorConfig& imu_config,
                   bool enable_spi_watchdog, motion::TimeUs now_us);
    void reset(motion::TimeUs now_us);
    void resetToVision(const motion::VisionObservationV2& vision,
                       motion::TimeUs now_us);

    // One SPI v2 exchange: `rx_frame` is the byte stream from CM4, `tx_frame`
    // is the feedback frame clocked back in the same transfer.  No additional
    // exchange or control cycle is waited for.
    bool onSpiFrame(const std::uint8_t* rx_frame, std::uint8_t* tx_frame,
                    motion::TimeUs now_us);
    bool applyConfigFrame(const std::uint8_t* frame, motion::TimeUs now_us);

    void step(motion::TimeUs now_us, motion::ControllerOutput* control);
    bool submitRawImu(const motion::RawImuSample& raw, motion::TimeUs now_us);
    void submitWheels(const motion::WheelSample& sample);
    void submitOptFlow(const motion::OptFlowSample& sample);
    void submitVision(const motion::VisionObservationV2& vision, motion::TimeUs now_us);
    void setAuxSnapshot(float battery_voltage, float capacitance_voltage,
                        float infrared_age_ms, bool have_imu,
                        const std::int16_t wheel_encoder[4]);

    // Match execution mode.  Disabled by default: every v3 byte stays exactly
    // as before.  When enabled the engine takes part in the startup capability
    // handshake and only accepts kicks after it completed.
    void setMatchExecutionEnabled(bool enabled);
    bool matchExecutionEnabled() const { return match_execution_.enabled(); }
    bool matchExecutionReady() const { return match_execution_.ready(); }
    bool matchExecutionRejected() const { return match_execution_.rejected(); }
    const char* matchExecutionError() const { return match_execution_.error(); }
    std::uint32_t matchExecutionCapabilityBits() const {
        return match_execution_.localCapabilities();
    }
    // Identity context of the shared match world.  The host (which emulates the
    // board) sets these before serving the SPI link; the real robot leaves the
    // defaults, so execution feedback carries the session it was created in.
    void setMatchExecutionContext(std::uint32_t world_session_id,
                                  std::uint32_t placement_generation);
    std::uint32_t matchWorldSessionId() const { return world_session_id_; }
    std::uint32_t matchPlacementGeneration() const { return placement_generation_; }

    // Commits the outcome of the event most recently approved by kickAllowed().
    // `discharge_us`/`executed_time_us` default to the approved values.  When
    // the board (or host world) does not call this, the approval itself is
    // reported as executed on the next feedback frame.
    void noteKickExecution(motion::TimeUs now_us, bool executed,
                           std::uint16_t discharge_us = 0,
                           motion::TimeUs executed_time_us = 0);
    // Command-direction ACK: removes exactly that unacknowledged event.  ACKs
    // are idempotent; unmatched duplicates are counted but never touch command
    // age, validity or any watchdog.
    bool acknowledgeKickExecution(const motion::KickExecutionAck& ack);

    std::size_t pendingKickExecutions() const { return kick_execution_count_; }
    std::uint32_t kickExecutionEvents() const { return kick_execution_events_; }
    std::uint32_t kickExecutionOverflows() const { return kick_execution_overflows_; }
    std::uint32_t kickExecutionAcks() const { return kick_execution_acks_; }
    std::uint32_t kickExecutionStaleAcks() const { return kick_execution_stale_acks_; }
    std::uint32_t kickExecutionDeduplicated() const {
        return kick_execution_deduplicated_;
    }
    std::uint32_t kickExecutionNotReady() const { return kick_execution_not_ready_; }
    std::uint32_t kickExecutionExecuted() const { return kick_execution_executed_; }
    std::uint32_t kickExecutionRejected() const { return kick_execution_rejected_; }
    // Last execution feedback block put on the wire (retransmissions included).
    // Diagnostics/telemetry only; the authoritative copy is the SPI frame.
    const motion::KickExecutionFeedback& lastKickExecutionFeedback() const {
        return last_kick_execution_feedback_;
    }

    const motion::MotionCore& core() const { return core_; }
    motion::MotionCore& core() { return core_; }
    const motion::ControllerOutput& control() const { return control_; }
    const motion::MotionFeedbackV2& feedback() const { return core_.feedback(); }
    const motion::MotionConfigV2& config() const { return config_; }

    // Unified safety gate outputs consumed by the firmware tasks.
    bool motorsEnabled() const;
    bool dribbleAllowed() const;
    bool kickAllowed(motion::TimeUs now_us);
    bool spiFresh(motion::TimeUs now_us) const;

    std::uint32_t crcErrors() const { return crc_errors_; }
    std::uint32_t framesReceived() const { return frames_received_; }
    std::uint32_t configFrames() const { return config_frames_; }

private:
    bool handleCommandFrame(const std::uint8_t* frame, motion::TimeUs now_us);
    void mapCommandTimes(motion::MotionCommandV2* command,
                         motion::VisionObservationV2* vision, bool* has_vision,
                         motion::TimeUs now_us);
    void encodeFeedback(std::uint8_t* frame, motion::TimeUs now_us);

    // Match execution state machine internals.
    struct PendingKick {
        motion::KickExecutionFeedback feedback;
        bool result_pending = false;  // approved; explicit outcome may still arrive
    };
    void beginKickExecution(motion::TimeUs now_us);
    void commitKickExecution(motion::KickExecutionState state, std::uint16_t discharge_us,
                             motion::TimeUs executed_time_us);
    void queueAuditKick(motion::KickExecutionState state, motion::TimeUs now_us,
                        std::uint16_t discharge_us);
    bool dequeueKickExecution(const motion::KickExecutionAck& ack);
    motion::KickExecutionFeedback nextRetransmission();
    void clearKickExecutions();

    motion::MotionCore core_;
    motion::MotionConfigV2 config_;
    motion::ImuPreprocessorConfig imu_config_;
    motion::TimeMapper command_time_map_;
    motion::SequenceGate config_sequences_;
    motion::ControllerOutput control_;
    bool spi_watchdog_enabled_ = false;
    std::uint32_t crc_errors_ = 0;
    std::uint32_t frames_received_ = 0;
    std::uint32_t config_frames_ = 0;
    std::uint32_t last_kick_sequence_ = 0;
    std::uint32_t last_kick_session_ = 0;
    motion::TimeUs kick_event_time_us_ = 0;
    bool kick_seen_ = false;
    bool kick_consumed_ = false;
    motion::TimeUs last_spi_frame_us_ = 0;
    bool has_spi_frame_ = false;
    float battery_voltage_ = 0.0f;
    float capacitance_voltage_ = 0.0f;
    float infrared_age_ms_ = -1.0f;
    bool have_imu_ = false;
    std::int16_t wheel_encoder_[4] = {0, 0, 0, 0};

    motion::MatchExecutionNegotiator match_execution_;
    std::uint32_t world_session_id_ = 0;
    std::uint32_t placement_generation_ = 0;
    bool kick_dedupe_counted_ = false;
    std::array<PendingKick, motion::kMatchMaxPendingKickExecutions> kick_executions_{};
    std::size_t kick_execution_count_ = 0;
    std::size_t kick_execution_send_ = 0;
    std::uint32_t kick_feedback_sequence_ = 0;
    // One-shot audit feedback (e.g. queue-full rejection).  Bounded, never
    // retransmitted, and never overwrites an unacknowledged execution event.
    motion::KickExecutionFeedback audit_kick_;
    bool has_audit_kick_ = false;
    std::uint32_t kick_execution_events_ = 0;
    std::uint32_t kick_execution_overflows_ = 0;
    std::uint32_t kick_execution_acks_ = 0;
    std::uint32_t kick_execution_stale_acks_ = 0;
    std::uint32_t kick_execution_deduplicated_ = 0;
    std::uint32_t kick_execution_not_ready_ = 0;
    std::uint32_t kick_execution_executed_ = 0;
    std::uint32_t kick_execution_rejected_ = 0;
    motion::KickExecutionFeedback last_kick_execution_feedback_;
};

}  // namespace stm32_motion

#endif  // MOTION_ENGINE_HPP
