#ifndef MOTION_ENGINE_HPP
#define MOTION_ENGINE_HPP

// STM32 production motion boundary.  This file is hardware independent and is
// compiled both into the ARM firmware and into the host integration runner;
// the firmware only supplies clock, sensor snapshots and the SPI buffers.

#include "motion/motion_core.hpp"
#include "motion/protocol.hpp"
#include "motion/time_mapper.hpp"

#include <cstdint>

namespace stm32_motion {

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
    void encodeFeedback(std::uint8_t* frame, motion::TimeUs now_us) const;

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
    bool kick_edge_state_ = false;
    motion::TimeUs last_spi_frame_us_ = 0;
    bool has_spi_frame_ = false;
    float battery_voltage_ = 0.0f;
    float capacitance_voltage_ = 0.0f;
    float infrared_age_ms_ = -1.0f;
    bool have_imu_ = false;
    std::int16_t wheel_encoder_[4] = {0, 0, 0, 0};
};

}  // namespace stm32_motion

#endif  // MOTION_ENGINE_HPP
