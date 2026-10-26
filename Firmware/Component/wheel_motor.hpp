#ifndef WHEEL_MOTOR_HPP
#define WHEEL_MOTOR_HPP
#include "Communication/can/can_helpers.hpp"
#include "component.hpp"
#include <cstdint>

class MotorDMH3510 {
public:
    struct Config_t { uint8_t control_id, feedback_id; float direction; };
    struct Parameter_t { float pmax = 3.1415926535f, vmax = 280.f, tmax = 1.f; };
    enum RegisterWriteCheckResult : uint8_t {
        kRegisterWriteNotAck = 0, kRegisterWriteOk = 1, kRegisterWriteMismatch = 2,
    };

    explicit MotorDMH3510(const Config_t& config);
    void pack_torque_data(float torque, uint8_t* tx_data);
    void parse_feedback_data(const uint8_t rx_data[8]);
    void build_set_torque_mode_msg(can_Message_t& msg);
    void build_enable_msg(can_Message_t& msg);
    void build_clear_error_msg(can_Message_t& msg);
    void build_set_pmax_msg(float pmax, can_Message_t& msg);
    RegisterWriteCheckResult check_write_register_feedback(const can_Message_t& msg);
    uint32_t command_can_id() const { return config_.control_id; }
    uint32_t feedback_can_id() const { return config_.feedback_id; }
    bool torque_mode_configured() const { return torque_mode_configured_; }
    bool is_writing_register() const { return writing_register_; }
    bool is_enabled() const { return enabled_; }
    float get_velocity() const { return velocity_rpm_; }
    void mark_feedback_received(std::uint64_t time_us) { feedback_time_us_ = time_us; }
    std::uint64_t feedback_time_us() const { return feedback_time_us_; }
    InputPort<float>* torque_ff_cmd_input_port() { return &torque_input_; }
    uint8_t pending_write_register_rid() const { return pending_write_register_rid_; }
    uint32_t pending_write_register_data() const { return pending_write_register_data_; }
    uint8_t last_write_register_rid() const { return last_write_register_rid_; }
    uint32_t last_write_register_data() const { return last_write_register_data_; }
private:
    void build_write_register_msg(uint8_t rid, uint32_t data, can_Message_t& msg);
    void update_wheel_speed_pll_gains();
    void reset_wheel_speed_pll(float measured_pos_rad, bool mark_prev_enabled);
    void update_wheel_speed_pll_from_pos(float measured_pos_rad, bool enabled);
    static float wrap_pm_pi(float angle_rad);
    static float uint_to_float(int value, float min, float max, int bits);
    Config_t config_;
    Parameter_t parameter_;
    InputPort<float> torque_input_;
    std::uint64_t feedback_time_us_ = 0;
    float velocity_rpm_ = 0;
    bool enabled_ = false, torque_mode_configured_ = false;
    bool writing_register_ = false;
    uint8_t pending_write_register_rid_ = 0, last_write_register_rid_ = 0;
    uint32_t pending_write_register_data_ = 0, last_write_register_data_ = 0;
    float pll_kp_ = 0, pll_ki_ = 0;
    float pll_pos_est_rad_ = 0, pll_vel_est_rad_s_ = 0;
    bool pll_initialized_ = false, pll_prev_enabled_ = false;
};
#endif
