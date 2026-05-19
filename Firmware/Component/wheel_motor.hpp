#ifndef __WHEEL_MOTOR_HPP
#define __WHEEL_MOTOR_HPP

#include "Task/utils.hpp"
#include "Communication/can/can_helpers.hpp"
#include "component.hpp"
#include <cstdint>
#include <limits>

class WheelMotorBase {
public:
    enum Type {
        kTypeDMH3510 = 0,
    };

    struct Config_t {
        uint8_t control_id; // 1~255
        uint8_t feedback_id;
        float direction;
        float ex_reduce_rate;
        bool remove_built_in_reducer;
        float limit_vel_curr_proportion = 0.9f;
    };

    struct Info_t {
        float motor_current_limit;
        float motor_torque_limit;
        float broad_current_limit;
        float raw_reduce_rate;
    };

    enum Mode : uint8_t {
        kModeMITControl = 1,
        kModePositionVelocityControl = 2,
        kModeVelocityControl = 3,
        kModeMixedControl = 4,
    };

    virtual ~WheelMotorBase() = default;

    virtual void pack_velocity_data(float velocity, uint8_t* tx_data) = 0;
    virtual void pack_mit_data(float position, float velocity, float kp, float kd, float torque_ff, uint8_t* tx_data) = 0;
    virtual void parse_feedback_data(const uint8_t rx_data[8]) = 0;

    virtual void build_set_mode_msg(Mode mode, can_Message_t& msg) = 0;
    virtual void build_enable_msg(can_Message_t& msg) = 0;
    virtual void build_disable_msg(can_Message_t& msg) = 0;
    virtual void build_clear_error_msg(can_Message_t& msg) = 0;
    virtual void build_save_zero_msg(can_Message_t& msg) = 0;
    virtual void build_set_velocity_kp_msg(float kp, can_Message_t& msg) = 0;
    virtual void build_set_velocity_ki_msg(float ki, can_Message_t& msg) = 0;
    virtual void build_set_acc_msg(float acc, can_Message_t& msg) = 0;
    virtual void build_set_dec_msg(float dec, can_Message_t& msg) = 0;
    virtual void build_set_pmax_msg(float pmax, can_Message_t& msg) = 0;
    virtual void build_set_vmax_msg(float vmax, can_Message_t& msg) = 0;
    virtual void build_set_tmax_msg(float tmax, can_Message_t& msg) = 0;
    virtual void reset_wheel_speed_pid() = 0;

    virtual uint32_t command_can_id() const = 0;
    virtual uint32_t feedback_can_id() const { return static_cast<uint32_t>(config_.feedback_id); }

    Mode get_mode() const { return mode_; }
    float get_angle() const { return angle_; }
    float get_velocity() const { return vel_; }
    float get_current() const { return current_; }
    float get_torque() const { return torque_; }
    Type get_type() const { return type_; }
    bool is_enabled() const { return enabled_; }

    OutputPort<float>* angle_output_port() { return &angle_output_port_; }
    OutputPort<float>* velocity_output_port() { return &velocity_output_port_; }
    OutputPort<float>* torque_output_port() { return &torque_output_port_; }
    OutputPort<float>* current_output_port() { return &current_output_port_; }
    OutputPort<float>* torque_cmd_output_port() { return &torque_cmd_output_port_; }

    InputPort<float>* velocity_cmd_input_port() { return &velocity_cmd_input_port_; }
    InputPort<float>* torque_ff_cmd_input_port() { return &torque_ff_cmd_input_port_; }

    void reset_ports();

protected:
    WheelMotorBase(Type type, const Config_t& config, const Info_t& info);
    void publish_feedback_ports();

    float angle_ = 0.0f;
    float vel_ = 0.0f;
    float current_ = 0.0f;
    float torque_ = 0.0f;

    Type type_;
    Info_t info_;
    Config_t config_;
    Mode mode_ = kModeVelocityControl;
    bool enabled_ = false;

    OutputPort<float> angle_output_port_{0.0f};
    OutputPort<float> velocity_output_port_{0.0f};
    OutputPort<float> torque_output_port_{0.0f};
    OutputPort<float> current_output_port_{0.0f};
    OutputPort<float> torque_cmd_output_port_{0.0f};

    InputPort<float> velocity_cmd_input_port_;
    InputPort<float> torque_ff_cmd_input_port_;
};

class MotorDMH3510 : public WheelMotorBase {
public:
    enum State : uint8_t {
        kStateMotorDisable = 0x0,
        kStateMotorEnable = 0x1,
        kStateOverVolt = 0x8,
        kStateUnderVolt = 0x9,
        kStateOverCurr = 0xA,
        kStateMosOverTemp = 0xB,
        kStateCoilOverTemp = 0xC,
        kStateCommLoss = 0xD,
        kStateOverload = 0xE,
    };

    enum Register : uint8_t {
        kRegACC = 0x04,
        kRegDEC = 0x05,
        kRegCTRL_MODE = 0x0A,
        kRegPMAX = 0x15,
        kRegVMAX = 0x16,
        kRegTMAX = 0x17,
        kRegKP_ASR = 0x19, // Kp for ASR (active speed regulation, i.e. velocity control)
        kRegKI_ASR = 0x1A, // Ki for ASR
    };

    struct Parameter_t {
        float acc = 10.0f;
        float dec = -10.0f;
        float kp_asr = 0.0f;
        float ki_asr = 0.0f;
        float pmax = 3.1415926535f;
        float vmax = 280.0f;
        float tmax = 1.0f;
    };

    explicit MotorDMH3510(const Config_t& config);

    void pack_velocity_data(float velocity, uint8_t* tx_data) override;
    void pack_mit_data(float position, float velocity, float kp, float kd, float torque_ff, uint8_t* tx_data) override;
    void parse_feedback_data(const uint8_t rx_data[8]) override;

    void build_set_mode_msg(Mode mode, can_Message_t& msg) override;
    void build_enable_msg(can_Message_t& msg) override;
    void build_disable_msg(can_Message_t& msg) override;
    void build_clear_error_msg(can_Message_t& msg) override;
    void build_save_zero_msg(can_Message_t& msg) override;
    void build_set_velocity_kp_msg(float kp, can_Message_t& msg);
    void build_set_velocity_ki_msg(float ki, can_Message_t& msg);
    void build_set_acc_msg(float acc, can_Message_t& msg);
    void build_set_dec_msg(float dec, can_Message_t& msg);
    void build_set_pmax_msg(float pmax, can_Message_t& msg);
    void build_set_vmax_msg(float vmax, can_Message_t& msg);
    void build_set_tmax_msg(float tmax, can_Message_t& msg);

    bool is_writing_register() { return writing_register_; }
    void reset_wheel_speed_pid() override;

    void build_write_register_msg(uint8_t rid, uint32_t data, can_Message_t& msg);

    uint32_t command_can_id() const override;

    Parameter_t parameter_;

private:
    State state_ = kStateMotorDisable;
    State last_error_ = kStateMotorDisable;
    bool writing_register_ = false;
    PID wheel_speed_pid_;
    float wheel_speed_pid_kp_alpha_ = 0.0f;
    bool wheel_speed_pid_prev_enabled_ = false;

    void update_wheel_speed_pll_gains();
    void update_wheel_speed_pid_kp_ramp();
    void reset_wheel_speed_pll(float measured_pos_rad, bool mark_prev_enabled);
    void update_wheel_speed_pll_from_pos(float measured_pos_rad, bool enabled_now);
    static float wrap_pm_pi(float angle_rad);

    float pll_kp_ = 0.0f;
    float pll_ki_ = 0.0f;
    float pll_pos_est_rad_ = 0.0f;
    float pll_vel_est_rad_s_ = 0.0f;
    float pll_vel_ramp_alpha_ = 0.0f;
    bool pll_initialized_ = false;
    bool pll_prev_enabled_ = false;
    bool pll_gain_unstable_ = false;

    // 3-state Luenberger observer [θ, ω, T_dist]
    float obs_theta_rad_ = 0.0f;
    float obs_omega_rad_s_ = 0.0f;
    float obs_t_dist_nm_ = 0.0f;
    bool obs_initialized_ = false;
    float last_torque_cmd_nm_ = 0.0f;

    // Velocity filter for damping path
    ButterworthLowPass2 obs_vel_filter_{{0.0f, 0.0f}};

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

extern const WheelMotorBase::Info_t motor_info_DMH3510;

#endif // __WHEEL_MOTOR_HPP
