#ifndef __CHASSIS_CONTROLLER_HPP
#define __CHASSIS_CONTROLLER_HPP

#include "component.hpp"
#include <array>
#include <cstddef>

class MixedLesoChassisController {
public:
    MixedLesoChassisController();

    InputPort<float>* chassis_vx_input_port() { return &chassis_vx_input_port_; }
    InputPort<float>* chassis_vy_input_port() { return &chassis_vy_input_port_; }
    InputPort<float>* chassis_omega_z_input_port() { return &chassis_omega_z_input_port_; }

    OutputPort<float>* wheel_torque_ff_output_port(std::size_t index) {
        return (index < wheel_torque_ff_output_ports_.size()) ? &wheel_torque_ff_output_ports_[index] : nullptr;
    }

    InputPort<float>* wheel_sent_torque_input_port(std::size_t index) {
        return (index < wheel_sent_torque_input_ports_.size()) ? &wheel_sent_torque_input_ports_[index] : nullptr;
    }

    void set_reference(const float vel_ref[3], const float acc_ref[3], float yaw_ref_rel_rad);
    void step(float dt_s);
    void reset();

private:
    bool inverse3x3(const float in[3][3], float out[3][3]) const;
    void precompute_mappings();

    InputPort<float> chassis_vx_input_port_;
    InputPort<float> chassis_vy_input_port_;
    InputPort<float> chassis_omega_z_input_port_;

    std::array<OutputPort<float>, 4> wheel_torque_ff_output_ports_ = {
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
    };

    float vel_ref_[3] = {0.0f, 0.0f, 0.0f};
    float acc_ref_[3] = {0.0f, 0.0f, 0.0f};

    // 2nd-order LESO states for vx/vy/wz: [z1=velocity, z2=disturbance]
    float vel_obs_[3][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};

    float j1_[3][4] = {{0.0f}};
    float j1_pinv_[4][3] = {{0.0f}};

    std::array<InputPort<float>, 4> wheel_sent_torque_input_ports_ = {};

    float last_chassis_vx_m_s_ = 0.0f;
    float last_chassis_vy_m_s_ = 0.0f;
    float last_chassis_omega_z_rad_s_ = 0.0f;
};

#endif // __CHASSIS_CONTROLLER_HPP
