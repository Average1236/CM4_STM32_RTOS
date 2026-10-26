#ifndef OPT_FLOW_HPP
#define OPT_FLOW_HPP
#include <cstdint>
class PositionPLL {
public:
    PositionPLL(float bandwidth_hz, float zero_snap_threshold, float max_jump_mm,
                uint8_t jump_confirm_frames);
    void update(float measured_pos_mm, float dt_s);
    void reset(float measured_pos_mm);
    float vel() const { return vel_est_mm_s_; }
    float pos() const { return pos_est_mm_; }
    bool  initialized() const { return initialized_; }

private:
    float pos_est_mm_;
    float vel_est_mm_s_;
    float kp_;
    float ki_;
    float zero_snap_threshold_;
    float max_jump_mm_;
    uint8_t jump_confirm_frames_;
    uint8_t jump_count_;
    bool  initialized_;
};


class OptFlow {
public:
    struct Data_t {
        float left_x, left_y, right_x, right_y;
        unsigned int tick_ms, valid_mask, left_tick_ms, right_tick_ms;
    };
    struct State_t {
        float body_vx = 0, body_vy = 0; // mm/s
        unsigned int sample_time_ms = 0;
        bool valid = false;
    };
    bool process(const Data_t& data);
    const State_t& get_state() const { return state_; }
private:
    struct Sensor {
        PositionPLL x{300.f, 1.f, 100.f, 3}, y{300.f, 1.f, 100.f, 3};
        std::uint32_t tick = 0;
        bool initialized = false;
    } sensors_[2];
    State_t state_;
};
#endif
