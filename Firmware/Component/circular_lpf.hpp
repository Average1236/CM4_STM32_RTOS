#ifndef __CIRCULAR_LPF_HPP
#define __CIRCULAR_LPF_HPP

#include "Task/utils.hpp"  // for wrap_to_pi, kPi

class CircularLowPassFilter {
public:
    void reset(float wrapped_init) { state_ = wrapped_init; }

    float step(float target, float alpha) {
        const float err = wrap_to_pi(target - state_);
        state_ = wrap_to_pi(state_ + alpha * err);
        return state_;
    }

    float state() const { return state_; }

private:
    float state_ = 0.0f;
};

#endif // __CIRCULAR_LPF_HPP
