#ifndef __UTILS_HPP
#define __UTILS_HPP

#include <cmath>


#define PI (3.1415926f)

constexpr float kPi = 3.1415926535f;

inline float wrap_to_pi(float angle_rad) {
    while (angle_rad > kPi) {
        angle_rad -= 2.0f * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0f * kPi;
    }
    return angle_rad;
}

template <typename T>
T limit(T data_t, T limit_t)
{
    if (limit_t < 0)
    {
        limit_t = -limit_t;
    }

    if (data_t > limit_t)
    {
        return limit_t;
    }
    else if (data_t < -limit_t)
    {
        return -limit_t;
    }
    else
    {
        return data_t;
    }
}

template <typename T>
T normalize(T data_t, T cycle_low, T cycle_high)
{
    if (cycle_low > cycle_high)
    {
        float t = cycle_low;
        cycle_low = cycle_high;
        cycle_high = t;
    }

    float cycle = cycle_high - cycle_low;

    while (data_t > cycle_high)
    {
        data_t -= cycle;
    }

    while (data_t < cycle_low)
    {
        data_t += cycle;
    }

    return data_t;
}

class PID {
public:
    struct Parameter_t
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float output_limit = 0.0f;
        float integ_limit = 0.0f;
        float dt = 0.0f;
        float back_calc_gain = 0.0f;
        float diff_cutoff_hz = 0.0f;  // 0 disables the derivative low-pass
    };

    PID() = default;
    explicit PID(const Parameter_t parameter) : parameter_(parameter) {};
    ~PID() = default;

    float calc(float ref, float fdb) {
        return calc(ref, fdb, parameter_);
    }

    float calc(float ref, float fdb, const Parameter_t& parameter) {
        constexpr float kEps = 1e-6f;
        parameter_ = parameter;

        last_err = err;
        err = ref - fdb;

        const float dt = (parameter.dt > kEps) ? parameter.dt : kEps;
        const float raw_diff = (err - last_err) / dt;

        if (parameter.diff_cutoff_hz > kEps) {
            constexpr float kTwoPi = 6.283185307f;
            const float tf = 1.0f / (kTwoPi * parameter.diff_cutoff_hz);
            diff_filt_ = diff_filt_ + (raw_diff - diff_filt_) * dt / (dt + tf);
        } else {
            diff_filt_ = raw_diff;
        }
        diff = diff_filt_;

        if (std::fabs(parameter.ki) > kEps) {
            const float integ_abs_limit = parameter.integ_limit / std::fabs(parameter.ki);
            const float integ_pre = limit<float>(integ + err * dt, integ_abs_limit);
            const float output_pre = parameter.kp * err + parameter.ki * integ_pre + parameter.kd * diff;
            const float output_sat = limit<float>(output_pre, parameter.output_limit);

            // Anti-windup by back-calculation: feed saturation residual back to integrator.
            integ = integ_pre + parameter.back_calc_gain * (output_sat - output_pre) * dt / parameter.ki;
            integ = limit<float>(integ, integ_abs_limit);
            output = output_sat;
        } else {
            output = limit<float>(parameter.kp * err + parameter.kd * diff, parameter.output_limit);
        }

        return output;
    }
    void set_parameter(const Parameter_t parameter) {
        parameter_ = parameter;
    }
    void reset() {
        integ = 0.0f;
        diff = 0.0f;
        diff_filt_ = 0.0f;
        last_err = 0.0f;
        output = 0.0f;
    }

    float get_integ() const {
        return integ;
    }

    float get_diff() const {
        return diff;
    }

private:
    Parameter_t parameter_;
    float err = 0.0f;
    float last_err = 0.0f;
    float integ = 0.0f;
    float diff = 0.0f;
    float diff_filt_ = 0.0f;
    float output = 0.0f;    
};

class ButterworthLowPass2 {
public:
    struct Parameter_t {
        float cutoff_hz;
        float dt;
    };

    explicit ButterworthLowPass2(Parameter_t parameter, float init = 0.0f)
        : parameter_(parameter) {
        compute_coeffs();
        reset(init);
    }

    float filter(float x) {
        if (!enabled_) {
            y_ = x;
            return y_;
        }

        // Transposed direct-form II implementation.
        const float y = b0_ * x + z1_;
        z1_ = b1_ * x - a1_ * y + z2_;
        z2_ = b2_ * x - a2_ * y;
        y_ = y;
        return y_;
    }

    void set_parameter(Parameter_t parameter) {
        parameter_ = parameter;
        compute_coeffs();
        reset(y_);
    }

    void reset(float value = 0.0f) {
        y_ = value;
        if (!enabled_) {
            z1_ = 0.0f;
            z2_ = 0.0f;
            return;
        }

        // Make filter output start at `value` for constant input `value`.
        z1_ = value * (1.0f - b0_);
        z2_ = value * (b2_ - a2_);
    }

    float output() const {
        return y_;
    }

private:
    void compute_coeffs() {
        constexpr float kEps = 1e-6f;
        constexpr float kQ = 0.70710678f; // 2nd-order Butterworth Q
        constexpr float kTwoPi = 6.283185307f;

        const float dt = (parameter_.dt > kEps) ? parameter_.dt : kEps;
        const float fs = 1.0f / dt;
        const float cutoff = parameter_.cutoff_hz;

        if (cutoff <= kEps || cutoff >= 0.49f * fs) {
            enabled_ = false;
            b0_ = 1.0f;
            b1_ = 0.0f;
            b2_ = 0.0f;
            a1_ = 0.0f;
            a2_ = 0.0f;
            return;
        }

        enabled_ = true;
        const float omega = kTwoPi * cutoff / fs;
        const float cosw = std::cos(omega);
        const float sinw = std::sin(omega);
        const float alpha = sinw / (2.0f * kQ);

        const float b0 = (1.0f - cosw) * 0.5f;
        const float b1 = 1.0f - cosw;
        const float b2 = (1.0f - cosw) * 0.5f;
        const float a0 = 1.0f + alpha;
        const float a1 = -2.0f * cosw;
        const float a2 = 1.0f - alpha;

        b0_ = b0 / a0;
        b1_ = b1 / a0;
        b2_ = b2 / a0;
        a1_ = a1 / a0;
        a2_ = a2 / a0;
    }

    Parameter_t parameter_;
    bool enabled_ = false;

    float b0_ = 1.0f;
    float b1_ = 0.0f;
    float b2_ = 0.0f;
    float a1_ = 0.0f;
    float a2_ = 0.0f;

    float z1_ = 0.0f;
    float z2_ = 0.0f;
    float y_ = 0.0f;
};

class TD {
public:
    struct Parameter_t
    {
        float r;
        float h;
        float dt;
        bool is_cycle;
        float cycle_low;
        float cycle_high;
    };

    TD(Parameter_t parameter, float init) : parameter_(parameter) {
        x1 = init;
        x1k = init;
        x2 = 0;
        x2k = 0;
        d = parameter_.r * parameter_.h;
        d0 = d * parameter_.h;
        cycle = parameter_.cycle_high - parameter_.cycle_low;
        data = init;
        diff = 0;
        last_raw_data_ = init;
        raw_data_ = init;
    }
    ~TD() = default;
    void calc(float raw_data) {
        if (parameter_.is_cycle) {
            float delta_raw_data = raw_data - last_raw_data_;
            if (delta_raw_data > cycle / 2.0) {
                raw_data_ += delta_raw_data - cycle;
            }
            else if (delta_raw_data < -cycle / 2.0){
                raw_data_ += delta_raw_data + cycle;
            }
            else {
                raw_data_ += delta_raw_data;
            }
        }
        else {
            raw_data_ = raw_data;
        }

        x1k = x1;
        x2k = x2;

        float tdy = x1k - raw_data_ + parameter_.h * x2k;
        float a0 = sqrt(d * d + 8 * parameter_.r * abs(tdy));

        float a = 0;

        if (abs(tdy) <= d0) {
            a = x2k + tdy / parameter_.h;
        }
        else {
            if (tdy > 0) {
                a = x2k + 0.5 * (a0 - d);
            }
            else {
                a = x2k - 0.5 * (a0 - d);
            }
        }

        float f = 0;
        if (abs(a) <= d) {
            f = -parameter_.r * a / d;
        }
        else {
            if (a > 0) {
                f = -parameter_.r;
            }
            else {
                f = parameter_.r;
            }
        }

        x1 = x1k + parameter_.dt * x2k;
        x2 = x2k + parameter_.dt * f;

        if (parameter_.is_cycle) {
            data = normalize<float>(x1, parameter_.cycle_low, parameter_.cycle_high);
        }
        else {
            data = x1;
        }

        diff = x2;
        last_raw_data_ = raw_data;
    }
    float get_data() const { return data; };
    float get_diff() const { return diff; };

private:
    Parameter_t parameter_;
    float x1;
    float x2;
    float x1k;
    float x2k;
    float d;
    float d0;
    float data;
    float diff;
    float last_raw_data_;
    float raw_data_;
    float cycle;
};

#endif // __UTILS_HPP
