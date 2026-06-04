#ifndef __YAW_S_CURVE_HPP
#define __YAW_S_CURVE_HPP

#include <cmath>

/// S-curve (jerk-limited) trajectory planner for yaw angle.
/// Computes a 7-phase plan on target change, samples position/velocity each cycle.
/// Handles cyclic angle wrapping — plans the shortest path.
class YawSCurve {
public:
    struct Config {
        float vmax;  // max angular velocity  (rad/s)
        float amax;  // max angular acceleration (rad/s²)
        float jmax;  // max angular jerk (rad/s³)
    };

    explicit YawSCurve(const Config& cfg) : cfg_(cfg) {}

    void set_config(const Config& cfg) { cfg_ = cfg; }

    /// Call when target changes. current_rad = measured yaw (unwrapped, rad).
    /// current_vel = measured angular velocity (rad/s).
    void set_target(float target_rad, float current_rad, float current_vel) {
        const float dq = wrap_pm_pi(target_rad - current_rad);
        if (fabsf(dq) < 1e-9f) {
            active_ = false;
            return;
        }

        sign_ = (dq >= 0.0f) ? 1.0f : -1.0f;
        const float q0 = 0.0f;
        const float q1 = sign_ * dq;
        start_pos_ = current_rad;
        start_vel_ = sign_ * current_vel;  // signed for positive-direction plan
        compute_plan(q0, q1, start_vel_);
        elapsed_ = 0.0f;
        active_ = (total_time_ > 1e-9f);
    }

    void step(float dt_s) {
        if (!active_) return;
        elapsed_ += dt_s;
        if (elapsed_ >= total_time_) {
            elapsed_ = total_time_;
            active_ = false;
        }
    }

    float position() const {
        if (!active_) return start_pos_ + sign_ * q1_;
        return start_pos_ + sign_ * sample_pos(elapsed_);
    }

    float velocity() const {
        if (!active_) return 0.0f;
        return sign_ * sample_vel(elapsed_);
    }

    bool active() const { return active_; }

private:
    Config cfg_;
    bool active_ = false;
    float sign_ = 1.0f;
    float start_pos_ = 0.0f;
    float start_vel_ = 0.0f;
    float elapsed_ = 0.0f;
    float total_time_ = 0.0f;

    float q1_ = 0.0f;
    float Tj1_ = 0.0f, Ta_ = 0.0f, Tv_ = 0.0f, Td_ = 0.0f, Tj2_ = 0.0f;
    float vlim_ = 0.0f, alima_ = 0.0f, alimd_ = 0.0f;
    float j_max_ = 0.0f;

    static float wrap_pm_pi(float x) {
        while (x > 3.1415926535f)  x -= 6.283185307f;
        while (x < -3.1415926535f) x += 6.283185307f;
        return x;
    }

    void compute_plan(float q0, float q1, float v0) {
        q1_ = q1;
        const float vmax = cfg_.vmax;
        float amax = cfg_.amax;
        const float jmax = cfg_.jmax;
        const float v1 = 0.0f;
        j_max_ = jmax;

        if (try_max_speed(q0, q1, v0, v1, vmax, amax, jmax)) return;
        if (try_full_amax(q0, q1, v0, v1, amax, jmax)) return;

        for (int i = 0; i < 200 && amax > 1e-8f; ++i) {
            amax *= 0.99f;
            if (try_full_amax(q0, q1, v0, v1, amax, jmax)) return;
        }
        fallback(q0, q1, amax);
    }

    bool try_max_speed(float q0, float q1, float v0, float v1,
                       float vmax, float amax, float jmax) {
        float Tj1, Ta_acc, a_lima;
        if ((vmax - v0) * jmax < amax * amax) {
            Tj1 = sqrtf(fmaxf((vmax - v0) / jmax, 0.0f));
            Ta_acc = 2.0f * Tj1;
            a_lima = jmax * Tj1;
        } else {
            Tj1 = amax / jmax;
            Ta_acc = Tj1 + (vmax - v0) / amax;
            a_lima = amax;
        }
        float Tj2, Td_dec, a_limd;
        if ((vmax - v1) * jmax < amax * amax) {
            Tj2 = sqrtf(fmaxf((vmax - v1) / jmax, 0.0f));
            Td_dec = 2.0f * Tj2;
            a_limd = -jmax * Tj2;
        } else {
            Tj2 = amax / jmax;
            Td_dec = Tj2 + (vmax - v1) / amax;
            a_limd = -amax;
        }
        float Tv;
        if (fabsf(vmax) < 1e-12f) {
            Tv = -1.0f;
        } else {
            Tv = (q1 - q0) / vmax
               - (Ta_acc / 2.0f) * (1.0f + v0 / vmax)
               - (Td_dec / 2.0f) * (1.0f + v1 / vmax);
        }
        if (Tv > 0.0f) {
            Tj1_ = Tj1; Tj2_ = Tj2;
            Ta_ = Ta_acc; Tv_ = Tv; Td_ = Td_dec;
            vlim_ = vmax; alima_ = a_lima; alimd_ = a_limd;
            total_time_ = Ta_ + Tv_ + Td_;
            return true;
        }
        return false;
    }

    bool try_full_amax(float q0, float q1, float v0, float v1,
                       float amax, float jmax) {
        const float Tj = amax / jmax;
        const float delta = (amax * amax * amax * amax) / (jmax * jmax)
                          + 2.0f * (v0 * v0 + v1 * v1)
                          + amax * (4.0f * (q1 - q0) - 2.0f * (amax / jmax) * (v0 + v1));
        if (delta < 0.0f) return false;
        const float sqrt_d = sqrtf(delta);
        float Ta = ((amax * amax / jmax) - 2.0f * v0 + sqrt_d) / (2.0f * amax);
        float Td = ((amax * amax / jmax) - 2.0f * v1 + sqrt_d) / (2.0f * amax);
        if (Ta < 0.0f || Td < 0.0f) return false;
        if (Ta < 2.0f * Tj || Td < 2.0f * Tj) return false;
        Tj1_ = Tj2_ = Tj;
        Ta_ = Ta; Tv_ = 0.0f; Td_ = Td;
        alima_ = amax; alimd_ = -amax;
        vlim_ = v0 + alima_ * (Ta - Tj);
        total_time_ = Ta_ + Tv_ + Td_;
        return true;
    }

    void fallback(float q0, float q1, float amax) {
        const float t_total = 2.0f * sqrtf(fabsf(q1 - q0) / (0.5f * amax));
        Tj1_ = Tj2_ = 0.0f;
        Ta_ = Td_ = t_total * 0.5f;
        Tv_ = 0.0f;
        alima_ = amax; alimd_ = -amax;
        vlim_ = 0.5f * amax * Ta_;
        total_time_ = t_total;
    }

    float sample_pos(float t) const {
        const float T = total_time_;
        if (t <= 0.0f) return 0.0f;
        const float q_acc_end = (vlim_ + start_vel_) * Ta_ / 2.0f;
        const float q_dec_start = q1_ - vlim_ * Td_ / 2.0f;  // v1=0
        if (t < Tj1_) {
            return start_vel_ * t + j_max_ * (t * t * t) / 6.0f;
        } else if (t < Ta_ - Tj1_) {
            return start_vel_ * t + (alima_ / 6.0f) * (3.0f * t * t - 3.0f * Tj1_ * t + Tj1_ * Tj1_);
        } else if (t < Ta_) {
            const float tr = Ta_ - t;
            return q_acc_end - vlim_ * tr + j_max_ * (tr * tr * tr) / 6.0f;
        } else if (t <= Ta_ + Tv_) {
            return q_acc_end + vlim_ * (t - Ta_);
        } else {
            const float tt = t - Ta_ - Tv_;
            if (tt < Tj2_) {
                return q_dec_start + vlim_ * tt - j_max_ * (tt * tt * tt) / 6.0f;
            } else if (tt < Td_ - Tj2_) {
                return q_dec_start + vlim_ * tt
                     + (alimd_ / 6.0f) * (3.0f * tt * tt - 3.0f * Tj2_ * tt + Tj2_ * Tj2_);
            } else if (t <= T) {
                const float tr = T - t;
                return q1_ - j_max_ * (tr * tr * tr) / 6.0f;
            }
        }
        return q1_;
    }

    float sample_vel(float t) const {
        const float T = total_time_;
        if (t <= 0.0f) return start_vel_;
        if (t < Tj1_) {
            return start_vel_ + j_max_ * t * t / 2.0f;
        } else if (t < Ta_ - Tj1_) {
            return start_vel_ + alima_ * (t - Tj1_ / 2.0f);
        } else if (t < Ta_) {
            const float tr = Ta_ - t;
            return vlim_ - j_max_ * (tr * tr) / 2.0f;
        } else if (t < Ta_ + Tv_) {
            return vlim_;
        } else {
            const float tt = t - Ta_ - Tv_;
            if (tt < Tj2_) {
                return vlim_ - j_max_ * (tt * tt) / 2.0f;
            } else if (tt < Td_ - Tj2_) {
                return vlim_ + alimd_ * (tt - Tj2_ / 2.0f);
            } else if (t <= T) {
                const float tr = T - t;
                return j_max_ * (tr * tr) / 2.0f;
            }
        }
        return 0.0f;
    }
};

#endif // __YAW_S_CURVE_HPP
