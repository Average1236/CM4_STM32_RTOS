你是一个嵌入式 C++ 固件修改助手。请在当前仓库中完成 yaw 角度规划逻辑改造，目标是解决“小幅快速变化的目标角度在高频重规划下导致 yaw 参考角停滞、角速度抖振、频繁刹停”的问题。

## 背景

当前代码中，`use_imu` 模式下 `SpiRx.vel[2]` 实际被当作目标 yaw 角度使用，而不是角速度。通信协议暂时不能修改，因此仍然保持：

```cpp
robot_vel[2] = SpiRx.vel[2] / 100.0f;
```

并在 `use_imu` 模式下把它解释为目标 yaw 角。

当前 `YawSCurve` 是基于固定终点的梯形/类 S 曲线角度规划器。每次目标角度变化超过 `kYawTargetReplanDeadbandRad` 时，会调用 `yaw_s_curve_.set_target(...)`，这会重建一段“到目标角并停车”的轨迹。对于高频小幅变化的目标角，这会导致：

1. 小目标变化被 deadband 吞掉；
2. 规划器频繁重置；
3. 参考角速度反复趋近 0；
4. 增大 yaw 加速度虽然能缓解滞后，但会造成速度不稳定和抖振；
5. 从实测角速度重新规划会因为机器人响应延迟进一步恶化问题。

## 总体要求

不要修改通信协议。
不要引入 Ruckig、Reflexxes 等复杂外部规划库。
优先复用仓库中已有的 PLL 或 TD 实现来估计目标 yaw 角速度。
如果现有 PLL/TD 接口不适合直接复用，可以在 `YawSCurve` 内部实现一个轻量二阶 PLL 作为 fallback。
尽量保持现有类名 `YawSCurve` 和外部调用接口兼容，减少对其他模块的影响。
不要再把目标角变化当作“重新规划一条到终点停车的轨迹”。
目标变化时只能更新目标测量值，不能重置 yaw 参考状态。
`yaw_ref / yaw_vel_ref / yaw_acc_ref` 必须在控制周期之间连续滚动。

## 需要重点修改的文件

请重点检查并修改以下文件：

```text
Firmware/Component/yaw_s_curve.hpp
Firmware/Component/robot.cpp
Firmware/Component/robot.hpp
Firmware/Component/control_params.hpp
```

如果实际路径略有不同，请在仓库中搜索 `YawSCurve`、`yaw_s_curve_`、`kYawTargetReplanDeadbandRad`、`prepare_yaw_control`、`pi_decode_spi` 来定位。

## 具体改造方案

### 1. 重写或替换 `YawSCurve` 内部实现

保留类名 `YawSCurve`，但将其从“梯形终点停车规划器”改为“增量式 yaw 参考跟踪器”。

内部至少维护以下状态：

```cpp
float q_ref_;             // 连续 yaw 参考角，unwrapped
float v_ref_;             // 连续 yaw 参考角速度
float a_ref_;             // 连续 yaw 参考角加速度

float target_meas_wrapped_; // 最新收到的目标 yaw 角，wrapped 到 [-pi, pi]
float target_unwrap_;       // 展开后的目标 yaw 角

float target_pos_est_;      // PLL/TD 估计后的目标角
float target_vel_est_;      // PLL/TD 估计后的目标角速度
```

必须提供以下方法或等效接口：

```cpp
void set_config(const Config& cfg);
void reset(float current_yaw_unwrapped, float current_omega, float target_wrapped);
void set_target_measurement(float target_wrapped);
void step(float dt_s);

float position() const;
float velocity() const;
float acceleration() const;
```

`step(dt_s)` 内部流程应为：

```text
1. unwrap 最新目标角；
2. 使用现有 PLL/TD 或轻量 PLL 估计 target_pos_est_ 和 target_vel_est_；
3. 用 target_pos_est_、target_vel_est_ 生成连续 yaw 参考；
4. 对 v_ref_ 限速度；
5. 对 a_ref_ 限加速度；
6. 对 a_ref_ 的变化量限 jerk；
7. 积分得到 q_ref_。
```

禁止在每次目标变化时重置 `q_ref_ / v_ref_ / a_ref_`。

### 2. 目标角 unwrap 逻辑

目标角来自 `[-pi, pi]`，必须先做连续展开：

```cpp
target_unwrap_ = target_unwrap_
               + wrap_pm_pi(target_meas_wrapped_ - wrap_pm_pi(target_unwrap_));
```

不要把 wrapped 角度直接送入 TD/PLL，否则跨越 `pi/-pi` 时会产生速度尖峰。

### 3. 目标角速度估计

优先在仓库中搜索是否已有可用的 TD 或 PLL，例如：

```text
TD
TrackingDifferentiator
PLL
PhaseLockedLoop
Pll
```

如果已有实现能直接用于目标角平滑和速度估计，则复用它。

如果没有合适接口，则在 `YawSCurve` 内部实现轻量二阶 PLL：

```cpp
void update_target_estimator(float dt_s) {
    const float target_unwrapped =
        target_unwrap_ + wrap_pm_pi(target_meas_wrapped_ - wrap_pm_pi(target_unwrap_));

    target_unwrap_ = target_unwrapped;

    const float bw = fmaxf(cfg_.target_estimator_bandwidth_hz, 1.0f);
    const float wn = 2.0f * kPi * bw;
    const float zeta = 0.707f;

    const float kp = 2.0f * zeta * wn;
    const float ki = wn * wn;

    const float e = target_unwrap_ - target_pos_est_;

    target_vel_est_ += ki * e * dt_s;
    target_vel_est_ = std::clamp(target_vel_est_, -fabsf(cfg_.vmax), fabsf(cfg_.vmax));

    target_pos_est_ += (target_vel_est_ + kp * e) * dt_s;
}
```

如果使用 TD，则必须保证 TD 输入的是 unwrapped 目标角，输出的速度估计需要 clamp 到 `[-vmax, vmax]`。

### 4. 增量式参考生成逻辑

`YawSCurve::step()` 中，在目标状态估计之后，执行类似以下逻辑：

```cpp
const float vmax = fabsf(cfg_.vmax);
const float amax = fabsf(cfg_.amax);
const float jmax = fabsf(cfg_.jmax);

const float target_pred =
    target_pos_est_ + cfg_.preview_time_sec * target_vel_est_;

const float e = target_pred - q_ref_;

float v_des = target_vel_est_ + cfg_.track_kp * e;
v_des = std::clamp(v_des, -vmax, vmax);

// 只有目标角速度接近 0 时才启用静止目标刹停包络。
// 目标正在移动时，不要强行把 v_des 压向 0。
if (fabsf(target_vel_est_) < cfg_.target_vel_zero_eps) {
    const float v_stop = sqrtf(fmaxf(0.0f, 2.0f * amax * fabsf(e)));
    v_des = std::clamp(v_des, -v_stop, v_stop);
}

float a_des = (v_des - v_ref_) / dt_s;
a_des = std::clamp(a_des, -amax, amax);

const float da_max = jmax * dt_s;
a_ref_ += std::clamp(a_des - a_ref_, -da_max, da_max);
a_ref_ = std::clamp(a_ref_, -amax, amax);

v_ref_ += a_ref_ * dt_s;
v_ref_ = std::clamp(v_ref_, -vmax, vmax);

q_ref_ += v_ref_ * dt_s;
```

在静止目标附近可以做吸附，但必须满足目标速度很小：

```cpp
if (fabsf(target_vel_est_) < cfg_.target_vel_zero_eps &&
    fabsf(target_pos_est_ - q_ref_) < cfg_.stop_band_rad &&
    fabsf(v_ref_) < sqrtf(2.0f * amax * cfg_.stop_band_rad)) {
    q_ref_ = target_pos_est_;
    v_ref_ = 0.0f;
    a_ref_ = 0.0f;
}
```

### 5. Config 参数

将 `YawSCurve::Config` 改为或扩展为：

```cpp
struct Config {
    float vmax;                          // rad/s
    float amax;                          // rad/s^2
    float jmax;                          // rad/s^3

    float target_estimator_bandwidth_hz; // TD/PLL 目标速度估计带宽
    float track_kp;                      // 角度误差到角速度的比例
    float preview_time_sec;              // 目标速度前瞻时间
    float stop_band_rad;                 // 静止目标附近吸附带
    float target_vel_zero_eps;           // 判断目标是否静止的速度阈值
};
```

在 `control_params.hpp` 中添加建议初值：

```cpp
inline constexpr float kYawTargetEstimatorBandwidthHz = 20.0f;
inline constexpr float kYawTargetTrackKp = 12.0f;
inline constexpr float kYawTargetPreviewTimeSec = 0.03f;
inline constexpr float kYawTargetStopBandRad = 0.003f;
inline constexpr float kYawTargetVelZeroEpsRadS = 0.02f;
```

`kYawTargetReplanDeadbandRad` 不再用于 yaw 目标重规划。可以删除，也可以保留但不再使用。

### 6. 修改 `robot.cpp` 中 yaw 目标处理

在 `pi_decode_spi()` 中，删除或停用如下逻辑：

```cpp
if (!yaw_target_initialized
    || fabsf(wrap_to_pi(new_target - yaw_target_rad)) > control_config::kYawTargetReplanDeadbandRad) {
    ...
    yaw_s_curve_.set_target(...);
}
```

改成仅更新目标角测量：

```cpp
if (use_imu) {
    yaw_target_rad = wrap_to_pi(robot_vel[2]);
} else {
    yaw_target_initialized = false;
}
```

不要在 `pi_decode_spi()` 中调用 `yaw_s_curve_.set_target()`。

### 7. 修改 `prepare_yaw_control(float dt_s)`

将 yaw 参考生成集中放到 `prepare_yaw_control(dt_s)` 中，每个控制周期执行：

```cpp
void Robot::prepare_yaw_control(float dt_s) {
    if (!use_imu) {
        yaw_target_initialized = false;
        return;
    }

    if (dt_s <= 1e-6f) {
        return;
    }

    yaw_s_curve_.set_config({
        yaw_max_vel,
        yaw_max_acc,
        yaw_max_jerk,
        control_config::kYawTargetEstimatorBandwidthHz,
        control_config::kYawTargetTrackKp,
        control_config::kYawTargetPreviewTimeSec,
        control_config::kYawTargetStopBandRad,
        control_config::kYawTargetVelZeroEpsRadS
    });

    if (!yaw_target_initialized) {
        const auto yaw = chassis_estimator.chassis_yaw_output_port()->any();
        const auto omega_z = chassis_estimator.chassis_omega_z_output_port()->any();

        const float init_yaw = yaw.has_value() ? *yaw : 0.0f;
        const float init_omega = omega_z.has_value() ? *omega_z : robot_real_vel[2];

        yaw_s_curve_.reset(init_yaw, init_omega, yaw_target_rad);
        yaw_target_initialized = true;
    }

    yaw_s_curve_.set_target_measurement(yaw_target_rad);
    yaw_s_curve_.step(dt_s);

    robot_real_vel[2] = yaw_s_curve_.velocity();
    robot_acc[2] = yaw_s_curve_.acceleration();
}
```

注意：初始化时可以使用实测 yaw 和实测 omega；正常运行中不要因为目标角变化而从实测 omega 重新规划。

### 8. 保持控制器接口兼容

`update_torque_feedforward()` 中原本可能有：

```cpp
if (use_imu) {
    chassis_controller.set_yaw_target(yaw_s_curve_.position(), yaw_s_curve_.velocity());
}
```

保持该逻辑，但此时 `yaw_s_curve_.position()` 和 `yaw_s_curve_.velocity()` 来自增量式参考生成器。

### 9. Debug 建议

如果已有调试变量，请增加或复用以下变量，便于观察：

```cpp
volatile float yaw_target_debug;
volatile float yaw_target_vel_est_debug;
volatile float yaw_ref_debug;
volatile float yaw_ref_vel_debug;
volatile float yaw_ref_acc_debug;
```

在 `prepare_yaw_control()` 后更新这些变量。

### 10. 验证标准

完成修改后，请执行以下检查：

1. 全仓库编译通过，无新增 warning。
2. 搜索确认 `kYawTargetReplanDeadbandRad` 不再参与 yaw 目标重规划。
3. 搜索确认 `YawSCurve::set_target()` 不再被 `robot.cpp` 中 yaw 控制逻辑调用；如果保留旧接口，不能用于当前 yaw 控制主路径。
4. 高频小幅变化目标角输入时：

   * `yaw_ref` 应连续变化；
   * `yaw_vel_ref` 不应反复掉到 0；
   * `yaw_acc_ref` 不应突变；
   * 电机不应出现周期性急刹或明显抖振。
5. 静止目标角输入时：

   * yaw 能稳定收敛；
   * 目标附近不会长期振荡；
   * 最终 `yaw_vel_ref` 能回到 0。

## 重要约束

不要修改 SPI 数据结构。
不要改变上位机发送字段含义。
不要把 `robot_vel[2]` 改成必须由上位机发送目标角速度。
不要引入外部大型轨迹库。
不要在每次目标角变化时 reset 规划器。
不要从实测角速度持续重规划，只允许初始化、IMU 失效恢复、控制模式切换时 reset。
必须保证 yaw 参考状态在正常运行中连续滚动。
