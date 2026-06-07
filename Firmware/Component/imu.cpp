#include "imu.hpp"
#include "control_params.hpp"

#include <cstring>

// Debug
volatile float imu_acc_x_debug = 0.0f;
volatile float imu_acc_y_debug = 0.0f;
volatile float imu_acc_z_debug = 0.0f;
volatile float imu_gyro_x_debug = 0.0f;
volatile float imu_gyro_y_debug = 0.0f;
volatile float imu_gyro_z_debug = 0.0f;
volatile float imu_omega_z_filt_debug = 0.0f;
volatile float imu_yaw_debug = 0.0f;

IMU::IMU(
    Model model,
    SPI_HandleTypeDef* hspi,
    GPIO_TypeDef* cs_port,
    uint16_t cs_pin
) : omega_filter_({control_config::kImuOmegaButterworthCutoffHz, 0.00125f}),
    model_(model), hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin) {}

bool IMU::init()
{
    return icm42688_init();
}

void IMU::process_once()
{
    bool updated = false;
    if (icm42688_read_burst(icm_rx_buffer_, IMU_ICM42688_BURST_DATA_LENGTH)) {
        updated = decode_icm42688(icm_rx_buffer_, IMU_ICM42688_BURST_DATA_LENGTH);
    }

    imu_acc_x_debug = data_[kAccX];
    imu_acc_y_debug = data_[kAccY];
    imu_acc_z_debug = data_[kAccZ];
    imu_gyro_x_debug = data_[kOmegaX];
    imu_gyro_y_debug = data_[kOmegaY];
    imu_gyro_z_debug = data_[kOmegaZ];

    if (updated) {
        publish_ports_from_cache();
    }
}

bool IMU::decode_icm42688(const uint8_t* raw_data, size_t len)
{
    if (raw_data == nullptr || len < IMU_ICM42688_BURST_DATA_LENGTH) {
        return false;
    }

    const int16_t ax = static_cast<int16_t>((static_cast<uint16_t>(raw_data[0]) << 8) | raw_data[1]);
    const int16_t ay = static_cast<int16_t>((static_cast<uint16_t>(raw_data[2]) << 8) | raw_data[3]);
    const int16_t az = static_cast<int16_t>((static_cast<uint16_t>(raw_data[4]) << 8) | raw_data[5]);
    const int16_t gx = static_cast<int16_t>((static_cast<uint16_t>(raw_data[6]) << 8) | raw_data[7]);
    const int16_t gy = static_cast<int16_t>((static_cast<uint16_t>(raw_data[8]) << 8) | raw_data[9]);
    const int16_t gz = static_cast<int16_t>((static_cast<uint16_t>(raw_data[10]) << 8) | raw_data[11]);

    data_[kAccX] = static_cast<float>(ax) * acc_sensitivity_;
    data_[kAccY] = static_cast<float>(ay) * acc_sensitivity_;
    data_[kAccZ] = static_cast<float>(az) * acc_sensitivity_;

    data_[kOmegaX] = static_cast<float>(gx) * gyro_sensitivity_;
    data_[kOmegaY] = static_cast<float>(gy) * gyro_sensitivity_;
    data_[kOmegaZ] = static_cast<float>(gz) * gyro_sensitivity_;

    data_[kOmegaZ] -= control_config::kImuOmegaBiasZ;

    return true;
}

bool IMU::icm42688_init()
{
    if (hspi_ == nullptr || cs_port_ == nullptr || cs_pin_ == 0U) {
        return false;
    }

    HAL_Delay(100); // Wait for sensor power up

    icm42688_write_reg(kIcm42688RegBankSel, 0x00);
    icm42688_write_reg(kIcm42688DeviceConfig, 0x01);
    HAL_Delay(100);

    const uint8_t who_am_i = icm42688_read_reg(kIcm42688WhoAmI);
    if (who_am_i != kIcm42688WhoAmIValue) {
        return false;
    }

    const uint8_t accel_cfg = static_cast<uint8_t>((kIcm42688Afs4G << 5) | kIcm42688Aodr1000Hz);
    const uint8_t gyro_cfg = static_cast<uint8_t>((kIcm42688Gfs1000Dps << 5) | kIcm42688Godr1000Hz);
    icm42688_write_reg(kIcm42688AccelConfig0, accel_cfg);
    icm42688_write_reg(kIcm42688GyroConfig0, gyro_cfg);

    uint8_t pwr_mgmt0 = icm42688_read_reg(kIcm42688PwrMgmt0);
    pwr_mgmt0 &= static_cast<uint8_t>(~(1U << 5));
    pwr_mgmt0 |= static_cast<uint8_t>(3U << 2);
    pwr_mgmt0 |= 3U;
    icm42688_write_reg(kIcm42688PwrMgmt0, pwr_mgmt0);
    HAL_Delay(1);

    acc_sensitivity_ = 4.0f * 9.8f / 32768.0f;
    gyro_sensitivity_ = 1000.0f / 32768.0f;
    return true;
}

uint8_t IMU::icm42688_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80U), 0x00};
    uint8_t rx[2] = {0};
    icm42688_cs_low();
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2U, 10U);
    icm42688_cs_high();
    if (status != HAL_OK) {
        return 0U;
    }
    return rx[1];
}

void IMU::icm42688_read_regs(uint8_t reg, uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0U) {
        return;
    }

    uint8_t tx_data[24] = {0};
    uint8_t rx_data[24] = {0};
    tx_data[0] = static_cast<uint8_t>(reg | 0x80U);

    icm42688_cs_low();
    if (HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, len + 1U, 10U) == HAL_OK) {
        std::memcpy(data, rx_data + 1, len);
    }
    icm42688_cs_high();
}

void IMU::icm42688_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};
    icm42688_cs_low();
    (void)HAL_SPI_TransmitReceive(hspi_, tx, rx, 2U, 10U);
    icm42688_cs_high();
}

bool IMU::icm42688_read_burst(uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0U) {
        return false;
    }
    icm42688_read_regs(kIcm42688AccelDataX1, data, len);
    return true;
}

void IMU::icm42688_cs_low()
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
}

void IMU::icm42688_cs_high()
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

void IMU::get_data(float out_data[9]) const
{
    out_data[0] = data_[kAccX];
    out_data[1] = data_[kAccY];
    out_data[2] = data_[kAccZ];
    // correct mapping: omega X/Y/Z
    out_data[3] = data_[kOmegaX];
    out_data[4] = data_[kOmegaY];
    out_data[5] = data_[kOmegaZ];
    
    out_data[6] = data_[kAngleX];
    out_data[7] = data_[kAngleY];
    out_data[8] = data_[kAngleZ];
}

void IMU::update_integrated_yaw(float dt_s) {
    // 使用动态估计的陀螺 Z 偏置（静止窗口 EMA 更新）
    const float omega_z_corrected = data_[kOmegaZ] - imu_stat_.bias_gz_dps;
    const float omega_z_filt = omega_filter_.filter(omega_z_corrected);
    imu_omega_z_filt_debug = omega_z_filt * (3.1415926535f / 180.0f);
    omega_z_port_ = omega_z_filt;
    integrated_yaw_deg_ += dt_s * omega_z_filt;
    integrated_yaw_deg_ = wrap_to_pi(integrated_yaw_deg_ * (3.1415926535f / 180.0f)) * (180.0f / 3.1415926535f);
    data_[kAngleZ] = integrated_yaw_deg_;
    yaw_port_ = integrated_yaw_deg_;
}

void IMU::publish_ports_from_cache() {
    omega_x_port_ = data_[kOmegaX];
    omega_y_port_ = data_[kOmegaY];
    acc_x_port_ = data_[kAccX];
    acc_y_port_ = data_[kAccY];
    acc_z_port_ = data_[kAccZ];
    // omega_z and yaw are published by update_integrated_yaw()
}

void IMU::update_roll_pitch(float dt_s) {
    // 锚定帧计数器（函数级静态，跨调用保持；仅 trust_accel 退出时清零）
    static uint8_t anchor_frame_count = 0;

    // 1. 去偏后陀螺积分（使用内部估计的 bias）
    const float omega_x = data_[kOmegaX] - imu_stat_.bias_gx_dps;
    const float omega_y = data_[kOmegaY] - imu_stat_.bias_gy_dps;
    integrated_roll_deg_  += dt_s * omega_x;
    integrated_pitch_deg_ += dt_s * omega_y;

    // 2. 加速度计锚定（仅在内部 trust_accel=true 时）
    if (imu_stat_.trust_accel) {
        const float alpha = (anchor_frame_count < 3)
            ? 0.5f : control_config::kImuRollPitchAlpha;
        anchor_frame_count++;

        const float gy = data_[kAccY];
        const float gz = data_[kAccZ];
        const float gx = data_[kAccX];

        constexpr float kRadToDeg = 180.0f / 3.1415926535f;
        const float acc_roll  = atan2f(gy, gz) * kRadToDeg;
        const float acc_pitch = atan2f(-gx, sqrtf(gy*gy + gz*gz)) * kRadToDeg;

        integrated_roll_deg_  = alpha * integrated_roll_deg_
                              + (1.0f - alpha) * acc_roll;
        integrated_pitch_deg_ = alpha * integrated_pitch_deg_
                              + (1.0f - alpha) * acc_pitch;
    } else {
        anchor_frame_count = 0;
    }

    data_[kAngleX] = integrated_roll_deg_;
    data_[kAngleY] = integrated_pitch_deg_;
    roll_port_  = integrated_roll_deg_;
    pitch_port_ = integrated_pitch_deg_;
}

void IMU::compute_stationary_and_bias() {
    const float ax = data_[kAccX], ay = data_[kAccY], az = data_[kAccZ];
    const float gx = data_[kOmegaX], gy = data_[kOmegaY], gz = data_[kOmegaZ];

    const float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    const float acc_var  = fabsf(acc_norm - prev_acc_norm_);
    prev_acc_norm_ = acc_norm;

    const bool accel_stable = (acc_var < control_config::kStationaryAccelVarThreshold);
    const bool gyro_still   = (fabsf(gx) < control_config::kStationaryGyroThresholdDegPerS)
                           && (fabsf(gy) < control_config::kStationaryGyroThresholdDegPerS)
                           && (fabsf(gz) < control_config::kStationaryGyroThresholdDegPerS);
    const bool all_still    = accel_stable && gyro_still;

    switch (imu_stat_phase_) {
    case ImuStatPhase::kNormal:
        if (all_still) {
            imu_stat_.confirm_count++;
            if (imu_stat_.confirm_count >= control_config::kStationaryConfirmFrames) {
                imu_stat_phase_ = ImuStatPhase::kCollecting;
                imu_stat_collect_count_ = 0;
                imu_stat_sum_gx_ = 0.0f; imu_stat_sum_gy_ = 0.0f; imu_stat_sum_gz_ = 0.0f;
            }
        } else {
            imu_stat_.confirm_count = 0;
        }
        imu_stat_.trust_accel = false;
        break;

    case ImuStatPhase::kCollecting:
        if (!all_still) {
            imu_stat_phase_ = ImuStatPhase::kNormal;
            imu_stat_.confirm_count = 0;
            imu_stat_.trust_accel = false;
            break;
        }
        imu_stat_sum_gx_ += gx; imu_stat_sum_gy_ += gy; imu_stat_sum_gz_ += gz;
        imu_stat_collect_count_++;
        if (imu_stat_collect_count_ >= control_config::kStationaryWindowFrames) {
            imu_stat_phase_ = ImuStatPhase::kVerifying;
        }
        imu_stat_.trust_accel = true;
        break;

    case ImuStatPhase::kVerifying:
        if (all_still) {
            const float n = static_cast<float>(imu_stat_collect_count_);
            const float mean_gx = imu_stat_sum_gx_ / n;
            const float mean_gy = imu_stat_sum_gy_ / n;
            const float mean_gz = imu_stat_sum_gz_ / n;

            const float alpha = control_config::kImuBiasAlpha;
            imu_stat_.bias_gx_dps = alpha * mean_gx + (1.0f - alpha) * imu_stat_.bias_gx_dps;
            imu_stat_.bias_gy_dps = alpha * mean_gy + (1.0f - alpha) * imu_stat_.bias_gy_dps;
            imu_stat_.bias_gz_dps = alpha * mean_gz + (1.0f - alpha) * imu_stat_.bias_gz_dps;

            // Continue collecting next window
            imu_stat_phase_ = ImuStatPhase::kCollecting;
            imu_stat_collect_count_ = 0;
            imu_stat_sum_gx_ = 0.0f; imu_stat_sum_gy_ = 0.0f; imu_stat_sum_gz_ = 0.0f;
            imu_stat_.trust_accel = true;
        } else {
            imu_stat_phase_ = ImuStatPhase::kNormal;
            imu_stat_.confirm_count = 0;
            imu_stat_.trust_accel = false;
        }
        break;
    }

    if (!all_still && imu_stat_phase_ == ImuStatPhase::kCollecting) {
        imu_stat_phase_ = ImuStatPhase::kNormal;
        imu_stat_.confirm_count = 0;
        imu_stat_.trust_accel = false;
    }
}

void IMU::reset_ports() {
    omega_x_port_.reset();
    omega_y_port_.reset();
    omega_z_port_.reset();
    yaw_port_.reset();
    roll_port_.reset();
    pitch_port_.reset();
    acc_x_port_.reset();
    acc_y_port_.reset();
    acc_z_port_.reset();
}