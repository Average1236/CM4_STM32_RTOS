#include "imu.hpp"
#include "control_params.hpp"

#include "cmsis_os.h"
#include <cstring>

// Debug
volatile float imu_acc_z_debug = 0.0f;
volatile float imu_gyro_z_debug = 0.0f;
volatile float imu_yaw_debug = 0.0f;

const float imu_k[3] = {16.0f * 9.8f / 32768.0f, 2000.0f / 32768.0f, 180.0f / 32768.0f};

IMU::IMU(
    Model model,
    SPI_HandleTypeDef* hspi,
    UART_HandleTypeDef* huart,
    GPIO_TypeDef* cs_port,
    uint16_t cs_pin
) : omega_filter_({control_config::kImuOmegaButterworthCutoffHz, 0.00125f}),
    model_(model), hspi_(hspi), huart_(huart), cs_port_(cs_port), cs_pin_(cs_pin) {}

bool IMU::init()
{
    if (model_ == Model::kIcm42688) {
        return icm42688_init();
    }
    return true;
}

bool IMU::start_acquisition()
{
    if (model_ == Model::kJy931) {
        if (huart_ == nullptr) {
            return false;
        }
        return HAL_UARTEx_ReceiveToIdle_DMA(huart_, uart_rx_buffer_, rx_buffer_len()) == HAL_OK;
    }
    return true;
}

uint32_t IMU::wait_timeout_ms() const
{
    if (model_ == Model::kIcm42688) {
        return 10U;
    }
    return osWaitForever;
}

void IMU::process_once()
{
    bool updated = false;
    if (model_ == Model::kJy931) {
        if (data_ready_) {
            updated = decode_jy931(uart_rx_buffer_, IMU_JY931_RX_DATA_LENGTH);
            data_ready_ = false;
        }
    } else if (model_ == Model::kIcm42688) {
        if (icm42688_read_burst(icm_rx_buffer_, IMU_ICM42688_BURST_DATA_LENGTH)) {
            updated = decode_icm42688(icm_rx_buffer_, IMU_ICM42688_BURST_DATA_LENGTH);
        }
    }

    imu_acc_z_debug = data_[kAccZ];
    imu_gyro_z_debug = data_[kOmegaZ];
    imu_yaw_debug = data_[kAngleZ];

    if (updated) {
        publish_ports_from_cache();
    }
}

bool IMU::decode_jy931(const uint8_t* raw_data, size_t len)
{
    if (raw_data == nullptr || len < 33U) {
        return false;
    }

    bool updated = false;
    for (uint8_t j = 0; j < 33; j++)
    {
        if (raw_data[j] != 0x55)
            continue;

        for (uint8_t i = 0; i < 3; i++)
        {
            if (raw_data[j + 0 + i * 11] == 0x55 && raw_data[j + 1 + i * 11] == (0x51 + i))
            {

                if (sumcrc(&(raw_data[j + 0 + i * 11])))
                {
                    data_[0 + i * 4] = (short)(((short)raw_data[j + 3 + i * 11] << 8) | raw_data[j + 2 + i * 11]) * imu_k[i];
                    data_[1 + i * 4] = (short)(((short)raw_data[j + 5 + i * 11] << 8) | raw_data[j + 4 + i * 11]) * imu_k[i];
                    data_[2 + i * 4] = (short)(((short)raw_data[j + 7 + i * 11] << 8) | raw_data[j + 6 + i * 11]) * imu_k[i];
                    // data_[kVoltage] = (short)(((short)raw_data[9] << 8) | raw_data[8]) / 100.0;
                    updated = true;
                }
            }
        }
    }

    return updated;
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

bool IMU::sumcrc(const uint8_t raw_data[11])
{
    uint16_t sum = 0x0;
    for (size_t i = 0; i < 10; i++)
    {
        sum += raw_data[i];
    }
    uint8_t crc = sum & 0xFF;
    return (crc == raw_data[10]);
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
    const float omega_z_filt = omega_filter_.filter(data_[kOmegaZ]);
    omega_z_port_ = omega_z_filt;
    integrated_yaw_deg_ += dt_s * omega_z_filt;
    integrated_yaw_deg_ = wrap_to_pi(integrated_yaw_deg_ * (3.1415926535f / 180.0f)) * (180.0f / 3.1415926535f);
    data_[kAngleZ] = integrated_yaw_deg_;
    yaw_port_ = integrated_yaw_deg_;
}

void IMU::publish_ports_from_cache() {
    omega_x_port_ = data_[kOmegaX];
    omega_y_port_ = data_[kOmegaY];
    if (model_ != Model::kIcm42688) {
        omega_z_port_ = data_[kOmegaZ];
        yaw_port_ = data_[kAngleZ];
    }
}

void IMU::reset_ports() {
    omega_x_port_.reset();
    omega_y_port_.reset();
    omega_z_port_.reset();
    yaw_port_.reset();
}