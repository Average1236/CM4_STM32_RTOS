#include "imu.hpp"
#include <cstring>

bool IMU::init() { return icm42688_init(); }
void IMU::process_once() {
    new_sample_ = false;
    uint8_t bytes[12];
    if (!icm42688_read_regs(kIcm42688AccelDataX1, bytes, sizeof(bytes))) return;
    for (int i = 0; i < 3; ++i) {
        const auto accel = static_cast<int16_t>((static_cast<uint16_t>(bytes[i*2]) << 8) | bytes[i*2+1]);
        const auto gyro = static_cast<int16_t>((static_cast<uint16_t>(bytes[i*2+6]) << 8) | bytes[i*2+7]);
        accel_[i] = accel * acc_sensitivity_;
        gyro_[i] = gyro * gyro_sensitivity_ * 0.017453292519943295f;
    }
    ++sequence_;
    new_sample_ = true;
}
bool IMU::consume_raw_sample(float accel[3], float gyro[3], std::uint32_t* sequence) {
    if (!new_sample_ || !accel || !gyro) return false;
    std::memcpy(accel, accel_, sizeof(accel_));
    std::memcpy(gyro, gyro_, sizeof(gyro_));
    if (sequence) *sequence = sequence_;
    new_sample_ = false;
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

bool IMU::icm42688_read_regs(uint8_t reg, uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0U) {
        return false;
    }

    uint8_t tx_data[24] = {0};
    uint8_t rx_data[24] = {0};
    tx_data[0] = static_cast<uint8_t>(reg | 0x80U);

    icm42688_cs_low();
    const bool success = HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, len + 1U, 10U) == HAL_OK;
    if (success) {
        std::memcpy(data, rx_data + 1, len);
    }
    icm42688_cs_high();
    return success;
}

void IMU::icm42688_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};
    icm42688_cs_low();
    (void)HAL_SPI_TransmitReceive(hspi_, tx, rx, 2U, 10U);
    icm42688_cs_high();
}

void IMU::icm42688_cs_low() { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET); }
void IMU::icm42688_cs_high() { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET); }
