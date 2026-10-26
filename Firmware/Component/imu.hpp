#ifndef IMU_HPP
#define IMU_HPP
#include <cstdint>
#include <cstddef>
#include "spi.h"

class IMU {
public:
    IMU(SPI_HandleTypeDef* spi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : hspi_(spi), cs_port_(cs_port), cs_pin_(cs_pin) {}
    bool init();
    void process_once();
    bool consume_raw_sample(float accel_m_s2[3], float gyro_rad_s[3], std::uint32_t* sequence);
private:
    static constexpr uint8_t kIcm42688WhoAmI = 0x75;
    static constexpr uint8_t kIcm42688WhoAmIValue = 0x47;
    static constexpr uint8_t kIcm42688RegBankSel = 0x76;
    static constexpr uint8_t kIcm42688DeviceConfig = 0x11;
    static constexpr uint8_t kIcm42688PwrMgmt0 = 0x4E;
    static constexpr uint8_t kIcm42688GyroConfig0 = 0x4F;
    static constexpr uint8_t kIcm42688AccelConfig0 = 0x50;
    static constexpr uint8_t kIcm42688AccelDataX1 = 0x1F;

    static constexpr uint8_t kIcm42688Afs4G = 0x02;
    static constexpr uint8_t kIcm42688Aodr1000Hz = 0x06;
    static constexpr uint8_t kIcm42688Gfs1000Dps = 0x01;
    static constexpr uint8_t kIcm42688Godr1000Hz = 0x06;

    bool icm42688_init();
    uint8_t icm42688_read_reg(uint8_t reg);
    bool icm42688_read_regs(uint8_t reg, uint8_t* data, uint16_t len);
    void icm42688_write_reg(uint8_t reg, uint8_t value);
    void icm42688_cs_low();
    void icm42688_cs_high();
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
    float acc_sensitivity_ = 4.f * 9.8f / 32768.f;
    float gyro_sensitivity_ = 1000.f / 32768.f;
    float accel_[3] = {}, gyro_[3] = {};
    std::uint32_t sequence_ = 0;
    bool new_sample_ = false;
};
#endif
