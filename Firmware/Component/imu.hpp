#ifndef __IMU_HPP
#define __IMU_HPP

#include <cstdint>
#include <cstddef>
#include "spi.h"
#include "component.hpp"
#include "Task/utils.hpp"

constexpr size_t IMU_ICM42688_BURST_DATA_LENGTH = 12U;

class IMU {
public:
    enum class Model : uint8_t {
        kIcm42688 = 0,
    };
    
    enum DataType {
        kAccX = 0,
        kAccY,
        kAccZ,
        kTemperature,
        kOmegaX,
        kOmegaY,
        kOmegaZ,
        kVoltage,
        kAngleX,
        kAngleY,
        kAngleZ,
        kVersion
    };

    enum Mode {
        kAcc = 0,
        kOmega,
        kAngle,
        kAuto
    };

    IMU(
        Model model,
        SPI_HandleTypeDef* hspi,
        GPIO_TypeDef* cs_port = nullptr,
        uint16_t cs_pin = 0U
    );
    ~IMU() = default;

    bool init();
    void process_once();

    Model model() const { return model_; }
    SPI_HandleTypeDef* spi_handle() const { return hspi_; }

    float get_data(const DataType type) const { return data_[type]; }
    void get_data(float out_data[9]) const;
    void publish_ports_from_cache();
    void reset_ports();

    OutputPort<float>* omega_x_port() { return &omega_x_port_; }
    OutputPort<float>* omega_y_port() { return &omega_y_port_; }
    OutputPort<float>* omega_z_port() { return &omega_z_port_; }
    OutputPort<float>* yaw_port() { return &yaw_port_; }
    OutputPort<float>* roll_port() { return &roll_port_; }
    OutputPort<float>* pitch_port() { return &pitch_port_; }

    ButterworthLowPass2 omega_filter_{{0.0f, 0.0f}};

    void update_integrated_yaw(float dt_s);
    void update_roll_pitch(float dt_s, bool trust_accel,
                           float bias_gx_dps, float bias_gy_dps);

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
    static constexpr uint8_t kIcm42688Aodr200Hz = 0x07;
    static constexpr uint8_t kIcm42688Aodr100Hz = 0x08;
    static constexpr uint8_t kIcm42688Gfs2000Dps = 0x00;
    static constexpr uint8_t kIcm42688Gfs1000Dps = 0x01;
    static constexpr uint8_t kIcm42688Godr2000Hz = 0x05;
    static constexpr uint8_t kIcm42688Godr1000Hz = 0x06;
    static constexpr uint8_t kIcm42688Godr200Hz = 0x07;
    static constexpr uint8_t kIcm42688Godr100Hz = 0x08;

    bool decode_icm42688(const uint8_t* raw_data, size_t len);

    bool icm42688_init();
    uint8_t icm42688_read_reg(uint8_t reg);
    void icm42688_read_regs(uint8_t reg, uint8_t* data, uint16_t len);
    void icm42688_write_reg(uint8_t reg, uint8_t value);
    bool icm42688_read_burst(uint8_t* data, uint16_t len);

    void icm42688_cs_low();
    void icm42688_cs_high();

private:
    Model model_;
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;

    float acc_sensitivity_ = 16.0f * 9.8f / 32768.0f;
    float gyro_sensitivity_ = 2000.0f / 32768.0f;

    uint8_t icm_rx_buffer_[IMU_ICM42688_BURST_DATA_LENGTH] = {0};

    float data_[12] = {0};
    OutputPort<float> omega_x_port_{0.0f};
    OutputPort<float> omega_y_port_{0.0f};
    OutputPort<float> omega_z_port_{0.0f};
    OutputPort<float> yaw_port_{0.0f};
    OutputPort<float> roll_port_{0.0f};
    OutputPort<float> pitch_port_{0.0f};
    float integrated_yaw_deg_ = 0.0f;
    float integrated_roll_deg_ = 0.0f;
    float integrated_pitch_deg_ = 0.0f;
};

#endif // __IMU_HPP