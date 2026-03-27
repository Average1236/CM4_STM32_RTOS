#ifndef __IMU_HPP
#define __IMU_HPP

#include <cstdint>
#include <cstddef>
#include "spi.h"
#include "component.hpp"

#define IMU_RX_DATA_LENGTH (11*3*2)
#define IMU_TX_DATA_LENGTH (5)

#define LSM6DS3_DATA_READY_PIN GPIO_PIN_1 // PC1
#define LSM6DS3_DATA_LENGTH 6 // gyro x/y/z

class IMU {
public:
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

    struct LSM6DS3_Registers_t {
        uint8_t INT1_CTRL = 0x0D;
        uint8_t INT2_CTRL = 0x0E;
        uint8_t WHO_AM_I = 0x0F;
        uint8_t CTRL1_XL = 0x10;
        uint8_t CTRL2_G = 0x11;
        uint8_t CTRL3_C = 0x12;
        uint8_t CTRL4_C = 0x13;
        uint8_t CTRL5_C = 0x14;
        uint8_t CTRL6_C = 0x15;
        uint8_t CTRL7_G = 0x16;
        uint8_t CTRL8_XL = 0x17;
        uint8_t CTRL9_XL = 0x18;
        uint8_t CTRL10_C = 0x19;

        uint8_t OUT_TEMP_L = 0x20;
        uint8_t OUT_TEMP_H = 0x21;
        uint8_t OUTX_L_G = 0x22;
        uint8_t OUTX_H_G = 0x23;
        uint8_t OUTY_L_G = 0x24;
        uint8_t OUTY_H_G = 0x25;
        uint8_t OUTZ_L_G = 0x26;
        uint8_t OUTZ_H_G = 0x27;
        uint8_t OUTX_L_XL = 0x28;
        uint8_t OUTX_H_XL = 0x29;
        uint8_t OUTY_L_XL = 0x2A;
        uint8_t OUTY_H_XL = 0x2B;
        uint8_t OUTZ_L_XL = 0x2C;
        uint8_t OUTZ_H_XL = 0x2D;
    };

    LSM6DS3_Registers_t lsm6ds3_regs_;

    const uint8_t get_acc_header[5] = {0xFF, 0xAA, 0x27, 0x34, 0x00};
    const uint8_t get_omega_header[5] = {0xFF, 0xAA, 0x27, 0x37, 0x00};
    const uint8_t get_angle_header[5] = {0xFF, 0xAA, 0x27, 0x3D, 0x00};

    IMU(SPI_HandleTypeDef* hspi) : hspi_(hspi) {
        // Initialize TX buffer first word: (Address | Read) << 8
        lsm6ds3_tx_data[0] = (static_cast<uint16_t>(0x80u | lsm6ds3_regs_.OUTX_L_G) << 8);
        // remaining bytes default to 0
    };
    ~IMU() = default;

    void decode(uint8_t raw_data[IMU_RX_DATA_LENGTH]);
    float get_data(const DataType type) const { return data_[type]; }
    void get_data(float out_data[9]) const;
    void lsm6ds3_init();
    void process_spi_data();
    void publish_ports_from_cache();
    void reset_ports();

    OutputPort<float>* acc_x_port() { return &acc_x_port_; }
    OutputPort<float>* acc_y_port() { return &acc_y_port_; }
    OutputPort<float>* acc_z_port() { return &acc_z_port_; }
    OutputPort<float>* omega_x_port() { return &omega_x_port_; }
    OutputPort<float>* omega_y_port() { return &omega_y_port_; }
    OutputPort<float>* omega_z_port() { return &omega_z_port_; }
    OutputPort<float>* yaw_port() { return &yaw_port_; }
    // void update(uint8_t imu_tx_date[5]);
    // void set_update_mode(const Mode mode) { mode_ = mode; }

    uint32_t data_ready_pin_ = LSM6DS3_DATA_READY_PIN; // PC1
    uint16_t lsm6ds3_tx_data[4] = {0}; // gyro x/y/z
    uint16_t lsm6ds3_rx_data[4] = {0}; // gyro x/y/z
    SPI_HandleTypeDef* hspi_ = &hspi1;

private:
    float data_[12] = {0};
    OutputPort<float> acc_x_port_{0.0f};
    OutputPort<float> acc_y_port_{0.0f};
    OutputPort<float> acc_z_port_{0.0f};
    OutputPort<float> omega_x_port_{0.0f};
    OutputPort<float> omega_y_port_{0.0f};
    OutputPort<float> omega_z_port_{0.0f};
    OutputPort<float> yaw_port_{0.0f};
    bool sumcrc(const uint8_t raw_data[11]);
    // Mode mode_ = kAuto;
    
};

#endif // __IMU_HPP
