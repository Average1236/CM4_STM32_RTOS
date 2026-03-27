#include "imu.hpp"

const float imu_k[3] = {16.0f * 9.8f / 32768.0f, 2000.0f / 32768.0f, 180.0f / 32768.0f};

void IMU::decode(uint8_t raw_data[IMU_RX_DATA_LENGTH])
{
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
                }
            }
        }
    }

    publish_ports_from_cache();
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

void IMU::publish_ports_from_cache() {
    acc_x_port_ = data_[kAccX];
    acc_y_port_ = data_[kAccY];
    acc_z_port_ = data_[kAccZ];
    omega_x_port_ = data_[kOmegaX];
    omega_y_port_ = data_[kOmegaY];
    omega_z_port_ = data_[kOmegaZ];
    yaw_port_ = data_[kAngleZ];
}

void IMU::reset_ports() {
    acc_x_port_.reset();
    acc_y_port_.reset();
    acc_z_port_.reset();
    omega_x_port_.reset();
    omega_y_port_.reset();
    omega_z_port_.reset();
    yaw_port_.reset();
}
