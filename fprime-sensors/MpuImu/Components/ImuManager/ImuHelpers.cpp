// ======================================================================
// \title  ImuHelpers.cpp
// \author mstarch
// \brief  cpp file for ImuManager component helper function implementations
// ======================================================================

#include "fprime-sensors/MpuImu/Components/ImuManager/ImuManager.hpp"
#include "fprime-sensors/MpuImu/Components/ImuManager/ImuTypes.hpp"

namespace MpuImu {

Drv::I2cStatus ImuManager ::reset() {
    // Attempt to write the reset data
    U8 reset_sequence[] = {POWER_MGMT_REGISTER, RESET_VALUE};
    Fw::Buffer writeBuffer(reset_sequence, sizeof(reset_sequence));
    Fw::Buffer readBuffer;
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::read_reset(U8& value) {
    U8 registerAddress = POWER_MGMT_REGISTER;
    Fw::Buffer writeBuffer(&registerAddress, sizeof(registerAddress));
    Fw::Buffer readBuffer(&value, sizeof(value));
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::enable() {
    U8 power_on_sequence[] = {POWER_MGMT_REGISTER, POWER_ON_VALUE};
    Fw::Buffer writeBuffer(power_on_sequence, sizeof(power_on_sequence));
    Fw::Buffer readBuffer;
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::configure_device() {
    Fw::ParamValid paramValid;
    Drv::I2cStatus status = Drv::I2cStatus::I2C_OK;
    // Read accelerometer parameter and configure
    {
        const AccelerationRange accelerationRange = this->paramGet_ACCELEROMETER_RANGE(paramValid);
        FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

        U8 accel_config_sequence[] = {ACCEL_CONFIG_REGISTER, this->accelerometer_range_to_register(accelerationRange)};
        Fw::Buffer writeBuffer(accel_config_sequence, sizeof(accel_config_sequence));
        Fw::Buffer readBuffer;
        status = this->bus_write(writeBuffer, readBuffer);
        if (status != Drv::I2cStatus::I2C_OK) {
            return status;
        }
    }
    // Read gyroscope parameter and configure
    {
        const GyroscopeRange gyroscopeRange = this->paramGet_GYROSCOPE_RANGE(paramValid);
        FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));
        U8 gyro_config_sequence[] = {GYRO_CONFIG_REGISTER, this->gyroscope_range_to_register(gyroscopeRange)};
        Fw::Buffer writeBuffer(gyro_config_sequence, sizeof(gyro_config_sequence));
        Fw::Buffer readBuffer;
        status = this->bus_write(writeBuffer, readBuffer);
        if (status != Drv::I2cStatus::I2C_OK) {
            return status;
        }
    }
    return status;
}

Drv::I2cStatus ImuManager ::read(ImuData& imuData) {
    U8 data[DATA_LENGTH];
    U8 registerAddress = DATA_BASE_REGISTER;

    Fw::Buffer writeBuffer(&registerAddress, 1);
    Fw::Buffer readBuffer(data, DATA_LENGTH);
    // If bus write fails, state machine is reset, so just return
    Drv::I2cStatus status = this->bus_write(writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return status;
    }
    RawImuData raw = this->deserialize_raw_data(readBuffer);

    // This code will read the currently scaled parameters
    Fw::ParamValid paramValid;
    const AccelerationRange accelerationRange = this->paramGet_ACCELEROMETER_RANGE(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));
    const GyroscopeRange gyroscopeRange = this->paramGet_GYROSCOPE_RANGE(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

    imuData = this->convert_raw_data(raw, accelerationRange, gyroscopeRange);
    return status;
}

RawImuData ImuManager ::deserialize_raw_data(Fw::Buffer& buffer) {
    auto deserializer = buffer.getDeserializer();
    RawImuData raw;
    deserializer.deserializeTo(raw.acceleration[0]);
    deserializer.deserializeTo(raw.acceleration[1]);
    deserializer.deserializeTo(raw.acceleration[2]);
    deserializer.deserializeTo(raw.temperature);
    deserializer.deserializeTo(raw.gyroscope[0]);
    deserializer.deserializeTo(raw.gyroscope[1]);
    deserializer.deserializeTo(raw.gyroscope[2]);
    return raw;
}

ImuData ImuManager ::convert_raw_data(const RawImuData& raw,
                                      const AccelerationRange& accelerationRange,
                                      const GyroscopeRange& gyroscopeRange) {
    // Set the values of the IMU data by multiplying by conversion factors
    MpuImu::ImuData imuData;
    imuData.get_acceleration().set_x(static_cast<F32>(raw.acceleration[0]) * 1.0f /
                                     static_cast<F32>(accelerationRange));
    imuData.get_acceleration().set_y(static_cast<F32>(raw.acceleration[1]) * 1.0f /
                                     static_cast<F32>(accelerationRange));
    imuData.get_acceleration().set_z(static_cast<F32>(raw.acceleration[2]) * 1.0f /
                                     static_cast<F32>(accelerationRange));
    imuData.set_temperature((static_cast<F32>(raw.temperature) / TEMPERATURE_SCALAR) + TEMPERATURE_OFFSET);
    imuData.get_rotation().set_x(static_cast<F32>(raw.gyroscope[0]) * 10.0f / static_cast<F32>(gyroscopeRange));
    imuData.get_rotation().set_y(static_cast<F32>(raw.gyroscope[1]) * 10.0f / static_cast<F32>(gyroscopeRange));
    imuData.get_rotation().set_z(static_cast<F32>(raw.gyroscope[2]) * 10.0f / static_cast<F32>(gyroscopeRange));
    return imuData;
}

U8 ImuManager ::accelerometer_range_to_register(AccelerationRange range) {
    U8 registerValue = 0;
    switch (range.e) {
        case AccelerationRange::RANGE_2G:
            registerValue = ACCEL_CONFIG_2G;
            break;
        case AccelerationRange::RANGE_4G:
            registerValue = ACCEL_CONFIG_4G;
            break;
        case AccelerationRange::RANGE_8G:
            registerValue = ACCEL_CONFIG_8G;
            break;
        case AccelerationRange::RANGE_16G:
            registerValue = ACCEL_CONFIG_16G;
            break;
        default:
            FW_ASSERT(0, range.e);
            break;
    }
    return registerValue;
}

U8 ImuManager ::gyroscope_range_to_register(GyroscopeRange range) {
    U8 registerValue = 0;
    switch (range.e) {
        case GyroscopeRange::RANGE_250DEG:
            registerValue = GYRO_CONFIG_250DEG;
            break;
        case GyroscopeRange::RANGE_500DEG:
            registerValue = GYRO_CONFIG_500DEG;
            break;
        case GyroscopeRange::RANGE_1000DEG:
            registerValue = GYRO_CONFIG_1000DEG;
            break;
        case GyroscopeRange::RANGE_2000DEG:
            registerValue = GYRO_CONFIG_2000DEG;
            break;
        default:
            FW_ASSERT(0, range.e);
            break;
    }
    return registerValue;
}

}  // namespace MpuImu
