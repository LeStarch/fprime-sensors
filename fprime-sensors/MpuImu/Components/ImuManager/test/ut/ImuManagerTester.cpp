// ======================================================================
// \title  ImuManagerTester.cpp
// \author mstarch
// \brief  cpp file for ImuManager component test harness implementation class
// ======================================================================

#include "ImuManagerTester.hpp"
#include "STest/Pick/Pick.hpp"

namespace MpuImu {

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

ImuManagerTester ::ImuManagerTester()
    : ImuManagerGTestBase("ImuManagerTester", ImuManagerTester::MAX_HISTORY_SIZE),
      component("ImuManager"),
      state(ImuManagerTester::State::RESET) {
    this->initComponents();
    this->connectPorts();
}

ImuManagerTester ::~ImuManagerTester() {
    this->component.deinit();
}

void ImuManagerTester ::tick() {
    this->invoke_to_run(0, 0);
}

void ImuManagerTester ::nominal_boot_sequence() {
    // Initial tick moves into RESET state
    this->tick();
    ASSERT_from_busWrite_SIZE(1);
    ASSERT_EVENTS_I2cError_SIZE(0);
    this->verify_state_and_clear(ImuManagerTester::State::WAIT_RESET);

    U32 randomValue = STest::Pick::lowerUpper(0, 100);
    // Wait a while before triggering reset finish
    for (FwSizeType i = 0; i < static_cast<FwSizeType>(randomValue); i++) {
        this->tick();
        ASSERT_from_busWriteRead_SIZE(1);
        this->verify_state_and_clear(ImuManagerTester::State::WAIT_RESET);
    }
    // Trigger reset finish to move out of WAIT_RESET
    this->state = ImuManagerTester::State::WAIT_RESET_FINISH;
    this->tick();
    ASSERT_from_busWriteRead_SIZE(1);
    ASSERT_from_busWrite_SIZE(0);
    this->verify_state_and_clear(ImuManagerTester::State::POWER_ON);

    // Next tick should push us through the ENABLE state
    this->tick();
    ASSERT_from_busWrite_SIZE(1);
    ASSERT_from_busWriteRead_SIZE(0);
    this->verify_state_and_clear(ImuManagerTester::State::CONFIGURE_ACCELEROMETER);
}

void ImuManagerTester ::reconfigure_sequence() {
    // Trigger configuration, will end in RUN state
    this->tick();
    ASSERT_from_busWrite_SIZE(2);
    this->verify_state_and_clear(ImuManagerTester::State::RUN);
}

void ImuManagerTester ::nominal_run_sequence() {
    // Run a few RUN cycles
    U32 randomValue = STest::Pick::lowerUpper(0, 20);
    for (FwSizeType i = 0; i < static_cast<FwSizeType>(randomValue); i++) {
        this->tick();
        ASSERT_from_busWriteRead_SIZE(1);
        ASSERT_TLM_Reading_SIZE(1);
        ASSERT_TLM_Reading(0, this->imuData);
        this->verify_state_and_clear(ImuManagerTester::State::RUN);
    }
}

void ImuManagerTester ::verify_register_write(U8 registerAddress, U8 registerValue, Fw::Buffer& writeBuffer) {
    ASSERT_EQ(writeBuffer.getSize(), 2);
    ASSERT_EQ(writeBuffer.getData()[0], registerAddress);
    ASSERT_EQ(writeBuffer.getData()[1], registerValue);
}

void ImuManagerTester ::verify_reset() {
    ASSERT_from_busWrite_SIZE(1);
    ImuManagerTester::FromPortEntry_busWrite entry = fromPortHistory_busWrite->at(0);
    ASSERT_EQ(entry.addr, 0x68);
    this->verify_register_write(0x6B, 0x80, entry.serBuffer);
}

void ImuManagerTester ::verify_state_and_clear(State state) {
    ASSERT_EQ(this->state, state);
    this->clearHistory();
    ASSERT_from_busWrite_SIZE(0);
    ASSERT_from_busWriteRead_SIZE(0);
}

void ImuManagerTester ::pick_acceleration_range() {
    U32 randomValue = STest::Pick::lowerUpper(0, 3);
    switch (randomValue) {
        case 0:
            this->accelerationRange = AccelerationRange::RANGE_2G;
            break;
        case 1:
            this->accelerationRange = AccelerationRange::RANGE_4G;
            break;
        case 2:
            this->accelerationRange = AccelerationRange::RANGE_8G;
            break;
        case 3:
            this->accelerationRange = AccelerationRange::RANGE_16G;
            break;
        default:
            this->accelerationRange = AccelerationRange::RANGE_2G;
    }
    this->paramSet_ACCELEROMETER_RANGE(this->accelerationRange, Fw::ParamValid::VALID);
    this->paramSend_ACCELEROMETER_RANGE(0, 0);
}

void ImuManagerTester ::pick_gyroscope_range() {
    U32 randomValue = STest::Pick::lowerUpper(0, 3);
    switch (randomValue) {
        case 0:
            this->gyroscopeRange = GyroscopeRange::RANGE_250DEG;
            break;
        case 1:
            this->gyroscopeRange = GyroscopeRange::RANGE_500DEG;
            break;
        case 2:
            this->gyroscopeRange = GyroscopeRange::RANGE_1000DEG;
            break;
        case 3:
            this->gyroscopeRange = GyroscopeRange::RANGE_2000DEG;
            break;
        default:
            this->gyroscopeRange = GyroscopeRange::RANGE_250DEG;
    }
    this->paramSet_GYROSCOPE_RANGE(this->gyroscopeRange, Fw::ParamValid::VALID);
    this->paramSend_GYROSCOPE_RANGE(0, 0);
}

void ImuManagerTester ::fill_read_data(Fw::Buffer& readBuffer) {
    RawImuData raw;
    raw.acceleration[0] = STest::Pick::lowerUpper(0, 0xFFFF);
    raw.acceleration[1] = STest::Pick::lowerUpper(0, 0xFFFF);
    raw.acceleration[2] = STest::Pick::lowerUpper(0, 0xFFFF);
    raw.temperature = STest::Pick::lowerUpper(0, 0xFFFF);
    raw.gyroscope[0] = STest::Pick::lowerUpper(0, 0xFFFF);
    raw.gyroscope[1] = STest::Pick::lowerUpper(0, 0xFFFF);
    raw.gyroscope[2] = STest::Pick::lowerUpper(0, 0xFFFF);
    auto serializer = readBuffer.getSerializer();
    serializer.serializeFrom(raw.acceleration[0]);
    serializer.serializeFrom(raw.acceleration[1]);
    serializer.serializeFrom(raw.acceleration[2]);
    serializer.serializeFrom(raw.temperature);
    serializer.serializeFrom(raw.gyroscope[0]);
    serializer.serializeFrom(raw.gyroscope[1]);
    serializer.serializeFrom(raw.gyroscope[2]);
    this->imuData = ImuManager::convert_raw_data(raw, this->accelerationRange, this->gyroscopeRange);
}

Drv::I2cStatus ImuManagerTester ::from_busWrite_handler(
    FwIndexType portNum,      //!< The port number
    U32 addr,                 //!< I2C slave device address
    Fw::Buffer& writeBuffer  //!< Buffer to write data to the i2c device
) {
    Fw::Buffer readBuffer;
    this->pushFromPortEntry_busWrite(addr, writeBuffer);
    return this->bus_handler_helper(addr, writeBuffer, readBuffer);
}


Drv::I2cStatus ImuManagerTester ::from_busWriteRead_handler(
    FwIndexType portNum,      //!< The port number
    U32 addr,                 //!< I2C slave device address
    Fw::Buffer& writeBuffer,  //!< Buffer to write data to the i2c device
    Fw::Buffer& readBuffer  //!< Buffer to read back data from the i2c device, must set size when passing in read buffer
) {
    this->pushFromPortEntry_busWriteRead(addr, writeBuffer, readBuffer);
    return this->bus_handler_helper(addr, writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManagerTester ::bus_handler_helper(
    U32 addr,                 //!< I2C slave device address
    Fw::Buffer& writeBuffer,  //!< Buffer to write data to the i2c device
    Fw::Buffer& readBuffer  //!< Buffer to read back data from the i2c device, must set size when passing in read buffer
) {
    switch (this->state) {
        // Check the output of the IMU when waiting for reset to finish
        case ImuManagerTester::State::RESET:
            EXPECT_EQ(this->state, ImuManagerTester::State::RESET); 
            this->verify_reset();
            this->state = ImuManagerTester::State::WAIT_RESET;
            break;
        // Check the output of the IMU when waiting for reset to finish
        case ImuManagerTester::State::WAIT_RESET:
            EXPECT_EQ(writeBuffer.getSize(), 1);
            EXPECT_EQ(writeBuffer.getData()[0], 0x6B);
            EXPECT_EQ(readBuffer.getSize(), 1);
            readBuffer.getData()[0] = 0x80;  // Indicate the reset is still in progress
            break;
        // Check the output of the IMU when waiting for reset to finish
        case ImuManagerTester::State::WAIT_RESET_FINISH:
            EXPECT_EQ(writeBuffer.getSize(), 1);
            EXPECT_EQ(writeBuffer.getData()[0], 0x6B);
            EXPECT_EQ(readBuffer.getSize(), 1);
            readBuffer.getData()[0] = 0x00;  // Indicate the reset is complete
            // Force entry to the POWER_ON state as this happens immediately
            this->state = ImuManagerTester::State::POWER_ON;
            break;
        case ImuManagerTester::State::POWER_ON:
            EXPECT_EQ(writeBuffer.getSize(), 2);
            EXPECT_EQ(writeBuffer.getData()[0], 0x6B);
            EXPECT_EQ(writeBuffer.getData()[1], 0x00);
            EXPECT_EQ(readBuffer.getSize(), 0);
            this->state = ImuManagerTester::State::CONFIGURE_ACCELEROMETER;
            break;
        case ImuManagerTester::State::CONFIGURE_ACCELEROMETER:
            EXPECT_EQ(writeBuffer.getSize(), 2);
            EXPECT_EQ(writeBuffer.getData()[0], 0x1C);
            EXPECT_EQ(writeBuffer.getData()[1], ImuManager::accelerometer_range_to_register(this->accelerationRange));
            EXPECT_EQ(readBuffer.getSize(), 0);
            this->state = ImuManagerTester::State::CONFIGURE_GYROSCOPE;
            break;
        case ImuManagerTester::State::CONFIGURE_GYROSCOPE:
            EXPECT_EQ(writeBuffer.getSize(), 2);
            EXPECT_EQ(writeBuffer.getData()[0], 0x1B);
            EXPECT_EQ(writeBuffer.getData()[1], ImuManager::gyroscope_range_to_register(this->gyroscopeRange));
            EXPECT_EQ(readBuffer.getSize(), 0);
            this->state = ImuManagerTester::State::RUN;
            break;
        case ImuManagerTester::State::RUN:
            EXPECT_EQ(writeBuffer.getSize(), 1);
            EXPECT_EQ(writeBuffer.getData()[0], 0x3B);
            EXPECT_EQ(readBuffer.getSize(), sizeof(U16) * 7);
            this->fill_read_data(readBuffer);
            break;
        default:
            break;
    }
    // Simulate failures at a given rate
    U8 failureCheck = STest::Pick::lowerUpper(0, 99);
    if (failureCheck < this->failureRate) {
        this->i2cFailure = (this->i2cFailure >= 5) ? 5 : this->i2cFailure + 1;
        this->state = ImuManagerTester::State::RESET;
        return Drv::I2cStatus::I2C_OTHER_ERR;
    }
    return Drv::I2cStatus::I2C_OK;
}

}  // namespace MpuImu
