// ======================================================================
// \title  ImuManager.cpp
// \author mstarch
// \brief  cpp file for ImuManager component implementation class
// ======================================================================

#include "fprime-sensors/MpuImu/Components/ImuManager/ImuManager.hpp"

namespace MpuImu {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

ImuManager ::ImuManager(const char* const compName) : ImuManagerComponentBase(compName), m_address(DEVICE_DEFAULT_ADDRESS) {}

ImuManager ::~ImuManager() {}

void ImuManager ::configure(U8 device_address) {
    this->m_address = device_address;
}

void ImuManager ::parameterUpdated(FwPrmIdType id) {
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    switch (id) {
        case PARAMID_ACCELEROMETER_RANGE: {
            // Read back the parameter value
            const AccelerationRange range = this->paramGet_ACCELEROMETER_RANGE(isValid);
            // NOTE: isValid is always VALID in parameterUpdated as it was just properly set
            FW_ASSERT(isValid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(isValid));
            this->log_ACTIVITY_HI_AccelerometerRangeUpdated(range);
            this->imuStateMachine_sendSignal_reconfigure();
            break;
        }
        case PARAMID_GYROSCOPE_RANGE: {
            // Read back the parameter value
            const GyroscopeRange range = this->paramGet_GYROSCOPE_RANGE(isValid);
            // NOTE: isValid is always VALID in parameterUpdated as it was just properly set
            FW_ASSERT(isValid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(isValid));
            this->log_ACTIVITY_HI_GyroscopeRangeUpdated(range);
            this->imuStateMachine_sendSignal_reconfigure();
            break;
        }
        default:
            FW_ASSERT(0, static_cast<FwAssertArgType>(id));
            break;
    }
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void ImuManager ::run_handler(FwIndexType portNum, U32 context) {
    this->imuStateMachine_sendSignal_tick();
    this->dispatchCurrentMessages();
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void ImuManager ::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // Reuse error call to force a reset
    this->imuStateMachine_sendSignal_error();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Implementations for internal state machine actions
// ----------------------------------------------------------------------

void ImuManager ::MpuImu_ImuStateMachine_action_doReset(SmId smId, MpuImu_ImuStateMachine::Signal signal) {
    // This function is implemented only for the specific instance "imuStateMachine"
    FW_ASSERT(smId == SmId::imuStateMachine);
    Drv::I2cStatus status = this->reset();
    // Transition to RESET state on failure
    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(this->m_address, status);
        this->imuStateMachine_sendSignal_error();
    } else {
        this->imuStateMachine_sendSignal_success();
    }
}

void ImuManager ::MpuImu_ImuStateMachine_action_checkReset(SmId smId, MpuImu_ImuStateMachine::Signal signal) {
    // This function is implemented only for the specific instance "imuStateMachine"
    FW_ASSERT(smId == SmId::imuStateMachine);
    U8 reset_val = 0;
    Drv::I2cStatus status = this->read_reset(reset_val);
    // When reset is complete, the reset bit will be 0
    if ((status != Drv::I2cStatus::I2C_OK)) {
        this->log_WARNING_HI_I2cError(this->m_address, status);
        this->imuStateMachine_sendSignal_error();
    } else if ((reset_val & RESET_VALUE) == 0) {
        this->imuStateMachine_sendSignal_success();
    }
}

void ImuManager ::MpuImu_ImuStateMachine_action_doEnable(SmId smId, MpuImu_ImuStateMachine::Signal signal) {
    // This function is implemented only for the specific instance "imuStateMachine"
    FW_ASSERT(smId == SmId::imuStateMachine);
    Drv::I2cStatus status = this->enable();
    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(this->m_address, status);
        this->imuStateMachine_sendSignal_error();
    } else {
        this->imuStateMachine_sendSignal_success();
    }
}

void ImuManager ::MpuImu_ImuStateMachine_action_doConfigure(SmId smId, MpuImu_ImuStateMachine::Signal signal) {
    // This function is implemented only for the specific instance "imuStateMachine"
    FW_ASSERT(smId == SmId::imuStateMachine);
    Drv::I2cStatus status = this->configure_device();
    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(this->m_address, status);
        this->imuStateMachine_sendSignal_error();
    } else {
        this->imuStateMachine_sendSignal_success();
    }
}

void ImuManager ::MpuImu_ImuStateMachine_action_doRead(SmId smId, MpuImu_ImuStateMachine::Signal signal) {
    // This function is implemented only for the specific instance "imuStateMachine"
    FW_ASSERT(smId == SmId::imuStateMachine);
    ImuData imuData;
    Drv::I2cStatus status = this->read(imuData);
    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_WARNING_HI_I2cError(this->m_address, status);
        this->imuStateMachine_sendSignal_error();
    } else {
        this->tlmWrite_Reading(imuData);
        if (this->isConnected_imuDataPush_OutputPort(0)) {
            this->imuDataPush_out(0, imuData);
        }
    }
}

// ----------------------------------------------------------------------
// Implementations for outgoing port calls
// ----------------------------------------------------------------------

Drv::I2cStatus ImuManager ::bus_write(Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer) {
    Drv::I2cStatus status;
    FW_ASSERT(writeBuffer.isValid());
    if (readBuffer.isValid()) {
        status = this->busWriteRead_out(0, this->m_address, writeBuffer, readBuffer);
    } else {
        status = this->busWrite_out(0, this->m_address, writeBuffer);
    }
    return status;
}

}  // namespace MpuImu
