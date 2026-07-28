// ======================================================================
// \title  Rfm69Manager.cpp
// \brief  cpp file for Rfm69Manager component implementation class
//
// F Prime handler implementations. Radio-specific register access and
// packet handling helpers live in Rfm69Helpers.cpp.
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Manager.hpp"

namespace Rfm69 {

namespace {

bool parameterValueIsUsable(Fw::ParamValid valid) {
    return (valid == Fw::ParamValid::VALID) || (valid == Fw::ParamValid::DEFAULT);
}

}  // namespace

// C++14 requires storage for these class constants when chrono binds them.
constexpr U32 Rfm69Manager::MODE_READY_TIMEOUT_USEC;
constexpr U32 Rfm69Manager::PACKET_DEADLINE_MARGIN_USEC;

// ----------------------------------------------------------------------
// Construction, initialization, and destruction
// ----------------------------------------------------------------------

Rfm69Manager ::Rfm69Manager(const char* const compName)
    : Rfm69ManagerComponentBase(compName),
      m_state(DETECT),
      m_configured(false),
      m_dataRate(Rfm69DataRate::BR_9600),
      m_bandwidthRx(Rfm69Bandwidth::BW_500_KHZ),
      m_txPower(Rfm69TxPower::DBM_13),
      m_packetsTransmitted(0),
      m_packetsReceived(0),
      m_transmitEnabled(TransmitState::ENABLED),
      m_resetPulsed(false),
      m_comStatusAnnounced(false),
      m_deferredBuffer(),
      m_deferredContext(),
      m_deferredValid(false),
      m_mosi{},
      m_miso{} {}

Rfm69Manager ::~Rfm69Manager() {}

void Rfm69Manager ::applyParameters() {
    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const Rfm69DataRate dataRate = this->paramGet_DATA_RATE(valid);
    if (parameterValueIsUsable(valid)) {
        this->m_dataRate = dataRate;
    }
    const Rfm69Bandwidth bandwidthRx = this->paramGet_BANDWIDTH_RX(valid);
    if (parameterValueIsUsable(valid)) {
        this->m_bandwidthRx = bandwidthRx;
    }
    const Rfm69TxPower txPower = this->paramGet_TX_POWER(valid);
    if (parameterValueIsUsable(valid)) {
        this->m_txPower = txPower;
    }
    this->m_configured = true;
}

void Rfm69Manager ::requestReconfigure() {
    // The radio will leave READY while registers are rewritten. Return a held
    // Com buffer with failure rather than silently losing ownership.
    this->finishDeferredTransmit(Fw::Success::FAILURE);
    if (this->m_state == DETECT) {
        return;
    }
    this->m_state = CONFIGURE;
}

void Rfm69Manager ::parameterUpdated(FwPrmIdType id) {
    Os::ScopeLock lock(this->m_lock);
    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    switch (id) {
        case PARAMID_DATA_RATE: {
            const Rfm69DataRate dataRate = this->paramGet_DATA_RATE(valid);
            FW_ASSERT(valid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(valid));
            this->m_dataRate = dataRate;
            break;
        }
        case PARAMID_BANDWIDTH_RX: {
            const Rfm69Bandwidth bandwidthRx = this->paramGet_BANDWIDTH_RX(valid);
            FW_ASSERT(valid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(valid));
            this->m_bandwidthRx = bandwidthRx;
            break;
        }
        case PARAMID_TX_POWER: {
            const Rfm69TxPower txPower = this->paramGet_TX_POWER(valid);
            FW_ASSERT(valid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(valid));
            this->m_txPower = txPower;
            break;
        }
        default:
            FW_ASSERT(0, static_cast<FwAssertArgType>(id));
            break;
    }
    this->m_configured = true;
    this->requestReconfigure();
}

void Rfm69Manager ::parametersLoaded() {
    // The topology invokes loadParameters() before starting rate groups. This
    // is the deployment's single source of radio configuration.
    Os::ScopeLock lock(this->m_lock);
    this->applyParameters();
    this->requestReconfigure();
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void Rfm69Manager ::dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    Os::ScopeLock lock(this->m_lock);
    Fw::Success status = Fw::Success::FAILURE;
    if (this->m_transmitEnabled == TransmitState::DISABLED) {
        // Commandable receive-only window: keep the radio in RX and drop the
        // downlink frame, but report success so Com flow control keeps moving.
        status = Fw::Success::SUCCESS;
    } else if ((this->m_state == READY) && !this->m_deferredValid) {
        // Listen-before-talk: defer the frame while a reception is in
        // progress; the run handler retries once the channel clears. The
        // buffer and com status are held until then, back-pressuring the
        // framer's com queue.
        if (this->channelBusy()) {
            this->m_deferredBuffer = data;
            this->m_deferredContext = context;
            this->m_deferredValid = true;
            return;
        }
        if (this->transmitFrame(data)) {
            status = Fw::Success::SUCCESS;
        }
    } else if ((this->m_state == READY) && this->m_deferredValid) {
        // Keep exactly one deferred frame. The new frame is returned below
        // with FAILURE; the deferred owner remains responsible for its frame.
        this->log_WARNING_HI_SendFailed(-1);
    } else {
        this->log_WARNING_HI_SendFailed(-1);
    }
    this->dataReturnOut_out(0, data, context);
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        this->comStatusOut_out(0, status);
    }
}

void Rfm69Manager ::dataReturnIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    this->deallocate_out(0, data);
}

void Rfm69Manager ::run_handler(FwIndexType portNum, U32 context) {
    Os::ScopeLock lock(this->m_lock);
    if (this->m_state == READY) {
        this->pollReceive();
        this->retryDeferredTransmit();
    } else {
        this->initializeRadio();
    }
}

// ----------------------------------------------------------------------
// Helper functions: radio state management
// ----------------------------------------------------------------------

void Rfm69Manager ::initializeRadio() {
    // Do not touch the bus before the deployment has loaded its parameters.
    if (!this->m_configured) {
        return;
    }
    if (this->m_state == DETECT) {
        if (!this->m_resetPulsed) {
            this->m_resetPulsed = this->pulseReset();
            if (!this->m_resetPulsed) {
                return;
            }
        }
        if (this->detectRadio()) {
            this->m_state = CONFIGURE;
        } else {
            this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
            return;
        }
    }
    if (this->m_state == CONFIGURE) {
        if (this->configureRadio() && this->setMode(Mode::RX)) {
            this->m_state = READY;
            // Com status is a one-time startup handshake. Sending an extra
            // SUCCESS after a hardware reset is invalid while the downstream
            // aggregator is already READY and causes it to assert.
            if (!this->m_comStatusAnnounced && this->isConnected_comStatusOut_OutputPort(0)) {
                Fw::Success ready = Fw::Success::SUCCESS;
                this->comStatusOut_out(0, ready);
                this->m_comStatusAnnounced = true;
            }
        } else {
            this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
        }
    }
}

bool Rfm69Manager ::transmitFrame(Fw::Buffer& data) {
    // The deployment's fixed 255-byte telemetry frame maps to exactly one
    // native RFM69 variable-length packet. There is intentionally no hidden
    // radio segmentation or reassembly contract.
    const FwSizeType size = data.getSize();
    if ((size == 0) || (size > MAX_PACKET_PAYLOAD)) {
        this->log_WARNING_HI_SendFailed(static_cast<I32>(size));
        return false;
    }
    const bool success = this->transmitPacket(data.getData(), size);
    if (!success) {
        this->log_WARNING_HI_SendFailed(-1);
    } else {
        this->log_WARNING_HI_ConfigurationFailed_ThrottleClear();
        this->log_WARNING_HI_SendFailed_ThrottleClear();
    }
    return success;
}

void Rfm69Manager ::finishDeferredTransmit(Fw::Success status) {
    if (!this->m_deferredValid) {
        return;
    }
    // Clear the state before invoking downstream ports: their synchronous
    // callbacks may cause a new frame to arrive as soon as this lock releases.
    Fw::Buffer buffer = this->m_deferredBuffer;
    const ComCfg::FrameContext context = this->m_deferredContext;
    this->m_deferredValid = false;
    this->m_deferredBuffer = Fw::Buffer();
    this->m_deferredContext = ComCfg::FrameContext();
    this->dataReturnOut_out(0, buffer, context);
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        this->comStatusOut_out(0, status);
    }
}

void Rfm69Manager ::retryDeferredTransmit() {
    if (!this->m_deferredValid) {
        return;
    }
    // TRANSMIT(DISABLED) is a receive-only window. A frame that happened to
    // be deferred immediately before the command must not escape later when
    // the channel goes idle.
    if (this->m_transmitEnabled == TransmitState::DISABLED) {
        this->finishDeferredTransmit(Fw::Success::SUCCESS);
        return;
    }
    if (this->channelBusy()) {
        return;
    }
    Fw::Buffer buffer = this->m_deferredBuffer;
    const ComCfg::FrameContext context = this->m_deferredContext;
    this->m_deferredValid = false;
    this->m_deferredBuffer = Fw::Buffer();
    this->m_deferredContext = ComCfg::FrameContext();
    Fw::Success status = Fw::Success::FAILURE;
    if (this->transmitFrame(buffer)) {
        status = Fw::Success::SUCCESS;
    }
    this->dataReturnOut_out(0, buffer, context);
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        this->comStatusOut_out(0, status);
    }
}

// ----------------------------------------------------------------------
// Command handler implementations
// ----------------------------------------------------------------------

void Rfm69Manager ::TRANSMIT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Rfm69::TransmitState enabled) {
    {
        Os::ScopeLock lock(this->m_lock);
        this->m_transmitEnabled = enabled;
        if (enabled == TransmitState::DISABLED) {
            // Match the behavior for a newly received disabled frame: drop
            // the held frame without RF transmission and release Com flow
            // control immediately.
            this->finishDeferredTransmit(Fw::Success::SUCCESS);
        }
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Rfm69Manager ::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    Fw::CmdResponse response = Fw::CmdResponse::EXECUTION_ERROR;
    {
        Os::ScopeLock lock(this->m_lock);
        // Return a pending downlink buffer before changing hardware state. The
        // next scheduler tick performs the normal detect/configure sequence.
        this->finishDeferredTransmit(Fw::Success::FAILURE);
        if (this->pulseReset()) {
            this->m_resetPulsed = true;
            this->m_state = DETECT;
            response = Fw::CmdResponse::OK;
        }
    }
    this->cmdResponse_out(opCode, cmdSeq, response);
}

bool Rfm69Manager ::pulseReset() {
    // A reset is optional for simulation and platforms that reset externally.
    if (!this->isConnected_resetGpio_OutputPort(0)) {
        return true;
    }
    Drv::GpioStatus status = this->resetGpio_out(0, Fw::Logic::HIGH);
    if (status == Drv::GpioStatus::OP_OK) {
        (void)Os::Task::delay(Fw::TimeInterval(0, 10000));
        status = this->resetGpio_out(0, Fw::Logic::LOW);
    }
    if (status == Drv::GpioStatus::OP_OK) {
        (void)Os::Task::delay(Fw::TimeInterval(0, 10000));
        return true;
    }
    return false;
}

void Rfm69Manager ::pollReceive() {
    U8 payload[MAX_PACKET_PAYLOAD];
    for (U32 i = 0; i < RX_PACKETS_PER_TICK; i++) {
        U8 flags1 = 0;
        U8 flags2 = 0;
        if ((this->readRegister(Reg::IRQ_FLAGS_1, flags1) != Drv::SpiStatus::SPI_OK) ||
            (this->readRegister(Reg::IRQ_FLAGS_2, flags2) != Drv::SpiStatus::SPI_OK)) {
            break;
        }
        // A packet is available (PayloadReady) or streaming in (SyncAddressMatch
        // with FIFO content). FifoNotEmpty alone is not sufficient: it is also
        // set while a transmission is loading the FIFO.
        const bool payloadReady = (flags2 & IrqFlags2::PAYLOAD_READY) != 0;
        const bool streamingIn =
            ((flags1 & IrqFlags1::SYNC_ADDRESS_MATCH) != 0) && ((flags2 & IrqFlags2::FIFO_NOT_EMPTY) != 0);
        if (!payloadReady && !streamingIn) {
            break;
        }
        this->updateRssi();
        const FwSizeType size = this->readReceivedPacket(payload, sizeof payload);
        if (size == 0) {
            break;
        }
        Fw::Buffer buffer = this->allocate_out(0, size);
        if (buffer.getSize() < size) {
            // Packet already drained from the radio; drop it
            this->log_WARNING_HI_AllocationFailed(size);
            this->deallocate_out(0, buffer);
            continue;
        }
        (void)::memcpy(buffer.getData(), payload, size);
        buffer.setSize(size);
        this->m_packetsReceived++;
        this->tlmWrite_PacketsReceived(this->m_packetsReceived);
        ComCfg::FrameContext context;
        this->dataOut_out(0, buffer, context);
        // readReceivedPacket() drains the entire variable-length packet and
        // restarts the receiver.  The IRQ state can take a short time to
        // settle after that restart, so defer a possible next packet to the
        // next polling tick instead of treating stale flags as another frame.
        break;
    }
}

}  // namespace Rfm69
