// ======================================================================
// \title  Rfm69Manager.cpp
// \brief  cpp file for Rfm69Manager component implementation class
//
// F Prime handler implementations. Radio-specific register access and
// packet handling helpers live in Rfm69Helpers.cpp.
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Manager.hpp"

namespace Rfm69 {

// ----------------------------------------------------------------------
// Construction, initialization, and destruction
// ----------------------------------------------------------------------

Rfm69Manager ::Rfm69Manager(const char* const compName)
    : Rfm69ManagerComponentBase(compName),
      m_state(DETECT),
      m_configured(false),
      m_packetsTransmitted(0),
      m_packetsReceived(0),
      m_transmitEnabled(TransmitState::ENABLED),
      m_resetPulsed(false),
      m_comStatusAnnounced(false),
      m_mosi{},
      m_miso{} {}

Rfm69Manager ::~Rfm69Manager() {}

void Rfm69Manager ::parameterUpdated(FwPrmIdType id) {
    // F´ already stored the new value; defer register rewrite to the next run
    // tick (LoRa re-reads params on each enableTx/enableRx instead).
    (void)id;
    Os::ScopeLock lock(this->m_lock);
    if (this->m_state != DETECT) {
        this->m_state = CONFIGURE;
    }
}

void Rfm69Manager ::parametersLoaded() {
    Os::ScopeLock lock(this->m_lock);
    this->m_configured = true;
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void Rfm69Manager ::dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    Os::ScopeLock lock(this->m_lock);
    Fw::Success status = Fw::Success::FAILURE;
    if (this->m_transmitEnabled == TransmitState::DISABLED) {
        // Receive-only window: drop downlink, keep Com flow control moving.
        status = Fw::Success::SUCCESS;
    } else if ((this->m_state == READY) && !this->channelBusy()) {
        // Dumb half-duplex: TX only when idle. If RX is in progress, drop below.
        if (this->transmitFrame(data)) {
            status = Fw::Success::SUCCESS;
        }
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
            // Same event as configure/RX failure: cannot bring the link up.
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
    // One Com buffer → one RF packet (1..255). No radio-layer split/reassembly.
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

// ----------------------------------------------------------------------
// Command handler implementations
// ----------------------------------------------------------------------

void Rfm69Manager ::TRANSMIT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Rfm69::TransmitState enabled) {
    {
        Os::ScopeLock lock(this->m_lock);
        this->m_transmitEnabled = enabled;
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Rfm69Manager ::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    Fw::CmdResponse response = Fw::CmdResponse::EXECUTION_ERROR;
    {
        Os::ScopeLock lock(this->m_lock);
        // Next scheduler tick performs the normal detect/configure sequence.
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
