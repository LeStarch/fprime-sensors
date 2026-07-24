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
      m_frequencyHz(DEFAULT_FREQUENCY_HZ),
      m_networkId(DEFAULT_NETWORK_ID),
      m_powerLevel(DEFAULT_POWER_LEVEL),
      m_packetsTransmitted(0),
      m_packetsReceived(0),
      m_transmitFailures(0),
      m_transmitsDeferred(0),
      m_deferredBuffer(),
      m_deferredContext(),
      m_deferredValid(false),
      m_transmitEnabled(TransmitState::ENABLED),
      m_resetPulsed(false),
      m_mosi{},
      m_miso{} {}

Rfm69Manager ::~Rfm69Manager() {}

void Rfm69Manager ::configure(U32 frequencyHz, U8 networkId, U8 powerLevel) {
    this->m_frequencyHz = frequencyHz;
    this->m_networkId = networkId;
    this->m_powerLevel = powerLevel;
    this->m_configured = true;
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
            this->m_transmitsDeferred++;
            this->log_ACTIVITY_LO_TransmitDeferred();
            this->tlmWrite_TransmitsDeferred(this->m_transmitsDeferred);
            return;
        }
        if (this->transmitFrame(data)) {
            status = Fw::Success::SUCCESS;
        }
    } else {
        this->log_WARNING_HI_RadioNotReady();
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
    // Wait for configure() before touching the bus
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
            this->log_WARNING_HI_RadioNotDetected();
            return;
        }
    }
    if (this->m_state == CONFIGURE) {
        if (this->configureRadio() && this->setMode(Mode::RX)) {
            this->m_state = READY;
            this->log_ACTIVITY_HI_RadioConfigured();
            // Signal readiness for the first frame
            if (this->isConnected_comStatusOut_OutputPort(0)) {
                Fw::Success ready = Fw::Success::SUCCESS;
                this->comStatusOut_out(0, ready);
            }
        } else {
            this->log_WARNING_HI_RadioConfigurationFailed();
        }
    }
}

bool Rfm69Manager ::transmitFrame(Fw::Buffer& data) {
    // Segment the frame into radio packets; the receiving side's frame
    // accumulator reassembles the byte stream
    const U8* const bytes = data.getData();
    const FwSizeType size = data.getSize();
    bool success = true;
    FwSizeType offset = 0;
    do {
        const FwSizeType chunk = FW_MIN(size - offset, MAX_PACKET_PAYLOAD);
        success = this->transmitPacket(&bytes[offset], chunk);
        offset += chunk;
    } while (success && (offset < size));
    if (!success) {
        this->m_transmitFailures++;
        this->log_WARNING_HI_TransmitFailed();
        this->tlmWrite_TransmitFailures(this->m_transmitFailures);
    }
    return success;
}

void Rfm69Manager ::retryDeferredTransmit() {
    if (!this->m_deferredValid || this->channelBusy()) {
        return;
    }
    this->m_deferredValid = false;
    Fw::Success status = Fw::Success::FAILURE;
    if (this->transmitFrame(this->m_deferredBuffer)) {
        status = Fw::Success::SUCCESS;
    }
    this->dataReturnOut_out(0, this->m_deferredBuffer, this->m_deferredContext);
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
    }
    this->tlmWrite_TransmitEnabled(enabled);
    this->log_ACTIVITY_HI_TransmitStateChanged(enabled);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
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
    this->log_WARNING_HI_ResetFailed(static_cast<U8>(status));
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
            this->log_WARNING_HI_ReceiveFailed();
            break;
        }
        Fw::Buffer buffer = this->allocate_out(0, size);
        if (buffer.getSize() < size) {
            // Packet already drained from the radio; drop it
            this->log_WARNING_HI_BufferAllocationFailed();
            this->deallocate_out(0, buffer);
            continue;
        }
        (void)::memcpy(buffer.getData(), payload, size);
        buffer.setSize(size);
        this->m_packetsReceived++;
        this->tlmWrite_PacketsReceived(this->m_packetsReceived);
        ComCfg::FrameContext context;
        this->dataOut_out(0, buffer, context);
    }
}

}  // namespace Rfm69
