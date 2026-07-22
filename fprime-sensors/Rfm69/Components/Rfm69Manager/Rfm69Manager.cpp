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
    Fw::Success status = Fw::Success::FAILURE;
    if (this->m_state == READY) {
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
        if (success) {
            status = Fw::Success::SUCCESS;
        } else {
            this->m_transmitFailures++;
            this->log_WARNING_HI_TransmitFailed();
            this->tlmWrite_TransmitFailures(this->m_transmitFailures);
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
    // Wait for configure() before touching the bus
    if (!this->m_configured) {
        return;
    }
    if (this->m_state == DETECT) {
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

void Rfm69Manager ::pollReceive() {
    U8 payload[MAX_PACKET_PAYLOAD + 1];
    for (U32 i = 0; i < RX_PACKETS_PER_TICK; i++) {
        U8 flags = 0;
        if (this->readRegister(Reg::IRQ_FLAGS_2, flags) != Drv::SpiStatus::SPI_OK) {
            break;
        }
        if ((flags & IrqFlags2::PAYLOAD_READY) == 0) {
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
