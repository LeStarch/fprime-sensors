// ======================================================================
// \title  Rfm69Sim.cpp
// \brief  cpp file for Rfm69Sim component implementation class
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Sim/Rfm69Sim.hpp"
#include <cstring>

namespace Rfm69 {

// ----------------------------------------------------------------------
// Construction, initialization, and destruction
// ----------------------------------------------------------------------

Rfm69Sim ::Rfm69Sim(const char* const compName) : Rfm69SimComponentBase(compName), m_model() {}

Rfm69Sim ::~Rfm69Sim() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

Drv::SpiStatus Rfm69Sim ::SpiWriteRead_handler(FwIndexType portNum, Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer) {
    if ((writeBuffer.getSize() == 0) || (readBuffer.getSize() < writeBuffer.getSize())) {
        return Drv::SpiStatus::SPI_MISMATCH_ERR;
    }
    this->m_model.spiTransaction(writeBuffer.getData(), readBuffer.getData(), writeBuffer.getSize());
    this->drainTransmittedPackets();
    return Drv::SpiStatus::SPI_OK;
}

void Rfm69Sim ::SpiReadWrite_handler(FwIndexType portNum, Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer) {
    (void)this->SpiWriteRead_handler(portNum, writeBuffer, readBuffer);
}

void Rfm69Sim ::airDataIn_handler(FwIndexType portNum, Fw::Buffer& buffer, const Drv::ByteStreamStatus& status) {
    if (status == Drv::ByteStreamStatus::OP_OK) {
        this->m_model.injectAirData(buffer.getData(), buffer.getSize());
    }
    this->airDataReturnOut_out(0, buffer);
}

void Rfm69Sim ::airReady_handler(FwIndexType portNum) {}

void Rfm69Sim ::drainTransmittedPackets() {
    U8 payload[MAX_PACKET_PAYLOAD + 1];
    for (FwSizeType size = this->m_model.retrievePacket(payload, sizeof payload); size > 0;
         size = this->m_model.retrievePacket(payload, sizeof payload)) {
        if (!this->isConnected_airDataOut_OutputPort(0)) {
            continue;
        }
        Fw::Buffer buffer = this->allocate_out(0, size);
        if (buffer.getSize() < size) {
            this->log_WARNING_HI_SimAllocationFailed();
            this->deallocate_out(0, buffer);
            continue;
        }
        (void)::memcpy(buffer.getData(), payload, size);
        buffer.setSize(size);
        (void)this->airDataOut_out(0, buffer);
        this->deallocate_out(0, buffer);
    }
}

}  // namespace Rfm69
