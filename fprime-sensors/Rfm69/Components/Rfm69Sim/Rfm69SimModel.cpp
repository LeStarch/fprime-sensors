// ======================================================================
// \title  Rfm69SimModel.cpp
// \brief  Register-level model of an RFM69HCW radio
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Sim/Rfm69SimModel.hpp"
#include <cstring>

namespace Rfm69 {

Rfm69SimModel::Rfm69SimModel()
    : m_fifoCount(0),
      m_fifoReadIndex(0),
      m_packetSent(false),
      m_payloadReady(false),
      m_fifoOverrun(false),
      m_airCount(0),
      m_txCount(0) {
    this->reset();
}

void Rfm69SimModel::reset() {
    (void)::memset(this->m_registers, 0, sizeof this->m_registers);
    (void)::memset(this->m_fifo, 0, sizeof this->m_fifo);
    (void)::memset(this->m_airBuffer, 0, sizeof this->m_airBuffer);
    (void)::memset(this->m_txQueue, 0, sizeof this->m_txQueue);
    (void)::memset(this->m_txSizes, 0, sizeof this->m_txSizes);
    // Datasheet Table 23 reset values for registers the manager interacts with
    this->m_registers[Reg::OP_MODE] = Mode::STANDBY;
    this->m_registers[Reg::BITRATE_MSB] = 0x1A;
    this->m_registers[Reg::BITRATE_LSB] = 0x0B;
    this->m_registers[Reg::FDEV_LSB] = 0x52;
    this->m_registers[Reg::FRF_MSB] = 0xE4;
    this->m_registers[Reg::FRF_MID] = 0xC0;
    this->m_registers[Reg::VERSION] = VERSION_VALUE;
    this->m_registers[Reg::PA_LEVEL] = 0x9F;
    this->m_registers[Reg::RX_BW] = 0x86;
    this->m_registers[Reg::RSSI_VALUE] = RSSI_READBACK;
    this->m_registers[Reg::DIO_MAPPING_2] = 0x05;
    this->m_registers[Reg::RSSI_THRESH] = 0xFF;
    this->m_registers[Reg::SYNC_CONFIG] = 0x98;
    this->m_registers[Reg::PACKET_CONFIG_1] = 0x10;
    this->m_registers[Reg::PAYLOAD_LENGTH] = 0x40;
    this->m_registers[Reg::FIFO_THRESH] = 0x0F;
    this->m_registers[Reg::PACKET_CONFIG_2] = 0x02;
    this->m_fifoCount = 0;
    this->m_fifoReadIndex = 0;
    this->m_packetSent = false;
    this->m_payloadReady = false;
    this->m_fifoOverrun = false;
    this->m_airCount = 0;
    this->m_txCount = 0;
}

void Rfm69SimModel::spiTransaction(const U8* mosi, U8* miso, FwSizeType size) {
    if ((mosi == nullptr) || (miso == nullptr) || (size == 0)) {
        return;
    }
    // First byte: wnr flag plus 7-bit address (datasheet section 5.2.1)
    const bool isWrite = (mosi[0] & SPI_WRITE_FLAG) != 0;
    U8 address = mosi[0] & SPI_ADDRESS_MASK;
    miso[0] = 0;
    for (FwSizeType i = 1; i < size; i++) {
        if (isWrite) {
            // The pre-write register value appears on MISO; FIFO writes must
            // not trigger the FIFO-pop side effect of a read
            miso[i] = (address == Reg::FIFO) ? 0 : this->m_registers[address & SPI_ADDRESS_MASK];
            this->writeRegister(address, mosi[i]);
        } else {
            miso[i] = this->readRegister(address);
        }
        // Burst access auto-increments the address; FIFO access does not
        if (address != Reg::FIFO) {
            address = (address + 1) & SPI_ADDRESS_MASK;
        }
    }
}

void Rfm69SimModel::injectAirData(const U8* data, FwSizeType size) {
    if ((data == nullptr) || (size == 0)) {
        return;
    }
    // Drop oldest bytes when the air buffer would overflow
    if (size > AIR_BUFFER_SIZE) {
        data = &data[size - AIR_BUFFER_SIZE];
        size = AIR_BUFFER_SIZE;
    }
    if ((this->m_airCount + size) > AIR_BUFFER_SIZE) {
        const FwSizeType excess = (this->m_airCount + size) - AIR_BUFFER_SIZE;
        (void)::memmove(this->m_airBuffer, &this->m_airBuffer[excess], this->m_airCount - excess);
        this->m_airCount -= excess;
    }
    (void)::memcpy(&this->m_airBuffer[this->m_airCount], data, size);
    this->m_airCount += size;
    this->tryLoadReceivePacket();
}

FwSizeType Rfm69SimModel::retrievePacket(U8* data, FwSizeType capacity) {
    if ((data == nullptr) || (this->m_txCount == 0)) {
        return 0;
    }
    const FwSizeType size = FW_MIN(this->m_txSizes[0], capacity);
    (void)::memcpy(data, this->m_txQueue[0], size);
    // Shift the queue forward
    for (FwSizeType i = 1; i < this->m_txCount; i++) {
        (void)::memcpy(this->m_txQueue[i - 1], this->m_txQueue[i], sizeof this->m_txQueue[i]);
        this->m_txSizes[i - 1] = this->m_txSizes[i];
    }
    this->m_txCount--;
    return size;
}

U8 Rfm69SimModel::readRegister(U8 address) {
    U8 value = 0;
    switch (address) {
        case Reg::FIFO:
            if (this->m_fifoReadIndex < this->m_fifoCount) {
                value = this->m_fifo[this->m_fifoReadIndex];
                this->m_fifoReadIndex++;
                if (this->m_fifoReadIndex >= this->m_fifoCount) {
                    // FIFO empty: PayloadReady clears; next packet may load
                    this->m_fifoCount = 0;
                    this->m_fifoReadIndex = 0;
                    this->m_payloadReady = false;
                    this->tryLoadReceivePacket();
                }
            }
            break;
        case Reg::IRQ_FLAGS_1:
            value = this->irqFlags1();
            break;
        case Reg::IRQ_FLAGS_2:
            value = this->irqFlags2();
            break;
        default:
            value = this->m_registers[address & SPI_ADDRESS_MASK];
            break;
    }
    return value;
}

void Rfm69SimModel::writeRegister(U8 address, U8 value) {
    switch (address) {
        case Reg::FIFO:
            if (this->m_fifoCount < FIFO_SIZE) {
                this->m_fifo[this->m_fifoCount] = value;
                this->m_fifoCount++;
            } else {
                // Overrun: flag set, data lost (datasheet section 5.2.2.3)
                this->m_fifoOverrun = true;
            }
            // In TX mode a complete packet transmits as soon as it is loaded
            if ((this->m_registers[Reg::OP_MODE] & Mode::MASK) == Mode::TX) {
                this->tryTransmit();
            }
            break;
        case Reg::IRQ_FLAGS_2:
            // Writing FifoOverrun=1 clears the flag and the FIFO
            if ((value & IrqFlags2::FIFO_OVERRUN) != 0) {
                this->m_fifoOverrun = false;
                this->m_fifoCount = 0;
                this->m_fifoReadIndex = 0;
                this->m_payloadReady = false;
            }
            break;
        case Reg::OP_MODE:
            this->m_registers[Reg::OP_MODE] = value;
            this->handleModeChange(value & Mode::MASK);
            break;
        default:
            this->m_registers[address & SPI_ADDRESS_MASK] = value;
            break;
    }
}

void Rfm69SimModel::handleModeChange(U8 mode) {
    if (mode == Mode::TX) {
        // PayloadReady is an RX flag; leaving RX clears it
        this->m_payloadReady = false;
        this->tryTransmit();
    } else if (mode == Mode::RX) {
        // PacketSent clears when exiting TX
        this->m_packetSent = false;
        this->tryLoadReceivePacket();
    } else {
        this->m_packetSent = false;
    }
}

void Rfm69SimModel::tryTransmit() {
    // Variable-length packet: first FIFO byte is the payload length
    const FwSizeType available = this->m_fifoCount - this->m_fifoReadIndex;
    if (available == 0) {
        return;
    }
    const U8 length = this->m_fifo[this->m_fifoReadIndex];
    if (available < (static_cast<FwSizeType>(length) + 1)) {
        return;  // Packet not fully loaded yet
    }
    if (this->m_txCount < TX_QUEUE_DEPTH) {
        const FwSizeType copySize = FW_MIN(static_cast<FwSizeType>(length), sizeof this->m_txQueue[0]);
        (void)::memcpy(this->m_txQueue[this->m_txCount], &this->m_fifo[this->m_fifoReadIndex + 1], copySize);
        this->m_txSizes[this->m_txCount] = copySize;
        this->m_txCount++;
    }
    // FIFO is drained by the transmission
    this->m_fifoCount = 0;
    this->m_fifoReadIndex = 0;
    this->m_packetSent = true;
}

void Rfm69SimModel::tryLoadReceivePacket() {
    // Deliver the next packet only when in RX with an empty FIFO
    if (((this->m_registers[Reg::OP_MODE] & Mode::MASK) != Mode::RX) || this->m_payloadReady ||
        (this->m_fifoCount != 0) || (this->m_airCount == 0)) {
        return;
    }
    const FwSizeType payloadSize = FW_MIN(this->m_airCount, MAX_PACKET_PAYLOAD);
    this->m_fifo[0] = static_cast<U8>(payloadSize);
    (void)::memcpy(&this->m_fifo[1], this->m_airBuffer, payloadSize);
    this->m_fifoCount = payloadSize + 1;
    this->m_fifoReadIndex = 0;
    (void)::memmove(this->m_airBuffer, &this->m_airBuffer[payloadSize], this->m_airCount - payloadSize);
    this->m_airCount -= payloadSize;
    this->m_payloadReady = true;
}

U8 Rfm69SimModel::irqFlags1() const {
    // The simulated radio settles instantaneously: ModeReady is always set
    U8 flags = IrqFlags1::MODE_READY;
    const U8 mode = this->m_registers[Reg::OP_MODE] & Mode::MASK;
    if (mode == Mode::RX) {
        flags |= IrqFlags1::RX_READY;
    } else if (mode == Mode::TX) {
        flags |= IrqFlags1::TX_READY;
    }
    return flags;
}

U8 Rfm69SimModel::irqFlags2() const {
    U8 flags = 0;
    const FwSizeType available = this->m_fifoCount - this->m_fifoReadIndex;
    if (available > 0) {
        flags |= IrqFlags2::FIFO_NOT_EMPTY;
    }
    if (available >= FIFO_SIZE) {
        flags |= IrqFlags2::FIFO_FULL;
    }
    if (this->m_fifoOverrun) {
        flags |= IrqFlags2::FIFO_OVERRUN;
    }
    if (this->m_packetSent) {
        flags |= IrqFlags2::PACKET_SENT;
    }
    if (this->m_payloadReady) {
        flags |= IrqFlags2::PAYLOAD_READY;
    }
    return flags;
}

}  // namespace Rfm69
