// ======================================================================
// \title  Rfm69SimModel.cpp
// \brief  Register-level model of an RFM69HCW radio
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Sim/Rfm69SimModel.hpp"
#include <cstring>

namespace Rfm69 {

Rfm69SimModel::Rfm69SimModel()
    : m_registerWriteCount(0),
      m_fifoCount(0),
      m_packetSent(false),
      m_payloadReady(false),
      m_fifoOverrun(false),
      m_stallPacketSent(false),
      m_rxRestartPending(false),
      m_txActive(false),
      m_txExpected(0),
      m_txCollected(0),
      m_rxActive(false),
      m_rxTotal(0),
      m_rxDelivered(0),
      m_rxCorrupt(false),
      m_airCount(0),
      m_txCount(0) {
    this->reset();
}

void Rfm69SimModel::reset() {
    (void)::memset(this->m_registers, 0, sizeof this->m_registers);
    (void)::memset(this->m_registerWriteAddresses, 0, sizeof this->m_registerWriteAddresses);
    (void)::memset(this->m_registerWriteValues, 0, sizeof this->m_registerWriteValues);
    (void)::memset(this->m_fifo, 0, sizeof this->m_fifo);
    (void)::memset(this->m_txAccum, 0, sizeof this->m_txAccum);
    (void)::memset(this->m_rxPacket, 0, sizeof this->m_rxPacket);
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
    this->m_registerWriteCount = 0;
    this->m_packetSent = false;
    this->m_payloadReady = false;
    this->m_fifoOverrun = false;
    this->m_stallPacketSent = false;
    this->m_rxRestartPending = false;
    this->m_txActive = false;
    this->m_txExpected = 0;
    this->m_txCollected = 0;
    this->m_rxActive = false;
    this->m_rxTotal = 0;
    this->m_rxDelivered = 0;
    this->m_rxCorrupt = false;
    this->m_airCount = 0;
    this->m_txCount = 0;
}

void Rfm69SimModel::spiTransaction(const U8* mosi, U8* miso, FwSizeType size) {
    if ((mosi == nullptr) || (miso == nullptr) || (size == 0)) {
        return;
    }
    // Each SPI byte exchanged advances the modeled air interface one byte
    // time, pacing transmission/reception against the manager's polling
    this->advanceClock(size);
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
    this->startReceive();
}

void Rfm69SimModel::injectCorruptFrame(U8 payloadLen) {
    // Only stage when the radio is idle in RX (mirrors startReceive() guards).
    if (((this->m_registers[Reg::OP_MODE] & Mode::MASK) != Mode::RX) || this->m_rxActive ||
        this->m_payloadReady || this->m_rxRestartPending || (this->m_fifoCount != 0)) {
        return;
    }
    // Cap so length byte + payload + trailing noise fits m_rxPacket.
    if (payloadLen > (MAX_PACKET_PAYLOAD - 40)) {
        payloadLen = static_cast<U8>(MAX_PACKET_PAYLOAD - 40);
    }
    // Trailing noise keeps FifoLevel asserted through the manager's final read so
    // the declared byte count completes without ever waiting on PayloadReady.
    const FwSizeType noise = 32;
    this->m_rxPacket[0] = payloadLen;
    for (FwSizeType i = 1; i <= static_cast<FwSizeType>(payloadLen) + noise; i++) {
        this->m_rxPacket[i] = static_cast<U8>(0xA5 ^ i);
    }
    this->m_rxTotal = static_cast<FwSizeType>(payloadLen) + 1 + noise;
    this->m_rxDelivered = 0;
    this->m_rxActive = true;
    this->m_rxCorrupt = true;
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

U8 Rfm69SimModel::readRegisterValue(U8 address) const {
    return this->m_registers[address & SPI_ADDRESS_MASK];
}

void Rfm69SimModel::setPacketSentStall(bool stall) {
    this->m_stallPacketSent = stall;
}

void Rfm69SimModel::clearRegisterWriteHistory() {
    this->m_registerWriteCount = 0;
}

bool Rfm69SimModel::wasRegisterWritten(U8 address, U8 value) const {
    const U8 normalizedAddress = address & SPI_ADDRESS_MASK;
    for (FwSizeType i = 0; i < this->m_registerWriteCount; i++) {
        if ((this->m_registerWriteAddresses[i] == normalizedAddress) && (this->m_registerWriteValues[i] == value)) {
            return true;
        }
    }
    return false;
}

void Rfm69SimModel::advanceClock(FwSizeType byteTimes) {
    const U8 mode = this->m_registers[Reg::OP_MODE] & Mode::MASK;
    for (FwSizeType t = 0; t < byteTimes; t++) {
        if (mode == Mode::TX) {
            if (!this->m_txActive && (this->m_fifoCount > 0)) {
                this->startTransmit();
            }
            // A stalled PacketSent leaves the completed TX active until the
            // manager changes mode. Do not consume or write beyond m_txAccum
            // after its expected payload has already been collected.
            if (this->m_txActive && (this->m_txCollected < this->m_txExpected) && (this->m_fifoCount > 0)) {
                this->m_txAccum[this->m_txCollected] = this->fifoPop();
                this->m_txCollected++;
                if (this->m_txCollected >= this->m_txExpected) {
                    if (this->m_stallPacketSent) {
                        // Leave TX active and PacketSent clear until the
                        // manager's timeout returns the device to RX.
                        continue;
                    }
                    // Packet fully clocked out over the air
                    if (this->m_txCount < TX_QUEUE_DEPTH) {
                        (void)::memcpy(this->m_txQueue[this->m_txCount], this->m_txAccum, this->m_txExpected);
                        this->m_txSizes[this->m_txCount] = this->m_txExpected;
                        this->m_txCount++;
                    }
                    this->m_txActive = false;
                    this->m_packetSent = true;
                }
            }
        } else if (mode == Mode::RX) {
            if (!this->m_rxActive && !this->m_rxRestartPending) {
                this->startReceive();
            }
            if (this->m_rxActive && (this->m_fifoCount < FIFO_SIZE)) {
                this->fifoPush(this->m_rxPacket[this->m_rxDelivered]);
                this->m_rxDelivered++;
                if (this->m_rxDelivered >= this->m_rxTotal) {
                    this->m_rxActive = false;
                    // A CRC-failing frame never asserts PayloadReady (the RFM69
                    // auto-clears on CRC failure). Leave the streamed bytes in
                    // the FIFO so FifoLevel stays asserted through the final read,
                    // exercising the manager's "completed without CRC" drop path.
                    this->m_payloadReady = !this->m_rxCorrupt;
                }
            }
        }
    }
}

U8 Rfm69SimModel::readRegister(U8 address) {
    U8 value = 0;
    switch (address) {
        case Reg::FIFO:
            if (this->m_fifoCount > 0) {
                value = this->fifoPop();
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
    const U8 normalizedAddress = address & SPI_ADDRESS_MASK;
    if ((normalizedAddress != Reg::FIFO) && (this->m_registerWriteCount < REGISTER_WRITE_HISTORY_SIZE)) {
        this->m_registerWriteAddresses[this->m_registerWriteCount] = normalizedAddress;
        this->m_registerWriteValues[this->m_registerWriteCount] = value;
        this->m_registerWriteCount++;
    }
    switch (address) {
        case Reg::FIFO:
            this->fifoPush(value);
            break;
        case Reg::IRQ_FLAGS_2:
            // Writing FifoOverrun=1 clears the flag and the FIFO
            if ((value & IrqFlags2::FIFO_OVERRUN) != 0) {
                this->m_fifoOverrun = false;
                this->m_fifoCount = 0;
                this->m_payloadReady = false;
                this->m_rxActive = false;
                // The manager follows this FIFO flush with RxRestart. Do not
                // begin consuming a queued packet in the SPI transaction
                // between those two writes.
                this->m_rxRestartPending = true;
            }
            break;
        case Reg::PACKET_CONFIG_2:
            // RxRestart (bit 2) is a command bit: it self-clears, flushes
            // stale receive state, and makes a queued air packet eligible for
            // a fresh receive cycle. AutoRxRestartOn (bit 1) remains set.
            this->m_registers[Reg::PACKET_CONFIG_2] = static_cast<U8>(value & static_cast<U8>(~0x04));
            if ((value & 0x04) != 0) {
                this->m_fifoCount = 0;
                this->m_payloadReady = false;
                this->m_fifoOverrun = false;
                this->m_rxActive = false;
                this->m_rxRestartPending = false;
                this->startReceive();
            }
            break;
        case Reg::OP_MODE: {
            const U8 previous = this->m_registers[Reg::OP_MODE] & Mode::MASK;
            this->m_registers[Reg::OP_MODE] = value;
            if ((value & Mode::MASK) != previous) {
                this->handleModeChange(value & Mode::MASK);
            }
            break;
        }
        default:
            this->m_registers[address & SPI_ADDRESS_MASK] = value;
            break;
    }
}

void Rfm69SimModel::handleModeChange(U8 mode) {
    if (mode == Mode::TX) {
        // PayloadReady is an RX flag; leaving RX abandons any reception
        this->m_payloadReady = false;
        this->m_rxActive = false;
    } else if (mode == Mode::RX) {
        // PacketSent clears when exiting TX; the FIFO clears on RX entry
        this->m_packetSent = false;
        this->m_txActive = false;
        this->m_fifoCount = 0;
        this->m_rxRestartPending = false;
        this->startReceive();
    } else {
        this->m_packetSent = false;
        this->m_txActive = false;
        this->m_rxActive = false;
    }
}

void Rfm69SimModel::fifoPush(U8 value) {
    if (this->m_fifoCount < FIFO_SIZE) {
        this->m_fifo[this->m_fifoCount] = value;
        this->m_fifoCount++;
    } else {
        // Overrun: flag set, data lost (datasheet section 5.2.2.3)
        this->m_fifoOverrun = true;
    }
}

U8 Rfm69SimModel::fifoPop() {
    U8 value = 0;
    if (this->m_fifoCount > 0) {
        value = this->m_fifo[0];
        this->m_fifoCount--;
        (void)::memmove(this->m_fifo, &this->m_fifo[1], this->m_fifoCount);
    }
    return value;
}

void Rfm69SimModel::startTransmit() {
    // Variable-length packet: first FIFO byte is the payload length
    const FwSizeType length = this->fifoPop();
    if (length == 0) {
        // Zero-length packet: nothing on the air, but the packet completes
        this->m_txExpected = 0;
        this->m_txCollected = 0;
        this->m_txActive = this->m_stallPacketSent;
        this->m_packetSent = !this->m_stallPacketSent;
        return;
    }
    this->m_txExpected = FW_MIN(length, sizeof this->m_txAccum);
    this->m_txCollected = 0;
    this->m_txActive = true;
}

void Rfm69SimModel::startReceive() {
    // Begin delivery only when in RX, idle, with a drained FIFO
    if (((this->m_registers[Reg::OP_MODE] & Mode::MASK) != Mode::RX) || this->m_rxActive || this->m_payloadReady ||
        this->m_rxRestartPending || (this->m_fifoCount != 0) || (this->m_airCount == 0)) {
        return;
    }
    const FwSizeType payloadSize = FW_MIN(this->m_airCount, MAX_PACKET_PAYLOAD);
    this->m_rxPacket[0] = static_cast<U8>(payloadSize);
    (void)::memcpy(&this->m_rxPacket[1], this->m_airBuffer, payloadSize);
    this->m_rxTotal = payloadSize + 1;
    this->m_rxDelivered = 0;
    this->m_rxActive = true;
    this->m_rxCorrupt = false;
    (void)::memmove(this->m_airBuffer, &this->m_airBuffer[payloadSize], this->m_airCount - payloadSize);
    this->m_airCount -= payloadSize;
}

U8 Rfm69SimModel::irqFlags1() const {
    // The simulated radio settles instantaneously: ModeReady is always set
    U8 flags = IrqFlags1::MODE_READY;
    const U8 mode = this->m_registers[Reg::OP_MODE] & Mode::MASK;
    if (mode == Mode::RX) {
        flags |= IrqFlags1::RX_READY;
        // A reception in progress asserts SyncAddressMatch and Rssi until
        // the payload is delivered and drained
        if (this->m_rxActive || this->m_payloadReady || (this->m_fifoCount > 0)) {
            flags |= IrqFlags1::SYNC_ADDRESS_MATCH | IrqFlags1::RSSI;
        }
    } else if (mode == Mode::TX) {
        flags |= IrqFlags1::TX_READY;
    }
    return flags;
}

U8 Rfm69SimModel::irqFlags2() const {
    U8 flags = 0;
    if (this->m_fifoCount > 0) {
        flags |= IrqFlags2::FIFO_NOT_EMPTY;
    }
    if (this->m_fifoCount > (this->m_registers[Reg::FIFO_THRESH] & 0x7F)) {
        flags |= IrqFlags2::FIFO_LEVEL;
    }
    if (this->m_fifoCount >= FIFO_SIZE) {
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
