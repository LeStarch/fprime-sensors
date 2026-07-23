// ======================================================================
// \title  Rfm69Helpers.cpp
// \brief  RFM69 radio-specific helper functions for the Rfm69Manager
//
// SPI register access and packet handling per the RFM69HCW datasheet
// (HopeRF, V1.1). Keeps the radio specifics out of the F Prime handler
// code in Rfm69Manager.cpp.
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Manager.hpp"

namespace Rfm69 {

// ----------------------------------------------------------------------
// Register access primitives
// ----------------------------------------------------------------------

Drv::SpiStatus Rfm69Manager ::readRegister(U8 address, U8& value) {
    // Single access read: address byte (wnr=0) then one dummy byte
    this->m_mosi[0] = address & SPI_ADDRESS_MASK;
    this->m_mosi[1] = 0;
    Fw::Buffer writeBuffer(this->m_mosi, 2);
    Fw::Buffer readBuffer(this->m_miso, 2);
    const Drv::SpiStatus status = this->spiWriteRead_out(0, writeBuffer, readBuffer);
    value = this->m_miso[1];
    return status;
}

Drv::SpiStatus Rfm69Manager ::writeRegister(U8 address, U8 value) {
    // Single access write: address byte with wnr=1 then the data byte
    this->m_mosi[0] = (address & SPI_ADDRESS_MASK) | SPI_WRITE_FLAG;
    this->m_mosi[1] = value;
    Fw::Buffer writeBuffer(this->m_mosi, 2);
    Fw::Buffer readBuffer(this->m_miso, 2);
    return this->spiWriteRead_out(0, writeBuffer, readBuffer);
}

// ----------------------------------------------------------------------
// Radio management
// ----------------------------------------------------------------------

bool Rfm69Manager ::detectRadio() {
    U8 version = 0;
    const Drv::SpiStatus status = this->readRegister(Reg::VERSION, version);
    return (status == Drv::SpiStatus::SPI_OK) && (version == VERSION_VALUE);
}

bool Rfm69Manager ::configureRadio() {
    // Frf register value: frequency / (32 MHz / 2^19) (datasheet section 4.2.4)
    const U32 frf = static_cast<U32>((static_cast<U64>(this->m_frequencyHz) * FRF_DIVISOR) / CRYSTAL_HZ);
    // Bit rate 55.555 kb/s and 50 kHz deviation: common RFM69 FSK settings
    constexpr U16 BITRATE = 0x0240;  // 32 MHz / 55555 bps
    constexpr U16 FDEV = 0x0333;     // 50 kHz / 61 Hz Fstep
    const struct {
        U8 address;
        U8 value;
    } configuration[] = {
        {Reg::OP_MODE, Mode::STANDBY},  // Sequencer on, standby
        {Reg::DATA_MODUL, 0x00},        // Packet mode, FSK, no shaping
        {Reg::BITRATE_MSB, static_cast<U8>(BITRATE >> 8)},
        {Reg::BITRATE_LSB, static_cast<U8>(BITRATE & 0xFF)},
        {Reg::FDEV_MSB, static_cast<U8>(FDEV >> 8)},
        {Reg::FDEV_LSB, static_cast<U8>(FDEV & 0xFF)},
        {Reg::FRF_MSB, static_cast<U8>(frf >> 16)},
        {Reg::FRF_MID, static_cast<U8>(frf >> 8)},
        {Reg::FRF_LSB, static_cast<U8>(frf & 0xFF)},
        // RFM69HCW: PA1 on (PA0 is not connected on this module)
        {Reg::PA_LEVEL, static_cast<U8>(0x40 | (this->m_powerLevel & 0x1F))},
        {Reg::RX_BW, 0x55},                                 // Recommended default (Table 23)
        {Reg::DIO_MAPPING_2, 0x07},                         // CLKOUT off
        {Reg::RSSI_THRESH, 0xE4},                           // Recommended default
        {Reg::SYNC_CONFIG, 0x88},                           // Sync on, 2 sync bytes
        {Reg::SYNC_VALUE_1, 0x2D},                          // Fixed first sync byte
        {Reg::SYNC_VALUE_2, this->m_networkId},             // Network ID
        {Reg::PACKET_CONFIG_1, 0x90},         // Variable length, CRC on
        {Reg::PAYLOAD_LENGTH, 0xFF},          // Max RX length: full 255-byte packets
        {Reg::FIFO_THRESH, static_cast<U8>(0x80 | FIFO_THRESHOLD)},  // TX start on FifoNotEmpty
        {Reg::PACKET_CONFIG_2, 0x02},                       // Auto RX restart
        {Reg::TEST_DAGC, 0x30},                             // Recommended default
    };
    for (FwSizeType i = 0; i < FW_NUM_ARRAY_ELEMENTS(configuration); i++) {
        if (this->writeRegister(configuration[i].address, configuration[i].value) != Drv::SpiStatus::SPI_OK) {
            return false;
        }
    }
    // Clear any FIFO residue from before configuration
    return this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN) == Drv::SpiStatus::SPI_OK;
}

bool Rfm69Manager ::setMode(U8 mode) {
    U8 opMode = 0;
    if (this->readRegister(Reg::OP_MODE, opMode) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    opMode = static_cast<U8>((opMode & static_cast<U8>(~Mode::MASK)) | (mode & Mode::MASK));
    if (this->writeRegister(Reg::OP_MODE, opMode) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    // Bounded poll for ModeReady (datasheet section 6, RegIrqFlags1)
    for (U32 i = 0; i < TX_POLL_LIMIT; i++) {
        U8 flags = 0;
        if (this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) {
            return false;
        }
        if ((flags & IrqFlags1::MODE_READY) != 0) {
            return true;
        }
    }
    return false;
}

bool Rfm69Manager ::channelBusy() {
    // Listen-before-talk: SyncAddressMatch indicates the radio is actively
    // clocking in a packet (datasheet section 6, RegIrqFlags1)
    U8 flags = 0;
    if (this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    return (flags & IrqFlags1::SYNC_ADDRESS_MATCH) != 0;
}

// ----------------------------------------------------------------------
// Packet handling
// ----------------------------------------------------------------------

bool Rfm69Manager ::writeFifo(const U8* data, FwSizeType size) {
    FW_ASSERT(data != nullptr);
    FW_ASSERT((size + 1) <= sizeof this->m_mosi, static_cast<FwAssertArgType>(size));
    this->m_mosi[0] = Reg::FIFO | SPI_WRITE_FLAG;
    (void)::memcpy(&this->m_mosi[1], data, size);
    Fw::Buffer writeBuffer(this->m_mosi, size + 1);
    Fw::Buffer readBuffer(this->m_miso, size + 1);
    return this->spiWriteRead_out(0, writeBuffer, readBuffer) == Drv::SpiStatus::SPI_OK;
}

bool Rfm69Manager ::transmitPacket(const U8* data, FwSizeType size) {
    FW_ASSERT(data != nullptr);
    FW_ASSERT(size <= MAX_PACKET_PAYLOAD, static_cast<FwAssertArgType>(size));
    // Load the initial FIFO fill while in standby: length byte then payload.
    // Packets larger than the FIFO are streamed: the remainder is topped up
    // during transmission whenever FifoLevel drops below the threshold
    // (datasheet section 5.2.2.3)
    if (!this->setMode(Mode::STANDBY)) {
        return false;
    }
    if (this->writeRegister(Reg::FIFO, static_cast<U8>(size)) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    FwSizeType offset = FW_MIN(size, FIFO_SIZE - 1);
    if (!this->writeFifo(data, offset)) {
        return false;
    }
    // Transmit, topping up the FIFO and awaiting PacketSent with a bounded poll
    if (!this->setMode(Mode::TX)) {
        return false;
    }
    bool sent = false;
    bool spiOk = true;
    for (U32 i = 0; (i < TX_POLL_LIMIT) && !sent && spiOk; i++) {
        U8 flags = 0;
        if (this->readRegister(Reg::IRQ_FLAGS_2, flags) != Drv::SpiStatus::SPI_OK) {
            spiOk = false;
            break;
        }
        if ((offset < size) && ((flags & IrqFlags2::FIFO_LEVEL) == 0)) {
            const FwSizeType chunk = FW_MIN(size - offset, TX_TOP_UP_CHUNK);
            spiOk = this->writeFifo(&data[offset], chunk);
            offset += chunk;
        } else if (offset >= size) {
            sent = (flags & IrqFlags2::PACKET_SENT) != 0;
        }
    }
    // Always return to receive so uplink data is not lost
    const bool rxOk = this->setMode(Mode::RX);
    if (sent && rxOk) {
        this->m_packetsTransmitted++;
        this->tlmWrite_PacketsTransmitted(this->m_packetsTransmitted);
    }
    return sent && rxOk;
}

FwSizeType Rfm69Manager ::readReceivedPacket(U8* data, FwSizeType capacity) {
    FW_ASSERT(data != nullptr);
    // Remain in RX and drain the FIFO as the packet streams in: packets
    // larger than the FIFO are read out while reception continues
    // (datasheet section 5.2.2.3)
    U8 length = 0;
    if (this->readRegister(Reg::FIFO, length) != Drv::SpiStatus::SPI_OK) {
        return 0;
    }
    if ((length == 0) || (static_cast<FwSizeType>(length) > capacity)) {
        // Invalid length: clear the FIFO to resynchronize
        (void)this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN);
        return 0;
    }
    FwSizeType received = 0;
    for (U32 i = 0; (i < RX_POLL_LIMIT) && (received < static_cast<FwSizeType>(length)); i++) {
        U8 flags = 0;
        if (this->readRegister(Reg::IRQ_FLAGS_2, flags) != Drv::SpiStatus::SPI_OK) {
            return 0;
        }
        if ((flags & IrqFlags2::FIFO_NOT_EMPTY) != 0) {
            U8 value = 0;
            if (this->readRegister(Reg::FIFO, value) != Drv::SpiStatus::SPI_OK) {
                return 0;
            }
            data[received++] = value;
        }
    }
    if (received < static_cast<FwSizeType>(length)) {
        // Reception stalled: clear the FIFO to resynchronize
        (void)this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN);
        return 0;
    }
    return received;
}

void Rfm69Manager ::updateRssi() {
    U8 rssi = 0;
    if (this->readRegister(Reg::RSSI_VALUE, rssi) == Drv::SpiStatus::SPI_OK) {
        // RSSI (dBm) = -RssiValue / 2 (datasheet section 6, RegRssiValue)
        this->tlmWrite_LastRssi(-static_cast<F32>(rssi) / 2.0f);
    }
}

}  // namespace Rfm69
