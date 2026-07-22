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
        {Reg::PACKET_CONFIG_1, 0x90},                       // Variable length, CRC on
        {Reg::PAYLOAD_LENGTH, static_cast<U8>(FIFO_SIZE)},  // Max RX length
        {Reg::FIFO_THRESH, 0x8F},                           // TX start on FifoNotEmpty
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

// ----------------------------------------------------------------------
// Packet handling
// ----------------------------------------------------------------------

bool Rfm69Manager ::transmitPacket(const U8* data, FwSizeType size) {
    FW_ASSERT(data != nullptr);
    FW_ASSERT(size <= MAX_PACKET_PAYLOAD, static_cast<FwAssertArgType>(size));
    // Load the FIFO while in standby: length byte then payload (variable-length mode)
    if (!this->setMode(Mode::STANDBY)) {
        return false;
    }
    this->m_mosi[0] = Reg::FIFO | SPI_WRITE_FLAG;
    this->m_mosi[1] = static_cast<U8>(size);
    (void)::memcpy(&this->m_mosi[2], data, size);
    Fw::Buffer writeBuffer(this->m_mosi, size + 2);
    Fw::Buffer readBuffer(this->m_miso, size + 2);
    if (this->spiWriteRead_out(0, writeBuffer, readBuffer) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    // Transmit and await PacketSent with a bounded poll
    if (!this->setMode(Mode::TX)) {
        return false;
    }
    bool sent = false;
    for (U32 i = 0; (i < TX_POLL_LIMIT) && !sent; i++) {
        U8 flags = 0;
        if (this->readRegister(Reg::IRQ_FLAGS_2, flags) != Drv::SpiStatus::SPI_OK) {
            break;
        }
        sent = (flags & IrqFlags2::PACKET_SENT) != 0;
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
    // Hold the radio in standby while draining the FIFO
    if (!this->setMode(Mode::STANDBY)) {
        return 0;
    }
    FwSizeType size = 0;
    U8 length = 0;
    if (this->readRegister(Reg::FIFO, length) == Drv::SpiStatus::SPI_OK) {
        if ((length > 0) && (static_cast<FwSizeType>(length) <= capacity) &&
            (static_cast<FwSizeType>(length) < FIFO_SIZE)) {
            // Burst read the payload out of the FIFO
            (void)::memset(this->m_mosi, 0, static_cast<FwSizeType>(length) + 1);
            this->m_mosi[0] = Reg::FIFO;
            Fw::Buffer writeBuffer(this->m_mosi, static_cast<FwSizeType>(length) + 1);
            Fw::Buffer readBuffer(this->m_miso, static_cast<FwSizeType>(length) + 1);
            if (this->spiWriteRead_out(0, writeBuffer, readBuffer) == Drv::SpiStatus::SPI_OK) {
                (void)::memcpy(data, &this->m_miso[1], length);
                size = length;
            }
        } else {
            // Invalid length: clear the FIFO to resynchronize
            (void)this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN);
        }
    }
    (void)this->setMode(Mode::RX);
    return size;
}

void Rfm69Manager ::updateRssi() {
    U8 rssi = 0;
    if (this->readRegister(Reg::RSSI_VALUE, rssi) == Drv::SpiStatus::SPI_OK) {
        // RSSI (dBm) = -RssiValue / 2 (datasheet section 6, RegRssiValue)
        this->tlmWrite_LastRssi(-static_cast<F32>(rssi) / 2.0f);
    }
}

}  // namespace Rfm69
