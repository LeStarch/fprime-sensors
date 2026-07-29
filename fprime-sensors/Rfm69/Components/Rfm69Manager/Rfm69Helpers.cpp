// ======================================================================
// \title  Rfm69Helpers.cpp
// \brief  RFM69 radio-specific helper functions for the Rfm69Manager
//
// SPI register access and packet handling per the RFM69HCW datasheet
// (HopeRF, V1.1). Keeps the radio specifics out of the F Prime handler
// code in Rfm69Manager.cpp.
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Manager.hpp"
#include <cstring>

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
    // Read operator params at configure time (same pattern as LoRa enableRx/Tx).
    // All other modem registers come from NATIVE_PACKET_PROFILE.
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    const Rfm69DataRate dataRateParam = this->paramGet_DATA_RATE(isValid);
    FW_ASSERT((isValid == Fw::ParamValid::VALID) || (isValid == Fw::ParamValid::DEFAULT),
              static_cast<FwAssertArgType>(isValid));
    const Rfm69Bandwidth bandwidthParam = this->paramGet_BANDWIDTH_RX(isValid);
    FW_ASSERT((isValid == Fw::ParamValid::VALID) || (isValid == Fw::ParamValid::DEFAULT),
              static_cast<FwAssertArgType>(isValid));
    const Rfm69TxPower txPowerParam = this->paramGet_TX_POWER(isValid);
    FW_ASSERT((isValid == Fw::ParamValid::VALID) || (isValid == Fw::ParamValid::DEFAULT),
              static_cast<FwAssertArgType>(isValid));

    DataRateSetting dataRate{};
    BandwidthSetting bandwidth{};
    TxPowerSetting power{};
    if (!getDataRateSetting(dataRateParam, dataRate) || !getBandwidthSetting(bandwidthParam, bandwidth) ||
        !getTxPowerSetting(txPowerParam, power)) {
        return false;
    }

    // Frf register value: frequency / (32 MHz / 2^19) (datasheet section 4.2.4)
    const U32 frf = static_cast<U32>((static_cast<U64>(FIXED_FREQUENCY_HZ) * FRF_DIVISOR) / CRYSTAL_HZ);
    const PacketProfile& packet = NATIVE_PACKET_PROFILE;
    const struct {
        U8 address;
        U8 value;
    } configuration[] = {
        {Reg::OP_MODE, Mode::STANDBY},  // Sequencer on, standby
        {Reg::DATA_MODUL, FIXED_DATA_MODUL},  // Packet FSK, no shaping
        {Reg::BITRATE_MSB, static_cast<U8>(dataRate.bitrateReg >> 8)},
        {Reg::BITRATE_LSB, static_cast<U8>(dataRate.bitrateReg & 0xFF)},
        {Reg::FDEV_MSB, static_cast<U8>(FIXED_FDEV_REGISTER >> 8)},
        {Reg::FDEV_LSB, static_cast<U8>(FIXED_FDEV_REGISTER & 0xFF)},
        {Reg::FRF_MSB, static_cast<U8>(frf >> 16)},
        {Reg::FRF_MID, static_cast<U8>(frf >> 8)},
        {Reg::FRF_LSB, static_cast<U8>(frf & 0xFF)},
        {Reg::PA_LEVEL, power.paLevel},  // HCW PA1/PA2 selection and output
        {Reg::OCP, Pa::OCP_NORMAL},
        {Reg::RX_BW, bandwidth.rxBw},
        {Reg::AFC_BW, bandwidth.afcBw},
        {Reg::DIO_MAPPING_2, 0x07},  // CLKOUT off
        {Reg::RSSI_THRESH, 0xE4},    // Recommended default
        {Reg::PREAMBLE_MSB, static_cast<U8>(packet.preambleBytes >> 8)},
        {Reg::PREAMBLE_LSB, static_cast<U8>(packet.preambleBytes & 0xFF)},
        {Reg::SYNC_CONFIG, 0xB8},  // Sync on, 8 sync bytes
        {Reg::SYNC_VALUE_1, packet.sync[0]},
        {Reg::SYNC_VALUE_2, packet.sync[1]},
        {Reg::SYNC_VALUE_3, packet.sync[2]},
        {Reg::SYNC_VALUE_4, packet.sync[3]},
        {Reg::SYNC_VALUE_5, packet.sync[4]},
        {Reg::SYNC_VALUE_6, packet.sync[5]},
        {Reg::SYNC_VALUE_7, packet.sync[6]},
        {Reg::SYNC_VALUE_8, packet.sync[7]},
        {Reg::PACKET_CONFIG_1, packet.packetConfig1},  // Variable length, whitening, CRC
        {Reg::PAYLOAD_LENGTH, static_cast<U8>(MAX_PACKET_PAYLOAD)},
        {Reg::FIFO_THRESH, static_cast<U8>(0x80 | packet.fifoThreshold)},
        {Reg::PACKET_CONFIG_2, packet.packetConfig2},
        {Reg::TEST_PA_1, Pa::TEST_PA_1_NORMAL},
        {Reg::TEST_PA_2, Pa::TEST_PA_2_NORMAL},
        {Reg::TEST_DAGC, packet.testDagc},  // Recommended default
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

bool Rfm69Manager ::recoverReceive() {
    // Clear FIFO overrun and re-arm RX (AutoRxRestartOn | RxRestart → 0x06).
    const bool fifoCleared =
        this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN) == Drv::SpiStatus::SPI_OK;
    const bool restarted = this->writeRegister(Reg::PACKET_CONFIG_2,
                                                static_cast<U8>(PacketConfig2::AUTO_RX_RESTART_ON |
                                                                PacketConfig2::RX_RESTART)) == Drv::SpiStatus::SPI_OK;
    return fifoCleared && restarted;
}

bool Rfm69Manager ::setPowerBoost(bool enabled) {
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    const Rfm69TxPower txPowerParam = this->paramGet_TX_POWER(isValid);
    FW_ASSERT((isValid == Fw::ParamValid::VALID) || (isValid == Fw::ParamValid::DEFAULT),
              static_cast<FwAssertArgType>(isValid));
    TxPowerSetting power{};
    if (!getTxPowerSetting(txPowerParam, power)) {
        return false;
    }
    if (!power.boost20dBm) {
        return !enabled;
    }

    const U8 ocp = enabled ? Pa::OCP_DISABLED : Pa::OCP_NORMAL;
    const U8 testPa1 = enabled ? Pa::TEST_PA_1_BOOST : Pa::TEST_PA_1_NORMAL;
    const U8 testPa2 = enabled ? Pa::TEST_PA_2_BOOST : Pa::TEST_PA_2_NORMAL;
    return (this->writeRegister(Reg::OCP, ocp) == Drv::SpiStatus::SPI_OK) &&
           (this->writeRegister(Reg::TEST_PA_1, testPa1) == Drv::SpiStatus::SPI_OK) &&
           (this->writeRegister(Reg::TEST_PA_2, testPa2) == Drv::SpiStatus::SPI_OK);
}

bool Rfm69Manager ::channelBusy() {
    // SyncAddressMatch: an uplink is being clocked in; TX must drop (half-duplex).
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

bool Rfm69Manager ::readFifo(U8* data, FwSizeType size) {
    FW_ASSERT(data != nullptr);
    FW_ASSERT((size + 1) <= sizeof this->m_mosi, static_cast<FwAssertArgType>(size));
    this->m_mosi[0] = Reg::FIFO;
    (void)::memset(&this->m_mosi[1], 0, size);
    Fw::Buffer writeBuffer(this->m_mosi, size + 1);
    Fw::Buffer readBuffer(this->m_miso, size + 1);
    if (this->spiWriteRead_out(0, writeBuffer, readBuffer) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    (void)::memcpy(data, &this->m_miso[1], size);
    return true;
}

bool Rfm69Manager ::transmitPacket(const U8* data, FwSizeType size) {
    FW_ASSERT(data != nullptr);
    FW_ASSERT(size <= MAX_PACKET_PAYLOAD, static_cast<FwAssertArgType>(size));
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    const Rfm69TxPower txPowerParam = this->paramGet_TX_POWER(isValid);
    FW_ASSERT((isValid == Fw::ParamValid::VALID) || (isValid == Fw::ParamValid::DEFAULT),
              static_cast<FwAssertArgType>(isValid));
    TxPowerSetting power{};
    if (!getTxPowerSetting(txPowerParam, power)) {
        return false;
    }
    // Load length + first FIFO fill in standby; stream the rest during TX
    // (FIFO is 66 bytes; payloads up to 255 need top-ups — datasheet 5.2.2.3).
    if (!this->setMode(Mode::STANDBY)) {
        return false;
    }
    if (this->writeRegister(Reg::FIFO, static_cast<U8>(size)) != Drv::SpiStatus::SPI_OK) {
        (void)this->setMode(Mode::RX);
        (void)this->recoverReceive();
        return false;
    }
    FwSizeType offset = FW_MIN(size, FIFO_SIZE - 1);
    if (!this->writeFifo(data, offset)) {
        (void)this->setMode(Mode::RX);
        (void)this->recoverReceive();
        return false;
    }
    bool boostEnabled = false;
    if (power.boost20dBm) {
        boostEnabled = this->setPowerBoost(true);
        if (!boostEnabled) {
            (void)this->setMode(Mode::RX);
            (void)this->recoverReceive();
            return false;
        }
    }
    // Transmit, topping up the FIFO and awaiting PacketSent with a bounded poll
    if (!this->setMode(Mode::TX)) {
        if (boostEnabled) {
            (void)this->setPowerBoost(false);
        }
        (void)this->setMode(Mode::RX);
        (void)this->recoverReceive();
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
    // Always return to receive so uplink data is not lost. High-power boost
    // is restored only once the PA has left TX.
    const bool rxOk = this->setMode(Mode::RX);
    const bool boostRestored = !boostEnabled || this->setPowerBoost(false);
    const bool recovered = sent ? true : this->recoverReceive();
    if (sent && rxOk) {
        this->m_packetsTransmitted++;
        this->tlmWrite_PacketsTransmitted(this->m_packetsTransmitted);
    }
    return sent && spiOk && rxOk && boostRestored && recovered;
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
        (void)this->recoverReceive();
        return 0;
    }
    FwSizeType received = 0;
    for (U32 i = 0; (i < RX_POLL_LIMIT) && (received < static_cast<FwSizeType>(length)); i++) {
        U8 flags = 0;
        if (this->readRegister(Reg::IRQ_FLAGS_2, flags) != Drv::SpiStatus::SPI_OK) {
            (void)this->recoverReceive();
            return 0;
        }
        if ((flags & IrqFlags2::FIFO_OVERRUN) != 0) {
            (void)this->recoverReceive();
            return 0;
        }

        // Do not single-byte poll a streaming packet. Once FifoLevel crosses
        // the programmed threshold, drain a threshold-sized burst. At
        // PayloadReady, the remaining bytes are all present and can be read
        // in one final burst (including packets smaller than the threshold).
        FwSizeType chunk = 0;
        if ((flags & IrqFlags2::PAYLOAD_READY) != 0) {
            chunk = static_cast<FwSizeType>(length) - received;
        } else if ((flags & IrqFlags2::FIFO_LEVEL) != 0) {
            chunk = FW_MIN(static_cast<FwSizeType>(FIFO_THRESHOLD), static_cast<FwSizeType>(length) - received);
        }
        if (chunk > 0) {
            if (!this->readFifo(&data[received], chunk)) {
                return 0;
            }
            received += chunk;
        }
    }
    if (received < static_cast<FwSizeType>(length)) {
        // Reception stalled: clear the FIFO to resynchronize
        (void)this->recoverReceive();
        return 0;
    }
    // PayloadReady remains asserted after a complete variable-length packet.
    // RegPacketConfig2.RxRestart is the documented receive re-arm command;
    // it self-clears and preserves AutoRxRestartOn (bit 1).
    if (this->writeRegister(Reg::PACKET_CONFIG_2,
                            static_cast<U8>(PacketConfig2::AUTO_RX_RESTART_ON | PacketConfig2::RX_RESTART)) !=
        Drv::SpiStatus::SPI_OK) {
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
