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
    // Parameter validity is ground-managed input: fail into ConfigurationFailed
    // and retry on the next tick rather than asserting.
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    const Rfm69DataRate dataRateParam = this->paramGet_DATA_RATE(isValid);
    if ((isValid != Fw::ParamValid::VALID) && (isValid != Fw::ParamValid::DEFAULT)) {
        return false;
    }
    const Rfm69Bandwidth bandwidthParam = this->paramGet_BANDWIDTH_RX(isValid);
    if ((isValid != Fw::ParamValid::VALID) && (isValid != Fw::ParamValid::DEFAULT)) {
        return false;
    }
    const Rfm69TxPower txPowerParam = this->paramGet_TX_POWER(isValid);
    if ((isValid != Fw::ParamValid::VALID) && (isValid != Fw::ParamValid::DEFAULT)) {
        return false;
    }

    DataRateSetting dataRate{};
    BandwidthSetting bandwidth{};
    TxPowerSetting power{};
    if (!getDataRateSetting(dataRateParam, dataRate) || !getBandwidthSetting(bandwidthParam, bandwidth) ||
        !getTxPowerSetting(txPowerParam, power)) {
        return false;
    }

    // The manager is driven by a 1 kHz rate group. Scale SPI polling with the
    // air rate: idle polls detect reception well before the 66-byte FIFO can
    // fill, while active polls drain one watermark faster than bytes arrive.
    // This avoids a blocking SPI ioctl on every 1 ms tick and prevents the
    // ActiveRateGroup queue from backing up under normal Linux scheduling jitter.
    this->m_rxActivePollDivisor = 1;
    if (dataRateParam == Rfm69DataRate::BR_38400) {
        this->m_rxIdlePollDivisor = 4;
        this->m_txTimeoutTicks = 150;
    } else if (dataRateParam == Rfm69DataRate::BR_19200) {
        this->m_rxIdlePollDivisor = 8;
        this->m_txTimeoutTicks = 250;
    } else if (dataRateParam == Rfm69DataRate::BR_9600) {
        this->m_rxIdlePollDivisor = 16;
        this->m_txTimeoutTicks = 400;
    } else if (dataRateParam == Rfm69DataRate::BR_4800) {
        this->m_rxIdlePollDivisor = 32;
        this->m_txTimeoutTicks = 750;
    } else {
        this->m_rxIdlePollDivisor = 64;
        this->m_txTimeoutTicks = 3000;
    }
    this->m_rxPollTicks = 0;

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
        {Reg::DIO_MAPPING_2, DIO_MAPPING_2_CLKOUT_OFF},
        {Reg::RSSI_THRESH, RSSI_THRESH_VALUE},
        {Reg::PREAMBLE_MSB, static_cast<U8>(packet.preambleBytes >> 8)},
        {Reg::PREAMBLE_LSB, static_cast<U8>(packet.preambleBytes & 0xFF)},
        {Reg::SYNC_CONFIG, SYNC_CONFIG_VALUE},
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
        {Reg::FIFO_THRESH, static_cast<U8>(TX_START_FIFO_NOT_EMPTY | packet.fifoThreshold)},
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
    if (!this->requestMode(mode)) {
        return false;
    }
    // Bounded poll for ModeReady (datasheet section 6, RegIrqFlags1).
    // Runtime TX uses the scheduler-driven state machine below instead; this
    // synchronous helper is limited to startup and exceptional RX recovery.
    for (U32 i = 0; i < MODE_READY_TIMEOUT_TICKS; i++) {
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

bool Rfm69Manager ::requestMode(U8 mode) {
    U8 opMode = 0;
    if (this->readRegister(Reg::OP_MODE, opMode) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    opMode = static_cast<U8>((opMode & static_cast<U8>(~Mode::MASK)) | (mode & Mode::MASK));
    if (this->writeRegister(Reg::OP_MODE, opMode) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    return true;
}

bool Rfm69Manager ::recoverReceive() {
    // Flush FIFO (FifoOverrun clear), then stand by and re-enter RX so
    // SyncAddressMatch cannot stick with a single leftover FIFO byte.
    const bool fifoCleared =
        this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN) == Drv::SpiStatus::SPI_OK;
    const bool standby = this->setMode(Mode::STANDBY);
    const bool rx = this->setMode(Mode::RX);
    const bool restarted = this->writeRegister(Reg::PACKET_CONFIG_2,
                                                static_cast<U8>(PacketConfig2::AUTO_RX_RESTART_ON |
                                                                PacketConfig2::RX_RESTART)) == Drv::SpiStatus::SPI_OK;
    this->m_rxStaleSyncTicks = 0;
    return fifoCleared && standby && rx && restarted;
}

bool Rfm69Manager ::setPowerBoost(bool enabled) {
    if (enabled) {
        // Enable is only supported for the +20 dBm setting.
        Fw::ParamValid isValid = Fw::ParamValid::INVALID;
        const Rfm69TxPower txPowerParam = this->paramGet_TX_POWER(isValid);
        if ((isValid != Fw::ParamValid::VALID) && (isValid != Fw::ParamValid::DEFAULT)) {
            return false;
        }
        TxPowerSetting power{};
        if (!getTxPowerSetting(txPowerParam, power)) {
            return false;
        }
        if (!power.boost20dBm) {
            return false;
        }
    }
    // Disable always restores the safe OCP/TestPa values unconditionally: the
    // TX_POWER parameter may have changed while a boosted transmit was in
    // flight, and boost registers must never remain active in RX.
    const U8 ocp = enabled ? Pa::OCP_DISABLED : Pa::OCP_NORMAL;
    const U8 testPa1 = enabled ? Pa::TEST_PA_1_BOOST : Pa::TEST_PA_1_NORMAL;
    const U8 testPa2 = enabled ? Pa::TEST_PA_2_BOOST : Pa::TEST_PA_2_NORMAL;
    return (this->writeRegister(Reg::OCP, ocp) == Drv::SpiStatus::SPI_OK) &&
           (this->writeRegister(Reg::TEST_PA_1, testPa1) == Drv::SpiStatus::SPI_OK) &&
           (this->writeRegister(Reg::TEST_PA_2, testPa2) == Drv::SpiStatus::SPI_OK);
}

bool Rfm69Manager ::channelBusy() {
    // Software drain in progress, or SyncAddressMatch: uplink clocking in.
    if (this->m_rxDraining) {
        return true;
    }
    U8 flags = 0;
    if (this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) {
        // A bus fault reads as "clear" so the transmit path is taken and its
        // own SPI failure handling reports RADIO_FAULT instead of silently
        // deferring downlink forever on a dead bus.
        return false;
    }
    return (flags & IrqFlags1::SYNC_ADDRESS_MATCH) != 0;
}

bool Rfm69Manager ::downlinkBlocked() {
    return this->channelBusy() || (this->m_rxTxHoldoffTicks > 0) || (this->m_txTxHoldoffTicks > 0);
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

    // Some SPI transports corrupt long FIFO read transactions (see the
    // RX_FIFO_SPI_CHUNK documentation), so the read is split into short
    // transfers. The loop bound is provable: each iteration transfers at
    // least one byte, so at most MAX_PACKET_PAYLOAD iterations occur.
    FwSizeType offset = 0;
    for (FwSizeType i = 0; (i < MAX_PACKET_PAYLOAD) && (offset < size); i++) {
        const FwSizeType chunk = FW_MIN(size - offset, RX_FIFO_SPI_CHUNK);
        this->m_mosi[0] = Reg::FIFO;
        (void)::memset(&this->m_mosi[1], 0, chunk);
        Fw::Buffer writeBuffer(this->m_mosi, chunk + 1);
        Fw::Buffer readBuffer(this->m_miso, chunk + 1);
        if (this->spiWriteRead_out(0, writeBuffer, readBuffer) != Drv::SpiStatus::SPI_OK) {
            return false;
        }
        (void)::memcpy(data + offset, &this->m_miso[1], chunk);
        offset += chunk;
    }
    return offset == size;
}

void Rfm69Manager ::abortTransmit() {
    this->log_WARNING_HI_SendFailed(Rfm69SendFailure::RADIO_FAULT);
    this->m_txWaitTicks = 0;
    this->m_txState = TX_ABORT_CLEAR_FIFO;
}

Rfm69Manager::TxProgress Rfm69Manager ::advanceTransmit() {
    FW_ASSERT(this->m_txActive);
    const FwSizeType size = this->m_txBuffer.getSize();
    const U8* const data = this->m_txBuffer.getData();
    FW_ASSERT(data != nullptr);
    FW_ASSERT((size > 0) && (size <= MAX_PACKET_PAYLOAD), static_cast<FwAssertArgType>(size));

    U8 flags = 0;
    switch (this->m_txState) {
        case TX_REQUEST_STANDBY:
            if (!this->requestMode(Mode::STANDBY)) {
                this->abortTransmit();
            } else {
                this->m_txWaitTicks = 0;
                this->m_txState = TX_WAIT_STANDBY;
            }
            break;

        case TX_WAIT_STANDBY:
            if (this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) {
                this->abortTransmit();
            } else if ((flags & IrqFlags1::MODE_READY) != 0) {
                this->m_txState = TX_CLEAR_FIFO;
            } else if (++this->m_txWaitTicks >= MODE_READY_TIMEOUT_TICKS) {
                this->abortTransmit();
            }
            break;

        case TX_CLEAR_FIFO:
            if (this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN) != Drv::SpiStatus::SPI_OK) {
                this->abortTransmit();
            } else {
                this->m_txState = TX_WRITE_LENGTH;
            }
            break;

        case TX_WRITE_LENGTH:
            if (this->writeRegister(Reg::FIFO, static_cast<U8>(size)) != Drv::SpiStatus::SPI_OK) {
                this->abortTransmit();
            } else {
                this->m_txState = TX_LOAD_INITIAL;
            }
            break;

        case TX_LOAD_INITIAL: {
            // FIFO is 66 bytes. Variable-length mode consumes one length byte,
            // leaving 65 payload bytes for the initial standby preload.
            const FwSizeType initial = FW_MIN(size, FIFO_SIZE - 1);
            if (!this->writeFifo(data, initial)) {
                this->abortTransmit();
            } else {
                this->m_txOffset = initial;
                this->m_txState = TX_ENABLE_BOOST;
            }
            break;
        }

        case TX_ENABLE_BOOST: {
            Fw::ParamValid isValid = Fw::ParamValid::INVALID;
            const Rfm69TxPower txPowerParam = this->paramGet_TX_POWER(isValid);
            TxPowerSetting power{};
            if (((isValid != Fw::ParamValid::VALID) && (isValid != Fw::ParamValid::DEFAULT)) ||
                !getTxPowerSetting(txPowerParam, power)) {
                this->abortTransmit();
            } else if (power.boost20dBm && !this->setPowerBoost(true)) {
                this->abortTransmit();
            } else {
                this->m_txBoostEnabled = power.boost20dBm;
                this->m_txState = TX_REQUEST_MODE;
            }
            break;
        }

        case TX_REQUEST_MODE:
            if (!this->requestMode(Mode::TX)) {
                this->abortTransmit();
            } else {
                this->m_txWaitTicks = 0;
                this->m_txElapsedTicks = 0;
                this->m_txState = TX_WAIT_MODE;
            }
            break;

        case TX_WAIT_MODE:
            if (this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) {
                this->abortTransmit();
            } else if ((flags & IrqFlags1::MODE_READY) != 0) {
                this->m_txState = TX_STREAM;
            } else if (++this->m_txWaitTicks >= MODE_READY_TIMEOUT_TICKS) {
                this->abortTransmit();
            }
            break;

        case TX_STREAM:
            if (++this->m_txElapsedTicks >= this->m_txTimeoutTicks) {
                this->abortTransmit();
                break;
            }
            if (this->readRegister(Reg::IRQ_FLAGS_2, flags) != Drv::SpiStatus::SPI_OK) {
                this->abortTransmit();
                break;
            }
            // Below FifoThreshold there is room for TX_TOP_UP_CHUNK without
            // risking FifoOverrun. At most one bounded burst is written per tick.
            if ((this->m_txOffset < size) && ((flags & IrqFlags2::FIFO_FULL) == 0) &&
                ((flags & IrqFlags2::FIFO_LEVEL) == 0)) {
                const FwSizeType chunk = FW_MIN(size - this->m_txOffset, TX_TOP_UP_CHUNK);
                if (!this->writeFifo(&data[this->m_txOffset], chunk)) {
                    this->abortTransmit();
                } else {
                    this->m_txOffset += chunk;
                }
            } else if ((this->m_txOffset >= size) && ((flags & IrqFlags2::PACKET_SENT) != 0)) {
                this->m_txState = TX_REQUEST_RX;
            }
            break;

        case TX_REQUEST_RX:
            if (!this->requestMode(Mode::RX)) {
                this->abortTransmit();
            } else {
                this->m_txWaitTicks = 0;
                this->m_txState = TX_WAIT_RX;
            }
            break;

        case TX_WAIT_RX:
            if (this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) {
                this->abortTransmit();
            } else if ((flags & IrqFlags1::MODE_READY) != 0) {
                this->m_txState = TX_DISABLE_BOOST;
            } else if (++this->m_txWaitTicks >= MODE_READY_TIMEOUT_TICKS) {
                this->abortTransmit();
            }
            break;

        case TX_DISABLE_BOOST:
            if (this->m_txBoostEnabled && !this->setPowerBoost(false)) {
                this->abortTransmit();
                break;
            }
            this->m_txBoostEnabled = false;
            this->m_packetsTransmitted++;
            this->tlmWrite_PacketsTransmitted(this->m_packetsTransmitted);
            this->log_WARNING_HI_ConfigurationFailed_ThrottleClear();
            this->log_WARNING_HI_SendFailed_ThrottleClear();
            return TX_SUCCEEDED;

        case TX_ABORT_CLEAR_FIFO:
            (void)this->writeRegister(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN);
            this->m_txState = TX_ABORT_REQUEST_RX;
            break;

        case TX_ABORT_REQUEST_RX:
            this->m_txWaitTicks = 0;
            if (this->requestMode(Mode::RX)) {
                this->m_txState = TX_ABORT_WAIT_RX;
            } else {
                this->m_txState = TX_ABORT_DISABLE_BOOST;
            }
            break;

        case TX_ABORT_WAIT_RX:
            if ((this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) ||
                ((flags & IrqFlags1::MODE_READY) != 0) ||
                (++this->m_txWaitTicks >= MODE_READY_TIMEOUT_TICKS)) {
                this->m_txState = TX_ABORT_DISABLE_BOOST;
            }
            break;

        case TX_ABORT_DISABLE_BOOST:
            if (this->m_txBoostEnabled) {
                (void)this->setPowerBoost(false);
            }
            this->m_txBoostEnabled = false;
            return TX_FAILED;

        case TX_IDLE:
        default:
            FW_ASSERT(false, static_cast<FwAssertArgType>(this->m_txState));
            break;
    }
    return TX_IN_PROGRESS;
}

void Rfm69Manager ::updateRssi() {
    U8 rssi = 0;
    if (this->readRegister(Reg::RSSI_VALUE, rssi) == Drv::SpiStatus::SPI_OK) {
        // RSSI (dBm) = -RssiValue / 2 (datasheet section 6, RegRssiValue)
        this->tlmWrite_LastRssi(-static_cast<F32>(rssi) / 2.0f);
    }
}

}  // namespace Rfm69
