// ======================================================================
// \title  Rfm69SimModelTestMain.cpp
// \brief  Register-level tests proving Rfm69SimModel behaves like the
//         RFM69HCW visible over SPI (datasheet HopeRF V1.1).
//
// The manager unit tests use this model as their radio; these tests
// validate the model itself so manager results are trustworthy.
// ======================================================================

#include <gtest/gtest.h>
#include <cstring>
#include "fprime-sensors/Rfm69/Components/Rfm69Sim/Rfm69SimModel.hpp"

namespace Rfm69 {

class Rfm69SimModelTest : public ::testing::Test {
  protected:
    //! Single-register read as the manager performs it (address, dummy)
    U8 readReg(U8 address) {
        U8 mosi[2] = {static_cast<U8>(address & SPI_ADDRESS_MASK), 0};
        U8 miso[2] = {0xFF, 0xFF};
        this->model.spiTransaction(mosi, miso, sizeof mosi);
        return miso[1];
    }

    //! Single-register write as the manager performs it (address|wnr, value)
    void writeReg(U8 address, U8 value) {
        U8 mosi[2] = {static_cast<U8>((address & SPI_ADDRESS_MASK) | SPI_WRITE_FLAG), value};
        U8 miso[2] = {0, 0};
        this->model.spiTransaction(mosi, miso, sizeof mosi);
    }

    //! Enter a mode and clock past any modeled settling time
    void enterMode(U8 mode, FwSizeType settleClocks = 0) {
        this->writeReg(Reg::OP_MODE, mode);
        for (FwSizeType i = 0; i < settleClocks; i++) {
            (void)this->readReg(Reg::IRQ_FLAGS_1);
        }
    }

    Rfm69SimModel model;
};

// ----------------------------------------------------------------------
// Register file semantics
// ----------------------------------------------------------------------

TEST_F(Rfm69SimModelTest, ResetRegisterDefaults) {
    // Datasheet Table 23 power-on values the manager relies on
    EXPECT_EQ(this->readReg(Reg::VERSION), VERSION_VALUE);
    EXPECT_EQ(this->readReg(Reg::OP_MODE) & Mode::MASK, Mode::STANDBY);
    EXPECT_EQ(this->readReg(Reg::BITRATE_MSB), 0x1A);
    EXPECT_EQ(this->readReg(Reg::BITRATE_LSB), 0x0B);
    EXPECT_EQ(this->readReg(Reg::PA_LEVEL), 0x9F);
    EXPECT_EQ(this->readReg(Reg::RSSI_VALUE), static_cast<U8>(Rfm69SimModel::RSSI_READBACK));
}

TEST_F(Rfm69SimModelTest, SingleWriteReadBack) {
    this->writeReg(Reg::SYNC_VALUE_1, 0xAB);
    EXPECT_EQ(this->readReg(Reg::SYNC_VALUE_1), 0xAB);
    EXPECT_EQ(this->model.readRegisterValue(Reg::SYNC_VALUE_1), 0xAB);
}

TEST_F(Rfm69SimModelTest, BurstWriteAutoIncrements) {
    // Burst access auto-increments the address (datasheet section 5.2.1)
    U8 mosi[4] = {static_cast<U8>(Reg::BITRATE_MSB | SPI_WRITE_FLAG), 0x0D, 0x05, 0x33};
    U8 miso[4] = {0};
    this->model.spiTransaction(mosi, miso, sizeof mosi);
    EXPECT_EQ(this->model.readRegisterValue(Reg::BITRATE_MSB), 0x0D);
    EXPECT_EQ(this->model.readRegisterValue(Reg::BITRATE_LSB), 0x05);
    EXPECT_EQ(this->model.readRegisterValue(Reg::FDEV_MSB), 0x33);
}

TEST_F(Rfm69SimModelTest, BurstReadAutoIncrements) {
    this->writeReg(Reg::FRF_MSB, 0xE4);
    this->writeReg(Reg::FRF_MID, 0xC0);
    this->writeReg(Reg::FRF_LSB, 0x12);
    U8 mosi[4] = {Reg::FRF_MSB, 0, 0, 0};
    U8 miso[4] = {0};
    this->model.spiTransaction(mosi, miso, sizeof mosi);
    EXPECT_EQ(miso[1], 0xE4);
    EXPECT_EQ(miso[2], 0xC0);
    EXPECT_EQ(miso[3], 0x12);
}

TEST_F(Rfm69SimModelTest, RegisterWriteHistory) {
    this->model.clearRegisterWriteHistory();
    this->writeReg(Reg::TEST_PA_1, 0x5D);
    EXPECT_TRUE(this->model.wasRegisterWritten(Reg::TEST_PA_1, 0x5D));
    EXPECT_FALSE(this->model.wasRegisterWritten(Reg::TEST_PA_1, 0x55));
    this->model.clearRegisterWriteHistory();
    EXPECT_FALSE(this->model.wasRegisterWritten(Reg::TEST_PA_1, 0x5D));
}

// ----------------------------------------------------------------------
// FIFO semantics
// ----------------------------------------------------------------------

TEST_F(Rfm69SimModelTest, FifoPushPopOrder) {
    // FIFO access does not auto-increment: repeated writes queue in order
    this->writeReg(Reg::FIFO, 0x11);
    this->writeReg(Reg::FIFO, 0x22);
    this->writeReg(Reg::FIFO, 0x33);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_NOT_EMPTY, 0);
    EXPECT_EQ(this->readReg(Reg::FIFO), 0x11);
    EXPECT_EQ(this->readReg(Reg::FIFO), 0x22);
    EXPECT_EQ(this->readReg(Reg::FIFO), 0x33);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_NOT_EMPTY, 0);
}

TEST_F(Rfm69SimModelTest, FifoLevelTracksThreshold) {
    this->writeReg(Reg::FIFO_THRESH, FIFO_THRESHOLD);
    for (FwSizeType i = 0; i < FIFO_THRESHOLD; i++) {
        this->writeReg(Reg::FIFO, static_cast<U8>(i));
    }
    // Count == threshold: FifoLevel still clear (asserts when count > threshold)
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_LEVEL, 0);
    this->writeReg(Reg::FIFO, 0xEE);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_LEVEL, 0);
}

TEST_F(Rfm69SimModelTest, FifoFullAndOverrun) {
    for (FwSizeType i = 0; i < FIFO_SIZE; i++) {
        this->writeReg(Reg::FIFO, static_cast<U8>(i));
    }
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_FULL, 0);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_OVERRUN, 0);
    // One more byte is lost and latches FifoOverrun (datasheet section 5.2.2.3)
    this->writeReg(Reg::FIFO, 0xFF);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_OVERRUN, 0);
    // Writing FifoOverrun=1 clears the flag and flushes the FIFO
    this->writeReg(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_OVERRUN, 0);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_NOT_EMPTY, 0);
}

// ----------------------------------------------------------------------
// Mode transitions and settling
// ----------------------------------------------------------------------

TEST_F(Rfm69SimModelTest, ModeReadyImmediateByDefault) {
    this->enterMode(Mode::RX);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::MODE_READY, 0);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::RX_READY, 0);
    this->enterMode(Mode::TX);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::TX_READY, 0);
}

TEST_F(Rfm69SimModelTest, ModeSettleDelaysModeReady) {
    // Oscillator/PLL settling deasserts ModeReady after each mode change
    this->model.setModeSettleByteTimes(8);
    this->writeReg(Reg::OP_MODE, Mode::RX);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::MODE_READY, 0);
    // Each IRQ read exchanges 2 SPI bytes = 2 byte clocks
    for (FwSizeType i = 0; i < 4; i++) {
        (void)this->readReg(Reg::IRQ_FLAGS_1);
    }
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::MODE_READY, 0);
}

TEST_F(Rfm69SimModelTest, EnteringRxClearsFifoAndPacketSent) {
    this->writeReg(Reg::FIFO, 0x77);
    this->enterMode(Mode::RX);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_NOT_EMPTY, 0);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::PACKET_SENT, 0);
}

// ----------------------------------------------------------------------
// Transmit path
// ----------------------------------------------------------------------

TEST_F(Rfm69SimModelTest, TransmitVariableLengthPacket) {
    // Variable-length packet: length byte then payload (datasheet section 5.2.2.2)
    const U8 payload[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x42};
    this->writeReg(Reg::FIFO, sizeof payload);
    for (FwSizeType i = 0; i < sizeof payload; i++) {
        this->writeReg(Reg::FIFO, payload[i]);
    }
    this->enterMode(Mode::TX);
    // Clock the packet onto the air: poll IRQ until PacketSent
    U8 flags2 = 0;
    for (FwSizeType i = 0; (i < 64) && ((flags2 & IrqFlags2::PACKET_SENT) == 0); i++) {
        flags2 = this->readReg(Reg::IRQ_FLAGS_2);
    }
    EXPECT_NE(flags2 & IrqFlags2::PACKET_SENT, 0);
    U8 out[MAX_PACKET_PAYLOAD] = {0};
    const FwSizeType size = this->model.retrievePacket(out, sizeof out);
    ASSERT_EQ(size, sizeof payload);
    EXPECT_EQ(::memcmp(out, payload, size), 0);
}

TEST_F(Rfm69SimModelTest, TransmitStreamsLargerThanFifo) {
    // A 200-byte packet cannot fit the 66-byte FIFO: stream by FifoLevel
    U8 payload[200];
    for (FwSizeType i = 0; i < sizeof payload; i++) {
        payload[i] = static_cast<U8>(i * 7);
    }
    this->writeReg(Reg::FIFO, sizeof payload);
    FwSizeType written = 0;
    // Initial fill leaving room for the length byte already pushed
    while (written < (FIFO_SIZE - 1)) {
        this->writeReg(Reg::FIFO, payload[written]);
        written++;
    }
    this->enterMode(Mode::TX);
    // Top up whenever FifoLevel clears, as the manager does (bounded loop)
    for (FwSizeType guard = 0; (guard < 4096) && (written < sizeof payload); guard++) {
        const U8 flags2 = this->readReg(Reg::IRQ_FLAGS_2);
        if ((flags2 & IrqFlags2::FIFO_LEVEL) == 0) {
            this->writeReg(Reg::FIFO, payload[written]);
            written++;
        }
    }
    ASSERT_EQ(written, sizeof payload);
    U8 flags2 = 0;
    for (FwSizeType i = 0; (i < 512) && ((flags2 & IrqFlags2::PACKET_SENT) == 0); i++) {
        flags2 = this->readReg(Reg::IRQ_FLAGS_2);
    }
    EXPECT_NE(flags2 & IrqFlags2::PACKET_SENT, 0);
    U8 out[MAX_PACKET_PAYLOAD] = {0};
    const FwSizeType size = this->model.retrievePacket(out, sizeof out);
    ASSERT_EQ(size, sizeof payload);
    EXPECT_EQ(::memcmp(out, payload, size), 0);
}

TEST_F(Rfm69SimModelTest, PacketSentStall) {
    this->model.setPacketSentStall(true);
    this->writeReg(Reg::FIFO, 2);
    this->writeReg(Reg::FIFO, 0xAA);
    this->writeReg(Reg::FIFO, 0xBB);
    this->enterMode(Mode::TX);
    U8 flags2 = 0;
    for (FwSizeType i = 0; i < 64; i++) {
        flags2 = this->readReg(Reg::IRQ_FLAGS_2);
    }
    // PacketSent never asserts and no packet is queued
    EXPECT_EQ(flags2 & IrqFlags2::PACKET_SENT, 0);
    U8 out[MAX_PACKET_PAYLOAD];
    EXPECT_EQ(this->model.retrievePacket(out, sizeof out), 0u);
}

TEST_F(Rfm69SimModelTest, MultipleQueuedTransmits) {
    for (U8 packet = 0; packet < 3; packet++) {
        this->enterMode(Mode::STANDBY);
        this->writeReg(Reg::IRQ_FLAGS_2, IrqFlags2::FIFO_OVERRUN);  // FIFO flush
        this->writeReg(Reg::FIFO, 1);
        this->writeReg(Reg::FIFO, static_cast<U8>(0xC0 + packet));
        this->enterMode(Mode::TX);
        U8 flags2 = 0;
        for (FwSizeType i = 0; (i < 64) && ((flags2 & IrqFlags2::PACKET_SENT) == 0); i++) {
            flags2 = this->readReg(Reg::IRQ_FLAGS_2);
        }
        ASSERT_NE(flags2 & IrqFlags2::PACKET_SENT, 0);
    }
    U8 out[MAX_PACKET_PAYLOAD];
    for (U8 packet = 0; packet < 3; packet++) {
        ASSERT_EQ(this->model.retrievePacket(out, sizeof out), 1u);
        EXPECT_EQ(out[0], static_cast<U8>(0xC0 + packet));
    }
    EXPECT_EQ(this->model.retrievePacket(out, sizeof out), 0u);
}

// ----------------------------------------------------------------------
// Receive path
// ----------------------------------------------------------------------

TEST_F(Rfm69SimModelTest, ReceiveSmallPacket) {
    this->enterMode(Mode::RX);
    const U8 uplink[4] = {0x01, 0x02, 0x03, 0x04};
    this->model.injectAirData(uplink, sizeof uplink);
    // Clock delivery into the FIFO until PayloadReady
    U8 flags2 = 0;
    for (FwSizeType i = 0; (i < 64) && ((flags2 & IrqFlags2::PAYLOAD_READY) == 0); i++) {
        flags2 = this->readReg(Reg::IRQ_FLAGS_2);
    }
    ASSERT_NE(flags2 & IrqFlags2::PAYLOAD_READY, 0);
    // First FIFO byte is the length in variable-length mode
    EXPECT_EQ(this->readReg(Reg::FIFO), sizeof uplink);
    for (FwSizeType i = 0; i < sizeof uplink; i++) {
        EXPECT_EQ(this->readReg(Reg::FIFO), uplink[i]);
    }
    // Draining the FIFO ends the reception (Rssi/SyncAddressMatch clear)
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::SYNC_ADDRESS_MATCH, 0);
}

TEST_F(Rfm69SimModelTest, ReceiveActiveAssertsRssiAndSync) {
    this->enterMode(Mode::RX);
    const U8 uplink[100] = {0x55};
    this->model.injectAirData(uplink, sizeof uplink);
    (void)this->readReg(Reg::IRQ_FLAGS_2);  // clock a few bytes in
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::SYNC_ADDRESS_MATCH, 0);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::RSSI, 0);
}

TEST_F(Rfm69SimModelTest, ChannelBusyControl) {
    this->enterMode(Mode::RX);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::SYNC_ADDRESS_MATCH, 0);
    this->model.setChannelBusy(true);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::SYNC_ADDRESS_MATCH, 0);
    EXPECT_NE(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::RSSI, 0);
    this->model.setChannelBusy(false);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_1) & IrqFlags1::SYNC_ADDRESS_MATCH, 0);
}

TEST_F(Rfm69SimModelTest, CorruptFrameNeverAssertsPayloadReady) {
    this->enterMode(Mode::RX);
    this->model.injectCorruptFrame(24);
    U8 flags2 = 0;
    FwSizeType drained = 0;
    for (FwSizeType i = 0; i < 256; i++) {
        flags2 = this->readReg(Reg::IRQ_FLAGS_2);
        ASSERT_EQ(flags2 & IrqFlags2::PAYLOAD_READY, 0);
        if ((flags2 & IrqFlags2::FIFO_NOT_EMPTY) != 0) {
            (void)this->readReg(Reg::FIFO);
            drained++;
        }
    }
    EXPECT_GT(drained, 24u);
}

TEST_F(Rfm69SimModelTest, RxRestartFlushesAndAllowsNextPacket) {
    this->enterMode(Mode::RX);
    const U8 first[8] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    this->model.injectAirData(first, sizeof first);
    U8 flags2 = 0;
    for (FwSizeType i = 0; (i < 64) && ((flags2 & IrqFlags2::PAYLOAD_READY) == 0); i++) {
        flags2 = this->readReg(Reg::IRQ_FLAGS_2);
    }
    ASSERT_NE(flags2 & IrqFlags2::PAYLOAD_READY, 0);
    // Abandon it: RxRestart command bit flushes and self-clears
    const U8 packetConfig2 = this->model.readRegisterValue(Reg::PACKET_CONFIG_2);
    this->writeReg(Reg::PACKET_CONFIG_2, packetConfig2 | 0x04);
    EXPECT_EQ(this->model.readRegisterValue(Reg::PACKET_CONFIG_2) & 0x04, 0);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::PAYLOAD_READY, 0);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_NOT_EMPTY, 0);
    // A following packet is received normally
    const U8 second[3] = {0xA1, 0xA2, 0xA3};
    this->model.injectAirData(second, sizeof second);
    flags2 = 0;
    for (FwSizeType i = 0; (i < 64) && ((flags2 & IrqFlags2::PAYLOAD_READY) == 0); i++) {
        flags2 = this->readReg(Reg::IRQ_FLAGS_2);
    }
    ASSERT_NE(flags2 & IrqFlags2::PAYLOAD_READY, 0);
    EXPECT_EQ(this->readReg(Reg::FIFO), sizeof second);
}

// ----------------------------------------------------------------------
// Fault injection controls
// ----------------------------------------------------------------------

TEST_F(Rfm69SimModelTest, AbsentRadioReadsZero) {
    this->model.setRadioPresent(false);
    EXPECT_EQ(this->readReg(Reg::VERSION), 0);
    this->writeReg(Reg::SYNC_VALUE_1, 0x77);  // latched nothing
    this->model.setRadioPresent(true);
    EXPECT_EQ(this->readReg(Reg::VERSION), VERSION_VALUE);
    EXPECT_EQ(this->readReg(Reg::SYNC_VALUE_1), 0);
}

TEST_F(Rfm69SimModelTest, RssiReadbackControl) {
    this->model.setRssiReadback(0xB4);  // -90 dBm
    EXPECT_EQ(this->readReg(Reg::RSSI_VALUE), 0xB4);
}

TEST_F(Rfm69SimModelTest, ResetRestoresPowerOnState) {
    this->writeReg(Reg::SYNC_VALUE_1, 0xAB);
    this->writeReg(Reg::FIFO, 0x11);
    this->enterMode(Mode::RX);
    this->model.reset();
    EXPECT_EQ(this->model.readRegisterValue(Reg::SYNC_VALUE_1), 0);
    EXPECT_EQ(this->model.readRegisterValue(Reg::OP_MODE) & Mode::MASK, Mode::STANDBY);
    EXPECT_EQ(this->readReg(Reg::IRQ_FLAGS_2) & IrqFlags2::FIFO_NOT_EMPTY, 0);
    EXPECT_EQ(this->readReg(Reg::VERSION), VERSION_VALUE);
}

}  // namespace Rfm69

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
