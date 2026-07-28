// ======================================================================
// \title  Rfm69ManagerTester.cpp
// \brief  cpp file for Rfm69Manager component test harness implementation class
// ======================================================================

#include "Rfm69ManagerTester.hpp"

namespace Rfm69 {

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

Rfm69ManagerTester ::Rfm69ManagerTester()
    : Rfm69ManagerGTestBase("Rfm69ManagerTester", Rfm69ManagerTester::MAX_HISTORY_SIZE),
      component("Rfm69Manager"),
      m_model(),
      m_spiFail(false),
      m_allocFail(false),
      m_allocation{} {
    this->initComponents();
    this->connectPorts();
}

Rfm69ManagerTester ::~Rfm69ManagerTester() {}

// ----------------------------------------------------------------------
// Handlers for typed from ports
// ----------------------------------------------------------------------

Drv::SpiStatus Rfm69ManagerTester ::from_spiWriteRead_handler(FwIndexType portNum,
                                                              Fw::Buffer& writeBuffer,
                                                              Fw::Buffer& readBuffer) {
    this->pushFromPortEntry_spiWriteRead(writeBuffer, readBuffer);
    if (this->m_spiFail) {
        return Drv::SpiStatus::SPI_WRITE_ERR;
    }
    EXPECT_EQ(writeBuffer.getSize(), readBuffer.getSize());
    this->m_model.spiTransaction(writeBuffer.getData(), readBuffer.getData(), writeBuffer.getSize());
    return Drv::SpiStatus::SPI_OK;
}

Fw::Buffer Rfm69ManagerTester ::from_allocate_handler(FwIndexType portNum, FwSizeType size) {
    this->pushFromPortEntry_allocate(size);
    if (this->m_allocFail || (size > sizeof this->m_allocation)) {
        return Fw::Buffer();
    }
    return Fw::Buffer(this->m_allocation, size);
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

void Rfm69ManagerTester ::setParameters(const Rfm69DataRate& dataRate,
                                        const Rfm69Bandwidth& bandwidthRx,
                                        const Rfm69TxPower& txPower) {
    this->paramSet_DATA_RATE(dataRate, Fw::ParamValid::VALID);
    this->paramSet_BANDWIDTH_RX(bandwidthRx, Fw::ParamValid::VALID);
    this->paramSet_TX_POWER(txPower, Fw::ParamValid::VALID);
}

void Rfm69ManagerTester ::setDefaultParameters() {
    const Rfm69DataRate dataRate = Rfm69DataRate::BR_9600;
    const Rfm69Bandwidth bandwidthRx = Rfm69Bandwidth::BW_500_KHZ;
    const Rfm69TxPower txPower = Rfm69TxPower::DBM_13;
    this->setParameters(dataRate, bandwidthRx, txPower);
}

void Rfm69ManagerTester ::makeReady() {
    this->setDefaultParameters();
    this->component.loadParameters();
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    this->clearHistory();
}

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------

void Rfm69ManagerTester ::test_initialization() {
    // Before FPP parameters are loaded, run must not touch the radio.
    this->invoke_to_run(0, 0);
    ASSERT_from_spiWriteRead_SIZE(0);
    ASSERT_EVENTS_SIZE(0);

    // Loading defaults is the deployment's single configuration path.
    this->setDefaultParameters();
    this->component.loadParameters();
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    // Initial com status is SUCCESS (ready for the first frame)
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
}

void Rfm69ManagerTester ::test_detection_retry() {
    this->setDefaultParameters();
    this->component.loadParameters();
    // SPI failure: radio not detected, no configuration attempted
    this->m_spiFail = true;
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(1);
    ASSERT_EVENTS_ConfigurationFailed(0, Rfm69Mode::Receive);
    ASSERT_from_comStatusOut_SIZE(0);
    // Radio (SPI) recovers: detection retries and succeeds
    this->m_spiFail = false;
    this->clearHistory();
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    ASSERT_from_comStatusOut_SIZE(1);
}

void Rfm69ManagerTester ::test_transmit() {
    this->makeReady();
    U8 data[32];
    for (FwSizeType i = 0; i < sizeof data; i++) {
        data[i] = static_cast<U8>(i);
    }
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);

    // Transmission succeeded and the buffer was returned
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_TLM_PacketsTransmitted(0, 1);

    // The model "transmitted" exactly the payload
    U8 transmitted[MAX_PACKET_PAYLOAD + 1];
    const FwSizeType size = this->m_model.retrievePacket(transmitted, sizeof transmitted);
    ASSERT_EQ(size, sizeof data);
    for (FwSizeType i = 0; i < size; i++) {
        ASSERT_EQ(transmitted[i], data[i]);
    }
    // No second packet pending
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), 0);
}

void Rfm69ManagerTester ::test_transmit_zero_size() {
    this->makeReady();
    U8 data[1] = {0};
    Fw::Buffer buffer(data, 0);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);

    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::FAILURE));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_EVENTS_SendFailed_SIZE(1);
    ASSERT_EVENTS_SendFailed(0, 0);
    ASSERT_TLM_PacketsTransmitted_SIZE(0);
    U8 transmitted[MAX_PACKET_PAYLOAD];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), 0);
}

void Rfm69ManagerTester ::test_transmit_oversize() {
    this->makeReady();
    U8 data[MAX_PACKET_PAYLOAD + 1];
    for (FwSizeType i = 0; i < sizeof data; i++) {
        data[i] = static_cast<U8>(i);
    }
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);

    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::FAILURE));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_EVENTS_SendFailed_SIZE(1);
    ASSERT_EVENTS_SendFailed(0, static_cast<I32>(MAX_PACKET_PAYLOAD + 1));
    ASSERT_TLM_PacketsTransmitted_SIZE(0);
    U8 transmitted[MAX_PACKET_PAYLOAD];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), 0);
}

void Rfm69ManagerTester ::test_transmit_large() {
    this->makeReady();
    // A full 255-byte packet: larger than the 66-byte FIFO, so it must
    // stream through with in-transmission top-ups
    U8 data[MAX_PACKET_PAYLOAD];
    for (FwSizeType i = 0; i < sizeof data; i++) {
        data[i] = static_cast<U8>((i * 7) & 0xFF);
    }
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);

    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_TLM_PacketsTransmitted(0, 1);

    U8 transmitted[MAX_PACKET_PAYLOAD + 1];
    const FwSizeType size = this->m_model.retrievePacket(transmitted, sizeof transmitted);
    ASSERT_EQ(size, sizeof data);
    for (FwSizeType i = 0; i < size; i++) {
        ASSERT_EQ(transmitted[i], data[i]);
    }
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), 0);
}

void Rfm69ManagerTester ::test_receive_large() {
    this->makeReady();
    // A full 255-byte packet must stream through the 66-byte FIFO
    U8 data[MAX_PACKET_PAYLOAD];
    for (FwSizeType i = 0; i < sizeof data; i++) {
        data[i] = static_cast<U8>((i * 3) & 0xFF);
    }
    this->m_model.injectAirData(data, sizeof data);
    this->invoke_to_run(0, 0);

    ASSERT_from_dataOut_SIZE(1);
    ASSERT_TLM_PacketsReceived(0, 1);
    const Fw::Buffer& received = this->fromPortHistory_dataOut->at(0).data;
    ASSERT_EQ(received.getSize(), sizeof data);
    for (FwSizeType i = 0; i < received.getSize(); i++) {
        ASSERT_EQ(received.getData()[i], data[i]);
    }
    // RxRestart is a self-clearing command bit; the model must be ready to
    // accept the next packet without carrying stale FIFO state forward.
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::PACKET_CONFIG_2), 0x02);
}

void Rfm69ManagerTester ::test_default_register_image() {
    this->makeReady();
    // This is the canonical default enum image mirrored by the ground station:
    // BR_9600 / BW_500_KHZ / DBM_13 plus the fixed native-packet profile.
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::DATA_MODUL), 0x00);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::BITRATE_MSB), 0x0D);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::BITRATE_LSB), 0x05);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FDEV_MSB), 0x01);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FDEV_LSB), 0x9A);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::RX_BW), 0xE0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::AFC_BW), 0xE0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::PA_LEVEL), 0x5F);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::OCP), Pa::OCP_NORMAL);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::TEST_PA_1), Pa::TEST_PA_1_NORMAL);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::TEST_PA_2), Pa::TEST_PA_2_NORMAL);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FRF_MSB), 0xE4);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FRF_MID), 0xC0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FRF_LSB), 0x00);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::SYNC_VALUE_2), 0xA7);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::PACKET_CONFIG_1), 0xD0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::PAYLOAD_LENGTH), 0xFF);
}

void Rfm69ManagerTester ::test_high_power_boost_recovery() {
    this->makeReady();

    // Change only TX_POWER through the generated parameter command. The
    // normal configured image keeps boost disabled until a packet is sent.
    const Rfm69TxPower txPower = Rfm69TxPower::DBM_20;
    this->paramSet_TX_POWER(txPower, Fw::ParamValid::VALID);
    this->paramSend_TX_POWER(0, 0);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, Rfm69ManagerComponentBase::OPCODE_TX_POWER_SET, 0, Fw::CmdResponse::OK);
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::PA_LEVEL), 0x7F);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::OCP), Pa::OCP_NORMAL);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::TEST_PA_1), Pa::TEST_PA_1_NORMAL);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::TEST_PA_2), Pa::TEST_PA_2_NORMAL);

    // The simulator's write history makes the short boost interval observable
    // even though transmitPacket() restores the normal RX settings before it
    // returns to the caller.
    this->m_model.clearRegisterWriteHistory();
    this->clearHistory();
    U8 data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);

    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_TRUE(this->m_model.wasRegisterWritten(Reg::OCP, Pa::OCP_DISABLED));
    ASSERT_TRUE(this->m_model.wasRegisterWritten(Reg::TEST_PA_1, Pa::TEST_PA_1_BOOST));
    ASSERT_TRUE(this->m_model.wasRegisterWritten(Reg::TEST_PA_2, Pa::TEST_PA_2_BOOST));
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::OCP), Pa::OCP_NORMAL);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::TEST_PA_1), Pa::TEST_PA_1_NORMAL);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::TEST_PA_2), Pa::TEST_PA_2_NORMAL);
}

void Rfm69ManagerTester ::test_invalid_data_rate_falls_back_to_default() {
    const Rfm69DataRate dataRate = Rfm69DataRate(static_cast<Rfm69DataRate::T>(0xFFFF));
    const Rfm69Bandwidth bandwidthRx = Rfm69Bandwidth::BW_500_KHZ;
    const Rfm69TxPower txPower = Rfm69TxPower::DBM_13;
    this->setParameters(dataRate, bandwidthRx, txPower);
    this->component.loadParameters();

    // The generated FPP parameter source rejects an unknown enum
    // serialization and supplies the declared default before the manager
    // applies it. The manager must configure the complete default profile.
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::BITRATE_MSB), 0x0D);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::BITRATE_LSB), 0x05);
}

void Rfm69ManagerTester ::test_bandwidth_update_reconfigure() {
    this->makeReady();

    // Generated FPP parameter-set commands call parameterUpdated(), which
    // schedules reconfiguration for the next rate-group tick. No bespoke
    // RECONFIGURE command is needed.
    const Rfm69Bandwidth bandwidthRx = Rfm69Bandwidth::BW_200_KHZ;
    this->paramSet_BANDWIDTH_RX(bandwidthRx, Fw::ParamValid::VALID);
    this->paramSend_BANDWIDTH_RX(0, 0);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, Rfm69ManagerComponentBase::OPCODE_BANDWIDTH_RX_SET, 0, Fw::CmdResponse::OK);
    ASSERT_from_spiWriteRead_SIZE(0);

    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    ASSERT_from_comStatusOut_SIZE(0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::RX_BW), 0xE9);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::AFC_BW), 0xE9);

    this->clearHistory();
    U8 data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    U8 transmitted[MAX_PACKET_PAYLOAD];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), sizeof data);
}

void Rfm69ManagerTester ::test_bandwidth_register_map() {
    const struct {
        Rfm69Bandwidth bandwidth;
        U8 registerValue;
    } expected[] = {
        {Rfm69Bandwidth::BW_100_KHZ, 0xEA},
        {Rfm69Bandwidth::BW_200_KHZ, 0xE9},
        {Rfm69Bandwidth::BW_250_KHZ, 0xE1},
        {Rfm69Bandwidth::BW_500_KHZ, 0xE0},
    };

    for (const auto& entry : expected) {
        BandwidthSetting setting{};
        ASSERT_TRUE(getBandwidthSetting(entry.bandwidth, setting));
        ASSERT_EQ(setting.rxBw, entry.registerValue);
        ASSERT_EQ(setting.afcBw, entry.registerValue);
    }
}

void Rfm69ManagerTester ::test_parameter_update_reconfigure() {
    this->makeReady();

    // The parameter set command invokes parameterUpdated(), which must defer
    // reconfiguration to the next rate-group tick rather than touching the
    // SPI bus in command-dispatch context.
    const Rfm69DataRate dataRate = Rfm69DataRate::BR_19200;
    this->paramSet_DATA_RATE(dataRate, Fw::ParamValid::VALID);
    this->paramSend_DATA_RATE(0, 0);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, Rfm69ManagerComponentBase::OPCODE_DATA_RATE_SET, 0, Fw::CmdResponse::OK);
    ASSERT_from_spiWriteRead_SIZE(0);

    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::BITRATE_MSB), 0x06);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::BITRATE_LSB), 0x83);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::DATA_MODUL), 0x00);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FDEV_MSB), 0x01);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FDEV_LSB), 0x9A);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::RX_BW), 0xE0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::AFC_BW), 0xE0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FRF_MSB), 0xE4);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FRF_MID), 0xC0);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::FRF_LSB), 0x00);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::SYNC_VALUE_2), 0xA7);
    ASSERT_from_comStatusOut_SIZE(0);

    this->clearHistory();
    U8 data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    U8 transmitted[MAX_PACKET_PAYLOAD];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), sizeof data);
}

void Rfm69ManagerTester ::test_reset_recovery() {
    this->makeReady();

    this->sendCmd_RESET(0, 0);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, Rfm69ManagerComponentBase::OPCODE_RESET, 0, Fw::CmdResponse::OK);
    ASSERT_from_resetGpio_SIZE(2);
    ASSERT_from_resetGpio(0, Fw::Logic::HIGH);
    ASSERT_from_resetGpio(1, Fw::Logic::LOW);

    // The normal next scheduler tick redetects and configures the model. The
    // startup Com SUCCESS is intentionally not repeated after a reset.
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_ConfigurationFailed_SIZE(0);
    ASSERT_from_comStatusOut_SIZE(0);

    this->clearHistory();
    U8 data[8] = {7, 6, 5, 4, 3, 2, 1, 0};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    U8 transmitted[MAX_PACKET_PAYLOAD];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), sizeof data);
}

void Rfm69ManagerTester ::test_transmit_dropped_when_busy() {
    this->makeReady();
    // A reception in progress: SyncAddressMatch is asserted by the model
    U8 uplink[40];
    for (FwSizeType i = 0; i < sizeof uplink; i++) {
        uplink[i] = static_cast<U8>(i);
    }
    this->m_model.injectAirData(uplink, sizeof uplink);

    // Dumb half-duplex: drop the downlink immediately (no deferred retry)
    U8 data[32] = {0xA5};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_EVENTS_SendFailed_SIZE(1);
    ASSERT_EVENTS_SendFailed(0, -1);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::FAILURE));
    ASSERT_from_dataReturnOut_SIZE(1);
    U8 transmitted[MAX_PACKET_PAYLOAD + 1];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), 0);

    // The next run tick still delivers the uplink
    this->invoke_to_run(0, 0);
    ASSERT_from_dataOut_SIZE(1);
    ASSERT_TLM_PacketsTransmitted_SIZE(0);
}

void Rfm69ManagerTester ::test_transmit_not_ready() {
    // No initialization: transmission must be refused with FAILURE status
    U8 data[8] = {0};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_EVENTS_SendFailed_SIZE(1);
    ASSERT_EVENTS_SendFailed(0, -1);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::FAILURE));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_from_spiWriteRead_SIZE(0);
}

void Rfm69ManagerTester ::test_transmit_spi_failure() {
    this->makeReady();
    this->m_spiFail = true;
    U8 data[8] = {0};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_EVENTS_SendFailed_SIZE(1);
    ASSERT_EVENTS_SendFailed(0, -1);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::FAILURE));
    ASSERT_from_dataReturnOut_SIZE(1);
}

void Rfm69ManagerTester ::test_transmit_timeout_recovery() {
    this->makeReady();
    this->m_model.setPacketSentStall(true);
    U8 data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);

    ASSERT_EVENTS_SendFailed_SIZE(1);
    ASSERT_EVENTS_SendFailed(0, -1);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::FAILURE));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_EQ(this->m_model.readRegisterValue(Reg::OP_MODE) & Mode::MASK, Mode::RX);
    U8 transmitted[MAX_PACKET_PAYLOAD];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), 0);

    // Remove the fault and prove the same radio model can transmit again.
    this->m_model.setPacketSentStall(false);
    this->clearHistory();
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_from_dataReturnOut_SIZE(1);
    const FwSizeType size = this->m_model.retrievePacket(transmitted, sizeof transmitted);
    ASSERT_EQ(size, sizeof data);
    for (FwSizeType i = 0; i < size; i++) {
        ASSERT_EQ(transmitted[i], data[i]);
    }
}

void Rfm69ManagerTester ::test_receive() {
    this->makeReady();
    // Inject 300 bytes: delivered as a 255-byte and a 45-byte packet
    U8 data[300];
    for (FwSizeType i = 0; i < sizeof data; i++) {
        data[i] = static_cast<U8>(0xFF - (i & 0xFF));
    }
    this->m_model.injectAirData(data, sizeof data);
    this->invoke_to_run(0, 0);

    // RxRestart separates complete variable-length packets. A second tick is
    // sufficient whether the implementation drains one or several packets per
    // service invocation.
    this->invoke_to_run(0, 0);

    ASSERT_from_dataOut_SIZE(2);
    ASSERT_TLM_PacketsReceived(1, 2);
    ASSERT_TLM_LastRssi(0, -40.0f);
    const Fw::Buffer& first = this->fromPortHistory_dataOut->at(0).data;
    const Fw::Buffer& second = this->fromPortHistory_dataOut->at(1).data;
    ASSERT_EQ(first.getSize(), 255u);
    ASSERT_EQ(second.getSize(), 45u);
    for (FwSizeType i = 0; i < second.getSize(); i++) {
        ASSERT_EQ(second.getData()[i], data[255 + i]);
    }
}

void Rfm69ManagerTester ::test_receive_allocation_failure() {
    this->makeReady();
    U8 data[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    this->m_model.injectAirData(data, sizeof data);
    this->m_allocFail = true;
    this->invoke_to_run(0, 0);
    // Packet dropped with an event; nothing delivered downstream
    ASSERT_EVENTS_AllocationFailed_SIZE(1);
    ASSERT_EVENTS_AllocationFailed(0, sizeof data);
    ASSERT_from_dataOut_SIZE(0);
    // Link stays alive: a later packet is delivered once allocation recovers
    this->m_allocFail = false;
    this->clearHistory();
    this->m_model.injectAirData(data, sizeof data);
    this->invoke_to_run(0, 0);
    ASSERT_from_dataOut_SIZE(1);
}

void Rfm69ManagerTester ::test_data_return() {
    U8 data[8] = {0};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataReturnIn(0, buffer, context);
    ASSERT_from_deallocate_SIZE(1);
}

void Rfm69ManagerTester ::test_transmit_disabled() {
    this->makeReady();
    // Command a receive-only window
    this->sendCmd_TRANSMIT(0, 0, Rfm69::TransmitState::DISABLED);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, Rfm69ManagerComponentBase::OPCODE_TRANSMIT, 0, Fw::CmdResponse::OK);
    this->clearHistory();

    // A downlink frame while disabled: no SPI transmit, but Com flow control
    // still advances with SUCCESS and the buffer is returned.
    U8 data[32];
    for (FwSizeType i = 0; i < sizeof data; i++) {
        data[i] = static_cast<U8>(i);
    }
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_from_spiWriteRead_SIZE(0);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_TLM_PacketsTransmitted_SIZE(0);

    // Re-enable transmit: the next frame is sent over SPI as normal
    this->sendCmd_TRANSMIT(0, 1, Rfm69::TransmitState::ENABLED);
    this->clearHistory();
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_TLM_PacketsTransmitted(0, 1);
}

}  // namespace Rfm69
