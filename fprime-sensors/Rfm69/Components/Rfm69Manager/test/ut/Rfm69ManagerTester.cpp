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

void Rfm69ManagerTester ::makeReady() {
    this->component.configure(Rfm69Manager::DEFAULT_FREQUENCY_HZ, Rfm69Manager::DEFAULT_NETWORK_ID,
                              Rfm69Manager::DEFAULT_POWER_LEVEL);
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_RadioConfigured_SIZE(1);
    this->clearHistory();
}

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------

void Rfm69ManagerTester ::test_initialization() {
    // Before configure() is called, run must not touch the radio
    this->invoke_to_run(0, 0);
    ASSERT_from_spiWriteRead_SIZE(0);
    ASSERT_EVENTS_SIZE(0);

    // After configure(), one tick detects and configures the radio
    this->component.configure(Rfm69Manager::DEFAULT_FREQUENCY_HZ, Rfm69Manager::DEFAULT_NETWORK_ID,
                              Rfm69Manager::DEFAULT_POWER_LEVEL);
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_RadioConfigured_SIZE(1);
    // Initial com status is SUCCESS (ready for the first frame)
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
}

void Rfm69ManagerTester ::test_detection_retry() {
    this->component.configure(Rfm69Manager::DEFAULT_FREQUENCY_HZ, Rfm69Manager::DEFAULT_NETWORK_ID,
                              Rfm69Manager::DEFAULT_POWER_LEVEL);
    // SPI failure: radio not detected, no configuration attempted
    this->m_spiFail = true;
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_RadioNotDetected_SIZE(1);
    ASSERT_from_comStatusOut_SIZE(0);
    // Radio (SPI) recovers: detection retries and succeeds
    this->m_spiFail = false;
    this->invoke_to_run(0, 0);
    ASSERT_EVENTS_RadioConfigured_SIZE(1);
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

void Rfm69ManagerTester ::test_transmit_segmentation() {
    this->makeReady();
    // 600 bytes segments into 255 + 255 + 90
    U8 data[600];
    for (FwSizeType i = 0; i < sizeof data; i++) {
        data[i] = static_cast<U8>(i & 0xFF);
    }
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);

    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_TLM_PacketsTransmitted(2, 3);

    // Reassembled packets match the original frame
    U8 reassembled[sizeof data];
    FwSizeType total = 0;
    U8 transmitted[MAX_PACKET_PAYLOAD + 1];
    const FwSizeType expectedSizes[] = {255, 255, 90};
    for (FwSizeType i = 0; i < FW_NUM_ARRAY_ELEMENTS(expectedSizes); i++) {
        const FwSizeType size = this->m_model.retrievePacket(transmitted, sizeof transmitted);
        ASSERT_EQ(size, expectedSizes[i]);
        (void)::memcpy(&reassembled[total], transmitted, size);
        total += size;
    }
    ASSERT_EQ(total, sizeof data);
    for (FwSizeType i = 0; i < sizeof data; i++) {
        ASSERT_EQ(reassembled[i], data[i]);
    }
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
}

void Rfm69ManagerTester ::test_transmit_deferred() {
    this->makeReady();
    // A reception in progress: SyncAddressMatch is asserted by the model
    U8 uplink[40];
    for (FwSizeType i = 0; i < sizeof uplink; i++) {
        uplink[i] = static_cast<U8>(i);
    }
    this->m_model.injectAirData(uplink, sizeof uplink);

    // Listen-before-talk defers the transmission: no status, no buffer return
    U8 data[32] = {0xA5};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_EVENTS_TransmitDeferred_SIZE(1);
    ASSERT_TLM_TransmitsDeferred(0, 1);
    ASSERT_from_comStatusOut_SIZE(0);
    ASSERT_from_dataReturnOut_SIZE(0);
    U8 transmitted[MAX_PACKET_PAYLOAD + 1];
    ASSERT_EQ(this->m_model.retrievePacket(transmitted, sizeof transmitted), 0);

    // The next run tick drains the reception, then retries the transmission
    this->invoke_to_run(0, 0);
    ASSERT_from_dataOut_SIZE(1);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_TLM_PacketsTransmitted(0, 1);
    const FwSizeType size = this->m_model.retrievePacket(transmitted, sizeof transmitted);
    ASSERT_EQ(size, sizeof data);
}

void Rfm69ManagerTester ::test_transmit_not_ready() {
    // No initialization: transmission must be refused with FAILURE status
    U8 data[8] = {0};
    Fw::Buffer buffer(data, sizeof data);
    ComCfg::FrameContext context;
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_EVENTS_RadioNotReady_SIZE(1);
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
    ASSERT_EVENTS_TransmitFailed_SIZE(1);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::FAILURE));
    ASSERT_from_dataReturnOut_SIZE(1);
    ASSERT_TLM_TransmitFailures(0, 1);
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
    ASSERT_EVENTS_BufferAllocationFailed_SIZE(1);
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
    ASSERT_EVENTS_TransmitStateChanged_SIZE(1);
    ASSERT_TLM_TransmitEnabled(0, Rfm69::TransmitState::DISABLED);
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
    ASSERT_TLM_TransmitEnabled(0, Rfm69::TransmitState::ENABLED);
    this->clearHistory();
    this->invoke_to_dataIn(0, buffer, context);
    ASSERT_from_comStatusOut(0, Fw::Success(Fw::Success::SUCCESS));
    ASSERT_TLM_PacketsTransmitted(0, 1);
}

}  // namespace Rfm69
