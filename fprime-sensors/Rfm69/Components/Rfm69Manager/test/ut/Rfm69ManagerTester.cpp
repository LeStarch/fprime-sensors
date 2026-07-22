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
    // 150 bytes segments into 64 + 64 + 22
    U8 data[150];
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
    const FwSizeType expectedSizes[] = {64, 64, 22};
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
    // Inject 100 bytes: delivered as a 64-byte and a 36-byte packet
    U8 data[100];
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
    ASSERT_EQ(first.getSize(), 64u);
    ASSERT_EQ(second.getSize(), 36u);
    for (FwSizeType i = 0; i < second.getSize(); i++) {
        ASSERT_EQ(second.getData()[i], data[64 + i]);
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

}  // namespace Rfm69
