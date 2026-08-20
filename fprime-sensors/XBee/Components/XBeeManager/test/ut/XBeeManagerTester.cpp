// ======================================================================
// \title  XBee.hpp
// \author mstarch
// \brief  cpp file for XBee test harness implementation class
// ======================================================================

#include "XBeeManagerTester.hpp"
#include <STest/Pick/Pick.hpp>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 100
#define RETRIES 3

U8 storage[RETRIES][10240];

namespace XBee {

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

XBeeManagerTester ::XBeeManagerTester()
    : XBeeManagerGTestBase("XBeeManagerTester", MAX_HISTORY_SIZE),
      component("XBeeManager"),
      m_send_mode(Drv::ByteStreamStatus::OP_OK),
      m_retries(0) {
    this->initComponents();
    this->connectPorts();
}

XBeeManagerTester ::~XBeeManagerTester() {
    this->component.deinit();
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------
void XBeeManagerTester ::fill_buffer(Fw::Buffer& buffer_to_fill) {
    U8 size = STest::Pick::lowerUpper(1, sizeof(buffer_to_fill.getSize()));
    for (U32 i = 0; i < size; i++) {
        buffer_to_fill.getData()[i] = STest::Pick::any();
    }
    buffer_to_fill.setSize(size);
}

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------
void XBeeManagerTester ::test_initial() {
    Fw::Success com_send_status = Fw::Success::SUCCESS;
    invoke_to_drvConnected(0);
    ASSERT_from_comStatusOut_SIZE(1);
    ASSERT_from_comStatusOut(0, com_send_status);
}

void XBeeManagerTester ::test_basic_io() {
    Fw::Buffer buffer(storage[0], sizeof(storage[0]));
    Fw::Success com_send_status = Fw::Success::SUCCESS;
    this->fill_buffer(buffer);

    // Downlink
    ComCfg::FrameContext context;
    invoke_to_dataIn(0, buffer, context);
    ASSERT_from_drvSendOut_SIZE(1);
    ASSERT_from_drvSendOut(0, buffer);
    ASSERT_from_comStatusOut(0, com_send_status);

    // Uplink
    Drv::ByteStreamStatus status = Drv::ByteStreamStatus::OP_OK;
    invoke_to_drvReceiveIn(0, buffer, status);
    ASSERT_from_dataOut_SIZE(1);
    ASSERT_from_dataOut(0, buffer, context);
}

void XBeeManagerTester ::test_fail() {
    Fw::Buffer buffer(storage[0], sizeof(storage[0]));
    this->fill_buffer(buffer);
    Fw::Success com_send_status = Fw::Success::FAILURE;
    m_send_mode = Drv::ByteStreamStatus::OTHER_ERROR;

    // Downlink
    ComCfg::FrameContext context;
    invoke_to_dataIn(0, buffer, context);
    ASSERT_from_drvSendOut_SIZE(1);
    ASSERT_from_drvSendOut(0, buffer);
    ASSERT_from_drvSendOut_SIZE(1);
    ASSERT_from_comStatusOut(0, com_send_status);

    // Uplink
    Drv::ByteStreamStatus status = Drv::ByteStreamStatus::OTHER_ERROR;
    invoke_to_drvReceiveIn(0, buffer, status);
    ASSERT_from_dataOut_SIZE(1);
    ASSERT_from_dataOut(0, buffer, context);
}

void XBeeManagerTester ::test_retry() {
    Fw::Buffer buffers[RETRIES];
    Fw::Success com_send_status = Fw::Success::SUCCESS;
    m_send_mode = Drv::ByteStreamStatus::SEND_RETRY;
    ComCfg::FrameContext context;

    for (U32 i = 0; i < RETRIES; i++) {
        buffers[i].set(storage[i], sizeof(storage[i]), i);
        this->fill_buffer(buffers[i]);
        invoke_to_dataIn(0, buffers[i], context);
        ASSERT_from_drvSendOut_SIZE((i + 1) * RETRIES);
        m_retries = 0;
    }
    ASSERT_from_drvSendOut_SIZE(RETRIES * RETRIES);
    ASSERT_from_comStatusOut_SIZE(3);
    for (U32 i = 0; i < RETRIES; i++) {
        for (U32 j = 0; j < RETRIES; j++) {
            ASSERT_from_drvSendOut((i * RETRIES) + j, buffers[i]);
        }
        ASSERT_from_comStatusOut(i, com_send_status);
    }
}

// ----------------------------------------------------------------------
// Handlers for typed from ports
// ----------------------------------------------------------------------

void XBeeManagerTester ::from_dataOut_handler(const FwIndexType portNum,
                                              Fw::Buffer& recvBuffer,
                                              const ComCfg::FrameContext& context) {
    this->pushFromPortEntry_dataOut(recvBuffer, context);
}

void XBeeManagerTester ::from_comStatusOut_handler(const FwIndexType portNum, Fw::Success& status) {
    this->pushFromPortEntry_comStatusOut(status);
}

Drv::ByteStreamStatus XBeeManagerTester ::from_drvSendOut_handler(const FwIndexType portNum, Fw::Buffer& sendBuffer) {
    this->pushFromPortEntry_drvSendOut(sendBuffer);
    m_retries = (m_send_mode == Drv::ByteStreamStatus::SEND_RETRY) ? (m_retries + 1) : m_retries;
    if (m_retries < RETRIES) {
        return m_send_mode;
    }
    return Drv::ByteStreamStatus::OP_OK;
}

}  // namespace XBee
