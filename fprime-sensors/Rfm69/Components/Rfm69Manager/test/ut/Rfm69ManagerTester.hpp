// ======================================================================
// \title  Rfm69ManagerTester.hpp
// \brief  hpp file for Rfm69Manager component test harness implementation class
// ======================================================================

#ifndef Rfm69_Rfm69ManagerTester_HPP
#define Rfm69_Rfm69ManagerTester_HPP

#include "Rfm69ManagerGTestBase.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Manager.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Sim/Rfm69SimModel.hpp"

namespace Rfm69 {

class Rfm69ManagerTester : public Rfm69ManagerGTestBase {
  public:
    // Maximum size of histories storing events, telemetry, and port outputs.
    // Streamed 255-byte packets are drained through many small SPI
    // transactions, each of which lands in the port history.
    static const U32 MAX_HISTORY_SIZE = 10000;

    // Instance ID supplied to the component instance under test
    static const FwEnumStoreType TEST_INSTANCE_ID = 0;

    // Size of the buffer pool backing the allocate port
    static const FwSizeType ALLOCATION_SIZE = 1024;

    // ----------------------------------------------------------------------
    // Construction and destruction
    // ----------------------------------------------------------------------

    //! Construct object Rfm69ManagerTester
    Rfm69ManagerTester();

    //! Destroy object Rfm69ManagerTester
    ~Rfm69ManagerTester();

    // ----------------------------------------------------------------------
    // Tests
    // ----------------------------------------------------------------------

    //! Detection, configuration, and initial com status (REQ-001/002/003)
    void test_initialization();

    //! Detection retry when the radio is absent (REQ-009)
    void test_detection_retry();

    //! Single-packet transmission (REQ-004/005/007/010)
    void test_transmit();

    //! Frame segmentation into multiple packets (REQ-004)
    void test_transmit_segmentation();

    //! Maximum-size (255-byte) packet streamed through the FIFO (REQ-004)
    void test_transmit_large();

    //! Maximum-size (255-byte) packet received through the FIFO (REQ-006)
    void test_receive_large();

    //! Listen-before-talk: transmission deferred during a reception (REQ-013)
    void test_transmit_deferred();

    //! Transmission before the radio is ready (REQ-012)
    void test_transmit_not_ready();

    //! Transmission with SPI failures (REQ-008)
    void test_transmit_spi_failure();

    //! Packet reception and delivery (REQ-006/007/010)
    void test_receive();

    //! Reception with buffer allocation failure (REQ-011)
    void test_receive_allocation_failure();

    //! Uplink buffer return path deallocates buffers
    void test_data_return();

  private:
    // ----------------------------------------------------------------------
    // Handlers for typed from ports
    // ----------------------------------------------------------------------

    //! Handler for from_spiWriteRead: backed by the register-level sim model
    Drv::SpiStatus from_spiWriteRead_handler(FwIndexType portNum,
                                             Fw::Buffer& writeBuffer,
                                             Fw::Buffer& readBuffer) override;

    //! Handler for from_allocate
    Fw::Buffer from_allocate_handler(FwIndexType portNum, FwSizeType size) override;

    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    //! Bring the component to the READY state
    void makeReady();

    //! Connect ports and initialize components (autocoded helpers)
    void connectPorts();
    void initComponents();

    // ----------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------

    //! The component under test
    Rfm69Manager component;

    //! Register-level radio model behind the SPI port
    Rfm69SimModel m_model;

    //! Force SPI transactions to fail
    bool m_spiFail;

    //! Force allocations to fail (return an empty buffer)
    bool m_allocFail;

    //! Backing memory for the allocate port
    U8 m_allocation[ALLOCATION_SIZE];
};

}  // namespace Rfm69

#endif
