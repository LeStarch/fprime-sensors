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
    //! Large enough for a slow-rate scheduler-driven TX plus configure/setup SPI.
    static const U32 MAX_HISTORY_SIZE = 50000;

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

    //! Zero-length frames are rejected: an RF packet must contain 1..255 bytes
    void test_transmit_zero_size();

    //! Frames above the native 255-byte RF payload limit are rejected
    void test_transmit_oversize();

    //! Maximum-size (255-byte) packet streamed through the FIFO (REQ-004)
    void test_transmit_large();

    //! FIFO-fit payload (65 B): single standby fill, no in-TX top-up
    void test_transmit_fifo_fit();

    //! Maximum-size (255-byte) packet received through the FIFO (REQ-006)
    void test_receive_large();

    //! FIFO-fit payload received in one PayloadReady drain
    void test_receive_fifo_fit();

    //! Default enum values program the canonical flight/ground register image
    void test_default_register_image();

    //! DBM_20 uses TestPa boost only for a TX and restores safe RX values
    void test_high_power_boost_recovery();

    //! Every exposed data-rate enum maps to the intended bitrate register
    void test_data_rate_register_map();

    //! A live BANDWIDTH_RX parameter update schedules and completes reconfiguration
    void test_bandwidth_update_reconfigure();

    //! Every exposed bandwidth enum maps to the intended paired RX/AFC registers
    void test_bandwidth_register_map();

    //! A live DATA_RATE parameter update schedules and completes reconfiguration
    void test_parameter_update_reconfigure();

    //! RESET pulses RST and returns the radio to READY without a process restart
    void test_reset_recovery();

    //! Half-duplex: TX while RX is in progress is dropped immediately
    void test_transmit_dropped_when_busy();

    //! Transmission before the radio is ready (REQ-012)
    void test_transmit_not_ready();

    //! Transmission with SPI failures (REQ-008)
    void test_transmit_spi_failure();

    //! A missing PacketSent response times out and returns the radio to RX
    void test_transmit_timeout_recovery();

    //! Packet reception and delivery (REQ-006/007/010)
    void test_receive();

    //! Reception with buffer allocation failure (REQ-011)
    void test_receive_allocation_failure();

    //! A CRC-failing frame is dropped, counted, and never forwarded downstream
    void test_receive_crc_drop();

    //! Uplink buffer return path deallocates buffers
    void test_data_return();

    //! TRANSMIT command gives a commandable receive-only window
    void test_transmit_disabled();

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

    //! Advance run ticks until a staged TX reports SUCCESS or FAILURE.
    void runUntilTransmitCompletes(U32 maxTicks = 4096);

    //! Seed the generated parameter source with the canonical default profile
    void setDefaultParameters();

    //! Seed the three FPP parameters before component.loadParameters()
    void setParameters(const Rfm69DataRate& dataRate,
                       const Rfm69Bandwidth& bandwidthRx,
                       const Rfm69TxPower& txPower);

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
