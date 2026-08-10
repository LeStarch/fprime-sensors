// ----------------------------------------------------------------------
// Rfm69ManagerTestMain.cpp
// ----------------------------------------------------------------------

#include "Rfm69ManagerTester.hpp"

TEST(Nominal, Initialization) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_initialization();
}

TEST(Nominal, Transmit) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit();
}

TEST(OffNominal, TransmitZeroSize) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_zero_size();
}

TEST(OffNominal, TransmitOversize) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_oversize();
}

TEST(Nominal, TransmitLarge) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_large();
}

TEST(Nominal, TransmitFifoFit) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_fifo_fit();
}

TEST(Nominal, ReceiveLarge) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_receive_large();
}

TEST(Nominal, ReceiveFifoFit) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_receive_fifo_fit();
}

TEST(Nominal, DefaultRegisterImage) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_default_register_image();
}

TEST(Nominal, HighPowerBoostRecovery) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_high_power_boost_recovery();
}

TEST(Nominal, DataRateRegisterMap) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_data_rate_register_map();
}

TEST(Nominal, BandwidthUpdateReconfigure) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_bandwidth_update_reconfigure();
}

TEST(Nominal, BandwidthRegisterMap) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_bandwidth_register_map();
}

TEST(Nominal, ParameterUpdateReconfigure) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_parameter_update_reconfigure();
}

TEST(Nominal, ResetRecovery) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_reset_recovery();
}

TEST(OffNominal, TransmitDroppedWhenBusy) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_dropped_when_busy();
}

TEST(Nominal, Receive) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_receive();
}

TEST(Nominal, DataReturn) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_data_return();
}

TEST(OffNominal, DetectionRetry) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_detection_retry();
}

TEST(OffNominal, TransmitNotReady) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_not_ready();
}

TEST(OffNominal, TransmitSpiFailure) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_spi_failure();
}

TEST(OffNominal, TransmitTimeoutRecovery) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_timeout_recovery();
}

TEST(OffNominal, ReceiveAllocationFailure) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_receive_allocation_failure();
}

TEST(OffNominal, ReceiveCrcDrop) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_receive_crc_drop();
}

TEST(Nominal, TransmitDisabled) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_disabled();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
