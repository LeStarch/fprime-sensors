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

TEST(Nominal, TransmitSegmentation) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_segmentation();
}

TEST(Nominal, TransmitLarge) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_large();
}

TEST(Nominal, ReceiveLarge) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_receive_large();
}

TEST(Nominal, TransmitDeferred) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_transmit_deferred();
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

TEST(OffNominal, ReceiveAllocationFailure) {
    Rfm69::Rfm69ManagerTester tester;
    tester.test_receive_allocation_failure();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
