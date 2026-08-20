// ======================================================================
// \title  GpsManagerTestMain.cpp
// \author starchmd
// \brief  cpp file for GpsManager component test main function
// ======================================================================

#include "GpsManagerTester.hpp"

namespace NmeaGps {
    // From https://en.wikipedia.org/wiki/NMEA_0183
    char GOOD_MESSAGE[] = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76";
    char IGNORED_MESSAGE[] = "$GPGSV,3,3,11,29,09,301,24,16,09,020,,36,,,*76";

    // Modified from aboce
    char FULL_MESSAGE[] = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,12,AAAA*76";
    char BAD_MESSAGE[] = "$GPGGA,,,,,,0,,,,,,,,*76";
    
    const F64 GOOD_LATITUDE = 53.361336666666666;
    const F64 GOOD_LONGITUDE = -6.50562;

    TEST_F(GpsManagerTester, WellFormedMessage) {
        Fw::Buffer data(reinterpret_cast<U8*>(GOOD_MESSAGE), sizeof(GOOD_MESSAGE));
        this->invoke_to_dataIn(0, data);
        ASSERT_EVENTS_InvalidData_SIZE(0);
        ASSERT_EVENTS_MalformedMessage_SIZE(0);
        ASSERT_TLM_Reading_SIZE(1);

        const GpsData& sent = this->tlmHistory_Reading->at(0).arg;
        EXPECT_DOUBLE_EQ(GOOD_LATITUDE, sent.get_latitude());
        EXPECT_DOUBLE_EQ(GOOD_LONGITUDE, sent.get_longitude());
        ASSERT_from_dataReturnOut_SIZE(1);
        ASSERT_EQ(data.getData(), this->fromPortHistory_dataReturnOut->at(0).fwBuffer.getData());
    }


    TEST_F(GpsManagerTester, FullWellFormedMessage) {
        Fw::Buffer data(reinterpret_cast<U8*>(FULL_MESSAGE), sizeof(FULL_MESSAGE));
        this->invoke_to_dataIn(0, data);
        ASSERT_EVENTS_InvalidData_SIZE(0);
        ASSERT_EVENTS_MalformedMessage_SIZE(0);
        ASSERT_TLM_Reading_SIZE(1);

        const GpsData& sent = this->tlmHistory_Reading->at(0).arg;
        EXPECT_DOUBLE_EQ(GOOD_LATITUDE, sent.get_latitude());
        EXPECT_DOUBLE_EQ(GOOD_LONGITUDE, sent.get_longitude());
        ASSERT_from_dataReturnOut_SIZE(1);
        ASSERT_EQ(data.getData(), this->fromPortHistory_dataReturnOut->at(0).fwBuffer.getData());
    }

    TEST_F(GpsManagerTester, IgnoredMessage) {
        Fw::Buffer data(reinterpret_cast<U8*>(IGNORED_MESSAGE), sizeof(IGNORED_MESSAGE));
        this->invoke_to_dataIn(0, data);
        ASSERT_EVENTS_InvalidData_SIZE(0);
        ASSERT_EVENTS_MalformedMessage_SIZE(0);
        ASSERT_TLM_Reading_SIZE(0);
        ASSERT_from_dataReturnOut_SIZE(1);
        ASSERT_EQ(data.getData(), this->fromPortHistory_dataReturnOut->at(0).fwBuffer.getData());
    }

    TEST_F(GpsManagerTester, BadMeessage) {
        Fw::Buffer data(reinterpret_cast<U8*>(BAD_MESSAGE), sizeof(BAD_MESSAGE));
        this->invoke_to_dataIn(0, data);
        ASSERT_EVENTS_InvalidData_SIZE(0);
        ASSERT_EVENTS_MalformedMessage_SIZE(1);
        ASSERT_TLM_Reading_SIZE(0);
        ASSERT_from_dataReturnOut_SIZE(1);
        ASSERT_EQ(data.getData(), this->fromPortHistory_dataReturnOut->at(0).fwBuffer.getData());
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
