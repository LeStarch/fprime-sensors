// ======================================================================
// \title  Rfm69Radio.hpp
// \brief  RFM69HCW register map, modem enum maps, and fixed packet profile
//
// Register addresses and values from the RFM69HCW datasheet (HopeRF, V1.1),
// Table 23 (register summary) and section 6 (register descriptions).
// ======================================================================

#ifndef Rfm69_Rfm69Radio_HPP
#define Rfm69_Rfm69Radio_HPP

#include <Fw/FPrimeBasicTypes.hpp>
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ManagerComponentAc.hpp"

namespace Rfm69 {

//! RFM69 register addresses (datasheet Table 23)
namespace Reg {
constexpr U8 FIFO = 0x00;             //!< FIFO read/write access
constexpr U8 OP_MODE = 0x01;          //!< Operating modes of the transceiver
constexpr U8 DATA_MODUL = 0x02;       //!< Data operation mode and modulation
constexpr U8 BITRATE_MSB = 0x03;      //!< Bit rate setting, MSB
constexpr U8 BITRATE_LSB = 0x04;      //!< Bit rate setting, LSB
constexpr U8 FDEV_MSB = 0x05;         //!< Frequency deviation, MSB
constexpr U8 FDEV_LSB = 0x06;         //!< Frequency deviation, LSB
constexpr U8 FRF_MSB = 0x07;          //!< RF carrier frequency, MSB
constexpr U8 FRF_MID = 0x08;          //!< RF carrier frequency, middle byte
constexpr U8 FRF_LSB = 0x09;          //!< RF carrier frequency, LSB
constexpr U8 VERSION = 0x10;          //!< Silicon version
constexpr U8 PA_LEVEL = 0x11;         //!< PA selection and output power
constexpr U8 OCP = 0x13;              //!< Over-current protection control
constexpr U8 RX_BW = 0x19;            //!< Channel filter bandwidth
constexpr U8 AFC_BW = 0x1A;           //!< AFC channel filter bandwidth
constexpr U8 DIO_MAPPING_2 = 0x26;    //!< DIO4/DIO5 mapping, CLKOUT frequency
constexpr U8 IRQ_FLAGS_1 = 0x27;      //!< Mode/PLL status flags
constexpr U8 IRQ_FLAGS_2 = 0x28;      //!< FIFO handling flags
constexpr U8 RSSI_VALUE = 0x24;       //!< RSSI value (-value/2 dBm)
constexpr U8 RSSI_THRESH = 0x29;      //!< RSSI threshold
constexpr U8 PREAMBLE_MSB = 0x2C;     //!< Preamble length, MSB
constexpr U8 PREAMBLE_LSB = 0x2D;     //!< Preamble length, LSB
constexpr U8 SYNC_CONFIG = 0x2E;      //!< Sync word recognition control
constexpr U8 SYNC_VALUE_1 = 0x2F;     //!< Sync word byte 1
constexpr U8 SYNC_VALUE_2 = 0x30;     //!< Sync word byte 2
constexpr U8 SYNC_VALUE_3 = 0x31;     //!< Sync word byte 3
constexpr U8 SYNC_VALUE_4 = 0x32;     //!< Sync word byte 4
constexpr U8 SYNC_VALUE_5 = 0x33;     //!< Sync word byte 5
constexpr U8 SYNC_VALUE_6 = 0x34;     //!< Sync word byte 6
constexpr U8 SYNC_VALUE_7 = 0x35;     //!< Sync word byte 7
constexpr U8 SYNC_VALUE_8 = 0x36;     //!< Sync word byte 8
constexpr U8 PACKET_CONFIG_1 = 0x37;  //!< Packet mode settings
constexpr U8 PAYLOAD_LENGTH = 0x38;   //!< Max payload length (variable mode)
constexpr U8 FIFO_THRESH = 0x3C;      //!< FIFO threshold, TX start condition
constexpr U8 PACKET_CONFIG_2 = 0x3D;  //!< Packet mode settings
constexpr U8 TEST_PA_1 = 0x5A;        //!< High-power PA test register 1
constexpr U8 TEST_PA_2 = 0x5C;        //!< High-power PA test register 2
constexpr U8 TEST_DAGC = 0x6F;        //!< Fading margin improvement
}  // namespace Reg

//! RegOpMode mode field values (bits 4-2)
namespace Mode {
constexpr U8 SLEEP = 0x00;
constexpr U8 STANDBY = 0x04;
constexpr U8 FS = 0x08;
constexpr U8 TX = 0x0C;
constexpr U8 RX = 0x10;
constexpr U8 MASK = 0x1C;  //!< Mode bits within RegOpMode
}  // namespace Mode

//! RegIrqFlags1 bit masks
namespace IrqFlags1 {
constexpr U8 MODE_READY = 0x80;
constexpr U8 RX_READY = 0x40;
constexpr U8 TX_READY = 0x20;
constexpr U8 RSSI = 0x08;                //!< RssiValue exceeded RssiThreshold
constexpr U8 SYNC_ADDRESS_MATCH = 0x01;  //!< Sync word detected: reception in progress
}  // namespace IrqFlags1

//! RegIrqFlags2 bit masks
namespace IrqFlags2 {
constexpr U8 FIFO_FULL = 0x80;
constexpr U8 FIFO_NOT_EMPTY = 0x40;
constexpr U8 FIFO_LEVEL = 0x20;  //!< FIFO fill exceeds FifoThreshold
constexpr U8 FIFO_OVERRUN = 0x10;
constexpr U8 PACKET_SENT = 0x08;
constexpr U8 PAYLOAD_READY = 0x04;
}  // namespace IrqFlags2

//! RegPacketConfig2 commands
namespace PacketConfig2 {
constexpr U8 AUTO_RX_RESTART_ON = 0x02;
constexpr U8 RX_RESTART = 0x04;
}  // namespace PacketConfig2

//! PA and high-power test values. DBM_20 enables boost only during TX and
//! restores normal/OCP-protected values immediately after returning to RX.
namespace Pa {
constexpr U8 OCP_NORMAL = 0x1A;
constexpr U8 OCP_DISABLED = 0x0F;
constexpr U8 TEST_PA_1_NORMAL = 0x55;
constexpr U8 TEST_PA_1_BOOST = 0x5D;
constexpr U8 TEST_PA_2_NORMAL = 0x70;
constexpr U8 TEST_PA_2_BOOST = 0x7C;
}  // namespace Pa

//! SPI address byte: write access flag (wnr bit, datasheet section 5.2.1)
constexpr U8 SPI_WRITE_FLAG = 0x80;
//! SPI address byte: register address mask
constexpr U8 SPI_ADDRESS_MASK = 0x7F;

//! Expected RegVersion value for the RFM69HCW
constexpr U8 VERSION_VALUE = 0x24;
//! Hardware FIFO size in bytes (datasheet section 5.2.2.2)
constexpr FwSizeType FIFO_SIZE = 66;
//! FIFO threshold programmed into RegFifoThresh (FifoLevel trip point)
constexpr U8 FIFO_THRESHOLD = 0x0F;
//! RFM69 variable-length packet payload maximum. Packets above the 66-byte
//! hardware FIFO are streamed while in flight.
constexpr FwSizeType MAX_PACKET_PAYLOAD = 255;
//! Bytes safe to burst-write into the FIFO while FifoLevel reads clear
//! (FIFO_SIZE less the threshold, with margin)
constexpr FwSizeType TX_TOP_UP_CHUNK = 48;
//! Frf register step size: 32 MHz crystal / 2^19 (datasheet section 4.2.4)
constexpr U32 CRYSTAL_HZ = 32000000;
constexpr U32 FRF_DIVISOR = 524288;  //!< 2^19

//! The register values below come from the RFM69 classical modem tables.
//! DATA_RATE, BANDWIDTH_RX, and TX_POWER are operator-selectable; the
//! remaining modem registers live in Rfm69Manager's fixed native-packet profile.
struct DataRateSetting {
    U16 bitrateReg;
    U32 bitsPerSecond;
};

struct BandwidthSetting {
    U8 rxBw;
    U8 afcBw;
};

struct TxPowerSetting {
    U8 paLevel;
    bool boost20dBm;
};

inline bool getDataRateSetting(const Rfm69DataRate& value, DataRateSetting& setting) {
    if (value == Rfm69DataRate::BR_1200) {
        setting = {0x682B, 1200};
    } else if (value == Rfm69DataRate::BR_4800) {
        setting = {0x1A0B, 4800};
    } else if (value == Rfm69DataRate::BR_9600) {
        setting = {0x0D05, 9600};
    } else if (value == Rfm69DataRate::BR_19200) {
        setting = {0x0683, 19200};
    } else if (value == Rfm69DataRate::BR_38400) {
        setting = {0x0341, 38400};
    } else {
        return false;
    }
    return true;
}

inline bool getBandwidthSetting(const Rfm69Bandwidth& value, BandwidthSetting& setting) {
    if (value == Rfm69Bandwidth::BW_100_KHZ) {
        setting = {0xEA, 0xEA};
    } else if (value == Rfm69Bandwidth::BW_200_KHZ) {
        setting = {0xE9, 0xE9};
    } else if (value == Rfm69Bandwidth::BW_250_KHZ) {
        setting = {0xE1, 0xE1};
    } else if (value == Rfm69Bandwidth::BW_500_KHZ) {
        setting = {0xE0, 0xE0};
    } else {
        return false;
    }
    return true;
}

inline bool getTxPowerSetting(const Rfm69TxPower& value, TxPowerSetting& setting) {
    // DBM_0..13: PA1. DBM_17: PA1+PA2. DBM_20: PA1+PA2 + boost only while TX.
    if (value == Rfm69TxPower::DBM_0) {
        setting = {0x52, false};  // PA1, -18 + 18 dBm
    } else if (value == Rfm69TxPower::DBM_5) {
        setting = {0x57, false};
    } else if (value == Rfm69TxPower::DBM_10) {
        setting = {0x5C, false};
    } else if (value == Rfm69TxPower::DBM_13) {
        setting = {0x5F, false};
    } else if (value == Rfm69TxPower::DBM_17) {
        setting = {0x7F, false};  // PA1 + PA2, normal test PA settings
    } else if (value == Rfm69TxPower::DBM_20) {
        setting = {0x7F, true};   // PA1 + PA2, boost only while transmitting
    } else {
        return false;
    }
    return true;
}

//! These packet-handler settings are deliberately not operator parameters.
//! DATA_RATE, BANDWIDTH_RX, and TX_POWER are the only operator-selectable
//! radio settings; the native F´ packet contract and sync word remain invariant.
struct PacketProfile {
    U16 preambleBytes;
    U8 sync[8];
    U8 packetConfig1;
    U8 fifoThreshold;
    U8 packetConfig2;
    U8 testDagc;
};

constexpr PacketProfile NATIVE_PACKET_PROFILE = {
    4,
    {0x2D, 0xA7, 0x5C, 0x39, 0xD1, 0x6E, 0x84, 0xF2},
    0xD0,  // PacketFormat=variable, DcFree=whitening, CrcOn; AES/unlimited off
    0x0F,  // FifoThresh; configureRadio ORs TxStartCondition → RegFifoThresh=0x8F
    0x02,  // AutoRxRestartOn
    0x30,
};

//! Preamble + sync + length + CRC bytes are on-air in addition to payload.
constexpr U32 PACKET_FIXED_AIR_BYTES = 4 + 8 + 1 + 2;

constexpr U32 packetAirtimeUsec(FwSizeType payloadBytes, U32 bitrateBps) {
    return static_cast<U32>(
        ((static_cast<U64>(payloadBytes + PACKET_FIXED_AIR_BYTES) * 8U * 1000000U) + bitrateBps - 1U) /
        bitrateBps);
}

}  // namespace Rfm69

#endif  // Rfm69_Rfm69Radio_HPP
