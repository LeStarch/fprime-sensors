// ======================================================================
// \title  Rfm69ModemMaps.hpp
// \brief  Validated RFM69 modem enum-to-register mappings
// ======================================================================

#ifndef Rfm69_Rfm69ModemMaps_HPP
#define Rfm69_Rfm69ModemMaps_HPP

#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ManagerComponentAc.hpp"

namespace Rfm69 {

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

}  // namespace Rfm69

#endif  // Rfm69_Rfm69ModemMaps_HPP
