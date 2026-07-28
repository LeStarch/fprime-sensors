// ======================================================================
// \title  Rfm69ModemMaps.hpp
// \brief  Validated RFM69 modem enum-to-register mappings
// ======================================================================

#ifndef Rfm69_Rfm69ModemMaps_HPP
#define Rfm69_Rfm69ModemMaps_HPP

#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ManagerComponentAc.hpp"

namespace Rfm69 {

//! The register values below come from the RFM69 classical modem tables:
//! RegBitrate = 32 MHz / bps and RegFdev = deviation / 61.035 Hz. RegRxBw
//! uses Table 14's FSK mantissa/exponent encoding with DCC frequency 4%.
struct DataRateSetting {
    U16 bitrateReg;
    U32 bitsPerSecond;
};

struct BandwidthSetting {
    U8 rxBw;
    U8 afcBw;
    U32 hertz;
};

struct DeviationSetting {
    U16 fdevReg;
    U32 hertz;
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
    if (value == Rfm69Bandwidth::BW_10_4_KHZ) {
        setting = {0xF5, 0xF5, 10417};
    } else if (value == Rfm69Bandwidth::BW_20_8_KHZ) {
        setting = {0xF4, 0xF4, 20833};
    } else if (value == Rfm69Bandwidth::BW_50_0_KHZ) {
        setting = {0xEB, 0xEB, 50000};
    } else if (value == Rfm69Bandwidth::BW_100_KHZ) {
        setting = {0xEA, 0xEA, 100000};
    } else if (value == Rfm69Bandwidth::BW_200_KHZ) {
        setting = {0xE9, 0xE9, 200000};
    } else if (value == Rfm69Bandwidth::BW_250_KHZ) {
        setting = {0xE1, 0xE1, 250000};
    } else if (value == Rfm69Bandwidth::BW_500_KHZ) {
        setting = {0xE0, 0xE0, 500000};
    } else {
        return false;
    }
    return true;
}

inline bool getDeviationSetting(const Rfm69Deviation& value, DeviationSetting& setting) {
    if (value == Rfm69Deviation::FDEV_5_KHZ) {
        setting = {0x0052, 5000};
    } else if (value == Rfm69Deviation::FDEV_10_KHZ) {
        setting = {0x00A4, 10000};
    } else if (value == Rfm69Deviation::FDEV_25_KHZ) {
        setting = {0x019A, 25000};
    } else if (value == Rfm69Deviation::FDEV_50_KHZ) {
        setting = {0x0333, 50000};
    } else if (value == Rfm69Deviation::FDEV_100_KHZ) {
        setting = {0x0666, 100000};
    } else {
        return false;
    }
    return true;
}

inline bool getModulationShapingRegister(const Rfm69ModulationShaping& value, U8& dataModul) {
    if (value == Rfm69ModulationShaping::FSK_NONE) {
        dataModul = 0x00;
    } else if (value == Rfm69ModulationShaping::GFSK_BT_1_0) {
        dataModul = 0x01;
    } else if (value == Rfm69ModulationShaping::GFSK_BT_0_5) {
        dataModul = 0x02;
    } else if (value == Rfm69ModulationShaping::GFSK_BT_0_3) {
        dataModul = 0x03;
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

inline bool modemSettingsAreCompatible(const DataRateSetting& dataRate,
                                       const BandwidthSetting& bandwidth,
                                       const DeviationSetting& deviation) {
    // Datasheet condition: bit rate must be less than twice the receive BW.
    // A deviation as wide as the receive filter cannot be demodulated cleanly.
    return (dataRate.bitsPerSecond < (2U * bandwidth.hertz)) && (deviation.hertz < bandwidth.hertz);
}

}  // namespace Rfm69

#endif  // Rfm69_Rfm69ModemMaps_HPP
