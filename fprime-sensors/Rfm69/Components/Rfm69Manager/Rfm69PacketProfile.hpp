// ======================================================================
// \title  Rfm69PacketProfile.hpp
// \brief  Invariant native-packet settings for Rfm69Manager
// ======================================================================

#ifndef Rfm69_Rfm69PacketProfile_HPP
#define Rfm69_Rfm69PacketProfile_HPP

#include <Fw/FPrimeBasicTypes.hpp>

namespace Rfm69 {

//! These packet-handler settings are deliberately not operator parameters.
//! DATA_RATE, BANDWIDTH_RX, and TX_POWER are the only operator-selectable
//! radio settings;
//! the native F´ packet contract and sync word remain invariant.
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
    0xD0,  // variable length, whitening, CRC, no address filter
    0x0F,
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

#endif  // Rfm69_Rfm69PacketProfile_HPP
