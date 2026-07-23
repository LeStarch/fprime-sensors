// ======================================================================
// \title  Rfm69Registers.hpp
// \brief  RFM69HCW register map and bit definitions
//
// Register addresses and values from the RFM69HCW datasheet (HopeRF, V1.1),
// Table 23 (register summary) and section 6 (register descriptions).
// ======================================================================

#ifndef Rfm69_Rfm69Registers_HPP
#define Rfm69_Rfm69Registers_HPP

#include <Fw/FPrimeBasicTypes.hpp>

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
constexpr U8 RX_BW = 0x19;            //!< Channel filter bandwidth
constexpr U8 DIO_MAPPING_2 = 0x26;    //!< DIO4/DIO5 mapping, CLKOUT frequency
constexpr U8 IRQ_FLAGS_1 = 0x27;      //!< Mode/PLL status flags
constexpr U8 IRQ_FLAGS_2 = 0x28;      //!< FIFO handling flags
constexpr U8 RSSI_VALUE = 0x24;       //!< RSSI value (-value/2 dBm)
constexpr U8 RSSI_THRESH = 0x29;      //!< RSSI threshold
constexpr U8 SYNC_CONFIG = 0x2E;      //!< Sync word recognition control
constexpr U8 SYNC_VALUE_1 = 0x2F;     //!< Sync word byte 1
constexpr U8 SYNC_VALUE_2 = 0x30;     //!< Sync word byte 2
constexpr U8 PACKET_CONFIG_1 = 0x37;  //!< Packet mode settings
constexpr U8 PAYLOAD_LENGTH = 0x38;   //!< Max payload length (variable mode)
constexpr U8 FIFO_THRESH = 0x3C;      //!< FIFO threshold, TX start condition
constexpr U8 PACKET_CONFIG_2 = 0x3D;  //!< Packet mode settings
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
//! Maximum payload bytes per variable-length packet: the length byte
//! ranges to 255; packets larger than the FIFO are streamed through it
//! (datasheet section 5.2.2.3)
constexpr FwSizeType MAX_PACKET_PAYLOAD = 255;
//! Bytes safe to burst-write into the FIFO while FifoLevel reads clear
//! (FIFO_SIZE less the threshold, with margin)
constexpr FwSizeType TX_TOP_UP_CHUNK = 48;
//! Frf register step size: 32 MHz crystal / 2^19 (datasheet section 4.2.4)
constexpr U32 CRYSTAL_HZ = 32000000;
constexpr U32 FRF_DIVISOR = 524288;  //!< 2^19

}  // namespace Rfm69

#endif  // Rfm69_Rfm69Registers_HPP
