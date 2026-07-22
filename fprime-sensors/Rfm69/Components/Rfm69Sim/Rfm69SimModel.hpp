// ======================================================================
// \title  Rfm69SimModel.hpp
// \brief  Register-level model of an RFM69HCW radio
//
// Plain C++ class (no component dependencies) modeling the RFM69HCW
// register file, FIFO, and packet-mode behavior visible over SPI.
// Used by the Rfm69Sim component and by the Rfm69Manager unit tests.
// ======================================================================

#ifndef Rfm69_Rfm69SimModel_HPP
#define Rfm69_Rfm69SimModel_HPP

#include <Fw/FPrimeBasicTypes.hpp>
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Registers.hpp"

namespace Rfm69 {

class Rfm69SimModel {
  public:
    //! Number of modeled registers
    static constexpr FwSizeType REGISTER_COUNT = 0x80;
    //! Bound on buffered air-interface (uplink) bytes
    static constexpr FwSizeType AIR_BUFFER_SIZE = 2048;
    //! Bound on queued transmitted packets awaiting retrieval
    static constexpr FwSizeType TX_QUEUE_DEPTH = 8;
    //! Fixed RSSI register readback (-value/2 dBm => -40 dBm)
    static constexpr U8 RSSI_READBACK = 0x50;

    Rfm69SimModel();

    //! Reset registers, FIFO, and buffers to power-on state
    void reset();

    //! Perform one SPI frame (NSS low..high): full-duplex, first byte address
    void spiTransaction(const U8* mosi, U8* miso, FwSizeType size);

    //! Inject bytes arriving over the simulated air interface (uplink)
    void injectAirData(const U8* data, FwSizeType size);

    //! Retrieve the next transmitted packet payload (downlink).
    //! \return payload size, or 0 when no packet is pending
    FwSizeType retrievePacket(U8* data, FwSizeType capacity);

  private:
    //! Read a register as seen over SPI, applying side effects (FIFO pop)
    U8 readRegister(U8 address);

    //! Write a register over SPI, applying side effects (FIFO push, mode change)
    void writeRegister(U8 address, U8 value);

    //! Handle a mode change written to RegOpMode
    void handleModeChange(U8 mode);

    //! Transmit the packet currently in the FIFO, if complete
    void tryTransmit();

    //! Load the next uplink packet into the FIFO when idle in RX
    void tryLoadReceivePacket();

    //! Current IRQ flag register values
    U8 irqFlags1() const;
    U8 irqFlags2() const;

    U8 m_registers[REGISTER_COUNT];

    // FIFO modeled as a simple queue
    U8 m_fifo[FIFO_SIZE];
    FwSizeType m_fifoCount;
    FwSizeType m_fifoReadIndex;

    // IRQ flag state
    bool m_packetSent;
    bool m_payloadReady;
    bool m_fifoOverrun;

    // Buffered uplink bytes awaiting packetization
    U8 m_airBuffer[AIR_BUFFER_SIZE];
    FwSizeType m_airCount;

    // Queue of transmitted packet payloads awaiting retrieval
    U8 m_txQueue[TX_QUEUE_DEPTH][MAX_PACKET_PAYLOAD + 1];
    FwSizeType m_txSizes[TX_QUEUE_DEPTH];
    FwSizeType m_txCount;
};

}  // namespace Rfm69

#endif  // Rfm69_Rfm69SimModel_HPP
