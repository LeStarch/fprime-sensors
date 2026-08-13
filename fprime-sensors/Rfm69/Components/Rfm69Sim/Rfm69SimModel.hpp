// ======================================================================
// \title  Rfm69SimModel.hpp
// \brief  Register-level model of an RFM69HCW radio
//
// Plain C++ class (no component dependencies) modeling the RFM69HCW
// register file, FIFO, and packet-mode behavior visible over SPI.
// Used by the Rfm69Sim component and by the Rfm69Manager unit tests.
//
// Time is modeled with a byte clock: each SPI byte exchanged advances
// the modeled air interface by one byte time, so packets larger than
// the FIFO stream through it as the manager polls, mirroring the real
// radio's behavior at a bit rate comparable to the SPI clock.
// ======================================================================

#ifndef Rfm69_Rfm69SimModel_HPP
#define Rfm69_Rfm69SimModel_HPP

#include <Fw/FPrimeBasicTypes.hpp>
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Radio.hpp"

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
    //! Bound on non-FIFO register writes retained for white-box assertions
    static constexpr FwSizeType REGISTER_WRITE_HISTORY_SIZE = 64;

    Rfm69SimModel();

    //! Reset registers, FIFO, and buffers to power-on state
    void reset();

    //! Perform one SPI frame (NSS low..high): full-duplex, first byte address
    void spiTransaction(const U8* mosi, U8* miso, FwSizeType size);

    //! Inject bytes arriving over the simulated air interface (uplink)
    void injectAirData(const U8* data, FwSizeType size);

    //! Inject a frame that fails the hardware CRC. The FIFO streams `payloadLen`
    //! declared bytes with enough trailing noise that FifoLevel stays asserted
    //! through the final read, but PayloadReady never asserts -- matching an
    //! RFM69 with CrcOn + CrcAutoClear on a corrupted/noise frame. Used to verify
    //! the manager drops such frames instead of forwarding them to the deframer.
    void injectCorruptFrame(U8 payloadLen);

    //! Retrieve the next transmitted packet payload (downlink).
    //! \return payload size, or 0 when no packet is pending
    FwSizeType retrievePacket(U8* data, FwSizeType capacity);

    //! Return a register value for white-box unit-test assertions. Dynamic IRQ
    //! registers are synthesized by the model and are therefore intentionally
    //! not exposed through this helper.
    U8 readRegisterValue(U8 address) const;

    //! Suppress PacketSent completion for an in-flight TX packet. This lets
    //! tests exercise the manager's bounded TX wait and RX recovery path.
    void setPacketSentStall(bool stall);

    //! Clear the bounded non-FIFO register-write history.
    void clearRegisterWriteHistory();

    //! Return whether a non-FIFO register write occurred since the history
    //! was last cleared. This exposes transient radio controls such as the
    //! DBM_20 TestPa boost without changing the simulation's functional API.
    bool wasRegisterWritten(U8 address, U8 value) const;

  private:
    //! Advance the modeled air interface by a number of byte times
    void advanceClock(FwSizeType byteTimes);

    //! Read a register as seen over SPI, applying side effects (FIFO pop)
    U8 readRegister(U8 address);

    //! Write a register over SPI, applying side effects (FIFO push, mode change)
    void writeRegister(U8 address, U8 value);

    //! Handle a mode change written to RegOpMode
    void handleModeChange(U8 mode);

    //! Push one byte into the FIFO
    void fifoPush(U8 value);

    //! Pop one byte from the front of the FIFO
    U8 fifoPop();

    //! Begin transmitting the packet whose length byte heads the FIFO
    void startTransmit();

    //! Begin delivering the next uplink packet from the air buffer
    void startReceive();

    //! Current IRQ flag register values
    U8 irqFlags1() const;
    U8 irqFlags2() const;

    U8 m_registers[REGISTER_COUNT];

    // Bounded non-FIFO write history for unit-test observation of transient
    // register values. FIFO data is intentionally excluded.
    U8 m_registerWriteAddresses[REGISTER_WRITE_HISTORY_SIZE];
    U8 m_registerWriteValues[REGISTER_WRITE_HISTORY_SIZE];
    FwSizeType m_registerWriteCount;

    // FIFO modeled as a simple queue (front at index 0)
    U8 m_fifo[FIFO_SIZE];
    FwSizeType m_fifoCount;

    // IRQ flag state
    bool m_packetSent;
    bool m_payloadReady;
    bool m_fifoOverrun;
    bool m_stallPacketSent;
    //! Hold a queued packet between FifoOverrun and RxRestart writes
    bool m_rxRestartPending;

    // In-progress transmission: bytes drain from the FIFO by the byte clock
    bool m_txActive;
    FwSizeType m_txExpected;
    FwSizeType m_txCollected;
    U8 m_txAccum[MAX_PACKET_PAYLOAD];

    // In-progress reception: bytes stream into the FIFO by the byte clock
    bool m_rxActive;
    FwSizeType m_rxTotal;
    FwSizeType m_rxDelivered;
    //! When set, the in-progress reception is a CRC-failing frame: PayloadReady
    //! is never asserted at completion (the RFM69 auto-clears on CRC failure).
    bool m_rxCorrupt;
    U8 m_rxPacket[MAX_PACKET_PAYLOAD + 1];

    // Buffered uplink bytes awaiting packetization
    U8 m_airBuffer[AIR_BUFFER_SIZE];
    FwSizeType m_airCount;

    // Queue of transmitted packet payloads awaiting retrieval
    U8 m_txQueue[TX_QUEUE_DEPTH][MAX_PACKET_PAYLOAD];
    FwSizeType m_txSizes[TX_QUEUE_DEPTH];
    FwSizeType m_txCount;
};

}  // namespace Rfm69

#endif  // Rfm69_Rfm69SimModel_HPP
