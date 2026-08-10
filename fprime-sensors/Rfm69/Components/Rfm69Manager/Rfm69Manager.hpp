// ======================================================================
// \title  Rfm69Manager.hpp
// \brief  hpp file for Rfm69Manager component implementation class
// ======================================================================

#ifndef Rfm69_Rfm69Manager_HPP
#define Rfm69_Rfm69Manager_HPP

#include "Os/Mutex.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ManagerComponentAc.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Radio.hpp"

namespace Rfm69 {

class Rfm69Manager final : public Rfm69ManagerComponentBase {
  public:
    //! Maximum number of bounded scheduler advances allowed while waiting for
    //! a radio mode transition. Each advance performs at most one status read.
    static constexpr U32 MODE_READY_TIMEOUT_TICKS = 20;
    //! Bound on packets completed per run invocation
    static constexpr U32 RX_PACKETS_PER_TICK = 1;
    //! Ticks of SyncAddressMatch without FifoLevel/PayloadReady before re-arm.
    //! Must exceed airtime to fill FIFO_THRESHOLD at the slowest supported rate
    //! (BR_1200: ~100 ms for 15 bytes); 250 ms leaves margin.
    static constexpr U32 RX_STALE_SYNC_TICKS = 250;
    //! After a completed uplink packet, defer downlink this many 1 kHz ticks.
    //! This is an RX-priority lease for the half-duplex link: each packet in a
    //! file burst refreshes it, so flight cannot begin a queued downlink in the
    //! gap before the next ground chunk (~450 ms in the HIL profile). Once the
    //! burst ends, TX resumes automatically and drains queued TM/events.
    static constexpr U32 RX_TX_HOLDOFF_TICKS = 600;
    //! Minimum quiet time between consecutive flight downlink packets. The
    //! peer must drain the CRC-accepted FIFO, restart RX, and reacquire sync;
    //! without this gap a short file-uplink handshake can be hidden behind an
    //! adjacent telemetry packet even though both flight transmissions succeed.
    static constexpr U32 TX_TX_HOLDOFF_TICKS = 50;

    // Fixed native-packet modem profile. DATA_RATE, BANDWIDTH_RX, and TX_POWER
    // are the only operator parameters; these values are deliberately compiled
    // into both the flight and ground-station implementations.
    static constexpr U8 FIXED_DATA_MODUL = 0x00;        //!< Packet FSK, no shaping
    static constexpr U16 FIXED_FDEV_REGISTER = 0x019A; //!< 25 kHz deviation
    static constexpr U32 FIXED_FREQUENCY_HZ = 915000000U;

    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct Rfm69Manager object
    Rfm69Manager(const char* const compName  //!< The component name
    );

    //! Destroy Rfm69Manager object
    ~Rfm69Manager();

  private:
    //! Software lifecycle: DETECT → CONFIGURE → READY
    //! (advanced at parametersLoaded; run retries after RESET / param updates)
    enum RadioState {
        DETECT,     //!< Looking for RegVersion on SPI
        CONFIGURE,  //!< Writing profile + entering RX
        READY       //!< Polling RX / accepting TX
    };

    //! Non-blocking downlink sequence. No state performs an airtime wait: the
    //! 1 kHz scheduler advances the sequence with bounded SPI transactions.
    enum TxState {
        TX_IDLE,
        TX_REQUEST_STANDBY,
        TX_WAIT_STANDBY,
        TX_CLEAR_FIFO,
        TX_WRITE_LENGTH,
        TX_LOAD_INITIAL,
        TX_ENABLE_BOOST,
        TX_REQUEST_MODE,
        TX_WAIT_MODE,
        TX_STREAM,
        TX_REQUEST_RX,
        TX_WAIT_RX,
        TX_DISABLE_BOOST,
        TX_ABORT_CLEAR_FIFO,
        TX_ABORT_REQUEST_RX,
        TX_ABORT_WAIT_RX,
        TX_ABORT_DISABLE_BOOST
    };

    enum TxProgress {
        TX_IN_PROGRESS,
        TX_SUCCEEDED,
        TX_FAILED
    };

    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for dataIn: transmit a frame over the radio
    void dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;

    //! Handler implementation for dataReturnIn: buffer ownership returned
    void dataReturnIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;

    //! Handler implementation for run: receive polling (or init retry if not READY)
    void run_handler(FwIndexType portNum, U32 context) override;

    // ----------------------------------------------------------------------
    // Command handler implementations
    // ----------------------------------------------------------------------

    //! Handler for the TRANSMIT command: enable/disable downlink
    void TRANSMIT_cmdHandler(FwOpcodeType opCode,     //!< The command opcode
                             U32 cmdSeq,              //!< The command sequence number
                             const Rfm69::TransmitState& enabled  //!< Desired transmit state
                             ) override;

    //! Handler for RESET: pulse hardware RST and reinitialize the radio
    void RESET_cmdHandler(FwOpcodeType opCode,  //!< The command opcode
                          U32 cmdSeq            //!< The command sequence number
                          ) override;

    //! Schedule CONFIGURE on the next run tick after a parameter set
    void parameterUpdated(FwPrmIdType id  //!< The parameter ID
                          ) override;

    //! Mark parameters loaded and bring the radio to READY before rate groups start
    void parametersLoaded() override;

    // ----------------------------------------------------------------------
    // Helper functions: radio state management (Rfm69Manager.cpp)
    // ----------------------------------------------------------------------

    //! Advance detection/configuration toward READY
    void initializeRadio();

    //! Poll for and deliver received packets (budgeted per 1 kHz tick)
    void pollReceive();

    //! Continue an in-progress FIFO drain; returns true when a full packet is ready
    bool continueReceiveDrain();

    //! Start a new drain after validating IRQ flags; returns true if draining
    bool beginReceiveDrain(U8 flags2);

    //! Deliver m_rxPayload[0..m_rxReceived) to the uplink Com path
    void deliverReceivedPacket();

    //! Abort an in-progress drain and re-arm the receiver
    void abortReceiveDrain();

    //! Request a reset pulse from the platform GPIO driver, if connected.
    //! \return true if unconnected or the HIGH/LOW pulse succeeded; false on
    //!         GPIO error. Callers must not treat false as fatal — reset is optional.
    bool pulseReset();

    //! Stage one native RF packet for scheduler-driven transmission.
    bool startTransmit(Fw::Buffer& data, const ComCfg::FrameContext& context);

    //! Advance the scheduler-driven transmit sequence by one bounded step.
    TxProgress advanceTransmit();

    //! Enter bounded abort recovery after a transmit SPI/timeout failure.
    void abortTransmit();

    //! Non-blocking bus lock for run: returns false if TX (or other owner) holds it
    bool tryAcquireBus();

    // ----------------------------------------------------------------------
    // Helper functions: RFM69 register access (Rfm69Helpers.cpp)
    // ----------------------------------------------------------------------

    //! \brief Read a single radio register
    Drv::SpiStatus readRegister(U8 address,  //!< Register address
                                U8& value    //!< [out] Register value
    );

    //! \brief Write a single radio register
    Drv::SpiStatus writeRegister(U8 address,  //!< Register address
                                 U8 value     //!< Value to write
    );

    //! \brief Detect the radio by checking the version register
    bool detectRadio();

    //! \brief Apply the packet-mode configuration to the radio
    bool configureRadio();

    //! \brief Command an operating mode and await ModeReady
    bool setMode(U8 mode  //!< Mode field value (Rfm69::Mode)
    );

    //! Request an operating mode without waiting for ModeReady.
    bool requestMode(U8 mode  //!< Mode field value (Rfm69::Mode)
    );

    //! Clear FIFO/re-arm packet RX after a timeout or malformed packet
    bool recoverReceive();

    //! Enable or restore the short-duration +20 dBm PA boost path.
    bool setPowerBoost(bool enabled);

    //! \brief True when a reception is in progress (defer TX via Com FAILURE)
    bool channelBusy();

    //! \brief True when downlink should wait (RX in progress or post-RX holdoff)
    bool downlinkBlocked();

    //! Emit comStatusOut when connected
    void emitComStatus(Fw::Success status);

    //! If a prior FAILURE left ComQueue waiting, clear resume flag once ready.
    //! Returns true when the caller should emit SUCCESS after releasing m_lock.
    bool maybeResumeComStatus();

    //! \brief Burst-write a block of payload bytes into the FIFO
    bool writeFifo(const U8* data,  //!< Bytes to load
                   FwSizeType size  //!< Byte count
    );

    //! Burst-read bytes from the FIFO.
    bool readFifo(U8* data,  //!< Destination bytes
                  FwSizeType size //!< Byte count
    );

    //! \brief Read the RSSI register and update telemetry
    void updateRssi();

    // ----------------------------------------------------------------------
    // Member variables
    // ----------------------------------------------------------------------

    //! Serializes radio access between dataIn (com queue thread) and run
    //! (rate group thread): both drive multi-transaction SPI sequences and
    //! share the SPI scratch buffers
    Os::Mutex m_lock;

    //! True while a downlink buffer is owned by the scheduler-driven TX state machine.
    bool m_txActive;
    TxState m_txState;
    Fw::Buffer m_txBuffer;
    ComCfg::FrameContext m_txContext;
    FwSizeType m_txOffset;
    U32 m_txWaitTicks;
    U32 m_txElapsedTicks;
    U32 m_txTimeoutTicks;
    bool m_txBoostEnabled;

    //! True while streaming a variable-length RX packet across run ticks
    bool m_rxDraining;
    U8 m_rxLength;              //!< Declared payload length from the length byte
    FwSizeType m_rxReceived;    //!< Bytes of payload copied so far
    //! True once PayloadReady was observed while draining the current packet.
    //! With CrcOn + CrcAutoClear (RegPacketConfig1=0xD0) the RFM69 only asserts
    //! PayloadReady on a CRC pass and flushes the FIFO on a CRC failure, so a
    //! packet whose byte count completes without PayloadReady is corrupt and must
    //! be dropped rather than handed to the deframer.
    bool m_rxSawPayloadReady;
    U8 m_rxPayload[MAX_PACKET_PAYLOAD];
    //! Divider state and bitrate-derived idle/streaming SPI poll periods.
    //! Idle polling only needs to notice a packet before the 66-byte FIFO fills;
    //! streaming polling must drain faster than bytes arrive.
    U32 m_rxPollTicks;
    U32 m_rxIdlePollDivisor;
    U32 m_rxActivePollDivisor;
    //! Consecutive ticks seeing sync+fifo without FifoLevel/PayloadReady
    U32 m_rxStaleSyncTicks;
    //! Remaining 1 kHz ticks to defer downlink after a completed uplink packet
    U32 m_rxTxHoldoffTicks;

    RadioState m_state;        //!< Radio management state
    bool m_configured;         //!< FPP parameters have been loaded
    U32 m_packetsTransmitted;  //!< Count of transmitted packets
    U32 m_packetsReceived;     //!< Count of received packets
    U32 m_rxCrcErrors;         //!< Count of RX frames dropped for failing CRC
    TransmitState m_transmitEnabled;  //!< Whether downlink transmit is permitted
    bool m_resetPulsed;                //!< True once optional RST was attempted (success or fail)
    bool m_comStatusAnnounced;          //!< Initial link-ready status has been sent
    //! Set after reporting FAILURE so run can emit SUCCESS when downlink is
    //! allowed again (holdoff expired, radio ready).
    bool m_comResumeNeeded;

    //! Scratch buffers for SPI transactions (address byte + length + payload)
    U8 m_mosi[MAX_PACKET_PAYLOAD + 2];
    U8 m_miso[MAX_PACKET_PAYLOAD + 2];
};

}  // namespace Rfm69

#endif  // Rfm69_Rfm69Manager_HPP
