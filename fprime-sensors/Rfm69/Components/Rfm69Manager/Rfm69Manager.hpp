// ======================================================================
// \title  Rfm69Manager.hpp
// \brief  hpp file for Rfm69Manager component implementation class
// ======================================================================

#ifndef Rfm69_Rfm69Manager_HPP
#define Rfm69_Rfm69Manager_HPP

#include "Os/Mutex.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/BringupMachineStateMachineAc.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ManagerComponentAc.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Radio.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/TransmitMachineStateMachineAc.hpp"

namespace Rfm69 {

class Rfm69Manager final : public Rfm69ManagerComponentBase {
  public:
    //! Maximum number of bounded scheduler advances allowed while waiting for
    //! a radio mode transition. Each advance performs at most one status read.
    static constexpr U32 MODE_READY_TIMEOUT_TICKS = 20;
    //! Ticks of SyncAddressMatch without FifoLevel/PayloadReady before re-arm.
    //! Must exceed airtime to fill FIFO_THRESHOLD at the slowest supported rate
    //! (BR_1200: ~100 ms for 15 bytes); 250 ms leaves margin.
    static constexpr U32 RX_STALE_SYNC_TICKS = 250;
    //! After a completed uplink packet, defer downlink this many 1 kHz ticks.
    //! This is an RX-priority lease for the half-duplex link: each packet in a
    //! file burst refreshes it, so flight cannot begin a queued downlink in the
    //! gap before the next ground chunk (~450 ms in the HIL profile). Once the
    //! burst ends, TX resumes automatically and drains queued TM/events.
    //! Ground-station local pending-TX hold ignores this timer (channelBusy only).
    static constexpr U32 RX_TX_HOLDOFF_TICKS = 500;
    //! Quiet time between consecutive local transmits (1 kHz ticks). At
    //! 19.2 kbps a ~150 B packet is ~60 ms on air; the peer then needs to
    //! drain FIFO, run FileUplink, and re-arm RX. Keep this under the GDS
    //! file-uplink cooldown (1.0 s) so paced chunks are unchanged and a UART
    //! burst cannot drain pending TX back-to-back.
    static constexpr U32 TX_TX_HOLDOFF_TICKS = 500;
    //! Upper bound on consecutive ticks a pending downlink may remain blocked
    //! by the RX-priority lease. Periodic uplink (e.g. GDS keepalives every
    //! 0.5 s) re-arms RX_TX_HOLDOFF_TICKS on every received packet, which
    //! would otherwise starve telemetry indefinitely. When the cap expires,
    //! the lease is dropped so one downlink window opens; an RX actively
    //! clocking in (channelBusy) is still honored.
    static constexpr U32 RX_HOLDOFF_STARVATION_TICKS = 2 * RX_TX_HOLDOFF_TICKS;

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
    //! RST must be held HIGH at least 100 us (datasheet section 7.2.2); one
    //! 1 kHz tick provides 1 ms with margin.
    static constexpr U32 RESET_HOLD_TICKS = 1;
    //! The radio needs 5 ms after RST release before use (datasheet 7.2.2).
    static constexpr U32 RESET_SETTLE_TICKS = 6;
    //! Register profile writes per CONFIGURE tick; bounds SPI work so the
    //! full-profile rewrite never exceeds the 1 kHz scheduler budget.
    static constexpr FwSizeType CONFIG_WRITES_PER_TICK = 4;
    //! Staged modem profile entries: 33 register writes plus the FIFO flush.
    static constexpr FwSizeType CONFIG_TABLE_SIZE = 34;

    enum TxProgress {
        TX_IN_PROGRESS,
        TX_SUCCEEDED,
        TX_FAILED
    };

    //! One staged register write of the modem profile.
    struct RegisterWrite {
        U8 address;
        U8 value;
    };

    //! Bring-up sequencing (BringupMachine in Rfm69Manager.fpp). Actions and
    //! guards delegate to the owning manager; ticks arrive under m_lock.
    class BringupSm final : public BringupMachineStateMachineBase {
      public:
        explicit BringupSm(Rfm69Manager& manager);

      private:
        void action_doAssertReset(Signal signal) final;
        void action_doHoldTick(Signal signal) final;
        void action_doSettleTick(Signal signal) final;
        void action_doDetect(Signal signal) final;
        void action_doPrepareReset(Signal signal) final;
        void action_doPrepareConfigure(Signal signal) final;
        void action_doConfigureStep(Signal signal) final;
        void action_doPollModeReady(Signal signal) final;
        void action_doConfigureFailed(Signal signal) final;
        void action_doAnnounceReady(Signal signal) final;

        bool guard_resetLineHeld(Signal signal) const final;
        bool guard_holdElapsed(Signal signal) const final;
        bool guard_settleElapsed(Signal signal) const final;
        bool guard_radioDetected(Signal signal) const final;
        bool guard_profileWritten(Signal signal) const final;
        bool guard_modeReady(Signal signal) const final;
        bool guard_modeWaitFailed(Signal signal) const final;

        Rfm69Manager& m_manager;
    };

    //! Downlink sequencing (TransmitMachine in Rfm69Manager.fpp). Actions and
    //! guards delegate to the owning manager; signals arrive under m_lock.
    class TransmitSm final : public TransmitMachineStateMachineBase {
      public:
        explicit TransmitSm(Rfm69Manager& manager);

      private:
        void action_doRequestStandby(Signal signal) final;
        void action_doPollModeReady(Signal signal) final;
        void action_doClearFifo(Signal signal) final;
        void action_doWriteLength(Signal signal) final;
        void action_doLoadInitial(Signal signal) final;
        void action_doEnableBoost(Signal signal) final;
        void action_doRequestTx(Signal signal) final;
        void action_doStream(Signal signal) final;
        void action_doRequestRx(Signal signal) final;
        void action_doDisableBoost(Signal signal) final;
        void action_doFinishSuccess(Signal signal) final;
        void action_doBeginAbort(Signal signal) final;
        void action_doAbortClearFifo(Signal signal) final;
        void action_doAbortRequestRx(Signal signal) final;
        void action_doPollAbortRx(Signal signal) final;
        void action_doAbortDisableBoost(Signal signal) final;
        void action_doFinishFailure(Signal signal) final;

        bool guard_stepOk(Signal signal) const final;
        bool guard_modeReady(Signal signal) const final;
        bool guard_modeWaitFailed(Signal signal) const final;
        bool guard_packetSent(Signal signal) const final;
        bool guard_abortRxDone(Signal signal) const final;

        Rfm69Manager& m_manager;
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

    //! Mark parameters available; bring-up itself is advanced by run() ticks
    void parametersLoaded() override;

    // ----------------------------------------------------------------------
    // Helper functions: radio state management (Rfm69Manager.cpp)
    // ----------------------------------------------------------------------

    //! True when bring-up has reached READY (RX polling / TX accepted)
    bool radioReady() const;

    //! True while a downlink buffer is owned by the transmit state machine
    bool txActive() const;

    //! Bring-up steps invoked by BringupMachine actions (one per run tick)
    void bringupAssertReset();
    void bringupHoldTick();
    void bringupSettleTick();
    void bringupDetect();
    void bringupPrepareReset();
    void bringupPrepareConfigure();
    void bringupConfigureStep();
    void bringupPollModeReady();
    void bringupConfigureFailed();
    void bringupAnnounceReady();

    //! Poll for and deliver received packets (budgeted per 1 kHz tick)
    void pollReceive();

    //! Continue an in-progress FIFO drain; returns true when a full packet is ready
    bool continueReceiveDrain();

    //! Start a new drain after validating IRQ flags; returns true if draining
    bool beginReceiveDrain();

    //! Mark a completed RX packet for delivery on this tick (under m_lock)
    void stageReceivedPacket();

    //! Allocate and emit a received packet on dataOut (outside m_lock)
    void deliverReceivedPacket(const U8* data, FwSizeType size);

    //! Abort an in-progress drain and re-arm the receiver
    void abortReceiveDrain();

    //! Write the RadioState telemetry channel.
    void reportRadioState(Rfm69RadioState state);

    //! Stage one native RF packet for scheduler-driven transmission.
    bool startTransmit(Fw::Buffer& data, const ComCfg::FrameContext& context);

    //! True when this deployment has no ComQueue retry path (comStatusOut unwired).
    //! Ground-station topologies use a local pending-TX hold instead of dropping.
    bool usesLocalTxHold() const;

    //! Enqueue one uplink buffer for a later run() tick. Returns false if full.
    bool enqueuePendingTransmit(Fw::Buffer& data, const ComCfg::FrameContext& context);

    //! If idle and clear, start the oldest pending uplink packet.
    //! Returns true when dropBuffer/dropContext must be returned after unlock.
    bool tryStartPendingTransmit(Fw::Buffer& dropBuffer, ComCfg::FrameContext& dropContext);

    //! Transmit steps invoked by TransmitMachine actions (one per run tick)
    void txRequestStandby();
    void txPollModeReady();
    void txClearFifo();
    void txWriteLength();
    void txLoadInitial();
    void txEnableBoost();
    void txRequestTx();
    void txStream();
    void txRequestRx();
    void txDisableBoost();
    void txFinishSuccess();
    void txBeginAbort();
    void txAbortClearFifo();
    void txAbortRequestRx();
    void txPollAbortRx();
    void txAbortDisableBoost();
    void txFinishFailure();

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

    //! \brief Stage the packet-mode register profile into m_configTable
    bool buildConfigTable();

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

    BringupSm m_bringupSm;   //!< Bring-up state machine (BringupMachine)
    TransmitSm m_txSm;       //!< Downlink state machine (TransmitMachine)

    Fw::Buffer m_txBuffer;            //!< Downlink buffer owned by the TX machine
    ComCfg::FrameContext m_txContext; //!< Frame context returned with m_txBuffer
    FwSizeType m_txOffset;            //!< Payload bytes already streamed to the FIFO
    U32 m_txWaitTicks;                //!< Ticks spent in the current ModeReady wait
    U32 m_txElapsedTicks;             //!< Total ticks since TX start (overall bound)
    U32 m_txTimeoutTicks;             //!< Bitrate-derived bound on m_txElapsedTicks
    bool m_txBoostEnabled;            //!< +20 dBm boost registers currently active
    bool m_txStepOk;                  //!< Last TX step's SPI transactions succeeded
    bool m_txModeReady;               //!< ModeReady observed by the last TX poll
    bool m_txModeWaitFailed;          //!< TX ModeReady wait failed or timed out
    bool m_txPacketSent;              //!< Payload streamed and PacketSent observed
    bool m_txAbortRxDone;             //!< Abort-path RX wait finished
    TxProgress m_txResult;            //!< Outcome of the last TX tick

    //! Local hold used when comStatusOut is unwired (no ComQueue). Mirrors the
    //! Arduino RadioHead ground-station usbHold/retry behavior for half-duplex.
    //! Sized to the GS frame-bin pool (8×272) so a short GDS burst cannot
    //! overflow and drop mid-file CFDP DATA packets (flight ignores this hold).
    static constexpr U32 PENDING_TX_DEPTH = 8;
    Fw::Buffer m_pendingTxBuffers[PENDING_TX_DEPTH];
    ComCfg::FrameContext m_pendingTxContexts[PENDING_TX_DEPTH];
    U32 m_pendingTxCount;

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
    static_assert(sizeof(m_rxPayload) == MAX_PACKET_PAYLOAD,
                  "RX drain bounds checks assume m_rxPayload holds MAX_PACKET_PAYLOAD bytes");
    //! Completed packet length staged for post-unlock delivery (0 = none)
    FwSizeType m_rxDeliverSize;
    //! Divider state and bitrate-derived idle/streaming SPI poll periods.
    //! Idle polling only needs to notice a packet before the 66-byte FIFO fills;
    //! streaming polling must drain faster than bytes arrive.
    U32 m_rxPollTicks;
    U32 m_rxIdlePollDivisor;
    U32 m_rxActivePollDivisor;
    //! Consecutive ticks seeing sync+fifo without FifoLevel/PayloadReady
    U32 m_rxStaleSyncTicks;
    //! Remaining 1 kHz ticks to defer flight downlink after a completed RX
    U32 m_rxTxHoldoffTicks;
    //! Remaining 1 kHz ticks to defer the next local TX after a completed TX
    U32 m_txTxHoldoffTicks;
    //! Consecutive ticks a pending downlink has been blocked by the RX lease
    U32 m_txStarvedTicks;

    U32 m_resetTicks;          //!< Ticks elapsed in the current reset phase
    bool m_resetLineHeld;      //!< RST is asserted (GPIO wired, write succeeded)
    bool m_holdElapsed;        //!< RST hold window elapsed and line released
    bool m_settleElapsed;      //!< Post-release settle window elapsed
    bool m_radioDetected;      //!< RegVersion matched on the last DETECT tick
    //! Staged modem profile written CONFIG_WRITES_PER_TICK entries per tick
    RegisterWrite m_configTable[CONFIG_TABLE_SIZE];
    FwSizeType m_configCount;  //!< Staged entries (0 = table must be rebuilt)
    FwSizeType m_configIndex;  //!< Next staged entry to write
    bool m_configRxRequested;  //!< RX mode requested after the profile write
    U32 m_bringupWaitTicks;    //!< Ticks spent in the bring-up ModeReady wait
    bool m_bringupModeReady;   //!< ModeReady observed by the last bring-up poll
    bool m_bringupWaitFailed;  //!< Bring-up ModeReady wait failed or timed out
    bool m_configured;         //!< FPP parameters have been loaded
    U32 m_packetsTransmitted;  //!< Count of transmitted packets
    U32 m_packetsReceived;     //!< Count of received packets
    U32 m_rxCrcErrors;         //!< Count of RX frames dropped for failing CRC
    TransmitState m_transmitEnabled;  //!< Whether downlink transmit is permitted
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
