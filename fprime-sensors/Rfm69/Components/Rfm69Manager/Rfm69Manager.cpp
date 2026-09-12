// ======================================================================
// \title  Rfm69Manager.cpp
// \brief  cpp file for Rfm69Manager component implementation class
//
// F Prime handler implementations. Radio-specific register access and
// packet handling helpers live in Rfm69Helpers.cpp.
// ======================================================================

#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Manager.hpp"
#include <Os/Mutex.hpp>
#include <cstring>

namespace Rfm69 {

// ----------------------------------------------------------------------
// Construction, initialization, and destruction
// ----------------------------------------------------------------------

Rfm69Manager ::Rfm69Manager(const char* const compName)
    : Rfm69ManagerComponentBase(compName),
      m_bringupSm(*this),
      m_txSm(*this),
      m_txBuffer(),
      m_txContext(),
      m_txOffset(0),
      m_txWaitTicks(0),
      m_txElapsedTicks(0),
      m_txTimeoutTicks(250),
      m_txBoostEnabled(false),
      m_txStepOk(false),
      m_txModeReady(false),
      m_txModeWaitFailed(false),
      m_txPacketSent(false),
      m_txAbortRxDone(false),
      m_txResult(TX_IN_PROGRESS),
      m_pendingTxBuffers{},
      m_pendingTxContexts{},
      m_pendingTxCount(0),
      m_rxDraining(false),
      m_rxLength(0),
      m_rxReceived(0),
      m_rxSawPayloadReady(false),
      m_rxPayload{},
      m_rxPollTicks(0),
      m_rxIdlePollDivisor(8),
      m_rxActivePollDivisor(4),
      m_rxStaleSyncTicks(0),
      m_rxDeliverSize(0),
      m_rxTxHoldoffTicks(0),
      m_txTxHoldoffTicks(0),
      m_txStarvedTicks(0),
      m_resetTicks(0),
      m_resetLineHeld(false),
      m_holdElapsed(false),
      m_settleElapsed(false),
      m_radioDetected(false),
      m_configTable{},
      m_configCount(0),
      m_configIndex(0),
      m_configRxRequested(false),
      m_bringupWaitTicks(0),
      m_bringupModeReady(false),
      m_bringupWaitFailed(false),
      m_configured(false),
      m_packetsTransmitted(0),
      m_packetsReceived(0),
      m_rxCrcErrors(0),
      m_transmitEnabled(TransmitState::ENABLED),
      m_comStatusAnnounced(false),
      m_comResumeNeeded(false),
      m_mosi{},
      m_miso{} {}

Rfm69Manager ::~Rfm69Manager() {}

void Rfm69Manager ::parameterUpdated(FwPrmIdType id) {
    // F´ already stored the new value; defer register rewrite to the next run
    // tick (LoRa re-reads params on each enableTx/enableRx instead).
    (void)id;
    Os::ScopeLock lock(this->m_lock);
    // Only the READY state handles reconfigure; mid-bring-up updates are
    // picked up when CONFIGURE rebuilds the staged profile anyway.
    this->m_bringupSm.sendSignal_reconfigure();
}

void Rfm69Manager ::parametersLoaded() {
    // No SPI or GPIO work here: bring-up (reset pulse, detect, configure) is
    // advanced one bounded step at a time by run() ticks.
    Os::ScopeLock lock(this->m_lock);
    this->m_configured = true;
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void Rfm69Manager ::dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    // Accepting a buffer only stages it. RF airtime is advanced by run(), so
    // this handler remains bounded even when a Com SUCCESS callback reaches it
    // synchronously from the 1 kHz rate-group thread.
    bool accepted = false;
    bool held = false;
    {
        Os::ScopeLock lock(this->m_lock);
        if (this->m_transmitEnabled == TransmitState::DISABLED) {
            // Keep a local hold on GS (no ComQueue); flight reports FAILURE so
            // ComQueue retries after TRANSMIT is re-enabled.
            if (this->usesLocalTxHold()) {
                held = this->enqueuePendingTransmit(data, context);
            } else {
                this->m_comResumeNeeded = true;
            }
        } else if (!this->radioReady() || this->txActive() || this->channelBusy() ||
                   (this->usesLocalTxHold() && (this->m_txTxHoldoffTicks > 0))) {
            // Half-duplex / not ready: ComQueue gets FAILURE; GS holds & retries.
            // GS ignores RX_TX_HOLDOFF (continuous TM would starve uplink) but
            // honors TX_TX_HOLDOFF so bursts cannot send back-to-back packets.
            if (this->usesLocalTxHold()) {
                held = this->enqueuePendingTransmit(data, context);
            } else {
                // Expected while busy: no SendFailed event (a WARNING_HI here
                // re-enters ComQueue and can reorder ground DATA chunks).
                this->m_comResumeNeeded = true;
            }
        } else if (this->downlinkBlocked() && !this->usesLocalTxHold()) {
            // Flight ComQueue path still respects holdoff via FAILURE/resume.
            this->m_comResumeNeeded = true;
        } else if (this->usesLocalTxHold() && (this->m_pendingTxCount > 0)) {
            // Keep FIFO order: a new frame must not jump ahead of held frames.
            held = this->enqueuePendingTransmit(data, context);
        } else {
            accepted = this->startTransmit(data, context);
            if (accepted) {
                this->m_comResumeNeeded = false;
            } else {
                // startTransmit only fails on empty/oversized payloads — never hold.
                this->m_comResumeNeeded = true;
            }
        }
    }
    if (!accepted && !held) {
        this->dataReturnOut_out(0, data, context);
        this->emitComStatus(Fw::Success::FAILURE);
    }
}

void Rfm69Manager ::dataReturnIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    this->deallocate_out(0, data);
}

void Rfm69Manager ::run_handler(FwIndexType portNum, U32 context) {
    TxProgress txProgress = TX_IN_PROGRESS;
    Fw::Buffer completedBuffer;
    ComCfg::FrameContext completedContext;
    bool txCompleted = false;
    Fw::Buffer dropBuffer;
    ComCfg::FrameContext dropContext;
    bool dropPending = false;
    bool resume = false;
    U8 rxPacket[MAX_PACKET_PAYLOAD];
    FwSizeType rxPacketSize = 0;
    {
    Os::ScopeLock lock(this->m_lock);
    if (this->txActive()) {
        this->m_txResult = TX_IN_PROGRESS;
        this->m_txSm.sendSignal_tick();
        txProgress = this->m_txResult;
        if (txProgress != TX_IN_PROGRESS) {
            txCompleted = true;
            completedBuffer = this->m_txBuffer;
            completedContext = this->m_txContext;
            this->m_txBuffer = Fw::Buffer();
            if (txProgress == TX_SUCCEEDED) {
                // Do not immediately re-enter dataIn through the synchronous
                // Com SUCCESS callback. Give the peer time to empty its FIFO,
                // restart RX, and reacquire the next preamble/sync word.
                this->m_txTxHoldoffTicks = TX_TX_HOLDOFF_TICKS;
                this->m_comResumeNeeded = false;
            } else {
                this->m_comResumeNeeded = true;
            }
        }
    } else if (this->radioReady()) {
        // Drain local GS hold before RX poll so uplink is not starved by TM.
        dropPending = this->tryStartPendingTransmit(dropBuffer, dropContext);
        if (!this->txActive()) {
            const U32 pollDivisor = this->m_rxDraining ? this->m_rxActivePollDivisor : this->m_rxIdlePollDivisor;
            this->m_rxPollTicks++;
            if (this->m_rxPollTicks >= pollDivisor) {
                this->m_rxPollTicks = 0;
                this->pollReceive();
            }
        }
    } else if (this->m_configured) {
        // Bring-up advances here one bounded step per tick; also covers
        // RESET, parameter reconfigure, and failed-detect retries. Do not
        // touch the bus before the deployment has loaded its parameters.
        this->m_bringupSm.sendSignal_tick();
    }
    if (this->m_rxTxHoldoffTicks > 0) {
        this->m_rxTxHoldoffTicks--;
    }
    if (this->m_txTxHoldoffTicks > 0) {
        this->m_txTxHoldoffTicks--;
    }
    // Downlink fairness: each received packet re-arms the RX lease, so
    // periodic uplink (GDS keepalives, paced file chunks) can otherwise defer
    // a pending downlink forever. Bound the total wait and drop the lease.
    if (this->m_comResumeNeeded && this->radioReady() &&
        (this->m_transmitEnabled == TransmitState::ENABLED) && (this->m_rxTxHoldoffTicks > 0)) {
        if (++this->m_txStarvedTicks >= RX_HOLDOFF_STARVATION_TICKS) {
            this->m_txStarvedTicks = 0;
            this->m_rxTxHoldoffTicks = 0;
        }
    } else {
        this->m_txStarvedTicks = 0;
    }
    // After holdoff decays, try a held GS uplink before emitting Com SUCCESS.
    if (!dropPending && !this->txActive() && this->radioReady()) {
        dropPending = this->tryStartPendingTransmit(dropBuffer, dropContext);
    }
    // Resume ComQueue after holdoff/TX FAILURE once the radio can accept TX.
    // A failed staged TX first reports FAILURE. Resume is deliberately deferred
    // to a later tick so ComRetry observes an ordered state transition rather
    // than FAILURE and SUCCESS in one synchronous callback chain.
    resume = txCompleted ? false : this->maybeResumeComStatus();
    // Copy a completed RX packet out under the lock; allocation and delivery
    // to downstream components happen after the lock is released.
    if (this->m_rxDeliverSize > 0) {
        rxPacketSize = this->m_rxDeliverSize;
        this->m_rxDeliverSize = 0;
        (void)::memcpy(rxPacket, this->m_rxPayload, rxPacketSize);
    }
    }
    if (rxPacketSize > 0) {
        this->deliverReceivedPacket(rxPacket, rxPacketSize);
    }
    if (txCompleted) {
        this->dataReturnOut_out(0, completedBuffer, completedContext);
        this->emitComStatus(txProgress == TX_SUCCEEDED ? Fw::Success::SUCCESS : Fw::Success::FAILURE);
    }
    if (dropPending) {
        this->dataReturnOut_out(0, dropBuffer, dropContext);
    }
    if (resume) {
        this->emitComStatus(Fw::Success::SUCCESS);
    }
}

void Rfm69Manager ::emitComStatus(Fw::Success status) {
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        this->comStatusOut_out(0, status);
    }
}

bool Rfm69Manager ::maybeResumeComStatus() {
    if (!this->m_comResumeNeeded) {
        return false;
    }
    if (this->m_transmitEnabled != TransmitState::ENABLED) {
        return false;
    }
    if (!this->radioReady() || this->downlinkBlocked()) {
        return false;
    }
    this->m_comResumeNeeded = false;
    return true;
}

// ----------------------------------------------------------------------
// Helper functions: radio state management
// ----------------------------------------------------------------------

bool Rfm69Manager ::radioReady() const {
    return this->m_bringupSm.getState() == BringupMachine_State::BRINGUP_READY;
}

bool Rfm69Manager ::txActive() const {
    return this->m_txSm.getState() != TransmitMachine_State::IDLE;
}

void Rfm69Manager ::bringupAssertReset() {
    // Optional hardware RST (recommended when a reset GPIO is wired —
    // cold power-up / shared-SPI glitches can leave the HCW in a bad
    // state). Never required: skip if unconnected or the write fails.
    this->m_resetLineHeld = this->isConnected_resetGpio_OutputPort(0) &&
                            (this->resetGpio_out(0, Fw::Logic::HIGH) == Drv::GpioStatus::OP_OK);
    if (this->m_resetLineHeld) {
        this->m_resetTicks = 0;
        this->reportRadioState(Rfm69RadioState::RESETTING);
    } else {
        this->reportRadioState(Rfm69RadioState::DETECT);
    }
}

void Rfm69Manager ::bringupHoldTick() {
    this->m_holdElapsed = false;
    if (++this->m_resetTicks >= RESET_HOLD_TICKS) {
        // Retry the release next tick on failure: advancing with RST
        // still asserted would leave the radio in reset and mask the
        // GPIO fault as a generic detection failure.
        if (this->resetGpio_out(0, Fw::Logic::LOW) == Drv::GpioStatus::OP_OK) {
            this->m_resetTicks = 0;
            this->m_holdElapsed = true;
        }
    }
}

void Rfm69Manager ::bringupSettleTick() {
    this->m_settleElapsed = (++this->m_resetTicks >= RESET_SETTLE_TICKS);
    if (this->m_settleElapsed) {
        this->reportRadioState(Rfm69RadioState::DETECT);
    }
}

void Rfm69Manager ::bringupDetect() {
    this->m_radioDetected = this->detectRadio();
    if (this->m_radioDetected) {
        this->reportRadioState(Rfm69RadioState::CONFIGURE);
    } else {
        this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
    }
}

void Rfm69Manager ::bringupPrepareReset() {
    this->m_resetTicks = 0;
    this->m_rxDraining = false;
    this->m_rxLength = 0;
    this->m_rxReceived = 0;
    this->m_rxPollTicks = 0;
    this->m_rxStaleSyncTicks = 0;
    this->reportRadioState(Rfm69RadioState::RESETTING);
}

void Rfm69Manager ::bringupPrepareConfigure() {
    this->m_configCount = 0;
    this->m_configIndex = 0;
    this->m_configRxRequested = false;
}

void Rfm69Manager ::bringupConfigureStep() {
    // One bounded chunk per tick: build the staged profile, write up to
    // CONFIG_WRITES_PER_TICK entries, then request RX. Any failure restarts
    // the profile write from the top on the next tick.
    if (this->m_configCount == 0) {
        if (!this->buildConfigTable()) {
            this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
            return;
        }
    }
    if (this->m_configIndex < this->m_configCount) {
        const FwSizeType end = FW_MIN(this->m_configIndex + CONFIG_WRITES_PER_TICK, this->m_configCount);
        for (; this->m_configIndex < end; this->m_configIndex++) {
            const RegisterWrite& entry = this->m_configTable[this->m_configIndex];
            if (this->writeRegister(entry.address, entry.value) != Drv::SpiStatus::SPI_OK) {
                this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
                this->bringupPrepareConfigure();
                return;
            }
        }
    } else if (this->requestMode(Mode::RX)) {
        this->m_configRxRequested = true;
        this->m_bringupWaitTicks = 0;
        this->m_bringupModeReady = false;
        this->m_bringupWaitFailed = false;
    } else {
        this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
        this->bringupPrepareConfigure();
    }
}

void Rfm69Manager ::bringupPollModeReady() {
    U8 flags = 0;
    if (this->readRegister(Reg::IRQ_FLAGS_1, flags) != Drv::SpiStatus::SPI_OK) {
        this->m_bringupWaitFailed = true;
    } else if ((flags & IrqFlags1::MODE_READY) != 0) {
        this->m_bringupModeReady = true;
    } else if (++this->m_bringupWaitTicks >= MODE_READY_TIMEOUT_TICKS) {
        this->m_bringupWaitFailed = true;
    }
}

void Rfm69Manager ::bringupConfigureFailed() {
    this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
    this->bringupPrepareConfigure();
}

void Rfm69Manager ::bringupAnnounceReady() {
    this->reportRadioState(Rfm69RadioState::READY);
    this->log_ACTIVITY_HI_RadioReady();
    // Com status is a one-time startup handshake. Sending an extra
    // SUCCESS after a hardware reset is invalid while the downstream
    // aggregator is already READY and causes it to assert.
    if (!this->m_comStatusAnnounced && this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success ready = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, ready);
        this->m_comStatusAnnounced = true;
    }
}

void Rfm69Manager ::reportRadioState(Rfm69RadioState state) {
    this->tlmWrite_RadioState(state);
}

bool Rfm69Manager ::startTransmit(Fw::Buffer& data, const ComCfg::FrameContext& context) {
    // One Com buffer → one RF packet (1..255). No radio-layer split/reassembly.
    const FwSizeType size = data.getSize();
    if ((data.getData() == nullptr) || (size == 0) || (size > MAX_PACKET_PAYLOAD)) {
        this->log_WARNING_HI_SendFailed(Rfm69SendFailure::INVALID_SIZE);
        return false;
    }
    this->m_txBuffer = data;
    this->m_txContext = context;
    this->m_txOffset = 0;
    this->m_txWaitTicks = 0;
    this->m_txElapsedTicks = 0;
    this->m_txBoostEnabled = false;
    this->m_txSm.sendSignal_start();
    return true;
}

bool Rfm69Manager ::usesLocalTxHold() const {
    // Flight wires comStatusOut into ComQueue/ComRetry. The Feather GS does not,
    // so a busy-radio reject would otherwise permanently drop uplink frames.
    return !this->isConnected_comStatusOut_OutputPort(0);
}

bool Rfm69Manager ::enqueuePendingTransmit(Fw::Buffer& data, const ComCfg::FrameContext& context) {
    if (this->m_pendingTxCount >= PENDING_TX_DEPTH) {
        this->log_WARNING_HI_SendFailed(Rfm69SendFailure::QUEUE_FULL);
        return false;
    }
    this->m_pendingTxBuffers[this->m_pendingTxCount] = data;
    this->m_pendingTxContexts[this->m_pendingTxCount] = context;
    this->m_pendingTxCount++;
    return true;
}

bool Rfm69Manager ::tryStartPendingTransmit(Fw::Buffer& dropBuffer, ComCfg::FrameContext& dropContext) {
    if ((this->m_pendingTxCount == 0) || this->txActive()) {
        return false;
    }
    if (this->m_transmitEnabled != TransmitState::ENABLED) {
        return false;
    }
    // Pace pending GS uplinks with the post-TX quiet window. RX holdoff is
    // ignored here so continuous TM cannot pin the pending queue. TX holdoff
    // is required so a UART burst cannot drain back-to-back.
    if (!this->radioReady() || this->channelBusy() || (this->m_txTxHoldoffTicks > 0)) {
        return false;
    }
    Fw::Buffer data = this->m_pendingTxBuffers[0];
    ComCfg::FrameContext context = this->m_pendingTxContexts[0];
    for (U32 i = 1; i < this->m_pendingTxCount; i++) {
        this->m_pendingTxBuffers[i - 1] = this->m_pendingTxBuffers[i];
        this->m_pendingTxContexts[i - 1] = this->m_pendingTxContexts[i];
    }
    this->m_pendingTxCount--;
    if (this->startTransmit(data, context)) {
        return false;
    }
    // Permanent reject (empty/oversized): caller returns ownership after unlock.
    dropBuffer = data;
    dropContext = context;
    return true;
}

// ----------------------------------------------------------------------
// Command handler implementations
// ----------------------------------------------------------------------

void Rfm69Manager ::TRANSMIT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Rfm69::TransmitState& enabled) {
    bool resume = false;
    {
        Os::ScopeLock lock(this->m_lock);
        if (enabled == TransmitState::ENABLED) {
            // Only resume if a FAILURE was actually reported while disabled: an
            // unsolicited SUCCESS while ComQueue is READY asserts downstream.
            if ((this->m_transmitEnabled == TransmitState::DISABLED) && this->m_comResumeNeeded) {
                this->m_comResumeNeeded = false;
                resume = true;
            }
            this->m_transmitEnabled = TransmitState::ENABLED;
        } else {
            this->m_transmitEnabled = TransmitState::DISABLED;
        }
    }
    if (resume) {
        this->emitComStatus(Fw::Success::SUCCESS);
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Rfm69Manager ::RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    Fw::Buffer abortBuffer;
    ComCfg::FrameContext abortContext;
    bool abortPending = false;
    {
        Os::ScopeLock lock(this->m_lock);
        // Fail an in-flight TX immediately: the radio is about to be held in
        // hardware reset, so further TX SPI traffic is pointless.
        abortPending = this->txActive();
        if (abortPending) {
            abortBuffer = this->m_txBuffer;
            abortContext = this->m_txContext;
            this->m_txBuffer = Fw::Buffer();
            this->m_txSm.sendSignal_cancel();
            this->m_txBoostEnabled = false;
            this->m_comResumeNeeded = true;
        }
        // Tick-driven: the optional RST pulse and re-detect are advanced one
        // bounded step per run() tick, so no command-context blocking occurs.
        this->m_bringupSm.sendSignal_reset();
    }
    if (abortPending) {
        this->dataReturnOut_out(0, abortBuffer, abortContext);
        this->emitComStatus(Fw::Success::FAILURE);
    }
    this->log_ACTIVITY_HI_ResetInitiated();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Rfm69Manager ::pollReceive() {
    // At most one packet's drain step per poll (per-tick SPI budget).
    if (this->m_rxDraining) {
        this->m_rxStaleSyncTicks = 0;
        if (this->continueReceiveDrain()) {
            this->stageReceivedPacket();
        }
        // Incomplete drains resume on the next 1 kHz tick (budgeted).
        return;
    }

    U8 flags2 = 0;
    if (this->readRegister(Reg::IRQ_FLAGS_2, flags2) != Drv::SpiStatus::SPI_OK) {
        return;
    }

    const bool payloadReady = (flags2 & IrqFlags2::PAYLOAD_READY) != 0;
    const bool fifoLevel = (flags2 & IrqFlags2::FIFO_LEVEL) != 0;
    const bool fifoNotEmpty = (flags2 & IrqFlags2::FIFO_NOT_EMPTY) != 0;

    // FifoLevel can only become set after packet reception starts. Avoid a
    // second idle IRQ register read: dataIn independently checks
    // SyncAddressMatch before taking the half-duplex channel for TX.
    if (!payloadReady && !(fifoNotEmpty && fifoLevel)) {
        // Stale-sync re-arm: a noise-triggered SyncAddressMatch that never
        // delivers a payload byte holds channelBusy() true and blocks downlink
        // forever. Bound that wait and restart RX. Any FIFO content resets the
        // bound: bytes present mean a reception is progressing (or a completed
        // frame is waiting), which the drain path above owns.
        U8 flags1 = 0;
        if (!fifoNotEmpty &&
            (this->readRegister(Reg::IRQ_FLAGS_1, flags1) == Drv::SpiStatus::SPI_OK) &&
            ((flags1 & IrqFlags1::SYNC_ADDRESS_MATCH) != 0)) {
            this->m_rxStaleSyncTicks += this->m_rxIdlePollDivisor;
            if (this->m_rxStaleSyncTicks >= RX_STALE_SYNC_TICKS) {
                this->m_rxStaleSyncTicks = 0;
                (void)this->recoverReceive();
            }
        } else {
            this->m_rxStaleSyncTicks = 0;
        }
        return;
    }
    this->m_rxStaleSyncTicks = 0;

    if (!this->beginReceiveDrain()) {
        return;
    }
    if (this->continueReceiveDrain()) {
        this->stageReceivedPacket();
    }
}

bool Rfm69Manager ::beginReceiveDrain() {
    this->updateRssi();
    U8 length = 0;
    if (this->readRegister(Reg::FIFO, length) != Drv::SpiStatus::SPI_OK) {
        return false;
    }
    if ((length == 0) || (static_cast<FwSizeType>(length) > MAX_PACKET_PAYLOAD)) {
        (void)this->recoverReceive();
        return false;
    }
    this->m_rxDraining = true;
    this->m_rxLength = length;
    this->m_rxReceived = 0;
    this->m_rxSawPayloadReady = false;
    return true;
}

bool Rfm69Manager ::continueReceiveDrain() {
    FW_ASSERT(this->m_rxDraining);
    // One FifoLevel watermark drain per tick while the packet is still arriving.
    // A tight multi-read loop can empty the FIFO before PayloadReady and clear
    // SyncAddressMatch (datasheet 5.2.3.1), corrupting the frame. On PayloadReady
    // (CRC pass) read the remaining tail in one burst.
    U8 flags = 0;
    if (this->readRegister(Reg::IRQ_FLAGS_2, flags) != Drv::SpiStatus::SPI_OK) {
        this->abortReceiveDrain();
        return false;
    }
    if ((flags & IrqFlags2::FIFO_OVERRUN) != 0) {
        this->abortReceiveDrain();
        return false;
    }
    if (((flags & IrqFlags2::PAYLOAD_READY) == 0) &&
        ((flags & IrqFlags2::FIFO_NOT_EMPTY) == 0)) {
        // CrcOn + CrcAutoClear flushes a bad frame and withholds PayloadReady.
        // Because every streaming drain preserves one FIFO byte, an empty FIFO
        // before completion unambiguously means the hardware rejected the CRC.
        this->m_rxCrcErrors++;
        this->tlmWrite_RxCrcErrors(this->m_rxCrcErrors);
        this->abortReceiveDrain();
        return false;
    }

    FwSizeType chunk = 0;
    if ((flags & IrqFlags2::PAYLOAD_READY) != 0) {
        this->m_rxSawPayloadReady = true;
        chunk = static_cast<FwSizeType>(this->m_rxLength) - this->m_rxReceived;
    } else if ((flags & IrqFlags2::FIFO_LEVEL) != 0) {
        chunk = FW_MIN(static_cast<FwSizeType>(RX_FIFO_DRAIN_CHUNK),
                       static_cast<FwSizeType>(this->m_rxLength) - this->m_rxReceived);
    }
    if (chunk == 0) {
        return false;  // Wait for airtime / next 1 kHz tick
    }
    if (!this->readFifo(&this->m_rxPayload[this->m_rxReceived], chunk)) {
        this->abortReceiveDrain();
        return false;
    }
    this->m_rxReceived += chunk;

    if (this->m_rxReceived < static_cast<FwSizeType>(this->m_rxLength)) {
        return false;  // Continue next 1 kHz tick
    }

    // PayloadReady is CRC-gated (see m_rxSawPayloadReady): a frame whose byte
    // count completed without ever observing it failed CRC and must be dropped.
    if (!this->m_rxSawPayloadReady) {
        this->m_rxCrcErrors++;
        this->tlmWrite_RxCrcErrors(this->m_rxCrcErrors);
        this->abortReceiveDrain();
        return false;
    }

    if (this->writeRegister(Reg::PACKET_CONFIG_2,
                            static_cast<U8>(PacketConfig2::AUTO_RX_RESTART_ON | PacketConfig2::RX_RESTART)) !=
        Drv::SpiStatus::SPI_OK) {
        this->abortReceiveDrain();
        return false;
    }
    this->m_rxDraining = false;
    return true;
}

void Rfm69Manager ::abortReceiveDrain() {
    this->m_rxDraining = false;
    this->m_rxLength = 0;
    this->m_rxReceived = 0;
    (void)this->recoverReceive();
}

void Rfm69Manager ::stageReceivedPacket() {
    // Called under m_lock: bookkeeping only. run_handler copies the payload
    // out and performs allocation/delivery after releasing the lock.
    this->m_rxDeliverSize = this->m_rxReceived;
    this->m_rxLength = 0;
    this->m_rxReceived = 0;
    this->m_packetsReceived++;
    this->tlmWrite_PacketsReceived(this->m_packetsReceived);
    // Keep the half-duplex channel clear for the next ground chunk.
    this->m_rxTxHoldoffTicks = RX_TX_HOLDOFF_TICKS;
}

void Rfm69Manager ::deliverReceivedPacket(const U8* data, FwSizeType size) {
    Fw::Buffer buffer = this->allocate_out(0, size);
    if (buffer.getSize() < size) {
        this->log_WARNING_HI_AllocationFailed(size);
        this->deallocate_out(0, buffer);
        return;
    }
    (void)::memcpy(buffer.getData(), data, size);
    buffer.setSize(size);
    ComCfg::FrameContext context;
    this->dataOut_out(0, buffer, context);
}

// ----------------------------------------------------------------------
// BringupMachine glue: actions and guards delegate to the manager
// ----------------------------------------------------------------------

Rfm69Manager::BringupSm ::BringupSm(Rfm69Manager& manager) : m_manager(manager) {
    this->initBase(0);
}

void Rfm69Manager::BringupSm ::action_doAssertReset(Signal signal) {
    (void)signal;
    this->m_manager.bringupAssertReset();
}

void Rfm69Manager::BringupSm ::action_doHoldTick(Signal signal) {
    (void)signal;
    this->m_manager.bringupHoldTick();
}

void Rfm69Manager::BringupSm ::action_doSettleTick(Signal signal) {
    (void)signal;
    this->m_manager.bringupSettleTick();
}

void Rfm69Manager::BringupSm ::action_doDetect(Signal signal) {
    (void)signal;
    this->m_manager.bringupDetect();
}

void Rfm69Manager::BringupSm ::action_doPrepareReset(Signal signal) {
    (void)signal;
    this->m_manager.bringupPrepareReset();
}

void Rfm69Manager::BringupSm ::action_doPrepareConfigure(Signal signal) {
    (void)signal;
    this->m_manager.bringupPrepareConfigure();
}

void Rfm69Manager::BringupSm ::action_doConfigureStep(Signal signal) {
    (void)signal;
    this->m_manager.bringupConfigureStep();
}

void Rfm69Manager::BringupSm ::action_doPollModeReady(Signal signal) {
    (void)signal;
    this->m_manager.bringupPollModeReady();
}

void Rfm69Manager::BringupSm ::action_doConfigureFailed(Signal signal) {
    (void)signal;
    this->m_manager.bringupConfigureFailed();
}

void Rfm69Manager::BringupSm ::action_doAnnounceReady(Signal signal) {
    (void)signal;
    this->m_manager.bringupAnnounceReady();
}

bool Rfm69Manager::BringupSm ::guard_resetLineHeld(Signal signal) const {
    (void)signal;
    return this->m_manager.m_resetLineHeld;
}

bool Rfm69Manager::BringupSm ::guard_holdElapsed(Signal signal) const {
    (void)signal;
    return this->m_manager.m_holdElapsed;
}

bool Rfm69Manager::BringupSm ::guard_settleElapsed(Signal signal) const {
    (void)signal;
    return this->m_manager.m_settleElapsed;
}

bool Rfm69Manager::BringupSm ::guard_radioDetected(Signal signal) const {
    (void)signal;
    return this->m_manager.m_radioDetected;
}

bool Rfm69Manager::BringupSm ::guard_profileWritten(Signal signal) const {
    (void)signal;
    return this->m_manager.m_configRxRequested;
}

bool Rfm69Manager::BringupSm ::guard_modeReady(Signal signal) const {
    (void)signal;
    return this->m_manager.m_bringupModeReady;
}

bool Rfm69Manager::BringupSm ::guard_modeWaitFailed(Signal signal) const {
    (void)signal;
    return this->m_manager.m_bringupWaitFailed;
}

// ----------------------------------------------------------------------
// TransmitMachine glue: actions and guards delegate to the manager
// ----------------------------------------------------------------------

Rfm69Manager::TransmitSm ::TransmitSm(Rfm69Manager& manager) : m_manager(manager) {
    this->initBase(1);
}

void Rfm69Manager::TransmitSm ::action_doRequestStandby(Signal signal) {
    (void)signal;
    this->m_manager.txRequestStandby();
}

void Rfm69Manager::TransmitSm ::action_doPollModeReady(Signal signal) {
    (void)signal;
    this->m_manager.txPollModeReady();
}

void Rfm69Manager::TransmitSm ::action_doClearFifo(Signal signal) {
    (void)signal;
    this->m_manager.txClearFifo();
}

void Rfm69Manager::TransmitSm ::action_doWriteLength(Signal signal) {
    (void)signal;
    this->m_manager.txWriteLength();
}

void Rfm69Manager::TransmitSm ::action_doLoadInitial(Signal signal) {
    (void)signal;
    this->m_manager.txLoadInitial();
}

void Rfm69Manager::TransmitSm ::action_doEnableBoost(Signal signal) {
    (void)signal;
    this->m_manager.txEnableBoost();
}

void Rfm69Manager::TransmitSm ::action_doRequestTx(Signal signal) {
    (void)signal;
    this->m_manager.txRequestTx();
}

void Rfm69Manager::TransmitSm ::action_doStream(Signal signal) {
    (void)signal;
    this->m_manager.txStream();
}

void Rfm69Manager::TransmitSm ::action_doRequestRx(Signal signal) {
    (void)signal;
    this->m_manager.txRequestRx();
}

void Rfm69Manager::TransmitSm ::action_doDisableBoost(Signal signal) {
    (void)signal;
    this->m_manager.txDisableBoost();
}

void Rfm69Manager::TransmitSm ::action_doFinishSuccess(Signal signal) {
    (void)signal;
    this->m_manager.txFinishSuccess();
}

void Rfm69Manager::TransmitSm ::action_doBeginAbort(Signal signal) {
    (void)signal;
    this->m_manager.txBeginAbort();
}

void Rfm69Manager::TransmitSm ::action_doAbortClearFifo(Signal signal) {
    (void)signal;
    this->m_manager.txAbortClearFifo();
}

void Rfm69Manager::TransmitSm ::action_doAbortRequestRx(Signal signal) {
    (void)signal;
    this->m_manager.txAbortRequestRx();
}

void Rfm69Manager::TransmitSm ::action_doPollAbortRx(Signal signal) {
    (void)signal;
    this->m_manager.txPollAbortRx();
}

void Rfm69Manager::TransmitSm ::action_doAbortDisableBoost(Signal signal) {
    (void)signal;
    this->m_manager.txAbortDisableBoost();
}

void Rfm69Manager::TransmitSm ::action_doFinishFailure(Signal signal) {
    (void)signal;
    this->m_manager.txFinishFailure();
}

bool Rfm69Manager::TransmitSm ::guard_stepOk(Signal signal) const {
    (void)signal;
    return this->m_manager.m_txStepOk;
}

bool Rfm69Manager::TransmitSm ::guard_modeReady(Signal signal) const {
    (void)signal;
    return this->m_manager.m_txModeReady;
}

bool Rfm69Manager::TransmitSm ::guard_modeWaitFailed(Signal signal) const {
    (void)signal;
    return this->m_manager.m_txModeWaitFailed;
}

bool Rfm69Manager::TransmitSm ::guard_packetSent(Signal signal) const {
    (void)signal;
    return this->m_manager.m_txPacketSent;
}

bool Rfm69Manager::TransmitSm ::guard_abortRxDone(Signal signal) const {
    (void)signal;
    return this->m_manager.m_txAbortRxDone;
}

}  // namespace Rfm69
