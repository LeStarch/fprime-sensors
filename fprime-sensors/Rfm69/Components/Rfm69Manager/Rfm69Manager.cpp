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
      m_txActive(false),
      m_txState(TX_IDLE),
      m_txBuffer(),
      m_txContext(),
      m_txOffset(0),
      m_txWaitTicks(0),
      m_txElapsedTicks(0),
      m_txTimeoutTicks(250),
      m_txBoostEnabled(false),
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
      m_rxTxHoldoffTicks(0),
      m_txTxHoldoffTicks(0),
      m_state(RESET_ASSERT),
      m_resetTicks(0),
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
    if (this->m_state == READY) {
        this->m_state = CONFIGURE;
    }
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
        } else if ((this->m_state != READY) || this->m_txActive || this->channelBusy() ||
                   (this->usesLocalTxHold() && (this->m_txTxHoldoffTicks > 0))) {
            // Half-duplex / not ready: ComQueue gets FAILURE; GS holds & retries.
            // GS still ignores RX_TX_HOLDOFF here (continuous TM would starve
            // uplink). It does honor TX_TX_HOLDOFF so a UART burst cannot start
            // the next packet in the same quiet window as the one that just
            // finished (that was "packet 5 after packet 0" on multi-chunk).
            if (this->usesLocalTxHold()) {
                held = this->enqueuePendingTransmit(data, context);
            } else {
                // Half-duplex busy is expected during RX/TX. Do not log
                // SendFailed: that WARNING_HI re-enters ComQueue and collides
                // with the next ground DATA chunk (FileUplink PacketOutOfOrder).
                this->m_comResumeNeeded = true;
            }
        } else if (this->downlinkBlocked() && !this->usesLocalTxHold()) {
            // Flight ComQueue path still respects holdoff via FAILURE/resume.
            this->m_comResumeNeeded = true;
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
    {
    Os::ScopeLock lock(this->m_lock);
    if (this->m_txActive) {
        txProgress = this->advanceTransmit();
        if (txProgress != TX_IN_PROGRESS) {
            txCompleted = true;
            completedBuffer = this->m_txBuffer;
            completedContext = this->m_txContext;
            this->m_txBuffer = Fw::Buffer();
            this->m_txState = TX_IDLE;
            this->m_txActive = false;
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
    } else if (this->m_state == READY) {
        // Drain local GS hold before RX poll so uplink is not starved by TM.
        dropPending = this->tryStartPendingTransmit(dropBuffer, dropContext);
        if (!this->m_txActive) {
            const U32 pollDivisor = this->m_rxDraining ? this->m_rxActivePollDivisor : this->m_rxIdlePollDivisor;
            this->m_rxPollTicks++;
            if (this->m_rxPollTicks >= pollDivisor) {
                this->m_rxPollTicks = 0;
                this->pollReceive();
            }
        }
    } else {
        // Startup normally reaches READY in parametersLoaded(). This path
        // covers RESET / parameter reconfigure and a failed initial detect.
        this->initializeRadio();
    }
    if (this->m_rxTxHoldoffTicks > 0) {
        this->m_rxTxHoldoffTicks--;
    }
    if (this->m_txTxHoldoffTicks > 0) {
        this->m_txTxHoldoffTicks--;
    }
    // After holdoff decays, try a held GS uplink before emitting Com SUCCESS.
    if (!dropPending && !this->m_txActive && (this->m_state == READY)) {
        dropPending = this->tryStartPendingTransmit(dropBuffer, dropContext);
    }
    // Resume ComQueue after holdoff/TX FAILURE once the radio can accept TX.
    // A failed staged TX first reports FAILURE. Resume is deliberately deferred
    // to a later tick so ComRetry observes an ordered state transition rather
    // than FAILURE and SUCCESS in one synchronous callback chain.
    resume = txCompleted ? false : this->maybeResumeComStatus();
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
    if (this->m_state != READY || this->downlinkBlocked()) {
        return false;
    }
    this->m_comResumeNeeded = false;
    return true;
}

// ----------------------------------------------------------------------
// Helper functions: radio state management
// ----------------------------------------------------------------------

void Rfm69Manager ::initializeRadio() {
    // Do not touch the bus before the deployment has loaded its parameters.
    if (!this->m_configured) {
        return;
    }
    switch (this->m_state) {
        case RESET_ASSERT:
            // Optional hardware RST (recommended when a reset GPIO is wired —
            // cold power-up / shared-SPI glitches can leave the HCW in a bad
            // state). Never required: skip if unconnected or the write fails.
            if (this->isConnected_resetGpio_OutputPort(0) &&
                (this->resetGpio_out(0, Fw::Logic::HIGH) == Drv::GpioStatus::OP_OK)) {
                this->m_resetTicks = 0;
                this->m_state = RESET_HOLD;
            } else {
                this->m_state = DETECT;
            }
            this->reportRadioState();
            break;
        case RESET_HOLD:
            if (++this->m_resetTicks >= RESET_HOLD_TICKS) {
                (void)this->resetGpio_out(0, Fw::Logic::LOW);
                this->m_resetTicks = 0;
                this->m_state = RESET_SETTLE;
            }
            break;
        case RESET_SETTLE:
            if (++this->m_resetTicks >= RESET_SETTLE_TICKS) {
                this->m_state = DETECT;
                this->reportRadioState();
            }
            break;
        case DETECT:
            if (this->detectRadio()) {
                this->m_state = CONFIGURE;
                this->reportRadioState();
            } else {
                this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
            }
            break;
        case CONFIGURE:
            if (this->configureRadio() && this->setMode(Mode::RX)) {
                this->m_state = READY;
                this->reportRadioState();
                this->log_ACTIVITY_HI_RadioReady();
                // Com status is a one-time startup handshake. Sending an extra
                // SUCCESS after a hardware reset is invalid while the downstream
                // aggregator is already READY and causes it to assert.
                if (!this->m_comStatusAnnounced && this->isConnected_comStatusOut_OutputPort(0)) {
                    Fw::Success ready = Fw::Success::SUCCESS;
                    this->comStatusOut_out(0, ready);
                    this->m_comStatusAnnounced = true;
                }
            } else {
                this->log_WARNING_HI_ConfigurationFailed(Rfm69Mode::Receive);
            }
            break;
        case READY:
        default:
            break;
    }
}

void Rfm69Manager ::beginResetSequence() {
    this->m_state = RESET_ASSERT;
    this->m_resetTicks = 0;
    this->m_rxDraining = false;
    this->m_rxLength = 0;
    this->m_rxReceived = 0;
    this->m_rxPollTicks = 0;
    this->m_rxStaleSyncTicks = 0;
    this->reportRadioState();
}

void Rfm69Manager ::reportRadioState() {
    Rfm69RadioState state = Rfm69RadioState::NOT_STARTED;
    switch (this->m_state) {
        case RESET_ASSERT:
        case RESET_HOLD:
        case RESET_SETTLE:
            state = Rfm69RadioState::RESETTING;
            break;
        case DETECT:
            state = Rfm69RadioState::DETECT;
            break;
        case CONFIGURE:
            state = Rfm69RadioState::CONFIGURE;
            break;
        case READY:
            state = Rfm69RadioState::READY;
            break;
        default:
            break;
    }
    this->tlmWrite_RadioState(state);
}

bool Rfm69Manager ::startTransmit(Fw::Buffer& data, const ComCfg::FrameContext& context) {
    // One Com buffer → one RF packet (1..255). No radio-layer split/reassembly.
    const FwSizeType size = data.getSize();
    if ((size == 0) || (size > MAX_PACKET_PAYLOAD)) {
        this->log_WARNING_HI_SendFailed(Rfm69SendFailure::INVALID_SIZE);
        return false;
    }
    this->m_txBuffer = data;
    this->m_txContext = context;
    this->m_txOffset = 0;
    this->m_txWaitTicks = 0;
    this->m_txElapsedTicks = 0;
    this->m_txBoostEnabled = false;
    this->m_txState = TX_REQUEST_STANDBY;
    this->m_txActive = true;
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
    if ((this->m_pendingTxCount == 0) || this->m_txActive) {
        return false;
    }
    if (this->m_transmitEnabled != TransmitState::ENABLED) {
        return false;
    }
    // Pace pending GS uplinks with the post-TX quiet window. RX holdoff is
    // ignored here so continuous TM cannot pin the pending queue. TX holdoff
    // is required so a UART burst cannot drain back-to-back.
    if ((this->m_state != READY) || this->channelBusy() || (this->m_txTxHoldoffTicks > 0)) {
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
            if (this->m_transmitEnabled == TransmitState::DISABLED) {
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
    {
        Os::ScopeLock lock(this->m_lock);
        // Tick-driven: the optional RST pulse and re-detect are advanced one
        // bounded step per run() tick, so no command-context blocking occurs.
        this->beginResetSequence();
    }
    this->log_ACTIVITY_HI_ResetInitiated();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Rfm69Manager ::pollReceive() {
    for (U32 packet = 0; packet < RX_PACKETS_PER_TICK; packet++) {
        if (this->m_rxDraining) {
            this->m_rxStaleSyncTicks = 0;
            if (this->continueReceiveDrain()) {
                this->deliverReceivedPacket();
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
            return;
        }

        if (!this->beginReceiveDrain()) {
            return;
        }
        if (this->continueReceiveDrain()) {
            this->deliverReceivedPacket();
        }
        return;
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

    // The declared byte count is satisfied, but a frame is only valid if the
    // hardware CRC passed. PayloadReady is CRC-gated: the RFM69 asserts it only on
    // a CRC pass and (with CrcAutoClear) flushes the FIFO and withholds it on a
    // failure. A genuine packet's sub-threshold tail is reachable ONLY under
    // PayloadReady (FifoLevel needs > threshold bytes present), so a real frame
    // always latches m_rxSawPayloadReady before its count completes. A frame that
    // streamed its whole count via FifoLevel alone (trailing noise kept the FIFO
    // above threshold) never passed CRC: drop it so corrupted bytes never reach
    // the deframer or command dispatcher. Do NOT re-read the flag here -- by now a
    // following packet's sync can assert PayloadReady and mask this frame's failure.
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

void Rfm69Manager ::deliverReceivedPacket() {
    const FwSizeType size = this->m_rxReceived;
    this->m_rxLength = 0;
    this->m_rxReceived = 0;
    Fw::Buffer buffer = this->allocate_out(0, size);
    if (buffer.getSize() < size) {
        this->log_WARNING_HI_AllocationFailed(size);
        this->deallocate_out(0, buffer);
        return;
    }
    (void)::memcpy(buffer.getData(), this->m_rxPayload, size);
    buffer.setSize(size);
    this->m_packetsReceived++;
    this->tlmWrite_PacketsReceived(this->m_packetsReceived);
    // Keep the half-duplex channel clear for the next ground chunk.
    this->m_rxTxHoldoffTicks = RX_TX_HOLDOFF_TICKS;
    ComCfg::FrameContext context;
    this->dataOut_out(0, buffer, context);
}

}  // namespace Rfm69
