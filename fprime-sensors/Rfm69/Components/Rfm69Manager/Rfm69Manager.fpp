module Rfm69 {
    @ Curated classical RFM69 FSK bit rates. Higher modes remain excluded until
    @ FIFO streaming has been demonstrated at those rates on flight hardware.
    enum Rfm69DataRate : U16 {
        BR_1200 = 1200
        BR_4800 = 4800
        BR_9600 = 9600
        BR_19200 = 19200
        BR_38400 = 38400
    }

    @ Curated RFM69 FSK receive/AFC filter bandwidth choices. The range starts
    @ at 100 kHz to leave margin for the fixed 25 kHz deviation at all supported
    @ bit rates; 500 kHz is the hardware-validated operational default.
    enum Rfm69Bandwidth : U16 {
        BW_100_KHZ = 100
        BW_200_KHZ = 200
        BW_250_KHZ = 250
        BW_500_KHZ = 500
    }

    @ Nominal RFM69HCW transmitter output power in dBm. The helper controls
    @ PA1/PA2, OCP, and TestPa boost registers as one validated configuration.
    enum Rfm69TxPower : I8 {
        DBM_0 = 0
        DBM_5 = 5
        DBM_10 = 10
        DBM_13 = 13
        DBM_17 = 17
        DBM_20 = 20
    }

    @ Radio mode associated with a configuration failure, mirroring LoRa.
    enum Rfm69Mode : U8 {
        Transmit
        Receive
    }

    @ Reason a staged downlink packet was not sent.
    enum Rfm69SendFailure : U8 {
        @ Payload was empty or exceeded the 255-byte RFM69 packet limit
        INVALID_SIZE
        @ SPI error or mode/PacketSent timeout; the radio was recovered to RX
        RADIO_FAULT
        @ Local pending-transmit hold was full (ground station only)
        QUEUE_FULL
    }

    @ Radio bring-up progress, reported as telemetry.
    enum Rfm69RadioState : U8 {
        @ Waiting for FPP parameters to load
        NOT_STARTED
        @ Driving / settling the optional hardware reset line
        RESETTING
        @ Probing RegVersion over SPI
        DETECT
        @ Writing the modem profile and entering receive
        CONFIGURE
        @ Receiving; downlink packets accepted
        READY
    }

    @ Controls whether the radio may leave receive mode to downlink.
    @ DISABLED: receive-only; downlink dataIn returns Com FAILURE (ComQueue pauses).
    @ ENABLED: normal TX; re-enable emits Com SUCCESS to resume the queue.
    enum TransmitState : U8 {
        ENABLED
        DISABLED
    }

    @ Tick-driven bring-up: hardware reset pulse, detection, chunked register
    @ configuration, and a one-poll-per-tick ModeReady wait into receive.
    @ Every action performs a bounded number of SPI transactions so a 1 kHz
    @ rate group never blocks on a full radio configuration in one tick.
    state machine BringupMachine {
        @ One bounded bring-up step (sent per run tick until READY)
        signal tick
        @ Restart bring-up from the hardware reset pulse
        signal reset
        @ Rewrite the modem profile after a parameter update (READY only)
        signal reconfigure

        @ Drive RST HIGH; skipped when the optional GPIO is unwired
        action doAssertReset
        @ Count the RST hold window, then release the line
        action doHoldTick
        @ Count the post-release settle window
        action doSettleTick
        @ Probe RegVersion over SPI
        action doDetect
        @ Reset counters and clear RX drain state for a fresh bring-up
        action doPrepareReset
        @ Invalidate the staged register profile so CONFIGURE rebuilds it
        action doPrepareConfigure
        @ Write one bounded chunk of the register profile (or request RX)
        action doConfigureStep
        @ Read IRQ flags once, looking for ModeReady
        action doPollModeReady
        @ Report a configuration failure and restart CONFIGURE
        action doConfigureFailed
        @ Report READY: RadioReady event and one-time com status
        action doAnnounceReady

        @ RST is asserted (GPIO wired and write succeeded)
        guard resetLineHeld
        @ RST hold window elapsed and the line was released
        guard holdElapsed
        @ Post-release settle window elapsed
        guard settleElapsed
        @ RegVersion matched
        guard radioDetected
        @ Full register profile written and RX mode requested
        guard profileWritten
        @ ModeReady observed
        guard modeReady
        @ ModeReady wait timed out or a poll failed on SPI
        guard modeWaitFailed

        initial enter BRINGUP

        state BRINGUP {
            initial enter RESET_ASSERT
            on reset do { doPrepareReset } enter RESET_ASSERT

            state RESET_ASSERT {
                on tick do { doAssertReset } enter ASSERT_CHOICE
            }
            choice ASSERT_CHOICE {
                if resetLineHeld enter RESET_HOLD else enter DETECT
            }
            state RESET_HOLD {
                on tick do { doHoldTick } enter HOLD_CHOICE
            }
            choice HOLD_CHOICE {
                if holdElapsed enter RESET_SETTLE else enter RESET_HOLD
            }
            state RESET_SETTLE {
                on tick do { doSettleTick } enter SETTLE_CHOICE
            }
            choice SETTLE_CHOICE {
                if settleElapsed enter DETECT else enter RESET_SETTLE
            }
            state DETECT {
                on tick do { doDetect } enter DETECT_CHOICE
            }
            choice DETECT_CHOICE {
                if radioDetected do { doPrepareConfigure } enter CONFIGURE else enter DETECT
            }
            state CONFIGURE {
                on tick do { doConfigureStep } enter CONFIGURE_CHOICE
            }
            choice CONFIGURE_CHOICE {
                if profileWritten enter WAIT_MODE_READY else enter CONFIGURE
            }
            state WAIT_MODE_READY {
                on tick do { doPollModeReady } enter MODE_READY_CHOICE
            }
            choice MODE_READY_CHOICE {
                if modeReady do { doAnnounceReady } enter READY else enter MODE_WAIT_CHOICE
            }
            choice MODE_WAIT_CHOICE {
                if modeWaitFailed do { doConfigureFailed } enter CONFIGURE else enter WAIT_MODE_READY
            }
            state READY {
                on reconfigure do { doPrepareConfigure } enter CONFIGURE
            }
        }
    }

    @ Scheduler-driven downlink sequence. No state performs an airtime wait:
    @ each tick action is a bounded number of SPI transactions, and failures
    @ divert through the ABORT_* recovery chain back to receive.
    state machine TransmitMachine {
        @ Begin transmitting the staged buffer
        signal start
        @ One bounded transmit step (sent per run tick while sending)
        signal tick
        @ Drop the sequence without SPI recovery (radio is being reset)
        signal cancel

        @ Request standby so the FIFO can be loaded
        action doRequestStandby
        @ Read IRQ flags once, looking for ModeReady
        action doPollModeReady
        @ Flush FIFO residue from any prior packet
        action doClearFifo
        @ Write the variable-length-mode length byte
        action doWriteLength
        @ Preload the FIFO with the initial payload chunk
        action doLoadInitial
        @ Enable the +20 dBm boost registers when configured
        action doEnableBoost
        @ Request TX mode and start the airtime bound
        action doRequestTx
        @ Top up the FIFO / watch for PacketSent (one bounded step)
        action doStream
        @ Request RX mode after PacketSent
        action doRequestRx
        @ Restore safe OCP/TestPa values after a boosted transmit
        action doDisableBoost
        @ Count the packet and clear failure throttles
        action doFinishSuccess
        @ Report SendFailed and begin bounded abort recovery
        action doBeginAbort
        @ Best-effort FIFO flush on the abort path
        action doAbortClearFifo
        @ Best-effort RX mode request on the abort path
        action doAbortRequestRx
        @ Poll for RX ModeReady, giving up on error or timeout
        action doPollAbortRx
        @ Best-effort boost disable on the abort path
        action doAbortDisableBoost
        @ Record the failed result for the run handler
        action doFinishFailure

        @ The step's SPI transactions all succeeded
        guard stepOk
        @ ModeReady observed
        guard modeReady
        @ ModeReady wait timed out or a poll failed on SPI
        guard modeWaitFailed
        @ Payload fully streamed and PacketSent observed
        guard packetSent
        @ Abort-path RX wait finished (ready, error, or timeout)
        guard abortRxDone

        initial enter IDLE

        state IDLE {
            on start enter SENDING
        }

        state SENDING {
            initial enter REQUEST_STANDBY
            on cancel enter IDLE

            state REQUEST_STANDBY {
                on tick do { doRequestStandby } enter REQUEST_STANDBY_CHOICE
            }
            choice REQUEST_STANDBY_CHOICE {
                if stepOk enter WAIT_STANDBY else enter ABORT_CLEAR_FIFO
            }
            state WAIT_STANDBY {
                on tick do { doPollModeReady } enter WAIT_STANDBY_CHOICE
            }
            choice WAIT_STANDBY_CHOICE {
                if modeReady enter CLEAR_FIFO else enter WAIT_STANDBY_TIMEOUT_CHOICE
            }
            choice WAIT_STANDBY_TIMEOUT_CHOICE {
                if modeWaitFailed enter ABORT_CLEAR_FIFO else enter WAIT_STANDBY
            }
            state CLEAR_FIFO {
                on tick do { doClearFifo } enter CLEAR_FIFO_CHOICE
            }
            choice CLEAR_FIFO_CHOICE {
                if stepOk enter WRITE_LENGTH else enter ABORT_CLEAR_FIFO
            }
            state WRITE_LENGTH {
                on tick do { doWriteLength } enter WRITE_LENGTH_CHOICE
            }
            choice WRITE_LENGTH_CHOICE {
                if stepOk enter LOAD_INITIAL else enter ABORT_CLEAR_FIFO
            }
            state LOAD_INITIAL {
                on tick do { doLoadInitial } enter LOAD_INITIAL_CHOICE
            }
            choice LOAD_INITIAL_CHOICE {
                if stepOk enter ENABLE_BOOST else enter ABORT_CLEAR_FIFO
            }
            state ENABLE_BOOST {
                on tick do { doEnableBoost } enter ENABLE_BOOST_CHOICE
            }
            choice ENABLE_BOOST_CHOICE {
                if stepOk enter REQUEST_TX else enter ABORT_CLEAR_FIFO
            }
            state REQUEST_TX {
                on tick do { doRequestTx } enter REQUEST_TX_CHOICE
            }
            choice REQUEST_TX_CHOICE {
                if stepOk enter WAIT_TX else enter ABORT_CLEAR_FIFO
            }
            state WAIT_TX {
                on tick do { doPollModeReady } enter WAIT_TX_CHOICE
            }
            choice WAIT_TX_CHOICE {
                if modeReady enter STREAM else enter WAIT_TX_TIMEOUT_CHOICE
            }
            choice WAIT_TX_TIMEOUT_CHOICE {
                if modeWaitFailed enter ABORT_CLEAR_FIFO else enter WAIT_TX
            }
            state STREAM {
                on tick do { doStream } enter STREAM_CHOICE
            }
            choice STREAM_CHOICE {
                if stepOk enter STREAM_SENT_CHOICE else enter ABORT_CLEAR_FIFO
            }
            choice STREAM_SENT_CHOICE {
                if packetSent enter REQUEST_RX else enter STREAM
            }
            state REQUEST_RX {
                on tick do { doRequestRx } enter REQUEST_RX_CHOICE
            }
            choice REQUEST_RX_CHOICE {
                if stepOk enter WAIT_RX else enter ABORT_CLEAR_FIFO
            }
            state WAIT_RX {
                on tick do { doPollModeReady } enter WAIT_RX_CHOICE
            }
            choice WAIT_RX_CHOICE {
                if modeReady enter DISABLE_BOOST else enter WAIT_RX_TIMEOUT_CHOICE
            }
            choice WAIT_RX_TIMEOUT_CHOICE {
                if modeWaitFailed enter ABORT_CLEAR_FIFO else enter WAIT_RX
            }
            state DISABLE_BOOST {
                on tick do { doDisableBoost } enter DISABLE_BOOST_CHOICE
            }
            choice DISABLE_BOOST_CHOICE {
                if stepOk do { doFinishSuccess } enter IDLE else enter ABORT_CLEAR_FIFO
            }
            state ABORT_CLEAR_FIFO {
                entry do { doBeginAbort }
                on tick do { doAbortClearFifo } enter ABORT_REQUEST_RX
            }
            state ABORT_REQUEST_RX {
                on tick do { doAbortRequestRx } enter ABORT_REQUEST_RX_CHOICE
            }
            choice ABORT_REQUEST_RX_CHOICE {
                if stepOk enter ABORT_WAIT_RX else enter ABORT_DISABLE_BOOST
            }
            state ABORT_WAIT_RX {
                on tick do { doPollAbortRx } enter ABORT_WAIT_RX_CHOICE
            }
            choice ABORT_WAIT_RX_CHOICE {
                if abortRxDone enter ABORT_DISABLE_BOOST else enter ABORT_WAIT_RX
            }
            state ABORT_DISABLE_BOOST {
                on tick do { doAbortDisableBoost, doFinishFailure } enter IDLE
            }
        }
    }

    @ Communication adapter (Svc.Com interface) for an RFM69HCW radio on SPI.
    passive component Rfm69Manager {
        @ Import the communication interface
        import Svc.Com

        # ----------------------------------------------------------------------
        # Implementation ports
        # ----------------------------------------------------------------------

        @ SPI bus transactions with the radio
        output port spiWriteRead: Drv.SpiWriteRead

        @ Reset requests through a platform-provided GPIO driver
        output port resetGpio: Drv.GpioWrite

        @ Allocation of buffers for received packets
        output port allocate: Fw.BufferGet

        @ Deallocation of buffers returned on dataReturnIn
        output port deallocate: Fw.BufferSend

        @ Rate-group tick: detection, configuration, and receive polling
        sync input port run: Svc.Sched

        # ----------------------------------------------------------------------
        # Special ports
        # ----------------------------------------------------------------------

        time get port timeCaller
        telemetry port tlmOut
        event port logOut
        text event port logTextOut
        command reg port CmdReg
        command recv port CmdDisp
        command resp port CmdStatus
        param get port prmGet
        param set port prmSet

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------

        @ Classical FSK bit rate; default is the next step above 9.6 kb/s.
        @ Explicit IDs preserve stored parameter compatibility after old
        @ fixed-profile parameters were removed. Ground Station must match.
        param DATA_RATE: Rfm69DataRate default Rfm69DataRate.BR_19200 id 0

        @ FSK receive/AFC filter bandwidth; default matches the ground-station image.
        param BANDWIDTH_RX: Rfm69Bandwidth default Rfm69Bandwidth.BW_500_KHZ id 1

        @ HCW transmitter power configuration, including PA boost when needed.
        param TX_POWER: Rfm69TxPower default Rfm69TxPower.DBM_13 id 4

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Enable or disable downlink. Disabled keeps the radio in receive mode.
        sync command TRANSMIT(enabled: TransmitState)

        @ Pulse hardware RST and then run normal detection/configuration.
        sync command RESET

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Event to indicate configuration failure
        event ConfigurationFailed(mode: Rfm69Mode) severity warning high \
            format "Failed to configure RFM69 into mode: {}" throttle 2

        @ Event to indicate send failure
        event SendFailed(reason: Rfm69SendFailure) severity warning high \
            format "Failed to send RFM69 message: {}" throttle 2

        @ Radio detected, configured, and receiving
        event RadioReady severity activity high \
            format "RFM69 radio configured and ready"

        @ Radio reset and re-initialization has begun
        event ResetInitiated severity activity high \
            format "RFM69 reset initiated"

        @ Event to indicate allocation failure
        event AllocationFailed(allocation_size: FwSizeType) severity warning high \
            format "Failed to allocate buffer of: {} bytes" throttle 2

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Successful RF packet transmissions
        telemetry PacketsTransmitted: U32

        @ RF packets delivered on dataOut
        telemetry PacketsReceived: U32

        @ RX frames dropped for failing hardware CRC (noise / corrupted uplink).
        @ A counter, not an event: corrupted frames are expected on a lossy link
        @ and must not spam WARNING events onto the downlink or the soak gate.
        telemetry RxCrcErrors: U32

        @ RSSI of last received packet (dBm)
        telemetry LastRssi: F32

        @ Radio bring-up state
        telemetry RadioState: Rfm69RadioState
    }
}
