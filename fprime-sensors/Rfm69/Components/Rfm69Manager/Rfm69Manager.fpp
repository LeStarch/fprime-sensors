module Rfm69 {
    @ Controls whether the radio is allowed to leave receive mode to downlink.
    @ Mirrors the fprime-zephyr LoRa driver's TRANSMIT command so operators can
    @ command deterministic receive-only (uplink) windows on the half-duplex link.
    enum TransmitState {
        @ Downlink frames are transmitted, then the radio returns to RX
        ENABLED
        @ Radio stays in receive mode; downlink frames are dropped
        DISABLED
    }

    @ Communication adapter (Svc.Com interface) for an RFM69HCW radio on a SPI bus
    passive component Rfm69Manager {

        import Svc.Com

        # ----------------------------------------------------------------------
        # Implementation ports
        # ----------------------------------------------------------------------

        @ SPI bus transactions with the radio
        output port spiWriteRead: Drv.SpiWriteRead

        @ Reset requests are issued through a platform-provided GPIO driver.
        @ The manager never owns GPIO hardware directly.
        output port resetGpio: Drv.GpioWrite

        @ Allocation of buffers for received packets
        output port allocate: Fw.BufferGet

        @ Deallocation of buffers returned on dataReturnIn
        output port deallocate: Fw.BufferSend

        @ Rate group tick: radio detection, configuration, and receive polling
        sync input port run: Svc.Sched

        # ----------------------------------------------------------------------
        # Special ports
        # ----------------------------------------------------------------------

        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Event port
        event port logOut

        @ Text event port
        text event port logTextOut

        @ Command registration port
        command reg port CmdReg

        @ Command receive port
        command recv port CmdDisp

        @ Command response port
        command resp port CmdStatus

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Enable or disable transmit. When disabled the radio remains in receive
        @ mode and downlink frames are dropped, giving a commandable receive-only
        @ window on the half-duplex link.
        sync command TRANSMIT(enabled: TransmitState)

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Radio detected and configured
        event RadioConfigured() severity activity high format "RFM69 radio detected and configured"

        @ Radio version register did not read back as expected
        event RadioNotDetected() severity warning high format "RFM69 radio not detected on SPI bus" throttle 5

        @ Failed to configure the radio
        event RadioConfigurationFailed() severity warning high format "RFM69 radio configuration failed" throttle 5

        @ A transmission failed
        event TransmitFailed() severity warning high format "RFM69 packet transmission failed" throttle 5

        @ Data received while the radio is not initialized
        event RadioNotReady() severity warning high format "RFM69 transmission requested before radio is ready" throttle 5

        @ Failed to allocate a buffer for a received packet
        event BufferAllocationFailed() severity warning high format "RFM69 buffer allocation failed; dropping received packet" throttle 5

        @ Failed to read a received packet out of the radio FIFO
        event ReceiveFailed() severity warning high format "RFM69 packet reception failed" throttle 5

        @ Transmission deferred because a reception is in progress
        event TransmitDeferred() severity activity low format "RFM69 transmission deferred: reception in progress" throttle 5

        @ Platform GPIO driver rejected an RFM69 reset request
        event ResetFailed(status: U8) severity warning high format "RFM69 reset GPIO operation failed: {}" throttle 5

        @ Transmit was enabled or disabled by command
        event TransmitStateChanged(enabled: TransmitState) severity activity high format "RFM69 transmit state set to {}"

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Number of radio packets transmitted
        telemetry PacketsTransmitted: U32

        @ Number of radio packets received
        telemetry PacketsReceived: U32

        @ Number of failed transmissions
        telemetry TransmitFailures: U32

        @ Number of transmissions deferred by listen-before-talk
        telemetry TransmitsDeferred: U32

        @ RSSI of the last received packet (dBm)
        telemetry LastRssi: F32

        @ Current transmit enable state
        telemetry TransmitEnabled: TransmitState update on change
    }
}
