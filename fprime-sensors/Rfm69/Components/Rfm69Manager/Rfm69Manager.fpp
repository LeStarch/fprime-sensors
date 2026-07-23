module Rfm69 {
    @ Communication adapter (Svc.Com interface) for an RFM69HCW radio on a SPI bus
    passive component Rfm69Manager {

        import Svc.Com

        # ----------------------------------------------------------------------
        # Implementation ports
        # ----------------------------------------------------------------------

        @ SPI bus transactions with the radio
        output port spiWriteRead: Drv.SpiWriteRead

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
    }
}
