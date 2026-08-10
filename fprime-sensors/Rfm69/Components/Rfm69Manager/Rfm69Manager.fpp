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

    @ Controls whether the radio may leave receive mode to downlink.
    @ DISABLED: receive-only; downlink dataIn returns Com FAILURE (ComQueue pauses).
    @ ENABLED: normal TX; re-enable emits Com SUCCESS to resume the queue.
    enum TransmitState : U8 {
        ENABLED
        DISABLED
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
        event SendFailed(status: I32) severity warning high \
            format "Failed to send RFM69 message: {}" throttle 2

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
    }
}
