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

    @ RFM69 FSK receive-channel filter bandwidth choices. Enum labels are
    @ operator-facing kHz approximations; helpers map each to Table 14 bytes.
    enum Rfm69Bandwidth : U16 {
        BW_10_4_KHZ = 10
        BW_20_8_KHZ = 21
        BW_50_0_KHZ = 50
        BW_100_KHZ = 100
        BW_200_KHZ = 200
        BW_250_KHZ = 250
        BW_500_KHZ = 500
    }

    @ Curated FSK frequency-deviation choices in kHz.
    enum Rfm69Deviation : U16 {
        FDEV_5_KHZ = 5
        FDEV_10_KHZ = 10
        FDEV_25_KHZ = 25
        FDEV_50_KHZ = 50
        FDEV_100_KHZ = 100
    }

    @ RFM69's physical analogue to LoRa coding-rate selection. RFM69 has no
    @ LoRa-style FEC; these values select RegDataModul FSK/GFSK shaping.
    enum Rfm69ModulationShaping : U8 {
        FSK_NONE = 0
        GFSK_BT_1_0 = 1
        GFSK_BT_0_5 = 2
        GFSK_BT_0_3 = 3
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
    enum TransmitState : U8 {
        ENABLED
        DISABLED
    }

    @ Communication adapter (Svc.Com interface) for an RFM69HCW radio on SPI.
    passive component Rfm69Manager {
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
        # LoRa-shaped, validated modem parameter surface
        # ----------------------------------------------------------------------

        @ Classical FSK bit rate; default is the hardware-validated 9.6 kb/s.
        param DATA_RATE: Rfm69DataRate default Rfm69DataRate.BR_9600

        @ FSK receive/AFC filter bandwidth; helpers reject an incompatible rate.
        param BANDWIDTH_RX: Rfm69Bandwidth default Rfm69Bandwidth.BW_500_KHZ

        @ FSK transmitter frequency deviation.
        param FREQUENCY_DEVIATION: Rfm69Deviation default Rfm69Deviation.FDEV_25_KHZ

        @ FSK/GFSK modulation shaping; RFM69's coding-rate analogue.
        param MODULATION_SHAPING: Rfm69ModulationShaping default Rfm69ModulationShaping.FSK_NONE

        @ HCW transmitter power configuration, including PA boost when needed.
        param TX_POWER: Rfm69TxPower default Rfm69TxPower.DBM_13

        @ Carrier frequency in Hz (e.g. 915000000).
        param FREQUENCY_HZ: U32 default 915000000

        @ Network ID written as the second byte of the eight-byte sync word.
        param NETWORK_ID: U8 default 167

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Enable or disable downlink. Disabled keeps the radio in receive mode.
        sync command TRANSMIT(enabled: TransmitState)

        @ Re-apply the loaded parameters to radio hardware on the next run tick.
        sync command RECONFIGURE

        @ Pulse hardware RST and then run normal detection/configuration.
        sync command RESET

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        event RadioConfigured() severity activity high format "RFM69 radio detected and configured"
        event RadioNotDetected() severity warning high format "RFM69 radio not detected on SPI bus" throttle 5
        event ConfigurationFailed(mode: Rfm69Mode) severity warning high \
            format "Failed to configure RFM69 into mode: {}" throttle 5
        event TransmitFailed() severity warning high format "RFM69 packet transmission failed" throttle 5
        event FrameTooLarge(frameSize: U32) severity warning high \
            format "RFM69 frame rejected: {} bytes (valid range is 1-255)" throttle 5
        event RadioNotReady() severity warning high \
            format "RFM69 transmission requested before radio is ready" throttle 5
        event BufferAllocationFailed() severity warning high \
            format "RFM69 buffer allocation failed; dropping received packet" throttle 5
        event ReceiveFailed() severity warning high format "RFM69 packet reception failed" throttle 5
        event TransmitDeferred() severity activity low \
            format "RFM69 transmission deferred: reception in progress" throttle 5
        event TransmitBusyDeferred() severity warning high \
            format "RFM69 transmission rejected: one deferred frame is pending" throttle 5
        event ResetFailed(status: U8) severity warning high \
            format "RFM69 reset GPIO operation failed: {}" throttle 5
        event RadioReset() severity activity high format "RFM69 hardware reset requested"
        event TransmitStateChanged(enabled: TransmitState) severity activity high \
            format "RFM69 transmit state set to {}"
        event DataRateUpdated(dataRate: Rfm69DataRate) severity activity high \
            format "RFM69 data rate set to {}"
        event BandwidthRxUpdated(bandwidth: Rfm69Bandwidth) severity activity high \
            format "RFM69 RX bandwidth set to {}"
        event FrequencyDeviationUpdated(deviation: Rfm69Deviation) severity activity high \
            format "RFM69 frequency deviation set to {}"
        event ModulationShapingUpdated(shaping: Rfm69ModulationShaping) severity activity high \
            format "RFM69 modulation shaping set to {}"
        event TxPowerUpdated(txPower: Rfm69TxPower) severity activity high \
            format "RFM69 TX power set to {}"
        event FrequencyUpdated(frequencyHz: U32) severity activity high \
            format "RFM69 frequency set to {} Hz"
        event NetworkIdUpdated(networkId: U8) severity activity high \
            format "RFM69 network ID set to {}"

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        telemetry PacketsTransmitted: U32
        telemetry PacketsReceived: U32
        telemetry TransmitFailures: U32
        telemetry TransmitsDeferred: U32
        telemetry LastRssi: F32
        telemetry TransmitEnabled: TransmitState update on change
        telemetry DataRate: Rfm69DataRate update on change
        telemetry BandwidthRx: Rfm69Bandwidth update on change
        telemetry FrequencyDeviation: Rfm69Deviation update on change
        telemetry ModulationShaping: Rfm69ModulationShaping update on change
        telemetry TxPower: Rfm69TxPower update on change
        telemetry FrequencyHz: U32 update on change
        telemetry NetworkId: U8 update on change
    }
}
