module Rfm69 {
    @ Register-level RFM69HCW simulation exposed over the Drv.Spi interface
    passive component Rfm69Sim {

        import Drv.Spi

        # ----------------------------------------------------------------------
        # Air interface: tunnels simulated RF packets over a byte stream driver
        # ----------------------------------------------------------------------

        @ Ready signal from the air interface driver (ignored)
        sync input port airReady: Drv.ByteStreamReady

        @ Bytes arriving over the simulated air interface (e.g. from the GDS)
        sync input port airDataIn: Drv.ByteStreamData

        @ Return of buffers received on airDataIn
        output port airDataReturnOut: Fw.BufferSend

        @ Transmitted packet payloads sent out the simulated air interface
        output port airDataOut: Drv.ByteStreamSend

        @ Allocation of buffers for transmitted packets
        output port allocate: Fw.BufferGet

        @ Deallocation of buffers for transmitted packets
        output port deallocate: Fw.BufferSend

        # ----------------------------------------------------------------------
        # Special ports
        # ----------------------------------------------------------------------

        @ Port for requesting the current time
        time get port timeCaller

        @ Event port
        event port logOut

        @ Text event port
        text event port logTextOut

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Failed to allocate a buffer for a transmitted packet
        event SimAllocationFailed() severity warning high format "RFM69 sim buffer allocation failed; dropping transmitted packet" throttle 5
    }
}
