module XBee {

    constant BASE_ID = 0xF0000000
    
    constant buffMgrId = 0xA000

    module Components {
        constant QUEUE_SIZE = 10
        constant STACK_SIZE = 64 * 1024
    }

    instance comDriver: Drv.LinuxUartDriver base id XBee.BASE_ID + 0x2000 {
        phase Fpp.ToCpp.Phases.configComponents """

        if (state.xbee.device != nullptr && state.xbee.baud != 0) {
            // Uplink is configured for receive so a socket task is started
            if (XBee::comDriver.open(state.xbee.device, static_cast<Drv::LinuxUartDriver::UartBaudRate>(state.xbee.baud),
                            Drv::LinuxUartDriver::NO_FLOW, Drv::LinuxUartDriver::PARITY_NONE, 1024)) {
                XBee::comDriver.start(100, XBee::Components::STACK_SIZE);
            } else {
                Fw::Logger::log("[ERROR] Failed to open UART device %s at baud rate %" PRIu32 "\n", state.xbee.device, state.xbee.baud);
            }
        }
        """
    }

    instance bufferManager: Svc.BufferManager base id XBee.BASE_ID + 0x03000 {
        phase Fpp.ToCpp.Phases.configObjects """
        Svc::BufferManager::BufferBins bins;
        """

        phase Fpp.ToCpp.Phases.configComponents """
        memset(&ConfigObjects::XBee_bufferManager::bins, 0, sizeof(ConfigObjects::XBee_bufferManager::bins));
        ConfigObjects::XBee_bufferManager::bins.bins[0].bufferSize = 2048;
        ConfigObjects::XBee_bufferManager::bins.bins[0].numBuffers = 5;
        XBee::bufferManager.setup(
            XBee::buffMgrId,
            0,
            XBee::Allocation::memAllocator,
            ConfigObjects::XBee_bufferManager::bins
        );
        """

        phase Fpp.ToCpp.Phases.tearDownComponents """
        XBee::bufferManager.cleanup();
        """
    }
}
