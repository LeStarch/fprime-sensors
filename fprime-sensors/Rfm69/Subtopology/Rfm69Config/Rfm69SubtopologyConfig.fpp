module Rfm69 {
    module SubtopologyConfig {
        constant BASE_ID = 0xC0000000
    }

    instance spiDriver: Drv.LinuxSpiDriver base id Rfm69.SubtopologyConfig.BASE_ID + 0x00002000 {
        phase Fpp.ToCpp.Phases.configComponents """
        if (not Rfm69::spiDriver.open(state.rfm69.device.device, state.rfm69.device.select, Drv::SPI_FREQUENCY_1MHZ)) {
            Fw::Logger::log("[ERROR] RFM69 SPI open failed\\n");
        }
        else {
            Fw::Logger::log("[INFO] RFM69 SPI open successful\\n");
        }
        """
    }

    instance rfm69Sim: Rfm69.Rfm69Sim base id Rfm69.SubtopologyConfig.BASE_ID + 0x00003000
}
