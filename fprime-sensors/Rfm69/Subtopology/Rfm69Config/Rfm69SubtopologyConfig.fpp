module Rfm69 {
    module SubtopologyConfig {
        constant BASE_ID = 0xC0000000
    }

    @ Linux platform implementations are intentionally isolated in the
    @ configuration layer. The topology and manager only use F' ports.
    instance spiDriver: Drv.LinuxSpiDriver base id Rfm69.SubtopologyConfig.BASE_ID + 0x00002000 {
        phase Fpp.ToCpp.Phases.configComponents """
        // The Pi/reference wiring shows intermittent MISO bit-7 sampling errors
        // at 5 MHz after the RFM69 has already accepted the RF CRC. One MHz is
        // comfortably inside the 1 kHz manager budget and is byte-exact in HIL.
        if (not Rfm69::spiDriver.open(state.rfm69.device.device, state.rfm69.device.select,
                                      Drv::SPI_FREQUENCY_1MHZ,
                                      Drv::SPI_MODE_CPOL_LOW_CPHA_LOW)) {
            Fw::Logger::log("[ERROR] RFM69 SPI open failed\\n");
        }
        """
    }

    @ RFM69 active-high reset line; pin assignment comes from topology state.
    instance resetGpio: Drv.LinuxGpioDriver base id Rfm69.SubtopologyConfig.BASE_ID + 0x00005000 {
        phase Fpp.ToCpp.Phases.configComponents """
        if (Rfm69::resetGpio.open(state.rfm69.device.resetGpioChip, state.rfm69.device.resetGpioPin,
                                  Drv::LinuxGpioDriver::GPIO_OUTPUT, Fw::Logic::LOW) != Os::File::OP_OK) {
            Fw::Logger::log("[ERROR] RFM69 reset GPIO open failed\\n");
        }
        """
    }

    instance rfm69Sim: Rfm69.Rfm69Sim base id Rfm69.SubtopologyConfig.BASE_ID + 0x00003000
}
