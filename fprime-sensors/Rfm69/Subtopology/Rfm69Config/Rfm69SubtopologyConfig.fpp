module Rfm69 {
    module SubtopologyConfig {
        constant BASE_ID = 0xC0000000
    }

    @ Linux platform implementations are intentionally isolated in the
    @ configuration layer. The topology and manager only use F' ports.
    instance spiDriver: Drv.LinuxSpiDriver base id Rfm69.SubtopologyConfig.BASE_ID + 0x00002000 {
        phase Fpp.ToCpp.Phases.configComponents """
        if (not Rfm69::spiDriver.open(0, 1, Drv::SPI_FREQUENCY_5MHZ,
                                      Drv::SPI_MODE_CPOL_LOW_CPHA_LOW)) {
            Fw::Logger::log("[ERROR] RFM69 SPI open failed\\n");
        }
        """
    }

    @ RFM69 active-high reset: BCM GPIO26, physical pin 37.
    instance resetGpio: Drv.LinuxGpioDriver base id Rfm69.SubtopologyConfig.BASE_ID + 0x00005000 {
        phase Fpp.ToCpp.Phases.configComponents """
        if (Rfm69::resetGpio.open("/dev/gpiochip0", 26,
                                  Drv::LinuxGpioDriver::GPIO_OUTPUT, Fw::Logic::LOW) != Os::File::OP_OK) {
            Fw::Logger::log("[ERROR] RFM69 reset GPIO26 open failed\\n");
        }
        """
    }

    instance rfm69Sim: Rfm69.Rfm69Sim base id Rfm69.SubtopologyConfig.BASE_ID + 0x00003000
}
