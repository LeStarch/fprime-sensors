module Rfm69 {

    @ Manager overseeing the RFM69 radio as a com interface
    instance rfm69Manager: Rfm69.Rfm69Manager base id Rfm69.SubtopologyConfig.BASE_ID + 0x00001000 {
        phase Fpp.ToCpp.Phases.configComponents """
        Rfm69::rfm69Manager.configure(Rfm69::Rfm69Manager::DEFAULT_FREQUENCY_HZ,
                                      Rfm69::Rfm69Manager::DEFAULT_NETWORK_ID,
                                      Rfm69::Rfm69Manager::DEFAULT_POWER_LEVEL);
        """
    }

    @ Hardware subtopology: RFM69 manager attached to a Linux SPI driver
    topology Subtopology {
        instance rfm69Manager
        instance spiDriver

        connections Rfm69 {
            rfm69Manager.spiWriteRead -> spiDriver.SpiWriteRead
        }
    }

    @ Simulation subtopology: RFM69 manager attached to a register-level simulation
    topology SimSubtopology {
        instance rfm69Manager
        instance rfm69Sim

        connections Rfm69Sim {
            rfm69Manager.spiWriteRead -> rfm69Sim.SpiWriteRead
        }
    }
}
