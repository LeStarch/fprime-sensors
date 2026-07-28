module Rfm69 {

    @ Manager overseeing the RFM69 radio as a com interface
    @ Configuration is loaded solely through Rfm69Manager's FPP parameters.
    instance rfm69Manager: Rfm69.Rfm69Manager base id Rfm69.SubtopologyConfig.BASE_ID + 0x00001000

    @ RFM69 subtopology. Its named hardware instances are configuration-layer
    @ implementations; the manager communicates with them only through F' ports.
    topology Subtopology {
        instance rfm69Manager
        instance spiDriver
        instance resetGpio

        connections Rfm69 {
            rfm69Manager.spiWriteRead -> spiDriver.SpiWriteRead
            rfm69Manager.resetGpio -> resetGpio.gpioWrite
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
