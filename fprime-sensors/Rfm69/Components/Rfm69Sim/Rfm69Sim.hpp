// ======================================================================
// \title  Rfm69Sim.hpp
// \brief  hpp file for Rfm69Sim component implementation class
// ======================================================================

#ifndef Rfm69_Rfm69Sim_HPP
#define Rfm69_Rfm69Sim_HPP

#include "fprime-sensors/Rfm69/Components/Rfm69Sim/Rfm69SimComponentAc.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Sim/Rfm69SimModel.hpp"

namespace Rfm69 {

class Rfm69Sim final : public Rfm69SimComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct Rfm69Sim object
    Rfm69Sim(const char* const compName  //!< The component name
    );

    //! Destroy Rfm69Sim object
    ~Rfm69Sim();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for SpiWriteRead
    Drv::SpiStatus SpiWriteRead_handler(FwIndexType portNum, Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer) override;

    //! Handler implementation for SpiReadWrite (deprecated Drv.Spi port)
    void SpiReadWrite_handler(FwIndexType portNum, Fw::Buffer& writeBuffer, Fw::Buffer& readBuffer) override;

    //! Handler implementation for airDataIn: bytes from the simulated air interface
    void airDataIn_handler(FwIndexType portNum, Fw::Buffer& buffer, const Drv::ByteStreamStatus& status) override;

    //! Handler implementation for airReady: air interface driver ready (no-op)
    void airReady_handler(FwIndexType portNum) override;

    //! Send any transmitted packets out the air interface
    void drainTransmittedPackets();

    Rfm69SimModel m_model;  //!< Register-level radio model
};

}  // namespace Rfm69

#endif  // Rfm69_Rfm69Sim_HPP
