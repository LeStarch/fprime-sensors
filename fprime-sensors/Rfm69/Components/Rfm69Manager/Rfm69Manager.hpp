// ======================================================================
// \title  Rfm69Manager.hpp
// \brief  hpp file for Rfm69Manager component implementation class
// ======================================================================

#ifndef Rfm69_Rfm69Manager_HPP
#define Rfm69_Rfm69Manager_HPP

#include "Os/Mutex.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ManagerComponentAc.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Registers.hpp"

namespace Rfm69 {

class Rfm69Manager final : public Rfm69ManagerComponentBase {
  public:
    //! Default carrier frequency (Hz): 915 MHz ISM band
    static constexpr U32 DEFAULT_FREQUENCY_HZ = 915000000;
    //! Default network ID (second sync word byte)
    static constexpr U8 DEFAULT_NETWORK_ID = 100;
    //! Default output power level (0-31, RegPaLevel OutputPower field)
    static constexpr U8 DEFAULT_POWER_LEVEL = 15;
    //! Bound on SPI polls awaiting PacketSent for a single packet
    static constexpr U32 TX_POLL_LIMIT = 10000;
    //! Bound on SPI polls draining one received packet from the FIFO
    static constexpr U32 RX_POLL_LIMIT = 10000;
    //! Bound on packets read out of the radio per run invocation
    static constexpr U32 RX_PACKETS_PER_TICK = 8;

    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct Rfm69Manager object
    Rfm69Manager(const char* const compName  //!< The component name
    );

    //! Destroy Rfm69Manager object
    ~Rfm69Manager();

    //! \brief Set the radio configuration; must be called before the radio is initialized
    //!
    //! Called during topology setup. Detection and register configuration
    //! occur on subsequent run invocations.
    void configure(U32 frequencyHz,  //!< Carrier frequency in Hz
                   U8 networkId,     //!< Network ID (second sync word byte)
                   U8 powerLevel     //!< Output power level (0-31)
    );

  private:
    //! Radio management states
    enum RadioState {
        DETECT,     //!< Radio not yet detected on the SPI bus
        CONFIGURE,  //!< Radio detected; configuration pending
        READY       //!< Radio configured and in receive mode
    };

    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for dataIn: transmit a frame over the radio
    void dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;

    //! Handler implementation for dataReturnIn: buffer ownership returned
    void dataReturnIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;

    //! Handler implementation for run: radio initialization and receive polling
    void run_handler(FwIndexType portNum, U32 context) override;

    // ----------------------------------------------------------------------
    // Command handler implementations
    // ----------------------------------------------------------------------

    //! Handler for the TRANSMIT command: enable/disable downlink
    void TRANSMIT_cmdHandler(FwOpcodeType opCode,     //!< The command opcode
                             U32 cmdSeq,              //!< The command sequence number
                             Rfm69::TransmitState enabled  //!< Desired transmit state
                             ) override;

    // ----------------------------------------------------------------------
    // Helper functions: radio state management (Rfm69Manager.cpp)
    // ----------------------------------------------------------------------

    //! Advance detection/configuration; returns true when READY
    void initializeRadio();

    //! Poll for and deliver received packets
    void pollReceive();

    //! Request a reset pulse from the platform GPIO driver, if connected.
    bool pulseReset();

    //! Transmit a deferred frame once the channel clears
    void retryDeferredTransmit();

    //! Segment and transmit a frame; returns true when all packets sent
    bool transmitFrame(Fw::Buffer& data);

    // ----------------------------------------------------------------------
    // Helper functions: RFM69 register access (Rfm69Helpers.cpp)
    // ----------------------------------------------------------------------

    //! \brief Read a single radio register
    Drv::SpiStatus readRegister(U8 address,  //!< Register address
                                U8& value    //!< [out] Register value
    );

    //! \brief Write a single radio register
    Drv::SpiStatus writeRegister(U8 address,  //!< Register address
                                 U8 value     //!< Value to write
    );

    //! \brief Detect the radio by checking the version register
    bool detectRadio();

    //! \brief Apply the packet-mode configuration to the radio
    bool configureRadio();

    //! \brief Command an operating mode and await ModeReady
    bool setMode(U8 mode  //!< Mode field value (Rfm69::Mode)
    );

    //! \brief Listen-before-talk: true when a reception is in progress
    bool channelBusy();

    //! \brief Burst-write a block of payload bytes into the FIFO
    bool writeFifo(const U8* data,  //!< Bytes to load
                   FwSizeType size  //!< Byte count
    );

    //! \brief Transmit one packet of at most MAX_PACKET_PAYLOAD bytes
    bool transmitPacket(const U8* data,  //!< Payload data
                        FwSizeType size  //!< Payload size
    );

    //! \brief Read one received packet out of the radio FIFO
    //! \return payload size read, or 0 on failure
    FwSizeType readReceivedPacket(U8* data,            //!< [out] Payload data
                                  FwSizeType capacity  //!< Capacity of data
    );

    //! \brief Read the RSSI register and update telemetry
    void updateRssi();

    // ----------------------------------------------------------------------
    // Member variables
    // ----------------------------------------------------------------------

    //! Serializes radio access between dataIn (com queue thread) and run
    //! (rate group thread): both drive multi-transaction SPI sequences and
    //! share the SPI scratch buffers
    Os::Mutex m_lock;

    RadioState m_state;        //!< Radio management state
    bool m_configured;         //!< configure() has been called
    U32 m_frequencyHz;         //!< Carrier frequency (Hz)
    U8 m_networkId;            //!< Network ID (sync word byte 2)
    U8 m_powerLevel;           //!< Output power level (0-31)
    U32 m_packetsTransmitted;  //!< Count of transmitted packets
    U32 m_packetsReceived;     //!< Count of received packets
    U32 m_transmitFailures;    //!< Count of failed transmissions
    U32 m_transmitsDeferred;   //!< Count of transmissions deferred by listen-before-talk
    TransmitState m_transmitEnabled;  //!< Whether downlink transmit is permitted
    bool m_resetPulsed;                //!< True once a reset pulse has been issued

    //! Frame deferred by listen-before-talk awaiting a clear channel
    Fw::Buffer m_deferredBuffer;
    ComCfg::FrameContext m_deferredContext;
    bool m_deferredValid;

    //! Scratch buffers for SPI transactions (address byte + length + payload)
    U8 m_mosi[MAX_PACKET_PAYLOAD + 2];
    U8 m_miso[MAX_PACKET_PAYLOAD + 2];
};

}  // namespace Rfm69

#endif  // Rfm69_Rfm69Manager_HPP
