// ======================================================================
// \title  Rfm69Manager.hpp
// \brief  hpp file for Rfm69Manager component implementation class
// ======================================================================

#ifndef Rfm69_Rfm69Manager_HPP
#define Rfm69_Rfm69Manager_HPP

#include "Os/Mutex.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ManagerComponentAc.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69ModemMaps.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69PacketProfile.hpp"
#include "fprime-sensors/Rfm69/Components/Rfm69Manager/Rfm69Registers.hpp"

namespace Rfm69 {

class Rfm69Manager final : public Rfm69ManagerComponentBase {
  public:
    //! Mode transitions normally take microseconds; never wait indefinitely.
    static constexpr U32 MODE_READY_TIMEOUT_USEC = 20000;
    //! Allow one packet's profile-derived airtime plus host/SPI scheduling room.
    static constexpr U32 PACKET_DEADLINE_MARGIN_USEC = 75000;
    //! Bound on packets read out of the radio per run invocation
    static constexpr U32 RX_PACKETS_PER_TICK = 8;

    // Fixed native-packet modem profile. DATA_RATE, BANDWIDTH_RX, and TX_POWER
    // are the only operator parameters; these values are deliberately compiled
    // into both the flight and RadioHead ground implementations.
    static constexpr U8 FIXED_DATA_MODUL = 0x00;        //!< Packet FSK, no shaping
    static constexpr U16 FIXED_FDEV_REGISTER = 0x019A; //!< 25 kHz deviation
    static constexpr U32 FIXED_FREQUENCY_HZ = 915000000U;

    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct Rfm69Manager object
    Rfm69Manager(const char* const compName  //!< The component name
    );

    //! Destroy Rfm69Manager object
    ~Rfm69Manager();

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

    //! Handler for RESET: pulse hardware RST and reinitialize the radio
    void RESET_cmdHandler(FwOpcodeType opCode,  //!< The command opcode
                          U32 cmdSeq            //!< The command sequence number
                          ) override;

    //! Apply an updated parameter value and schedule radio reconfiguration
    void parameterUpdated(FwPrmIdType id  //!< The parameter ID
                          ) override;

    //! Copy the loaded FPP parameter values before the rate groups start
    void parametersLoaded() override;

    // ----------------------------------------------------------------------
    // Helper functions: radio state management (Rfm69Manager.cpp)
    // ----------------------------------------------------------------------

    //! Advance detection/configuration; returns true when READY
    void initializeRadio();

    //! Request a reconfigure cycle on the next run tick (CONFIGURE or DETECT)
    void requestReconfigure();

    //! Copy the DATA_RATE, BANDWIDTH_RX, and TX_POWER FPP parameter values into members
    void applyParameters();

    //! Return a deferred Com buffer and report its final status exactly once
    void finishDeferredTransmit(Fw::Success status);

    //! Poll for and deliver received packets
    void pollReceive();

    //! Request a reset pulse from the platform GPIO driver, if connected.
    bool pulseReset();

    //! Transmit a deferred frame once the channel clears
    void retryDeferredTransmit();

    //! Transmit exactly one native RF packet; reject zero or >255-byte frames
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

    //! Clear FIFO/re-arm packet RX after a timeout or malformed packet
    bool recoverReceive();

    //! Enable or restore the short-duration +20 dBm PA boost path.
    bool setPowerBoost(bool enabled);

    //! Deadline for a packet of the supplied payload length at active DATA_RATE
    U32 packetDeadlineUsec(FwSizeType payloadBytes) const;

    //! \brief Listen-before-talk: true when a reception is in progress
    bool channelBusy();

    //! \brief Burst-write a block of payload bytes into the FIFO
    bool writeFifo(const U8* data,  //!< Bytes to load
                   FwSizeType size  //!< Byte count
    );

    //! Burst-read bytes from the FIFO.
    bool readFifo(U8* data,  //!< Destination bytes
                  FwSizeType size //!< Byte count
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
    bool m_configured;         //!< FPP parameters have been loaded
    Rfm69DataRate m_dataRate;  //!< Curated FSK bit rate
    Rfm69Bandwidth m_bandwidthRx;  //!< Curated RX/AFC bandwidth
    Rfm69TxPower m_txPower;    //!< HCW PA configuration
    U32 m_packetsTransmitted;  //!< Count of transmitted packets
    U32 m_packetsReceived;     //!< Count of received packets
    U32 m_transmitFailures;    //!< Count of failed transmissions
    U32 m_transmitsDeferred;   //!< Count of transmissions deferred by listen-before-talk
    TransmitState m_transmitEnabled;  //!< Whether downlink transmit is permitted
    bool m_resetPulsed;                //!< True once a reset pulse has been issued
    bool m_comStatusAnnounced;          //!< Initial link-ready status has been sent

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
