# Rfm69::Rfm69Manager

The `Rfm69Manager` is a communication adapter implementing the F Prime `Svc.Com`
interface on top of an RFM69HCW packet radio attached to a SPI bus (`Drv.Spi`
interface, e.g. `Drv.LinuxSpiDriver`). It transmits framed downlink data as one
or more RFM69 packets and delivers received RFM69 packet payloads to the uplink
frame accumulator.

Radio-specific register access and packet handling are implemented as helper
functions in `Rfm69Helpers.cpp`, keeping the F Prime handler code in
`Rfm69Manager.cpp` clean.

Reference: [RFM69HCW datasheet (HopeRF, V1.1)](https://cdn.sparkfun.com/datasheets/Wireless/General/RFM69HCW-V1.1.pdf)

## Requirements

| ID | Shall Statement | Description / Context | Test Method |
|---|---|---|---|
| REQ-RFM69MGR-001 | The component shall detect the RFM69 radio by reading the silicon version register (`RegVersion`, 0x10) via SPI. | Confirms the radio is present and the SPI link works before configuration. Expected value 0x24. | Unit test |
| REQ-RFM69MGR-002 | The component shall configure the radio for variable-length packet mode with FSK modulation, CRC enabled, and the configured carrier frequency, sync word network ID, and output power. | Standard packet-mode operation per datasheet §5.1.2 / §5.5. | Unit test |
| REQ-RFM69MGR-003 | The component shall report `Fw::Success::SUCCESS` on `comStatusOut` once the radio has been detected and configured. | Signals the framer/queue that the interface is ready for the first frame. | Unit test |
| REQ-RFM69MGR-004 | The component shall transmit data received on `dataIn` by segmenting it into radio packets of at most 64 payload bytes, writing each packet to the radio FIFO, commanding transmit mode, and confirming `PacketSent` for each packet. | RFM69 FIFO is 66 bytes; variable-length packets carry a length byte plus payload. Frames larger than one packet are segmented; the receiving side's frame accumulator reassembles the byte stream. | Unit test |
| REQ-RFM69MGR-005 | The component shall report the status of each `dataIn` transmission on `comStatusOut` (`SUCCESS` on complete transmission, `FAILURE` otherwise) and return buffer ownership via `dataReturnOut`. | Com adapter protocol required by `Svc.ComQueue` / framer. | Unit test |
| REQ-RFM69MGR-006 | The component shall poll the radio on its `run` port and, when `PayloadReady` is set, read the received packet payload from the FIFO into an allocated buffer and emit it on `dataOut`. | Deadline-driven (rate group) polling of `RegIrqFlags2`; avoids dependence on DIO interrupt lines. | Unit test |
| REQ-RFM69MGR-007 | The component shall return the radio to receive mode after each transmission and after each packet reception. | The radio idles in RX so uplink data is not lost. | Unit test |
| REQ-RFM69MGR-008 | The component shall emit a WARNING_HI event and report `FAILURE` if a transmission times out or a SPI transaction fails during transmission. | Fault visibility for operators. | Unit test |
| REQ-RFM69MGR-009 | The component shall emit a throttled WARNING_HI event when the radio cannot be detected, and shall retry detection on subsequent `run` invocations. | Radio may be powered late or wired incorrectly; recover without reboot. | Unit test |
| REQ-RFM69MGR-010 | The component shall telemeter counts of packets transmitted, packets received, and transmit failures, and the RSSI of the last received packet. | Basic link health monitoring. | Unit test |
| REQ-RFM69MGR-011 | The component shall drop received data and emit a throttled WARNING_HI event if a buffer cannot be allocated for a received packet. | Off-nominal memory exhaustion handling; radio FIFO is cleared to keep the link alive. | Unit test |
| REQ-RFM69MGR-012 | The component shall not transmit data received on `dataIn` before the radio is initialized, reporting `FAILURE` instead. | Protects against use before configuration completes. | Unit test |

## Interface Summary

| Direction | Data | Peer |
|---|---|---|
| Input | Framed downlink data (`dataIn`, `Svc.ComDataWithContext`) | `Svc.Ccsds.TmFramer` / `Svc.Framer` |
| Output | Transmission status (`comStatusOut`, `Fw.SuccessCondition`) | Framer / `Svc.ComQueue` |
| Output | Received uplink bytes (`dataOut`, `Svc.ComDataWithContext`) | `Svc.FrameAccumulator` |
| Output | Returned downlink buffers (`dataReturnOut`) | Framer |
| Input | Returned uplink buffers (`dataReturnIn`) | `Svc.FrameAccumulator` |
| Output | SPI write/read transactions (`spiWriteRead`, `Drv.SpiWriteRead`) | `Drv.LinuxSpiDriver` or `Rfm69.Rfm69Sim` |
| Output | Buffer allocation/deallocation (`allocate`/`deallocate`) | `Svc.BufferManager` |
| Input | Rate group tick (`run`, `Svc.Sched`) | `Svc.ActiveRateGroup` |

## State

| State | Description |
|---|---|
| `DETECT` | Radio not yet found; each `run` tick attempts to read `RegVersion`. |
| `CONFIGURE` | Radio detected; apply configuration registers and enter RX. |
| `READY` | Radio in RX; `dataIn` transmissions allowed; `run` polls for received packets. |

## Design Notes

- **Component kind**: `passive`. All work is deadline-driven from the rate
  group (`run`) or synchronous with `dataIn`; no queue or thread is needed.
- **Polling, not interrupts**: `RegIrqFlags2` is polled over SPI. This trades a
  small amount of SPI traffic for independence from platform GPIO/interrupt
  wiring. Transmit completion uses a bounded poll (`TX_POLL_LIMIT`) so the
  component cannot spin forever on a wedged radio.
- **Segmentation**: The com adapter contract delivers arbitrarily sized
  frames, while RFM69 packets carry at most 64 payload bytes here. Frames are
  segmented on transmit; both the flight and ground sides run frame
  accumulators/deframers over the resulting byte stream, so no reassembly
  header is required.
- **Configuration**: `configure()` is called during topology setup
  (`configComponents` phase) with the carrier frequency, network ID (sync
  word byte 2), and output power. Register values follow the datasheet
  recommended defaults (Table 23/24) with a 55.555 kb/s bit rate and 50 kHz
  frequency deviation, matching common RFM69 usage.
- **High-power PA**: The RFM69HCW routes power through PA1 (PA0 is not
  connected on the HCW module); `RegPaLevel` is set accordingly. The +20 dBm
  high-power sequence (PA1+PA2 with boost registers) is not enabled.
