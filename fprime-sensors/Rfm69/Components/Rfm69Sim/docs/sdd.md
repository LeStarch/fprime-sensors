# Rfm69::Rfm69Sim

The `Rfm69Sim` is a register-level simulation of the RFM69HCW radio presented
as the far side of the F Prime `Drv.Spi` interface. It allows a deployment (or
unit test) to run the `Rfm69Manager` against a simulated radio with no
hardware: the manager's SPI transactions operate on a simulated register file
and 66-byte FIFO, and the simulated "RF air interface" is tunneled over a
`Drv.ByteStreamDriver` (e.g. `Drv.TcpServer`) so the GDS can communicate with
the deployment end-to-end.

The register model itself is a plain C++ class (`Rfm69SimModel`) with no
F Prime dependencies beyond basic types, so it is also linked directly into
the `Rfm69Manager` unit tests.

## Requirements

| ID | Shall Statement | Description / Context | Test Method |
|---|---|---|---|
| REQ-RFM69SIM-001 | The simulation shall implement the `Drv.Spi` interface, emulating single, burst, and FIFO SPI register access per datasheet §5.2.1 (wnr address bit, address auto-increment, FIFO address 0x00 non-incrementing). | Full-duplex transaction semantics: MISO byte i is produced while MOSI byte i is consumed. | Unit test |
| REQ-RFM69SIM-002 | The simulation shall model the RFM69 register file with datasheet reset values, including `RegVersion` (0x10) = 0x24. | Enables the manager's detection logic to function unmodified. | Unit test |
| REQ-RFM69SIM-003 | The simulation shall model the 66-byte FIFO and the `RegIrqFlags1`/`RegIrqFlags2` flags `ModeReady`, `SyncAddressMatch`, `Rssi`, `FifoNotEmpty`, `FifoLevel`, `FifoFull`, `PacketSent`, and `PayloadReady`. | Flag set used by packet-mode drivers, including FIFO streaming and listen-before-talk. | Unit test |
| REQ-RFM69SIM-004 | While in transmit mode, the simulation shall drain the variable-length packet from the FIFO at the byte-clock rate, "transmit" the packet payload out its air interface when the declared length has been clocked out, and set `PacketSent`. | Pacing exercises the manager's FIFO top-up path for packets larger than the FIFO. | Unit test |
| REQ-RFM69SIM-005 | While in receive mode, the simulation shall packetize bytes received on its air interface into variable-length packets of at most 255 payload bytes and stream them into the FIFO at the byte-clock rate, setting `SyncAddressMatch` during delivery and `PayloadReady` when the packet is fully delivered. | Pacing exercises the manager's in-reception FIFO drain path for packets larger than the FIFO. | Unit test |
| REQ-RFM69SIM-006 | The simulation shall bound buffered air-interface data, dropping the oldest data when the bound is exceeded. | Radios drop packets; the sim must not grow memory without bound. | Unit test |

## Interface Summary

| Direction | Data | Peer |
|---|---|---|
| Input | SPI transactions (`SpiWriteRead`, guarded) | `Rfm69Manager.spiWriteRead` |
| Input | Uplink bytes from the GDS "air" tunnel (`airDataIn`, `Drv.ByteStreamData`) | `Drv.TcpServer.$recv` |
| Output | Return of air receive buffers (`airDataReturnOut`) | `Drv.TcpServer.recvReturnIn` |
| Output | Transmitted packet payloads to the "air" tunnel (`airDataOut`, `Drv.ByteStreamSend`) | `Drv.TcpServer.$send` |
| Output | Buffer allocation/deallocation (`allocate`/`deallocate`) | `Svc.BufferManager` |

## Design Notes

- **Component kind**: `passive`. The SPI port is `guarded` (per `Drv.Spi`) and
  the air input is synchronous; the guard serializes access to the model.
- **Byte clock**: Time is modeled by advancing the air interface one byte
  time per SPI byte exchanged. Transmissions drain the FIFO and receptions
  fill it at that rate, so packets larger than the 66-byte FIFO stream
  through it as the manager polls, and `SyncAddressMatch` is observable while
  a reception is in progress (exercising listen-before-talk).
- **Fidelity**: The model covers the packet-mode subset exercised by
  `Rfm69Manager` (register access modes, FIFO, mode transitions, IRQ flags).
  Modulation, exact bit-rate timing, RSSI dynamics, AES, and address
  filtering are not modeled; `RegRssiValue` reads back a fixed nominal value.
- **This is a test article, not flight software.**
