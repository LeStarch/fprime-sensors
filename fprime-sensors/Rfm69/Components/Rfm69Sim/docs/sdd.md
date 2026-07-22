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
| REQ-RFM69SIM-003 | The simulation shall model the 66-byte FIFO and the `RegIrqFlags1`/`RegIrqFlags2` flags `ModeReady`, `FifoNotEmpty`, `FifoFull`, `PacketSent`, and `PayloadReady`. | Minimum flag set used by packet-mode drivers. | Unit test |
| REQ-RFM69SIM-004 | When commanded into transmit mode with a variable-length packet in the FIFO, the simulation shall "transmit" the packet payload out its air interface and set `PacketSent`. | Transmit completes instantaneously in simulation. | Unit test |
| REQ-RFM69SIM-005 | While in receive mode, the simulation shall packetize bytes received on its air interface into variable-length packets of at most 64 payload bytes, load them into the FIFO one packet at a time, and set `PayloadReady`. | Mirrors real radio behavior where each received packet is read out of the FIFO before the next is delivered. | Unit test |
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
- **Fidelity**: The model covers the packet-mode subset exercised by
  `Rfm69Manager` (register access modes, FIFO, mode transitions, IRQ flags).
  Modulation, timing, RSSI dynamics, AES, and address filtering are not
  modeled; `RegRssiValue` reads back a fixed nominal value.
- **This is a test article, not flight software.**
