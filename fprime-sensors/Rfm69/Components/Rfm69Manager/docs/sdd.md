# Rfm69::Rfm69Manager

F´ `Svc.Com` adapter for an RFM69HCW on SPI: one Com buffer → one native RF
packet (variable-length mode with hardware CRC and whitening; FIFO streamed
because the chip FIFO is only 66 bytes).

| File | Role |
| --- | --- |
| `Rfm69Manager.cpp` | Ports, commands, params, `DETECT`/`CONFIGURE`/`READY`, buffer ownership |
| `Rfm69Helpers.cpp` | SPI registers, TX/RX FIFO streaming, PA boost |
| `Rfm69Radio.hpp` | Register map, modem enum→register maps, fixed packet profile |

## Operator surface

| Kind | Name | Notes |
| --- | --- | --- |
| Param | `DATA_RATE` | Default `BR_9600` |
| Param | `BANDWIDTH_RX` | Default `BW_500_KHZ` (RX/AFC filter) |
| Param | `TX_POWER` | Default `DBM_13` |
| Cmd | `TRANSMIT` | `ENABLED`/`DISABLED` (downlink only; RX stays on) |
| Cmd | `RESET` | Optional RST GPIO, then detect/configure |
| Event | `ConfigurationFailed` | Detect or configure/RX entry failed |
| Event | `SendFailed` | TX rejected or failed |
| Event | `AllocationFailed` | RX buffer allocate failed |
| Tlm | `PacketsTransmitted`, `PacketsReceived`, `LastRssi` | |

All other modem settings are fixed in `NATIVE_PACKET_PROFILE` / helpers (915 MHz
FSK, 25 kHz deviation, sync `2D A7…`, `PacketConfig1=0xD0`). Maps live in
`Rfm69Radio.hpp`. Default flight/ground-station image: `BR_9600` + `BW_500_KHZ` +
`DBM_13`. Changing rate/BW on flight requires a matching ground-station rebuild.

## Behavior

- Lifecycle: `DETECT` → `CONFIGURE` → `READY` on `run`; params/`RESET` re-enter configure or detect.
- Radio accepts **1–255** byte payloads; rejects 0 and >255 via `SendFailed` (no segmentation). Reference GDS/ground-station path uses **255-byte** records — see `GroundStationRadioHead/gds/README.md`.
- At most one deferred TX while RX is in progress (listen-before-talk).
- TX/RX waits use airtime from `DATA_RATE` + 75 ms margin; failures recover to RX.
- `DBM_20` enables PA boost only during TX.

## Requirements

| ID | Shall | Verify |
| --- | --- | --- |
| 001 | Detect HCW (`RegVersion=0x24`) | UT + HIL |
| 002 | Apply fixed profile + three FPP params | UT + HIL |
| 003 | Enter `READY`; one initial Com `SUCCESS` | UT |
| 004 | TX 1–255 bytes with FIFO streaming | UT + HIL |
| 005 | Reject 0 / >255 without splitting | UT |
| 006 | Return every Com buffer once with status | UT |
| 007 | Poll, deliver, recover RX | UT + HIL |
| 008 | Bound waits; recover to RX on failure | UT + HIL |
| 009 | Defer at most one TX during RX | UT |
| 010 | `TRANSMIT DISABLED` blocks downlink only | UT + HIL |
| 011 | `RESET` re-inits without process restart | UT + HIL |
| 012 | Telemeter TX/RX packet counts and `LastRssi` | UT + GDS |

## HIL

Default profile: 255-byte TM reaches radio GDS; short command via fixed-TC
plugin reaches flight; `TRANSMIT DISABLED` stops TM but allows uplink; `RESET`
restores traffic; no sustained FIFO/timeout/config failures. Flight binary needs
`cap_sys_nice` for configured priorities.
