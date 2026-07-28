# Rfm69::Rfm69Manager

`Rfm69Manager` is the F´ `Svc.Com` adapter for an RFM69HCW packet radio on
SPI. It transports one F´ Com frame as one native RFM69 packet. The component
uses variable-length packet mode, CRC, whitening, and FIFO streaming. It does
not use RadioHead envelopes, unlimited mode, AES, radio-layer segmentation, or
radio-layer reassembly.

Radio SPI/FIFO work is implemented in `Rfm69Helpers.cpp`. F´ port, parameter,
command, state, and buffer-ownership work is implemented in
`Rfm69Manager.cpp`.

## Design invariants

- The native RF payload is 1–255 bytes. A 255-byte packet is valid even though
  the RFM69 FIFO holds only 66 bytes; the packet is streamed while it is sent
  or received.
- `RegPacketConfig1=0xD0` selects variable length, whitening, and CRC. AES and
  unlimited mode remain disabled.
- The component never splits a frame greater than 255 bytes. It rejects the
  frame with `FrameTooLarge`, returns its buffer, and reports Com failure.
- FPP parameters are the sole source of modem configuration. There is no
  duplicate scalar setup path and no raw-register command.
- The FPP surface deliberately follows the operational shape of the Zephyr
  LoRa driver where RFM69 physics allow it: curated enum parameters, a
  transmit-enable command, and an explicit configuration-failure mode.

## Packet and GDS contract

The radio itself accepts a native payload from 1 through 255 bytes. The
reference deployment and its RadioHead Feather bridge use a stricter GDS
transport contract:

| Path | Contract |
| --- | --- |
| Flight downlink | One 255-byte `TmFrameFixedSize` frame → one 255-byte RF packet. |
| Feather downlink to USB | Only one complete 255-byte RF packet is emitted as one 255-byte USB record. Short or stalled RF packets are discarded and RX is restarted. |
| GDS uplink | One 255-byte USB record → one 255-byte RF packet. The bridge never uses an idle timeout, fragment, or reassembly operation. |
| Flight uplink | The 255-byte fixed TC transfer frame is validated by the ordinary F´ TC/Space Packet deframers. The embedded Space Packet retains its original, shorter declared length. |

Ordinary GDS commands are normally shorter than 255 bytes. The
`GroundStationRadioHead/gds` plugin therefore pads a normal, CRC-valid TC
transfer frame to exactly 255 bytes, changes its TC frame-length field, and
recomputes the FECF. It does **not** append arbitrary serial padding. The F´
Space Packet deframer uses the embedded packet's declared length, so the zero
fill is not dispatched as command data. See the ground-station
[`gds/README.md`](../../../../../../../../GroundStationRadioHead/gds/README.md)
for the required GDS environment and framing selection.

This strict USB rule belongs to the GDS/Feather bridge, not to the native
RFM69 packet handler. It makes every radio GDS record deterministic and keeps
the flight implementation free of piecemeal serial reconstruction.

## LoRa-shaped parameter surface

RFM69 is FSK/GFSK hardware, not a LoRa modem. The parameter names mirror the
useful parts of the LoRa control surface while mapping each value to an RFM69
register image.

| LoRa-style concept | RFM69 parameter | RFM69 meaning |
| --- | --- | --- |
| `DATA_RATE` | `DATA_RATE: Rfm69DataRate` | Classical FSK bit rate (`RegBitrate`). |
| Receiver bandwidth | `BANDWIDTH_RX: Rfm69Bandwidth` | FSK receive/AFC filter bandwidth (`RegRxBw`/`RegAfcBw`). |
| Coding-rate-like air-format selection | `MODULATION_SHAPING: Rfm69ModulationShaping` | FSK/GFSK shaping in `RegDataModul`; RFM69 has no LoRa-style FEC. |
| Transmit enable | `TRANSMIT(TransmitState)` | Allows or suppresses downlink while receive remains active. |
| Configuration failure direction | `Rfm69Mode` | Identifies receive or transmit context in `ConfigurationFailed`. |
| RFM69-specific FSK requirement | `FREQUENCY_DEVIATION: Rfm69Deviation` | FSK deviation (`RegFdev`). |
| RFM69-specific PA control | `TX_POWER: Rfm69TxPower` | Validated PA/OCP/TestPa configuration. |
| Carrier and network | `FREQUENCY_HZ`, `NETWORK_ID` | FRF synthesizer setting and sync byte two. |

The full enum-to-register table is in [modem-params.md](modem-params.md).
The default, hardware-matched image is recorded in
[modem-profile-9600.md](modem-profile-9600.md).

### Parameters and commands

| Kind | Name | Default / behavior |
| --- | --- | --- |
| Parameter | `DATA_RATE` | `BR_9600` |
| Parameter | `BANDWIDTH_RX` | `BW_500_KHZ` |
| Parameter | `FREQUENCY_DEVIATION` | `FDEV_25_KHZ` |
| Parameter | `MODULATION_SHAPING` | `FSK_NONE` |
| Parameter | `TX_POWER` | `DBM_13` |
| Parameter | `FREQUENCY_HZ` | `915000000` Hz |
| Parameter | `NETWORK_ID` | `167` (`0xA7`) |
| Command | `TRANSMIT(ENABLED \| DISABLED)` | `DISABLED` intentionally drops downlink with successful Com flow control while leaving uplink receive enabled. |
| Command | `RECONFIGURE` | Reloads the FPP values and requests a hardware configuration cycle. |
| Command | `RESET` | Pulses the optional RST GPIO, then uses normal detect/apply flow on the next scheduler tick. |

The deployment loads parameters before rate groups start. A parameter update
or `RECONFIGURE` takes the radio out of `READY`, rewrites its image on a later
`run` invocation, and returns any held listen-before-talk frame with failure.

## Validation and default image

Before it writes modem registers, `configureRadio()` resolves every enum and
validates both of these conditions:

```text
BitRate < 2 × RxBw
Fdev < RxBw
```

An unknown enum value, invalid FRF, or incompatible rate/bandwidth/deviation
combination emits `ConfigurationFailed(Rfm69Mode.Receive)`. The manager stays
out of `READY` and does not send its initial successful Com-status handshake.

The default validated tuple is:

```text
BR_9600 + BW_500_KHZ + FDEV_25_KHZ + FSK_NONE + DBM_13
+ FREQUENCY_HZ=915000000 + NETWORK_ID=0xA7
```

It produces the following must-match register values on flight and on the
compiled default Feather sketch:

| Setting | Default register image |
| --- | --- |
| Carrier | `RegFrf=E4 C0 00` (915 MHz) |
| Modem | `RegDataModul=00`, `RegBitrate=0D 05`, `RegFdev=01 9A` |
| Receive filters | `RegRxBw=E0`, `RegAfcBw=E0` |
| PA | `RegPaLevel=5F` (+13 dBm PA1), `RegOcp=1A`, normal TestPa values |
| Sync | `RegSyncConfig=B8`; `2D A7 5C 39 D1 6E 84 F2` |
| Packet handler | `PacketConfig1=D0`, `PayloadLength=FF`, `FifoThresh=8F`, `PacketConfig2=02` |

If a flight modem, frequency, network, or power parameter changes, the
Feather sketch must be rebuilt with the matching register image before use.
The checked-in sketch deliberately represents the validated default rather
than offering an unrelated second configuration API.

## FIFO service, timing, and PA behavior

The default 255-byte packet places 270 bytes on air: four preamble bytes,
eight sync bytes, one length byte, 255 payload bytes, and two CRC bytes. At
9,600 bit/s this is about 225 ms. Packet TX/RX deadlines are calculated from
the active `DATA_RATE` and payload length plus a 75 ms scheduler/SPI margin;
they are not bounded by a count of SPI polls.

`rfmRateGroup` is driven every 1 ms in the reference deployment, separately
from 1 Hz housekeeping. This cadence is required to drain/refill the 66-byte
FIFO in threshold-sized bursts at the supported rates through 38.4 kb/s. A
future faster enum must not be enabled until that cadence and FIFO servicing
are proven on the real-time runner.

`DBM_0` through `DBM_13` use PA1. `DBM_17` uses PA1+PA2 with normal OCP and
TestPa values. `DBM_20` uses the PA1+PA2 path and enables the RFM69 high-power
boost registers only during a TX packet (`OCP=0x0F`, `TestPa1=0x5D`,
`TestPa2=0x7C`); recovery restores OCP and normal TestPa values before RX.
Use `DBM_20` only in accordance with the radio module, power-supply,
duty-cycle, and local RF regulations.

All failed or expired TX/RX paths return to RX, clear FIFO overrun state, and
write `AutoRxRestartOn | RxRestart` (`RegPacketConfig2=0x06`). This prevents a
partial packet or absent `PacketSent` from wedging the half-duplex link.

## Requirements

| ID | Shall statement | Verification |
| --- | --- | --- |
| REQ-RFM69MGR-001 | Detect an RFM69HCW by reading `RegVersion=0x24`. | Unit + HIL |
| REQ-RFM69MGR-002 | Apply the validated LoRa-shaped FPP parameter image and reject incompatible modem combinations. | Unit + HIL |
| REQ-RFM69MGR-003 | Enter `READY` and emit one initial successful Com status only after valid configuration and RX mode entry. | Unit |
| REQ-RFM69MGR-004 | Transmit exactly one 1–255-byte native RF packet while streaming the 66-byte FIFO as needed. | Unit + HIL |
| REQ-RFM69MGR-005 | Reject zero-length and greater-than-255-byte frames without segmentation. | Unit |
| REQ-RFM69MGR-006 | Return every immediate or deferred Com buffer exactly once with final status. | Unit |
| REQ-RFM69MGR-007 | Poll, stream, deliver, and recover from one received packet. | Unit + HIL |
| REQ-RFM69MGR-008 | Bound mode/TX/RX waits by active-rate airtime and recover to RX after failure. | Unit + HIL |
| REQ-RFM69MGR-009 | Defer at most one TX during an active reception. | Unit |
| REQ-RFM69MGR-010 | Keep uplink enabled when `TRANSMIT DISABLED` suppresses downlink. | Unit + HIL |
| REQ-RFM69MGR-011 | Reset and reconfigure without restarting the deployment. | Unit + HIL |
| REQ-RFM69MGR-012 | Telemeter packet/failure/defer counts, RSSI, TX state, and active parameter values. | Unit + GDS |
| REQ-RFM69MGR-013 | Exchange fixed 255-byte GDS radio records using the strict Feather bridge and fixed-TC framing plugin. | Plugin test + HIL |

## HIL acceptance

For the default tuple, acceptance is a Pi/Feather radio session in which a
255-byte TM frame reaches radio GDS, a normal short GDS command reaches flight
through the fixed-TC plugin, `TRANSMIT DISABLED` suppresses TM but permits that
uplink, `RESET` restores traffic, and repeated packets produce no FIFO,
timeout, or configuration failures. The final Pi deployment binary is granted
`cap_sys_nice=eip` so its configured Linux scheduling priorities are available.
