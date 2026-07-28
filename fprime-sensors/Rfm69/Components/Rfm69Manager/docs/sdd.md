# Rfm69::Rfm69Manager

`Rfm69Manager` is the F´ `Svc.Com` adapter for an RFM69HCW packet radio on
SPI. It transports one F´ Com frame as one native RFM69 packet. The component
uses variable-length packet mode, CRC, whitening, and FIFO streaming. It does
not use RadioHead envelopes, unlimited mode, AES, radio-layer segmentation, or
radio-layer reassembly.

Radio SPI/FIFO work is implemented in `Rfm69Helpers.cpp`. F´ ports,
parameters, commands, state, and buffer ownership are implemented in
`Rfm69Manager.cpp`. Register constants, modem enum maps, and the fixed
native-packet profile live in `Rfm69Radio.hpp`.

## Design invariants

- The native RF payload is 1–255 bytes. A 255-byte packet is valid even
  though the RFM69 FIFO holds only 66 bytes; the packet is streamed while it
  is sent or received.
- `RegPacketConfig1=0xD0` selects variable length, whitening, and CRC. AES
  and unlimited mode remain disabled.
- The component never splits a frame greater than 255 bytes. It rejects the
  frame with `FrameTooLarge`, returns its buffer, and reports Com failure.
- The FPP configuration surface is intentionally small: data rate, receive/AFC
  bandwidth, and transmit power are operator parameters. The rest of the
  interoperable RF profile is a reviewed, fixed flight implementation detail.

## Configuration and control surface

The names follow the useful operational shape of the Zephyr LoRa driver where
RFM69 physics permit it, but RFM69 is an FSK/GFSK radio, not a LoRa modem.

| Kind | Name | Default / behavior |
| --- | --- | --- |
| Parameter | `DATA_RATE: Rfm69DataRate` | `BR_9600`; selects the Pi radio's `RegBitrate`. |
| Parameter | `BANDWIDTH_RX: Rfm69Bandwidth` | `BW_500_KHZ`; selects both `RegRxBw` and `RegAfcBw`. |
| Parameter | `TX_POWER: Rfm69TxPower` | `DBM_13`; selects the Pi PA/OCP/TestPa behavior. |
| Command | `TRANSMIT(TransmitState)` | `ENABLED` permits downlink; `DISABLED` keeps uplink receive active and suppresses downlink. |
| Command | `RESET` | Pulses optional RST GPIO and follows normal detect/configure flow. |
| Diagnostic enum | `Rfm69Mode` | Identifies receive or transmit context in `ConfigurationFailed`; it is not a parameter. |

`TransmitState` is a synchronous command state, not a modem parameter. It has
only `ENABLED` and `DISABLED`; there is deliberately no `DISABLING` state. A
successful synchronous command leaves the manager in one of those two stable
states. Disabling downlink never disables packet reception.

`Rfm69DataRate`, `Rfm69Bandwidth`, and `Rfm69TxPower` are the only modem
values that an operator can change through FPP. `Rfm69Mode` remains valuable
as diagnostic context, but it does not select radio settings. The
enum-to-register maps for the three parameters are in `Rfm69Radio.hpp`.

## Fixed interoperable radio profile

Except for `DATA_RATE`, `BANDWIDTH_RX`, and `TX_POWER`, flight always applies
this profile:

| Setting | Fixed value / register image |
| --- | --- |
| Carrier | 915 MHz; `RegFrf=E4 C0 00` |
| Modulation | Packet FSK with no shaping; `RegDataModul=00` |
| Frequency deviation | 25 kHz; `RegFdev=01 9A` |
| Receive/AFC bandwidth | `BANDWIDTH_RX`; default 500 kHz: `RegRxBw=E0`, `RegAfcBw=E0` |
| Preamble and sync | Four-byte preamble; `RegSyncConfig=B8`; sync `2D A7 5C 39 D1 6E 84 F2` |
| Packet handler | `PacketConfig1=D0`, `PayloadLength=FF`, `FifoThresh=8F`, `PacketConfig2=02` |
| Receiver support | `DioMapping2=07`, `RssiThresh=E4`, `TestDagc=30` |
| Default PA state | `TX_POWER=DBM_13`: `PaLevel=5F`, `Ocp=1A`, `TestPa1=55`, `TestPa2=70` |

The selected data rate and receive/AFC bandwidth are the variable modem timing
settings. The exposed range begins at 100 kHz to retain margin for the fixed
25 kHz deviation at every supported rate; 500 kHz is the hardware-validated
operational default. The default is 500 kHz, not 25 MHz: at the fixed 915 MHz
carrier its nominal receiver passband is approximately 914.75–915.25 MHz,
within the stated 902–928 MHz allocation. It is a receive filter, not a claim
about occupied transmit bandwidth or regulatory approval.
There is no parameter or raw-register command for deviation, shaping,
frequency, or network ID.

The checked-in `GroundStationRadioHead` Feather sketch is compiled for the
default `BR_9600`, `BW_500_KHZ`, and `DBM_13` image above. Changing flight
`DATA_RATE` or `BANDWIDTH_RX` requires rebuilding and reflashing the Feather
with the matching register image. Changing flight `TX_POWER` changes the Pi
transmitter only; reflash the Feather too only when matching Feather transmit
power is required. The complete default image is the fixed profile table above
plus `BR_9600` / `BW_500_KHZ` / `DBM_13`.

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

## Configuration lifecycle

The deployment loads `DATA_RATE`, `BANDWIDTH_RX`, and `TX_POWER` before rate
groups start. FPP rejects unknown enum serializations; the manager retains a
valid loaded value or uses its compiled default. A generated FPP parameter-set
command invokes `parameterUpdated()`, takes the radio out of `READY`, reapplies
the fixed profile plus those three values on a later `run` invocation, and
returns any held listen-before-talk frame with failure. There is no handwritten
`RECONFIGURE` command.

`ConfigurationFailed(Rfm69Mode.Receive)` denotes failure to apply the
resolved valid profile to radio hardware or to enter RX mode. It is not an
invalid-parameter event. On that hardware failure the manager stays out of
`READY` and does not send its initial successful Com-status handshake.

## FIFO service, timing, and PA behavior

At the default 9,600 bit/s rate, a 255-byte packet places 270 bytes on air:
four preamble bytes, eight sync bytes, one length byte, 255 payload bytes, and
two CRC bytes. That is about 225 ms. Packet TX/RX deadlines are calculated
from the active `DATA_RATE` and payload length plus a 75 ms scheduler/SPI
margin; they are not bounded by a count of SPI polls.

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
| REQ-RFM69MGR-002 | Apply the fixed 915 MHz FSK profile plus the three FPP-validated parameters; retain or default a valid value when FPP rejects an invalid serialization. | Unit + HIL |
| REQ-RFM69MGR-003 | Enter `READY` and emit one initial successful Com status only after valid configuration and RX mode entry. | Unit |
| REQ-RFM69MGR-004 | Transmit exactly one 1–255-byte native RF packet while streaming the 66-byte FIFO as needed. | Unit + HIL |
| REQ-RFM69MGR-005 | Reject zero-length and greater-than-255-byte frames without segmentation. | Unit |
| REQ-RFM69MGR-006 | Return every immediate or deferred Com buffer exactly once with final status. | Unit |
| REQ-RFM69MGR-007 | Poll, stream, deliver, and recover from one received packet. | Unit + HIL |
| REQ-RFM69MGR-008 | Bound mode/TX/RX waits by active-rate airtime and recover to RX after failure. | Unit + HIL |
| REQ-RFM69MGR-009 | Defer at most one TX during an active reception. | Unit |
| REQ-RFM69MGR-010 | Keep uplink enabled when `TRANSMIT DISABLED` suppresses downlink. | Unit + HIL |
| REQ-RFM69MGR-011 | Reset and reconfigure without restarting the deployment. | Unit + HIL |
| REQ-RFM69MGR-012 | Telemeter packet/failure/defer counts, RSSI, TX state, active data rate, active receive bandwidth, and active TX power. | Unit + GDS |
| REQ-RFM69MGR-013 | Exchange fixed 255-byte GDS radio records using the strict Feather bridge and fixed-TC framing plugin. | Plugin test + HIL |

## HIL acceptance

For the default profile, acceptance is a Pi/Feather radio session in which a
255-byte TM frame reaches radio GDS, a normal short GDS command reaches flight
through the fixed-TC plugin, `TRANSMIT DISABLED` suppresses TM but permits that
uplink, `RESET` restores traffic, and repeated packets produce no FIFO,
timeout, or configuration failures. The final Pi deployment binary is granted
`cap_sys_nice=eip` so its configured Linux scheduling priorities are available.
