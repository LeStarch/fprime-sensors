# Default RFM69 must-match image: 9.6 kb/s FSK

This document records the default Pi ↔ Feather hardware profile. Flight
exposes `DATA_RATE`, `BANDWIDTH_RX`, and `TX_POWER` as FPP parameters. All
other values below are hardcoded in `Rfm69Manager`; they are not an operator
configuration surface.

```text
DATA_RATE=BR_9600
BANDWIDTH_RX=BW_500_KHZ
TX_POWER=DBM_13

fixed: 915 MHz, 25 kHz Fdev, packet FSK/no shaping,
       four-byte preamble, sync 2D A7 5C 39 D1 6E 84 F2
```

| Register/group | Value | Purpose |
| --- | --- | --- |
| Carrier | `RegFrf=E4 C0 00` | Fixed 915 MHz |
| `RegDataModul` | `00` | Fixed packet-mode FSK, no shaping |
| `RegBitrate` | `0D 05` | `DATA_RATE=BR_9600` (9,600 bit/s) |
| `RegFdev` | `01 9A` | Fixed 25 kHz deviation |
| `RegRxBw` / `RegAfcBw` | `E0` / `E0` | `BANDWIDTH_RX=BW_500_KHZ`: 500 kHz receive/AFC filters |
| Preamble | `00 04` | Fixed four bytes |
| Sync | `RegSyncConfig=B8`; `2D A7 5C 39 D1 6E 84 F2` | Fixed sync enabled, eight bytes; `0xA7` is the network byte |
| Packet configuration | `PacketConfig1=D0`, `PayloadLength=FF`, `FifoThresh=8F`, `PacketConfig2=02` | Variable length, whitening, CRC, 255-byte maximum, TX start on FIFO-not-empty, auto RX restart |
| RX recovery | `PacketConfig2=06` | `AutoRxRestartOn | RxRestart`, written after completion/failure |
| Receiver support | `DioMapping2=07`, `RssiThresh=E4`, `TestDagc=30` | CLKOUT disabled and recommended receiver defaults |
| PA normal state | `PaLevel=5F`, `Ocp=1A`, `TestPa1=55`, `TestPa2=70` | `TX_POWER=DBM_13`: PA1 +13 dBm, OCP protected, no high-power boost |

The native RFM69 application payload is variable length from 1 to 255 bytes.
It has a 66-byte FIFO, not a 66-byte packet limit: flight and the Feather
stream packets larger than the FIFO. A full default packet has 270 on-air
bytes (preamble + sync + length + payload + CRC), about 225 ms at 9.6 kb/s.
The flight deadline adds 75 ms of scheduling/SPI margin, and the Feather
restarts a stalled RX after 300 ms.

The 25 kHz setting is fixed FSK frequency deviation, not a 25 MHz bandwidth.
The default receiver/AFC filter is 500 kHz (0.5 MHz). Centered on the fixed
915 MHz carrier, its nominal passband is approximately 914.75–915.25 MHz,
which is inside the stated 902–928 MHz allocation. This is a profile-placement
calculation only; final operation still requires applicable device-emissions
and regulatory-compliance verification.

The checked-in `GroundStationRadioHead` sketch uses this default profile.
Changing flight `DATA_RATE` or `BANDWIDTH_RX` requires rebuilding and
reflashing the sketch with the matching register image. Changing flight
`TX_POWER` changes only the Pi unless the Feather is separately rebuilt and
reflashed with matching PA registers.

The radio GDS USB link is stricter than the native packet handler: exactly one
255-byte USB record maps to exactly one 255-byte RF packet in either direction.
Use the `GroundStationRadioHead/gds` fixed-TC framing plugin for short GDS
commands; never rely on a serial idle timeout or arbitrary padding.
