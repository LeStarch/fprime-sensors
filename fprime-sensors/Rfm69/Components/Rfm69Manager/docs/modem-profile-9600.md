# Default RFM69 must-match image: 9.6 kb/s FSK

This document records the **default** enum image used for Pi ↔ Feather
hardware testing. It is not a single-profile modem API: flight exposes the
LoRa-shaped FPP parameters documented in [modem-params.md](modem-params.md).
The checked-in Feather sketch is deliberately compiled for this default image,
so rebuild it with the corresponding values before changing a flight modem,
frequency, network, or PA parameter.

```text
DATA_RATE=BR_9600
BANDWIDTH_RX=BW_500_KHZ
FREQUENCY_DEVIATION=FDEV_25_KHZ
MODULATION_SHAPING=FSK_NONE
TX_POWER=DBM_13
FREQUENCY_HZ=915000000
NETWORK_ID=0xA7
```

| Register/group | Value | Purpose |
| --- | --- | --- |
| Carrier | `RegFrf=E4 C0 00` | 915 MHz (`frequency × 2^19 / 32 MHz`) |
| `RegDataModul` | `00` | Packet-mode FSK, no shaping |
| `RegBitrate` | `0D 05` | 9,600 bit/s |
| `RegFdev` | `01 9A` | 25 kHz deviation |
| `RegRxBw` / `RegAfcBw` | `E0` / `E0` | 500 kHz receive/AFC filters |
| Preamble | `00 04` | Four bytes |
| Sync | `RegSyncConfig=B8`; `2D A7 5C 39 D1 6E 84 F2` | Sync enabled, eight bytes; byte two is `NETWORK_ID` |
| Packet configuration | `PacketConfig1=D0`, `PayloadLength=FF`, `FifoThresh=8F`, `PacketConfig2=02` | Variable length, whitening, CRC, 255-byte maximum, TX start on FIFO-not-empty, auto RX restart |
| RX recovery | `PacketConfig2=06` | `AutoRxRestartOn | RxRestart`, written after completion/failure |
| Receiver support | `DioMapping2=07`, `RssiThresh=E4`, `TestDagc=30` | CLKOUT disabled and recommended receiver defaults |
| PA normal state | `PaLevel=5F`, `Ocp=1A`, `TestPa1=55`, `TestPa2=70` | PA1 +13 dBm, OCP protected, no high-power boost |

The native RFM69 application payload is variable length from 1 to 255 bytes.
It has a 66-byte FIFO, not a 66-byte packet limit: flight and the Feather
stream packets larger than the FIFO. A full default packet has 270 on-air
bytes (preamble + sync + length + payload + CRC), about 225 ms at 9.6 kb/s.
The flight deadline adds 75 ms of scheduling/SPI margin, and the Feather
restarts a stalled RX after 300 ms.

The radio GDS USB link is stricter than the native packet handler: exactly one
255-byte USB record maps to exactly one 255-byte RF packet in either direction.
Use the `GroundStationRadioHead/gds` fixed-TC framing plugin for short GDS
commands; never rely on a serial idle timeout or arbitrary padding.
