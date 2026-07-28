# RFM69 configuration reference

`Rfm69Manager` exposes exactly three modem FPP parameters:

| Parameter | Type | Default | Scope |
| --- | --- | --- | --- |
| `DATA_RATE` | `Rfm69DataRate` | `BR_9600` | Pi RFM69 bitrate and airtime deadlines |
| `BANDWIDTH_RX` | `Rfm69Bandwidth` | `BW_500_KHZ` | Pi RFM69 receive and AFC filter bandwidth |
| `TX_POWER` | `Rfm69TxPower` | `DBM_13` | Pi RFM69 PA/OCP/TestPa behavior |

They are curated choices, not raw register writes. FPP rejects an unknown
enum serialization; the manager retains a valid loaded value or falls back to
its compiled default. F´'s generated parameter-set command invokes
`parameterUpdated()`, which schedules a configuration cycle that reapplies the
fixed radio profile plus those resolved valid values. There is no separate
handwritten reconfigure command. `ConfigurationFailed(Receive)` denotes
failure to apply that resolved profile to the radio hardware, not parameter
deserialization failure.

`Rfm69Mode` is diagnostic event context, not configuration. `TransmitState`
is the argument to the synchronous `TRANSMIT` command, not a parameter:
`ENABLED` permits downlink and `DISABLED` creates a receive-only window. There
is no transient `DISABLING` state.

## `DATA_RATE: Rfm69DataRate`

`RegBitrate = 32 MHz / bit rate`.

| Enum | Bit rate | `RegBitrate` |
| --- | ---: | --- |
| `BR_1200` | 1,200 bit/s | `0x682B` |
| `BR_4800` | 4,800 bit/s | `0x1A0B` |
| `BR_9600` | 9,600 bit/s | `0x0D05` |
| `BR_19200` | 19,200 bit/s | `0x0683` |
| `BR_38400` | 38,400 bit/s | `0x0341` |

Rates above 38.4 kb/s are intentionally absent until FIFO streaming is proven
on the target real-time runner. The exposed bandwidth range starts at 100 kHz
to retain margin for the fixed 25 kHz deviation at every listed rate; 500 kHz
remains the hardware-validated operational default.

## `BANDWIDTH_RX: Rfm69Bandwidth`

`BANDWIDTH_RX` sets both RFM69 receiver filters: `RegRxBw` and `RegAfcBw`.
It is a receiver/AFC filter setting, not the carrier frequency and not a
request to transmit an RF signal of that width. The intentionally small enum
contains only values valid with the fixed 25 kHz frequency deviation and the
supported data rates.

| Enum | Receive/AFC bandwidth | `RegRxBw` / `RegAfcBw` |
| --- | ---: | --- |
| `BW_100_KHZ` | 100 kHz | `0xEA` / `0xEA` |
| `BW_200_KHZ` | 200 kHz | `0xE9` / `0xE9` |
| `BW_250_KHZ` | 250 kHz | `0xE1` / `0xE1` |
| `BW_500_KHZ` | 500 kHz | `0xE0` / `0xE0` |

The default is 500 kHz (0.5 MHz), not 25 MHz. With the fixed 915 MHz carrier,
that nominal receiver passband is approximately 914.75–915.25 MHz, well
inside the stated 902–928 MHz allocation. The 25 kHz value elsewhere in this
profile is FSK frequency deviation, not bandwidth. This spectral-placement
check is not a substitute for device-level emissions and regulatory-compliance
testing.

## `TX_POWER: Rfm69TxPower`

The value is a nominal RFM69HCW output setting, not a raw PA code. The helper
keeps the Pi radio in an OCP-protected, normal-TestPa state outside a packet
TX.

| Enum | `RegPaLevel` | PA path / special behavior |
| --- | --- | --- |
| `DBM_0` | `0x52` | PA1, nominal 0 dBm |
| `DBM_5` | `0x57` | PA1, nominal +5 dBm |
| `DBM_10` | `0x5C` | PA1, nominal +10 dBm |
| `DBM_13` | `0x5F` | PA1, nominal +13 dBm |
| `DBM_17` | `0x7F` | PA1+PA2, normal OCP/TestPa values |
| `DBM_20` | `0x7F` | PA1+PA2; during TX only: `Ocp=0x0F`, `TestPa1=0x5D`, `TestPa2=0x7C` |

For every setting other than active `DBM_20` transmission, the helper writes
`Ocp=0x1A`, `TestPa1=0x55`, and `TestPa2=0x70`. It restores those values before
returning to RX. Ensure the module supply, permitted duty cycle, antenna, and
local RF regulations support any high-power selection.

## Fixed profile (not FPP parameters)

The following are hardcoded in the flight manager to keep the Pi and Feather
interoperable:

| Setting | Value / register image |
| --- | --- |
| Carrier | 915 MHz; `RegFrf=E4 C0 00` |
| Modulation | Packet FSK, no shaping; `RegDataModul=00` |
| Frequency deviation | 25 kHz; `RegFdev=01 9A` |
| Receive/AFC bandwidth | `BANDWIDTH_RX`; default 500 kHz, `RegRxBw=E0`, `RegAfcBw=E0` |
| Preamble and sync | Four-byte preamble; `2D A7 5C 39 D1 6E 84 F2` (`0xA7` network byte) |
| Packet handler | Variable length, whitening, CRC, no AES: `PacketConfig1=D0`, `PayloadLength=FF`, `FifoThresh=8F`, `PacketConfig2=02` |
| RX recovery | `PacketConfig2=06` (`AutoRxRestartOn | RxRestart`) |

There is intentionally no FPP parameter for deviation, modulation shaping,
frequency, or network ID. The complete default image is recorded in
[modem-profile-9600.md](modem-profile-9600.md).

## Feather matching rule

The checked-in `GroundStationRadioHead` sketch is compiled for the default
profile with `BR_9600`, `BW_500_KHZ`, and `DBM_13`. A flight `DATA_RATE` or
`BANDWIDTH_RX` change must be matched by rebuilding and reflashing that
Feather sketch with the same register image. A flight `TX_POWER` change
affects the Pi transmitter only; rebuild and reflash the Feather only if its
transmit power must match too.

The strict USB radio-GDS contract always remains one 255-byte USB record per
one 255-byte RF packet. Use its fixed-TC plugin for ordinary short commands.
