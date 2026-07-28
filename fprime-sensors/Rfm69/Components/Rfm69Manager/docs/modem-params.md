# RFM69 modem parameter reference

`Rfm69Manager` uses curated FPP enums rather than raw modem-register commands.
The names track the operational surface of the Zephyr LoRa driver, while the
values below map to classical RFM69 FSK/GFSK registers. This protects the
native 255-byte packet contract from accidental unsupported modem images.

## Configuration lifecycle

The deployment loads all parameters before starting rate groups. `DATA_RATE`,
`BANDWIDTH_RX`, `FREQUENCY_DEVIATION`, `MODULATION_SHAPING`, `TX_POWER`,
`FREQUENCY_HZ`, and `NETWORK_ID` are copied to the manager and then resolved
as one image before any modem register is changed.

A parameter update or `RECONFIGURE` schedules a new configuration cycle. An
unmappable enum, invalid carrier register value, or incompatible modem tuple
emits `ConfigurationFailed(Receive)` and leaves the component out of `READY`.
The values are therefore discrete operator choices, not arbitrary register
pokes.

## Enum-to-register maps

### `DATA_RATE: Rfm69DataRate`

`RegBitrate = 32 MHz / bit rate`.

| Enum | Bit rate | `RegBitrate` |
| --- | ---: | --- |
| `BR_1200` | 1,200 bit/s | `0x682B` |
| `BR_4800` | 4,800 bit/s | `0x1A0B` |
| `BR_9600` | 9,600 bit/s | `0x0D05` |
| `BR_19200` | 19,200 bit/s | `0x0683` |
| `BR_38400` | 38,400 bit/s | `0x0341` |

Rates above 38.4 kb/s are intentionally absent until FIFO streaming is proven
on the target real-time runner.

### `BANDWIDTH_RX: Rfm69Bandwidth`

The enum labels are operator-friendly approximations in kHz. Flight maps them
to the exact FSK Table-14 `RegRxBw`/`RegAfcBw` encodings, with DCC frequency
4%.

| Enum | Actual filter bandwidth | `RegRxBw` / `RegAfcBw` |
| --- | ---: | --- |
| `BW_10_4_KHZ` | 10,417 Hz | `0xF5` / `0xF5` |
| `BW_20_8_KHZ` | 20,833 Hz | `0xF4` / `0xF4` |
| `BW_50_0_KHZ` | 50,000 Hz | `0xEB` / `0xEB` |
| `BW_100_KHZ` | 100,000 Hz | `0xEA` / `0xEA` |
| `BW_200_KHZ` | 200,000 Hz | `0xE9` / `0xE9` |
| `BW_250_KHZ` | 250,000 Hz | `0xE1` / `0xE1` |
| `BW_500_KHZ` | 500,000 Hz | `0xE0` / `0xE0` |

### `FREQUENCY_DEVIATION: Rfm69Deviation`

`RegFdev` is the deviation divided by the RFM69 frequency step
(approximately 61.035 Hz).

| Enum | Deviation | `RegFdev` |
| --- | ---: | --- |
| `FDEV_5_KHZ` | 5 kHz | `0x0052` |
| `FDEV_10_KHZ` | 10 kHz | `0x00A4` |
| `FDEV_25_KHZ` | 25 kHz | `0x019A` |
| `FDEV_50_KHZ` | 50 kHz | `0x0333` |
| `FDEV_100_KHZ` | 100 kHz | `0x0666` |

### `MODULATION_SHAPING: Rfm69ModulationShaping`

RFM69 has no LoRa-style FEC/coding-rate parameter. This enum is its structural
air-format analogue and selects the FSK/GFSK shaping field in
`RegDataModul`.

| Enum | `RegDataModul` | Meaning |
| --- | --- | --- |
| `FSK_NONE` | `0x00` | Packet FSK with no shaping |
| `GFSK_BT_1_0` | `0x01` | GFSK BT 1.0 shaping |
| `GFSK_BT_0_5` | `0x02` | GFSK BT 0.5 shaping |
| `GFSK_BT_0_3` | `0x03` | GFSK BT 0.3 shaping |

### `TX_POWER: Rfm69TxPower`

The value is a nominal RFM69HCW output setting, not a raw power code. The
helper keeps the radio in an OCP-protected, normal-TestPa state outside a
packet TX.

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

### Carrier and network

| Parameter | Register mapping |
| --- | --- |
| `FREQUENCY_HZ` | `RegFrf = frequency_hz × 2^19 / 32,000,000`; 915 MHz is `0xE4C000`. |
| `NETWORK_ID` | The second byte of the eight-byte sync word: `2D <network-id> 5C 39 D1 6E 84 F2`. |

## Compatibility rule

Before programming the chip, the resolved settings must satisfy:

```text
BitRate < 2 × RxBw
Fdev < RxBw
```

The first condition is the RFM69 receive-filter rule. The second prevents a
deviation as wide as the selected filter. A failure is reported as
`ConfigurationFailed(Receive)` and no partially configured radio is declared
ready.

## Default ground image and packet settings

The default tuple is `BR_9600`, `BW_500_KHZ`, `FDEV_25_KHZ`, `FSK_NONE`,
`DBM_13`, 915 MHz, and network ID `0xA7`. The Feather RadioHead sketch mirrors
that exact image. It also shares the invariant packet settings:

| Packet setting | Register/value |
| --- | --- |
| Native payload limit | 1–255 bytes; `RegPayloadLength=0xFF` in variable-length mode |
| Packet mode | `RegPacketConfig1=0xD0`: whitening and CRC enabled, AES disabled |
| FIFO policy | 66-byte FIFO streamed in threshold bursts; `RegFifoThresh=0x8F` |
| RX restart | `RegPacketConfig2=0x02`; recovery writes `0x06` |
| Preamble and sync | Four-byte preamble; `2D A7 5C 39 D1 6E 84 F2` at defaults |

The Feather has no runtime FPP parameter service. Changing flight parameters
requires rebuilding its compiled register image to the same values before
testing the link. The strict USB radio-GDS contract is always fixed at 255
bytes; use its fixed-TC plugin for ordinary short commands.
