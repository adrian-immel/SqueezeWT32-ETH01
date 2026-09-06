# SqueezeWT32-ETH01

Squeezebox / Logitech Media Server (LMS) player for the **Wireless-Tag
WT32-ETH01** board (ESP32-D0WD, no PSRAM) using the on-board **LAN8720A
Ethernet** (no WiFi) and a **PCM5102A I2S DAC**.

This is a port of the [SqueezeEsp32](https://github.com/bgiraut/SqueezeEsp32)
project. Differences with the original:

- WiFi + WiFiManager captive portal removed, wired **Ethernet only**
- Audio output is a **PCM5102A I2S DAC** driven by the
  [ESP8266Audio](https://github.com/earlephilhower/ESP8266Audio) software
  decoders (FLAC / MP3 / WAV) plus a small native decoder for raw **PCM**
  streams (SlimProto format `'p'`, as transcoded by the server for some
  sources)
- Advertised codecs are only the ones that work: `flc`, `pcm`, `mp3`
- VS1053 module, ESP8266 and flac_plugin code removed
- Bigger audio pipeline buffers (the WiFi stack no longer eats the RAM)
- Software volume control (gain applied to the samples, the PCM5102A has no
  volume register)
- PlatformIO build

## Platform note (why not the official espressif32 platform)

The stock PlatformIO `espressif32` platform ships an Arduino core stuck at
ESP-IDF 4.x. ESP8266Audio requires IDF 5.x (the new I2S driver) and fails with
`cannot open source file "driver/i2s_std.h"`. As recommended by the
[ESP8266Audio README](https://github.com/earlephilhower/ESP8266Audio#esp32-and-platformio),
this project uses the community
[pioarduino](https://github.com/pioarduino/platform-espressif32) platform,
which is built from the current Espressif Arduino core (here pinned to
`55.03.311` = Arduino core 3.3.11 / IDF 5.5.5).

## Wiring

| PCM5102A | WT32-ETH01 |
|----------|------------|
| VIN      | 5V         |
| GND      | GND        |
| BCK      | IO33       |
| LRCK (WS)| IO32       |
| DIN      | IO17       |
| LOUT/ROUT| amplifier / speakers |

Power both boards from the same 5 V rail. The PCM5102A runs in I2S mode with
no external MCLK needed. IO17/IO32/IO33 do not conflict with the Ethernet
(which uses the internal RMII pins, MDC/MDIO = 23/18 and the oscillator on
GPIO0/GPIO16).

## Ethernet

The WT32-ETH01 Ethernet is configured in `src/main.cpp` for the LAN8720A PHY:

```
ETH.begin(ETH_PHY_LAN8720, 1, 23, 18, 16, ETH_CLOCK_GPIO0_IN);
```

This is the Arduino-ESP32 >= 3.1.0 argument order
(`type, phy_addr, mdc, mdio, power, clk_mode`). On the older core 3.0.x the
first two arguments were swapped (`phy_addr, power, mdc, mdio, type, clk`).

- PHY address 1, MDC = GPIO23, MDIO = GPIO18
- GPIO16 is driven high to enable the external 50 MHz RMII oscillator
- 50 MHz REFCLK enters on GPIO0 (`ETH_CLOCK_GPIO0_IN`)

The player finds the LMS by the slimproto **UDP autodiscovery** broadcast
(like the original project). You must be on the same LAN as your server.

## Audio transport

Playback uses the native SlimProto streaming model (as SqueezeLite does) : the
player opens a TCP connection to the LMS stream address carried in the `strm s`
command (falling back to `LMS_PORT`), sends the HTTP request header verbatim,
skips the HTTP response headers and feeds the remaining bytes to the
ESP8266Audio decoder.

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```sh
pio run                                # build
pio run -t upload                      # flash over the serial adapter
pio device monitor                     # logs, 115200 baud
```

The WT32-ETH01 has no USB connector: program it with a 3.3 V USB-serial
adapter or a downloader gizmo (see the
[wt32-eth01 notes](https://github.com/egnor/wt32-eth01)).

## Configuration

Everything is in `src/config.h`:

- I2S pins for the PCM5102A
- `AUDIO_BUFFER_SIZE` : pre-decode network jitter buffer (default 128 KB).
  When the server feeds faster than real time (FLAC/MP3) the buffer fills and
  absorbs a stalling stream. Raw PCM (`'p'` format) arrives at real time, so
  it only holds whatever the server already pushed. Lower it if you get
  allocation failures.
- `AUDIO_BUFFER_REFILL_GOAL` : a blocking prefill returns once this much data
  has arrived (~16 KB = ~93 ms of 44.1 kHz stereo PCM), keeping start latency
  low while the buffer tops itself up in the background.
- `AUDIO_DMA_BUFFER_COUNT` / `AUDIO_DMA_BUFFER_BYTES` : I2S DMA ring depth.
- `UDP_PORT` / `LMS_PORT` : slimproto discovery/control port (default 3483).

## Serial log

At 115200 baud you should see the ethernet link-up, the DHCP address, then:

```
Found LMS server @ 192.168.1.10
Connecting to server @ 192.168.1.10
Connection Ok, send hello to LMS
slimproto connected to LMS @ 192.168.1.10 - free heap ...
```

Start a playback from the LMS web UI (the player shows up as *SqueezeEsp*).
