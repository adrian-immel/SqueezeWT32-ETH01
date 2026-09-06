# SqueezeWT32-ETH01

Squeezebox / Logitech Media Server (LMS) player for the **Wireless-Tag
WT32-ETH01** board (ESP32-D0WD, no PSRAM) using the on-board **LAN8720A
Ethernet** (no WiFi) and a **PCM5102A I2S DAC**.

This is a port of the [SqueezeEsp32](https://github.com/bgiraut/SqueezeEsp32)
project. Differences with the original:

- WiFi + WiFiManager captive portal removed, wired **Ethernet only**
- Audio output is a **PCM5102A I2S DAC** driven by the
  [ESP8266Audio](https://github.com/earlephilhower/ESP8266Audio) software
  decoders (FLAC / MP3 / WAV). ESP8266Audio is pinned to **v2.2.0**: newer
  releases require the new `i2s_std` driver that the classic ESP32 does not
  have (only ESP32-S2/S3/C3).
- VS1053 module, ESP8266 and flac_plugin code removed
- Bigger audio pipeline buffers (the WiFi stack no longer eats the RAM)
- Software volume control (gain applied to the samples, the PCM5102A has no
  volume register)
- PlatformIO build

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

- PHY address 1, MDC = GPIO23, MDIO = GPIO18
- GPIO16 is driven high to enable the external 50 MHz RMII oscillator
- 50 MHz REFCLK enters on GPIO0 (`ETH_CLOCK_GPIO0_IN`)

The player finds the LMS by the slimproto **UDP autodiscovery** broadcast
(like the original project). You must be on the same LAN as your server.

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```sh
cp src/config.h.example src/config.h   # optional : tweak buffer sizes
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
- `AUDIO_BUFFER_SIZE` : pre-decode network jitter buffer (default 24 KB,
  was 4 KB). Raise it if your network has bursts, lower it if you get
  allocation failures.
- `AUDIO_DMA_BUFFER_COUNT` : I2S DMA ring depth (ESP8266Audio constructor arg).
- `UDP_PORT` / `LMS_PORT` / `LMS_HTTP_PORT` : slimproto ports.

## Serial log

At 115200 baud you should see the ethernet link-up, the DHCP address, then:

```
Found LMS server @ 192.168.1.10
Connecting to server @ 192.168.1.10
Connection Ok, send hello to LMS
slimproto connected to LMS @ 192.168.1.10 - free heap ...
```

Start a playback from the LMS web UI (the player shows up as *SqueezeEsp*).
