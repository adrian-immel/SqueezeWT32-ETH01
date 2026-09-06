/*
 * SqueezeWT32-ETH01 - configuration
 *
 * Copy this file to "src/config.h" and adjust if needed.
 *
 * Note: unlike the original SqueezeESP32 project there is no WiFi and no
 * WiFiManager captive portal. This build uses the WT32-ETH01 wired Ethernet
 * only, so the server is found with the slimproto UDP autodiscovery broadcast.
 */
#ifndef config_h
#define config_h

#define I2S_DAC_MODULE

/* -------------------------------------------------------------------------
 * PCM5102A I2S wiring
 *
 *   PCM5102A   WT32-ETH01
 *   BCK         IO33
 *   LRCK (WS)   IO32
 *   DIN         IO17
 *   VIN         5V
 *   GND         GND
 *   LOUT/ROUT   amplifier / speakers
 * ---------------------------------------------------------------------- */
#define I2S_BCK_PIN    33
#define I2S_LRCK_PIN   32
#define I2S_DOUT_PIN   17

/* -------------------------------------------------------------------------
 * Logitech Media Server ports
 * ---------------------------------------------------------------------- */
#define UDP_PORT       3483   /* slimproto discovery + command connection  */
#define LMS_PORT       3483   /* command/stream TCP connection to the LMS  */

/* -------------------------------------------------------------------------
 * Buffers.
 *
 * Static RAM use of this build is about 33 KB (see the linker map), the rest
 * of the internal DRAM is heap. The WiFi stack is gone (wired Ethernet only)
 * so there is plenty of room for a deep audio pipeline. Tune these down if
 * you get allocation failures in the serial log.
 *
 * AUDIO_BUFFER_SIZE      pre-decode network jitter buffer. The decoder reads
 *                        the compressed bytes from here, so bigger = longer
 *                        stall tolerance : when the server feeds faster than
 *                        real time (FLAC/MP3) the buffer fills and absorbs a
 *                        stalling stream. Raw PCM (SlimProto format 'p')
 *                        arrives at real time, so the buffer only holds
 *                        whatever burst the server already pushed - it can
 *                        not grow on its own.
 * AUDIO_BUFFER_REFILL_GOAL
 *                        a blocking prefill returns as soon as this many
 *                        bytes have arrived instead of waiting for a full
 *                        AUDIO_BUFFER_SIZE. Keeps the start-of-song gap at
 *                        ~93 ms of 44.1 kHz stereo PCM (~16 KB) and lets the
 *                        buffer top itself up in the background afterwards.
 * PCM_PREROLL_BYTES      raw PCM ('p') streams arrive at real time, so they
 *                        can not build up a cushion while playing. Wait for
 *                        this many bytes before starting a PCM stream : the
 *                        head start then absorbs short server-side feed gaps
 *                        (e.g. when the LMS transcode hiccups on a volume
 *                        change). ~0.37 s of 44.1 kHz stereo at 64 KB.
 * AUDIO_DMA_BUFFER_COUNT / AUDIO_DMA_BUFFER_BYTES
 *                        I2S DMA ring depth (SetBuffers()). More/larger DMA
 *                        buffers smooth out decoder timing glitches.
 * ---------------------------------------------------------------------- */
#define AUDIO_BUFFER_SIZE        131072   /* 128 KB */
#define AUDIO_BUFFER_REFILL_GOAL 16384
#define PCM_PREROLL_BYTES        65536    /* ~0.37 s of 44.1 kHz stereo PCM */
#define AUDIO_DMA_BUFFER_COUNT   8
#define AUDIO_DMA_BUFFER_BYTES   2304

#endif
