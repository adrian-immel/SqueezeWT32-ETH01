#include "slimproto.h"

#include <WiFi.h>   // WiFiClient (NetworkClient) concrete type for the stream

/*
 * NOTE : This is a trimmed down, Ethernet + PCM5102A (I2S) version of the
 * original SqueezeESP32 slimproto code. The VS1053 hardware decoder path and
 * the whole ESP8266 branch have been removed. Audio is decoded in software by
 * the ESP8266Audio library and pushed to the I2S DAC.
 *
 * Audio transport follows the native SlimProto behaviour (like SqueezeLite) :
 * the player opens a TCP connection to the LMS stream address given in the
 * "strm s" command, sends the HTTP request header verbatim, skips the HTTP
 * response headers and feeds the remaining bytes to the decoder.
 */

// Called when there's a warning or error (like a buffer underflow or decode hiccup)
void StatusCallback(void *cbData, int code, const char *string)
{
  const char *ptr = reinterpret_cast<const char *>(cbData);
  char s1[64];
  strncpy_P(s1, string, sizeof(s1));
  s1[sizeof(s1)-1]=0;
  Serial.printf("STATUS(%s) '%d' = '%s'\n", ptr, code, s1);
  Serial.flush();
}

/**
 * AudioFileSource that reads the bytes coming from an already connected TCP
 * stream client.
 *
 * read() collects until the request is fully satisfied (that is what keeps
 * the jitter buffer pre-filled, required for real-time rate streams like
 * Spoton PCM). A gap in the data is NOT end of stream : endless streams
 * (Spoton) legitimately pause feeding while the socket stays open. Only a
 * closed socket terminates the stream.
 *
 * While waiting for bytes the LMS control socket is pumped (pause/unpause/
 * volume handled inline, start/stop parked as a pending command) so the
 * player stays responsive even though the decoder sits inside this read
 * ("stuck audio" fix), without ever tearing down the chain from under us.
 */
class StreamAudioSource : public AudioFileSource {
public:
  StreamAudioSource(Client & client, slimproto * owner) :
    vClient(client), vOwner(owner) {
    vOpen = vClient.connected();
  }
  virtual ~StreamAudioSource() override {
    close();
  }

  virtual uint32_t read(void *data, uint32_t len) override {
    if (!vOpen) return 0;
    uint8_t *ptr = (uint8_t *)data;
    uint32_t got = 0;
    // Watchdog for a half-dead stream (socket open, server silent while it
    // claims to be playing). While paused the LMS stops sending on purpose,
    // so the watchdog is disabled in that state.
    unsigned long viDeadline = millis() + 30000UL;

    while (got < len)
    {
      int avail = vClient.available();
      if (avail > 0)
      {
        uint32_t want = (avail < (int)(len - got)) ? (uint32_t)avail : (len - got);
        int r = vClient.read(ptr + got, want);
        if (r > 0)
        {
          got += (uint32_t)r;
          if (vOwner)
            vOwner->AddStreamBytes((uint32_t)r);
          viDeadline = millis() + 30000UL;
          continue;  // keep collecting until the request is satisfied
        }
        if (r < 0)
          break;
      }

      // No new bytes right now : closed and drained = real end of stream
      if (!vClient.connected() && vClient.available() == 0)
      {
        if (got == 0)
        {
          Serial.println("read: stream socket closed by server (EOS)");
          vOpen = false;
        }
        break;
      }

      // Keep the LMS link alive while waiting : processes pause/unpause/
      // volume/track-position inline, parks new stream commands.
      if (vOwner)
      {
        if (!vOwner->PumpControlDuringAudio())
        {
          vOpen = false;   // control link died
          break;
        }
        if (vOwner->StreamAbortPending())
        {
          Serial.printf("read: unblocked by parked strm cmd (t=%lums)\r\n", millis());
          break;           // song change/stop parked : unblock the decoder
        }
        // Feed silence into the I2S DMA while stalled, otherwise the
        // circular DMA re-emits its last contents ("stuck CD").
        vOwner->FeedIdleSamples();
      }

      // Jitter-buffer sized refill : a 16 KB head start is pre-fill enough,
      // return instead of waiting for the full 24 KB (halves the audible
      // gap when refilling after an underflow ; AudioFileSourceBuffer keeps
      // topping the buffer up opportunistically afterwards). Small decoder
      // requests still wait for the full length (alignment matters there).
      if (got >= 16384)
        break;

      // Half-dead stream watchdog (not while paused on purpose)
      if (vOwner && !vOwner->IsPaused() &&
          (long)(millis() - viDeadline) >= 0)
      {
        Serial.println("Stream watchdog : no data for 30s");
        if (got == 0)
          vOpen = false;
        break;
      }

      delay(2);
    }
    return got;
  }

  virtual uint32_t readNonBlock(void *data, uint32_t len) override {
    if (!vOpen || vClient.available() == 0) return 0;
    int r = vClient.read((uint8_t *)data, len);
    return (r > 0) ? (uint32_t)r : 0;
  }

  virtual bool close() override {
    vClient.stop();
    vOpen = false;
    return true;
  }

  virtual bool isOpen() override {
    return vOpen && (vClient.connected() || vClient.available());
  }

  virtual uint32_t getSize() override {
    return 0;
  }

  virtual bool loop() override {
    return vOpen;
  }

private:
  Client & vClient;
  slimproto * vOwner;
  bool vOpen;
};

/**
 * Read bytes from the stream socket until the end of the HTTP response
 * headers (an empty line : "\r\n\r\n") has been seen.
 */
static bool skipResponseHeaders(Client & client, uint32_t timeoutMs)
{
  unsigned long viStart = millis();
  int viCrlf = 0;

  while (viCrlf < 4)
  {
    if (client.available() > 0)
    {
      char viCh = client.read();
      if (viCh == '\r' || viCh == '\n') viCrlf++;
      else viCrlf = 0;
      continue;
    }

    if (!client.connected())
      return false;

    if (millis() - viStart > timeoutMs)
      return false;

    delay(2);
  }
  return true;
}

/**
 * Translate the SlimProto sample rate code to Hz
 * ('0'=11k '1'=22k '2'=32k '3'=44.1k '4'=48k '5'=8k '6'=12k '7'=16k
 *  '8'=24k '9'=96k)
 */
static uint32_t pcmRateFromCode(byte pCode)
{
  switch (pCode)
  {
    case '0': return 11025;
    case '1': return 22050;
    case '2': return 32000;
    case '3': return 44100;
    case '4': return 48000;
    case '5': return 8000;
    case '6': return 12000;
    case '7': return 16000;
    case '8': return 24000;
    case '9': return 96000;
    default:  return 44100;
  }
}

/**
 * Minimal generator that plays a raw little/big-endian PCM stream (SlimProto
 * format 'p', as sent e.g. by the Spoton/Spotify plugin). Sample rate,
 * number of channels and bit depth come from the "strm s" command fields.
 */
class AudioGeneratorPCM : public AudioGenerator {
public:
  AudioGeneratorPCM(uint32_t pRate, uint8_t pChannels, uint8_t pBits,
                    bool pBigEndian)
    : vRate(pRate), vChannels(pChannels), vBits(pBits), vBigEndian(pBigEndian)
  {
  }

  virtual bool begin(AudioFileSource *source, AudioOutput *output) override {
    file = source;
    this->output = output;
    if (!file || !output) return false;
    if (!file->isOpen()) return false;

    output->begin();
    output->SetRate(vRate);
    output->SetChannels(vChannels);

    vBufPtr = 0;
    vBufLen = 0;
    lastSample[0] = 0;
    lastSample[1] = 0;
    running = true;
    return true;
  }

  virtual bool loop() override {
    if (!running) goto done;

    if (!output->ConsumeSample(lastSample))
      goto done;    // Output busy, try again later

    do {
      if (!GetSample(lastSample[AudioOutput::LEFTCHANNEL])) {
        stop();
        break;
      }
      if (vChannels == 2) {
        if (!GetSample(lastSample[AudioOutput::RIGHTCHANNEL])) {
          stop();
          break;
        }
      } else {
        lastSample[AudioOutput::RIGHTCHANNEL] = lastSample[AudioOutput::LEFTCHANNEL];
      }
    } while (running && output->ConsumeSample(lastSample));

done:
    file->loop();
    output->loop();
    return running;
  }

  virtual bool stop() override {
    running = false;
    return true;
  }

  virtual bool isRunning() override {
    return running;
  }

private:
  bool GetBytes(uint32_t pLen, uint8_t *pDst) {
    while (pLen) {
      if (vBufPtr >= vBufLen) {
        vBufPtr = 0;
        vBufLen = file->read(vBuf, sizeof(vBuf));
        if (!vBufLen)
          return false;    // End of stream
      }
      *pDst++ = vBuf[vBufPtr++];
      pLen--;
    }
    return true;
  }

  bool GetSample(int16_t & pSample) {
    if (vBits == 8) {
      uint8_t u8;
      if (!GetBytes(1, &u8)) return false;
      pSample = ((int16_t) u8 - 128) << 8;
    } else {
      uint8_t b[2];
      if (!GetBytes(2, b)) return false;
      int16_t s = (int16_t) (b[0] | (b[1] << 8));   // little endian
      if (vBigEndian)
        s = (int16_t) ((s << 8) | ((s >> 8) & 0x00ff));
      pSample = s;
    }
    return true;
  }

  uint32_t vRate;
  uint8_t  vChannels;
  uint8_t  vBits;
  bool     vBigEndian;

  uint8_t  vBuf[1024];
  uint32_t vBufPtr;
  uint32_t vBufLen;
};


/**
 * I2S output with the DMA state exposed (feed-idle-silence guard).
 * Never call flush() on it : that plays out the queued samples.
 */
class DacOutput : public AudioOutputI2S {
public:
  bool isActive() { return i2sOn; }
};


responseBase::responseBase(Client * pClient)
{
  vcClient = pClient;
}

responseBase::~responseBase()
{
}


reponseHelo::reponseHelo(Client * pClient, const uint8_t pMac[6]) : responseBase(pClient)
{
  memset((void *) &vcResponse, 0, sizeof(vcResponse));

  memcpy((void *) vcResponse.command, "HELO", 4);
  vcResponse.diviceID = 0x08;
  vcResponse.firmwareRevision = 0x0b;

  memcpy(vcResponse.macAddr, pMac, 6);

  // Build a unique UUID out of the MAC address
  memcpy(vcResponse.uuid, pMac, 6);
  vcResponse.uuid[15] = 0x01;

  vcResponse.language[0] = 'E';
  vcResponse.language[1] = 'N';

  const char capabilites[] = "Model=SqueezeEsp,ModelName=SqueezeEsp,Firmware=7,flc,pcm,mp3,SampleRate=44100,HasPreAmp";
  memcpy(vcResponse.capabilites, capabilites, strlen(capabilites));
}

void reponseHelo::sendResponse()
{
  vcResponse.sizeResponse = __builtin_bswap32(sizeof(vcResponse) - 8); // N'inclus pas la commande ni la taille.
  vcClient->write((const uint8_t *) &vcResponse, sizeof(vcResponse));
}


reponseSTAT::reponseSTAT(Client * pClient) : responseBase(pClient)
{
  memset((void *) &vcResponse, '\0', sizeof(vcResponse));
}

void reponseSTAT::sendResponse()
{
  memcpy((void *) vcResponse.opcode, "STAT", 4);
  vcResponse.sizeResponse = __builtin_bswap32(sizeof(vcResponse) - 8);
  vcClient->write((const uint8_t *) &vcResponse, sizeof(vcResponse));
}


/**
 * Constructor
 **/
slimproto::slimproto(String pAdrLMS, Client * pClient)
{
  vcDacAudioGen = 0;
  vcDacSrc = 0;
  vcDacBuff = 0;
  vcDacOut = 0;
  vcStreamClient = 0;

  vcAdrLMS = pAdrLMS;
  vcCommandSize = 0;

  vcPlayerStat = StopStatus;    /* 0 = stop , 1 = play , 2 = pause */

  vcClient = pClient;

  LastStatMsg = millis();
  StartTimeCurrentSong = 0;
  EndTimeCurrentSong = 0;
  ByteReceivedCurrentSong = 0;

  Serial.printf("slimproto connected to LMS @ %s - free heap %u bytes\n",
                vcAdrLMS.c_str(), ESP.getFreeHeap());
}

slimproto::~slimproto()
{
  StopDacPlayback();
}

/**
 * Handle one incoming control message ; returns false when the LMS control
 * link must be considered down.
 */
int slimproto::HandleMessages()
{
  if(vcClient->connected())
  {
    if(vcCommandSize == 0 && vcClient->available() >= 2)
    {
      uint8_t viExtractSize[2];
      vcClient->read(viExtractSize, 2);

      // Convert big endian length into integer
      vcCommandSize = (viExtractSize[0] << 8) | viExtractSize[1];
    }

    if(vcCommandSize > 250)
    {
      // Out of spec, flush the garbage
      Serial.println("Oversized command received, flushing");
      while(vcClient->available() && vcCommandSize--)
        vcClient->read();
      vcCommandSize = 0;
    }

    if(vcCommandSize && vcClient->available() >= vcCommandSize)
    {
      uint8_t viExtractCommand[vcCommandSize];
      int viSizeRead = vcClient->read(viExtractCommand, vcCommandSize);

      HandleCommand(viExtractCommand, viSizeRead);
      vcCommandSize = 0;
    }
  }

  // Force a reconnect if the server went quiet (no stat requests) for 60s
  if(millis() - LastStatMsg >= (60 * 1000))
  {
    Serial.println("No Stat request from 60 seconds");
    return false;
  }

  return true;
}


int slimproto::HandleAudio()
{
  // Run a "strm q/s" command that was parked while the decoder sat blocked
  // inside a stream read (see StreamAudioSource::read / PumpControlDuringAudio)
  if(vcPendingStreamCmd)
    ExecutePendingStreamCmd();

  // While paused the I2S channel stays enabled, so keep pushing silence :
  // otherwise the circular DMA would re-emit its last content ("stuck CD").
  if(vcPlayerStat == PauseStatus)
  {
    FeedIdleSamples();
    return 0;
  }

  if(vcPlayerStat != PlayStatus)
    return 0;

  if(!vcDacAudioGen)
    return 0;

  if(!vcDacAudioGen->isRunning())
  {
    // A parked song change stops the current decoder (read returned 0) ; it
    // is executed at the next call (top of this function).
    return 0;
  }

  if(!vcDacAudioGen->loop())
  {
    if(vcPendingStreamCmd)
      return 0;   // interrupted by a parked strm q/s, not a real EOS

    // Decoder reached a genuine end of stream (server closed the socket).
    // Tell the server decoding finished ("STMd") : without it LMS/Spoton
    // keeps the player in playing state and never sends a new stream
    // (audio stays dead until the Connect session is re-created).
    // Safe to send here : EOS is detected within ms of the socket close and
    // a "strm" command that arrived first is parked (checked above), so a
    // late STMd can no longer hit the NEW stream.
    Serial.println("Audio stream ended");

    reponseSTAT viDone(vcClient);
    memcpy((void *) viDone.vcResponse.event, "STMd", 4);
    viDone.vcResponse.elapsed_seconds = (millis() - StartTimeCurrentSong) / 1000;
    viDone.vcResponse.bytes_received_L = ByteReceivedCurrentSong;
    viDone.sendResponse();

    vcPlayerStat = StopStatus;
    StopDacPlayback();
  }

  return 0;
}


/**
 * Pump the LMS control socket from inside a blocking stream read.
 * Pause/unpause/volume/status are executed inline (they only touch the I2S
 * output / volume, never the decoder chain) ; stream start/stop commands
 * are parked for HandleAudio() because running them here would delete the
 * decoder chain while the decoder is decoding.
 */
int slimproto::PumpControlDuringAudio()
{
  vcPumpingFromAudio = true;
  int viRes = HandleMessages();
  vcPumpingFromAudio = false;
  return viRes;
}


/**
 * Push silence into the I2S DMA while the stream is stalled, so the circular
 * DMA buffer gets overwritten with silence instead of re-emitting its last
 * content (the "stuck CD" sound heard when the server stops feeding data,
 * e.g. on a Spotify stop before the "strm q" arrives).
 */
void slimproto::FeedIdleSamples()
{
  if (vcDacOut && vcDacOut->isActive())
  {
    static int16_t silence[512];   // zeroed once : 256 stereo pairs
    vcDacOut->ConsumeSamples(silence, 256);
  }
}


void slimproto::DeferStreamCmd(byte pCommand [], int pSize)
{
  if(pSize < 5 || pSize > (int)sizeof(vcPendingCmd))
  {
    Serial.println("Invalid strm command size, dropped");
    return;
  }
  ByteArrayCpy(vcPendingCmd, pCommand, pSize);
  vcPendingSize = pSize;
  vcPendingStreamCmd = true;   // executed by HandleAudio()
}


void slimproto::ExecutePendingStreamCmd()
{
  if(!vcPendingStreamCmd)
    return;

  vcPendingStreamCmd = false;

  if(vcPendingSize >= 5 && vcPendingCmd[4] == 'q')
    HandleStrmQCmd(vcPendingCmd, vcPendingSize);
  else
    HandleStrmSCmd(vcPendingCmd, vcPendingSize);
}


/**
 * Stop command
 */
void slimproto::HandleStrmQCmd(byte pCommand [], int pSize)
{
  Serial.println("strm q received");
  StopDacPlayback();

  vcPlayerStat = StopStatus;
  EndTimeCurrentSong = millis();

  reponseSTAT * viResponse = new reponseSTAT(vcClient);
  memcpy((void *) viResponse->vcResponse.event, "STMf", 4);
  viResponse->vcResponse.elapsed_seconds = 0;
  viResponse->sendResponse();
  delete viResponse;
}

/**
 * Status command
 */
void slimproto::HandleStrmTCmd(byte pCommand [], int pSize)
{
  reponseSTAT viResponse(vcClient);
  memcpy((void *) viResponse.vcResponse.event, "STMt", 4);
  viResponse.vcResponse.bytes_received_L = ByteReceivedCurrentSong;

  // If current stat is 'stop' send 0 as elapsed time
  if(vcPlayerStat == StopStatus)
    viResponse.vcResponse.elapsed_seconds = 0;
  else
    viResponse.vcResponse.elapsed_seconds = (millis() - StartTimeCurrentSong) / 1000;

  viResponse.sendResponse();
  LastStatMsg = millis();
}

/**
 * Handle start command
 */
void slimproto::HandleStrmSCmd(byte pCommand [], int pSize)
{
  StrmStruct strmInfo;
  memcpy(&strmInfo, pCommand + 4, sizeof(strmInfo));

  Serial.print("strm s - format : ");
  Serial.println((char) strmInfo.formatbyte);

  // Pick the software decoder matching the stream format
  AudioGenerator * viNewGen = 0;
  switch (strmInfo.formatbyte)
  {
    case 'm':  // MP3
      viNewGen = new AudioGeneratorMP3();
      viNewGen->RegisterStatusCB(StatusCallback, (void*) "mp3");
      break;
    case 'f':  // FLAC
      viNewGen = new AudioGeneratorFLAC();
      viNewGen->RegisterStatusCB(StatusCallback, (void*) "flac");
      break;
    case 'w':  // WAV
      viNewGen = new AudioGeneratorWAV();
      viNewGen->RegisterStatusCB(StatusCallback, (void*) "wav");
      break;
    case 'p':  // raw PCM (Spoton/Spotify)
    {
      uint32_t viRate = pcmRateFromCode(strmInfo.pcmsamplerate);
      uint8_t  viCh   = (strmInfo.pcmchannels == '1') ? 1 : 2;
      uint8_t  viBits = (strmInfo.pcmsamplesize == '0') ? 8 : 16;
      bool     viBE   = (strmInfo.pcmendian == '0');
      Serial.printf("PCM %u Hz, %u ch, %u bit\n", viRate, viCh, viBits);
      viNewGen = new AudioGeneratorPCM(viRate, viCh, viBits, viBE);
      viNewGen->RegisterStatusCB(StatusCallback, (void*) "pcm");
      break;
    }
    default:
      Serial.print("Format not supported in DAC mode : ");
      Serial.println((char) strmInfo.formatbyte);
      reponseSTAT viStop(vcClient);
      memcpy((void *) viStop.vcResponse.event, "STMf", 4);
      viStop.sendResponse();
      return;
  }

  // Grab the stream request (starts right after "strm" + strmInfo).
  // Per the SlimProto protocol this is an HTTP request header used to obtain
  // the stream data, e.g. :
  //     GET /stream.mp3?player=00:04:20:aa:bb:cc HTTP/1.0\r\n\r\n
  const int viMaxReq = 220;
  int viReqLen = pSize - sizeof(strmInfo) - 4;
  if (viReqLen > viMaxReq) viReqLen = viMaxReq;
  if (viReqLen < 0) viReqLen = 0;
  byte viReq[viMaxReq + 8];
  memset(viReq, 0, sizeof(viReq));
  ByteArrayCpy(viReq, pCommand + sizeof(strmInfo) + 4, viReqLen);

  // The request must end with an empty line
  if (viReqLen < 4
      || !(viReq[viReqLen - 4] == '\r' && viReq[viReqLen - 3] == '\n'
        && viReq[viReqLen - 2] == '\r' && viReq[viReqLen - 1] == '\n'))
  {
    viReq[viReqLen++] = '\r';
    viReq[viReqLen++] = '\n';
    viReq[viReqLen++] = '\r';
    viReq[viReqLen++] = '\n';
  }

  // Cleanup previous playback (if LMS did not send a strmq first)
  StopDacPlayback();

  // Stream server address : explicit one from LMS, else the control server
  IPAddress viStreamIp;
  if (strmInfo.server_ip[0] != 0)
    viStreamIp = IPAddress(strmInfo.server_ip[0], strmInfo.server_ip[1],
                           strmInfo.server_ip[2], strmInfo.server_ip[3]);
  else
    viStreamIp.fromString(vcAdrLMS);

  uint16_t viStreamPort = ((uint16_t) strmInfo.server_port[0] << 8)
                        | strmInfo.server_port[1];
  if (viStreamPort == 0) viStreamPort = LMS_PORT;

  Serial.print("Opening stream to ");
  Serial.print(viStreamIp);
  Serial.print(":");
  Serial.println(viStreamPort);

  vcStreamClient = new WiFiClient();
  if (!vcStreamClient->connect(viStreamIp, viStreamPort))
  {
    Serial.println("Unable to connect to the stream socket");
    delete vcStreamClient, vcStreamClient = 0;
    delete viNewGen;
    reponseSTAT viStop(vcClient);
    memcpy((void *) viStop.vcResponse.event, "STMf", 4);
    viStop.sendResponse();
    return;
  }

  // Send the HTTP request header we received from the server
  vcStreamClient->write((const uint8_t *) viReq, viReqLen);

  // Skip the HTTP response headers that come back before the audio body
  if (!skipResponseHeaders(*vcStreamClient, 8000))
  {
    Serial.println("No stream response headers from server");
    StopDacPlayback();
    delete viNewGen;
    reponseSTAT viStop(vcClient);
    memcpy((void *) viStop.vcResponse.event, "STMf", 4);
    viStop.sendResponse();
    return;
  }

  // Wire the decode chain : stream socket -> jitter buffer -> gen -> I2S
  vcDacSrc = new StreamAudioSource(*vcStreamClient, this);

  vcDacBuff = new AudioFileSourceBuffer(vcDacSrc, AUDIO_BUFFER_SIZE);
  vcDacBuff->RegisterStatusCB(StatusCallback, (void*) "buffer");

  vcDacOut = new DacOutput();
  vcDacOut->SetBuffers(AUDIO_DMA_BUFFER_COUNT, AUDIO_DMA_BUFFER_BYTES);
  vcDacOut->SetPinout(I2S_BCK_PIN, I2S_LRCK_PIN, I2S_DOUT_PIN);
  vcDacOut->SetGain(vcVolumeGain);

  vcDacAudioGen = viNewGen;
  if (!vcDacAudioGen->begin(vcDacBuff, vcDacOut))
  {
    Serial.println("Decoder failed to start");
    StopDacPlayback();
    reponseSTAT viStop(vcClient);
    memcpy((void *) viStop.vcResponse.event, "STMf", 4);
    viStop.sendResponse();
    return;
  }

  vcPlayerStat = PlayStatus;
  StartTimeCurrentSong = millis();
  ByteReceivedCurrentSong = 0;

  Serial.printf("Playback started - free heap %u bytes\n", ESP.getFreeHeap());

  // Tell the server the stream is connected/started
  reponseSTAT viResponseSTMc(vcClient);
  memcpy((void *) viResponseSTMc.vcResponse.event, "STMc", 4);
  viResponseSTMc.sendResponse();

  reponseSTAT viResponseSTMe(vcClient);
  memcpy((void *) viResponseSTMe.vcResponse.event, "STMe", 4);
  viResponseSTMe.sendResponse();

  reponseSTAT viResponseSTMh(vcClient);
  memcpy((void *) viResponseSTMh.vcResponse.event, "STMh", 4);
  viResponseSTMh.sendResponse();

  reponseSTAT viResponseSTMs(vcClient);
  memcpy((void *) viResponseSTMs.vcResponse.event, "STMs", 4);
  viResponseSTMs.sendResponse();
}

/**
 * Handle pause command
 *
 * The decoder/generator is left running (so we can resume where we left off)
 * and the I2S output keeps running : HandleAudio() feeds silence while we
 * are paused. (Stopping the I2S channel here worked, but restarting it on
 * unpause made the PCM5102A pop/crackle, and refills start instantly this
 * way.)
 */
void slimproto::HandleStrmPCmd(byte pCommand [], int pSize)
{
  Serial.printf("Pause (jitter %u B, tcp pending %d B)\r\n",
                vcDacBuff ? vcDacBuff->getFillLevel() : 0,
                vcStreamClient ? vcStreamClient->available() : 0);

  vcPlayerStat = PauseStatus;

  // Send Pause confirm
  reponseSTAT viResponseSTMp(vcClient);
  memcpy((void *) viResponseSTMp.vcResponse.event, "STMp", 4);
  viResponseSTMp.sendResponse();
}

/**
 * Handle unpause command
 */
void slimproto::HandleStrmUCmd(byte pCommand [], int pSize)
{
  Serial.println("Unpause");

  // The I2S channel is normally kept running during pause (silence fed by
  // HandleAudio), so it usually needs no restart. Restart defensively only
  // if it is not active (e.g. a bare unpause).
  if (vcDacAudioGen && vcDacOut && !vcDacOut->isActive())
  {
    if (!vcDacOut->begin())
      Serial.println("I2S restart failed on unpause");
  }

  vcPlayerStat = PlayStatus;

  // Send UnPause confirm
  reponseSTAT viResponseSTMp(vcClient);
  memcpy((void *) viResponseSTMp.vcResponse.event, "STMr", 4);
  viResponseSTMp.sendResponse();
}

/**
 * Handle volume control (software gain applied in the I2S output)
 */
void slimproto::HandleAudgCmd(byte pCommand [], int pSize)
{
  u32_t viVol = unpackN((u32_t *) (pCommand + 14));   // audg gainL
  ApplyVolume(viVol);
}

/**
 * Map a slimproto audg gain (16.16 fixed point, 65536 = 100%) to a software
 * gain on the samples (the PCM5102A has no volume register). LMS already
 * applies its volume curve server side, so the value is used as-is.
 */
void slimproto::ApplyVolume(u32_t pVolume)
{
  float vGain = (float) pVolume / 65536.0f;
  if(vGain > 1.0f) vGain = 1.0f;
  if(vGain < 0.0f) vGain = 0.0f;

  vcVolumeGain = vGain;

  if(vcDacOut)
    vcDacOut->SetGain(vcVolumeGain);
}


/**
 * Tear down the whole decoder -> I2S chain
 */
void slimproto::StopDacPlayback()
{
  // NOTE : do not call vcDacOut->flush() here ! flush() plays out the whole
  // queued DMA (that is the "stuck CD" replay). stop() below disables the
  // channel and resets the DMA, discarding the pending samples.
  if (vcDacAudioGen)
  {
    vcDacAudioGen->stop();
    delete vcDacAudioGen, vcDacAudioGen = 0;
  }
  if (vcDacOut)
  {
    vcDacOut->stop();
    delete vcDacOut, vcDacOut = 0;
  }
  if (vcDacBuff)
  {
    vcDacBuff->close();
    delete vcDacBuff, vcDacBuff = 0;
  }
  if (vcDacSrc)
  {
    vcDacSrc->close();
    delete vcDacSrc, vcDacSrc = 0;
  }
  if (vcStreamClient)
  {
    vcStreamClient->stop();
    delete vcStreamClient, vcStreamClient = 0;
  }
  vcPlayerStat = StopStatus;
}


void slimproto::HandleCommand(byte pCommand[], int pSize)
{
  byte viCommand[5] = {0};

  ByteArrayCpy(viCommand, pCommand, 4);

  if(strcmp((const char *) viCommand, "strm") == 0)
  {
    unsigned char viSubCmd = (unsigned char) pCommand[4];

    switch (viSubCmd)
    {
      case 'q':
      case 's':
        // Tear-down / start : never run inline while the decoder sits in a
        // blocking stream read (would free the chain from under it). Parked
        // by HandleAudio() otherwise.
        if (vcPumpingFromAudio)
        {
          DeferStreamCmd(pCommand, pSize);
          break;
        }
        if (viSubCmd == 'q')
          HandleStrmQCmd(pCommand, pSize);
        else
          HandleStrmSCmd(pCommand, pSize);
        break;
      case 't':
        HandleStrmTCmd(pCommand, pSize);
        break;
      case 'p':
        HandleStrmPCmd(pCommand, pSize);
        break;
      case 'u':
        HandleStrmUCmd(pCommand, pSize);
        break;
      default:
        break;
    }
  }
  else if (strcmp((const char *) viCommand, "audg") == 0)
  {
    HandleAudgCmd(pCommand, pSize);
  }
}

void slimproto::ByteArrayCpy(byte * pDst, byte * pSrv, int pSize)
{
  for(int i = 0; i < pSize; i++)
    pDst[i] = pSrv[i];
}


u32_t slimproto::unpackN(u32_t *src)
{
  u8_t *ptr = (u8_t *) src;
  return *(ptr) << 24 | *(ptr + 1) << 16 | *(ptr + 2) << 8 | *(ptr + 3);
}
