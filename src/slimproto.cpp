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
 * the jitter buffer pre-filled, required for real-time raw PCM streams). A
 * gap in the data is NOT end of stream : endless streams can legitimately
 * pause feeding while the socket stays open. Only a closed socket terminates
 * the stream.
 *
 * While waiting for bytes the LMS control socket is pumped (pause/unpause/
 * volume handled inline, start/stop parked as a pending command) so the
 * player stays responsive even though the decoder sits inside this read,
 * without tearing the decoder chain down from under it.
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
        // circular DMA re-emits its last contents.
        vOwner->FeedIdleSamples();
      }

      // Refill head start : once the jitter buffer has this much data again,
      // return instead of waiting for the full AUDIO_BUFFER_SIZE so the
      // decoder can resume quickly after an underflow. The buffer keeps
      // topping itself up opportunistically afterwards (AudioFileSourceBuffer
      // calls readNonBlock on every read). Small decoder requests still wait
      // for their full length (alignment matters there).
      if (got >= AUDIO_BUFFER_REFILL_GOAL)
        break;

      // Half-dead stream watchdog (not while paused on purpose). Do not
      // accumulate paused time either : refresh the deadline so that a long
      // pause followed by an unpause starts a fresh 30s countdown.
      if (vOwner && vOwner->IsPaused())
        viDeadline = millis() + 30000UL;

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
    // Live stream : length unknown. Never return 0 here - the FLAC decoder
    // asks its eof callback (getPos() >= getSize()) BEFORE every read and
    // would see 0 >= 0 and bail out as end-of-stream without reading a byte.
    return 0xFFFFFFFFUL;
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
 * format 'p', as transcoded by the server for some sources). Sample rate,
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
 *
 * NOTE : never call flush() on this output. flush() plays out the whole DMA
 * queue before returning ; on a paused/stalled stream that replays the last
 * buffered audio. stop() disables the channel and discards the DMA instead.
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
  vcResponse.sizeResponse = __builtin_bswap32(sizeof(vcResponse) - 8); // excludes the command opcode and size field
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

    // Out of spec : nothing we send or receive is larger than the parked
    // "strm" command buffer (600 bytes), so anything bigger is framing garbage.
    // Flush it to resynchronise on the next length prefix.
    if(vcCommandSize > (int) sizeof(vcPendingCmd))
    {
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
  // otherwise the circular DMA would re-emit its last content.
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

    // Genuine end of stream (server closed the socket). Announce it to the
    // server ("STMd") : without it LMS keeps the player in its playing state
    // and never sends a new stream, so the next track would never start.
    // Safe to send here : EOS is detected within ms of the socket close and a
    // "strm" command that arrived first is parked (checked above), so a late
    // STMd can never hit the NEW stream.
    SendStatEvent("STMd", (millis() - StartTimeCurrentSong) / 1000,
                  ByteReceivedCurrentSong);

    StopDacPlayback();   // resets vcPlayerStat to StopStatus
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
 * content (e.g. while the server has stopped feeding data but the stop
 * command has not arrived yet).
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
  memcpy(vcPendingCmd, pCommand, pSize);
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
 * Send a SlimProto STAT event message.
 */
void slimproto::SendStatEvent(const char pEvent[4], uint32_t pElapsedSeconds,
                              uint32_t pBytesReceived)
{
  reponseSTAT viResponse(vcClient);
  memcpy((void *) viResponse.vcResponse.event, pEvent, 4);
  viResponse.vcResponse.elapsed_seconds = pElapsedSeconds;
  viResponse.vcResponse.bytes_received_L = pBytesReceived;
  viResponse.sendResponse();
}

/**
 * Stop command
 */
void slimproto::HandleStrmQCmd(byte pCommand [], int pSize)
{
  Serial.println("strm q received");
  StopDacPlayback();   // resets vcPlayerStat to StopStatus

  SendStatEvent("STMf", 0, 0);
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
  if (pSize < (int)(sizeof(StrmStruct) + 4))
  {
    Serial.println("strm s too short, dropped");
    return;
  }

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
    case 'p':  // raw PCM
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
      StopDacPlayback();
      delete viNewGen;
      SendStatEvent("STMf", 0, 0);
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
  memcpy(viReq, pCommand + sizeof(strmInfo) + 4, viReqLen);

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
    SendStatEvent("STMf", 0, 0);
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
    SendStatEvent("STMf", 0, 0);
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
    SendStatEvent("STMf", 0, 0);
    return;
  }

  vcPlayerStat = PlayStatus;
  StartTimeCurrentSong = millis();
  ByteReceivedCurrentSong = 0;

  Serial.printf("Playback started - free heap %u bytes\n", ESP.getFreeHeap());

  // Tell the server the stream is connected/started
  SendStatEvent("STMc", 0, 0);
  SendStatEvent("STMe", 0, 0);
  SendStatEvent("STMh", 0, 0);
  SendStatEvent("STMs", 0, 0);
}

/**
 * Handle pause command
 *
 * The decoder/generator is left running (so we can resume exactly where we
 * left off) and the I2S output keeps running : HandleAudio() feeds silence
 * while paused. Stopping the I2S channel on pause is not done : re-enabling
 * it on unpause made the PCM5102A pop/crackle, and keeping it running makes
 * refills resume instantly.
 */
void slimproto::HandleStrmPCmd(byte pCommand [], int pSize)
{
  Serial.printf("Pause (jitter %u B, tcp pending %d B)\r\n",
                vcDacBuff ? vcDacBuff->getFillLevel() : 0,
                vcStreamClient ? vcStreamClient->available() : 0);

  vcPlayerStat = PauseStatus;

  // Send Pause confirm
  SendStatEvent("STMp", 0, 0);
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
  SendStatEvent("STMr", 0, 0);
}

/**
 * Handle volume control (software gain applied in the I2S output).
 *
 * The "adjust" flag tells us who applies the LMS volume (same behaviour as
 * SqueezeLite) :
 *  - adjust != 0 : LMS sends a full-scale stream and the player must scale it
 *    to the sent gain. This is the case for native MP3/FLAC streams, which
 *    LMS cannot attenuate without decoding.
 *  - adjust == 0 : LMS has already scaled the stream data to its volume
 *    (raw PCM transcodes, where the server can apply the gain during
 *    conversion) and expects the player to stay at unity gain. Applying the
 *    sent gain again would attenuate the stream twice and make it very quiet
 *    compared to MP3 at the same volume.
 */
void slimproto::HandleAudgCmd(byte pCommand [], int pSize)
{
  // gainL / adjust are at fixed offsets (see the audg_packet layout)
  if (pSize < (int) (offsetof(AudgStruct, gainL) + sizeof(u32_t)))
    return;

  const AudgStruct * pAudg = (const AudgStruct *) pCommand;

  if (pAudg->adjust)
  {
    // gainL is 16.16 fixed point
    u32_t viVol = unpackN((u32_t *) (pCommand + offsetof(AudgStruct, gainL)));
    Serial.printf("audg volume : %u (adjust)\n", viVol);
    ApplyVolume(viVol);
  }
  else
  {
    Serial.println("audg volume : 100%% (stream pre-scaled by LMS)");
    ApplyVolume(65536);   // 100 % : the stream data already carries the volume
  }
}

/**
 * Map a slimproto audg gain (16.16 fixed point, 65536 = 100%) to a software
 * gain on the samples (the PCM5102A has no volume register).
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
  // NOTE : do not call vcDacOut->flush() here. flush() plays out the whole
  // queued DMA first, so a stopped/paused stream would replay its last audio.
  // stop() below disables the channel and discards the pending DMA samples.
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
  if (pSize < 4)
    return;   // too short to even hold the 4-byte command opcode

  byte viCommand[5] = {0};
  memcpy(viCommand, pCommand, 4);

  if(strcmp((const char *) viCommand, "strm") == 0)
  {
    if (pSize < 5)
      return;   // "strm" sub-command byte required

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

u32_t slimproto::unpackN(u32_t *src)
{
  u8_t *ptr = (u8_t *) src;
  return *(ptr) << 24 | *(ptr + 1) << 16 | *(ptr + 2) << 8 | *(ptr + 3);
}
