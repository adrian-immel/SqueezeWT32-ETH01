#include "slimproto.h"

/*
 * NOTE : This is a trimmed down, Ethernet + PCM5102A (I2S) version of the
 * original SqueezeESP32 slimproto code. The VS1053 hardware decoder path and
 * the whole ESP8266 branch have been removed. Audio is decoded in software by
 * the ESP8266Audio library (pinned to v2.2.0 for the classic ESP32) and
 * pushed to the I2S DAC.
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

  const char capabilites[] = "Model=SqueezeEsp,ModelName=SqueezeEsp,Firmware=7,flc,mp3,SampleRate=44100,HasPreAmp";
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
  vcDacFile = 0;
  vcDacBuff = 0;
  vcDacOut = 0;

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

      if(vcCommandSize != 172)
        Serial.print("Expected command size : "), Serial.println(vcCommandSize);
    }

    if(vcCommandSize > 250)
    {
      uint8_t availableSize = vcClient->available();
      Serial.println("Expected command size to big ??!!??");
      Serial.print("Available size : "), Serial.println(availableSize);

      uint8_t viExtractCommand[availableSize];
      int viSizeRead = vcClient->read(viExtractCommand, availableSize);
      PrintByteArray(viExtractCommand, viSizeRead);
      vcCommandSize = 0;
    }

    if(vcCommandSize && vcClient->available() >= vcCommandSize)
    {
      uint8_t viExtractCommand[vcCommandSize];
      int viSizeRead = vcClient->read(viExtractCommand, vcCommandSize);

      if(viSizeRead != vcCommandSize)
        Serial.println("Not enought data as expected !!!");

      if(vcCommandSize != 172)
        PrintByteArray(viExtractCommand, viSizeRead);

      HandleCommand(viExtractCommand, viSizeRead);
      vcCommandSize = 0;
    }
  }

  // Send Stat Message if last one is more than 60 seconds
  if(millis() - LastStatMsg >= (60 * 1000))
  {
    Serial.println("No Stat request from 60 seconds, Is there any probem ?");
    return false;
  }

  return true;
}


int slimproto::HandleAudio()
{
  if(vcPlayerStat != PlayStatus)
    return 0;

  if(!vcDacAudioGen)
    return 0;

  if(!vcDacAudioGen->isRunning())
    return 0;

  if(!vcDacAudioGen->loop())
  {
    // Decoder reached the end of the HTTP stream
    Serial.println("Audio stream ended");
    vcPlayerStat = StopStatus;
    StopDacPlayback();
  }

  return 0;
}


/**
 * Stop command
 */
void slimproto::HandleStrmQCmd(byte pCommand [], int pSize)
{
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
    default:
      Serial.print("Format not supported in DAC mode : ");
      Serial.println((char) strmInfo.formatbyte);
      reponseSTAT viStop(vcClient);
      memcpy((void *) viStop.vcResponse.event, "STMf", 4);
      viStop.sendResponse();
      return;
  }

  // Grab the stream path (starts right after "strm" + strmInfo)
  int viPathSize = pSize - sizeof(strmInfo) - 4;
  if(viPathSize > 190) viPathSize = 190;
  char viTmpUrl[192] = {0};
  ByteArrayCpy((byte *) viTmpUrl, pCommand + sizeof(strmInfo) + 4, viPathSize);

  String viPath = String((char *) viTmpUrl);
  if(viPath.length() == 0) viPath = "/stream.mp3";
  if(!viPath.startsWith("/")) viPath = "/" + viPath;

  // Build the http URL of the stream
  String viUrl;
  if(strmInfo.server_ip[0] == 0)
  {
    // No explicit stream server : use the LMS we are connected to
    viUrl = "http://" + vcAdrLMS + ":" + String(LMS_HTTP_PORT) + viPath;
  }
  else
  {
    String viSrvIp = String(strmInfo.server_ip[0]) + "." + strmInfo.server_ip[1]
                   + "." + strmInfo.server_ip[2] + "." + strmInfo.server_ip[3];
    viUrl = "http://" + viSrvIp + ":" + String(LMS_HTTP_PORT) + viPath;
  }
  Serial.print("Stream url : "), Serial.println(viUrl);

  // Cleanup previous playback (if LMS did not send a strmq first)
  StopDacPlayback();

  // Open the HTTP stream and wire the decode chain
  vcDacFile = new AudioFileSourceICYStream(viUrl.c_str());
  if(!vcDacFile->isOpen())
  {
    Serial.println("Unable to open the HTTP stream");
    delete vcDacFile, vcDacFile = 0;
    delete viNewGen;
    reponseSTAT viStop(vcClient);
    memcpy((void *) viStop.vcResponse.event, "STMf", 4);
    viStop.sendResponse();
    return;
  }

  vcDacBuff = new AudioFileSourceBuffer(vcDacFile, AUDIO_BUFFER_SIZE);
  vcDacBuff->RegisterStatusCB(StatusCallback, (void*) "buffer");

  vcDacOut = new AudioOutputI2S(0, AudioOutputI2S::EXTERNAL_I2S,
                                AUDIO_DMA_BUFFER_COUNT);
  vcDacOut->SetPinout(I2S_BCK_PIN, I2S_LRCK_PIN, I2S_DOUT_PIN);
  vcDacOut->SetGain(vcVolumeGain);

  vcDacAudioGen = viNewGen;
  if(!vcDacAudioGen->begin(vcDacBuff, vcDacOut))
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
 */
void slimproto::HandleStrmPCmd(byte pCommand [], int pSize)
{
  Serial.println("Pause");

  // Send Pause confirm
  reponseSTAT viResponseSTMp(vcClient);
  memcpy((void *) viResponseSTMp.vcResponse.event, "STMp", 4);
  viResponseSTMp.sendResponse();

  vcPlayerStat = PauseStatus;
}

/**
 * Handle unpause command
 */
void slimproto::HandleStrmUCmd(byte pCommand [], int pSize)
{
  Serial.println("Unpause");

  // Send UnPause confirm
  reponseSTAT viResponseSTMp(vcClient);
  memcpy((void *) viResponseSTMp.vcResponse.event, "STMr", 4);
  viResponseSTMp.sendResponse();

  vcPlayerStat = PlayStatus;
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
 * Map a slimproto volume value (0..~65536) to a software gain (0..1).
 * The PCM5102A has no volume register, so we use a gain on the samples,
 * with a squared taper to get closer to a perceived-loudness curve.
 */
void slimproto::ApplyVolume(u32_t pVolume)
{
  float vPct = (float) pVolume / 65536.0f;
  if(vPct > 1.0f) vPct = 1.0f;
  if(vPct < 0.0f) vPct = 0.0f;

  vcVolumeGain = vPct * vPct;

  Serial.print("Volume raw : "), Serial.print(pVolume);
  Serial.print(" / gain : "), Serial.println(vcVolumeGain);

  if(vcDacOut)
    vcDacOut->SetGain(vcVolumeGain);
}


/**
 * Tear down the whole decoder -> I2S chain
 */
void slimproto::StopDacPlayback()
{
  if(vcDacAudioGen)
  {
    vcDacAudioGen->stop();
    delete vcDacAudioGen, vcDacAudioGen = 0;
  }
  if(vcDacOut)
  {
    vcDacOut->stop();
    delete vcDacOut, vcDacOut = 0;
  }
  if(vcDacBuff)
  {
    vcDacBuff->close();
    delete vcDacBuff, vcDacBuff = 0;
  }
  if(vcDacFile)
  {
    vcDacFile->close();
    delete vcDacFile, vcDacFile = 0;
  }
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
        HandleStrmQCmd(pCommand, pSize);
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
      case 's':
        HandleStrmSCmd(pCommand, pSize);
        break;
      default:
        Serial.println("default SubCommand");
    }
  }
  else if (strcmp((const char *) viCommand, "audg") == 0)
  {
    HandleAudgCmd(pCommand, pSize);
  }
  else
  {
    Serial.print("Other command : [");
    Serial.print((char *) viCommand);
    Serial.print("] Size : ");
    Serial.println(pSize);
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


void slimproto::PrintByteArray(byte * psrc, int pSize)
{
  Serial.print("Array(");
  Serial.print(pSize);
  Serial.print(") : ");

  char tmp[16];
  for(int i = 0; i < pSize; i++)
  {
    sprintf(tmp, "%.2X", psrc[i]);
    Serial.print(tmp);
    Serial.print(" ");
  }
  Serial.println("");
}
