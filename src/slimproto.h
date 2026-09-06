#ifndef slimproto_h
#define slimproto_h

#include "config.h"
#include <Arduino.h>
#include <Client.h>

#include <AudioFileSourceICYStream.h>
#include <AudioFileSourceBuffer.h>
#include <AudioGeneratorMP3.h>
#include <AudioGeneratorFLAC.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>

struct __attribute__((packed)) StrmStructDef
{
   byte command;
   byte autostart;
   byte formatbyte;
   byte pcmsamplesize;
   byte pcmsamplerate;
   byte pcmchannels;
   byte pcmendian;
   byte threshold;
   byte spdif_enable;
   byte trans_period;
   byte trans_type;
   byte flags;
   byte output_threshold;
   byte RESERVED;
   u32_t replay_gain;
   byte server_port[2];
   byte server_ip[4];
};

struct __attribute__((packed)) audg_packet {
  char  opcode[4];
  u32_t old_gainL;     // unused
  u32_t old_gainR;     // unused
  u8_t  adjust;
  u8_t  preamp;        // unused
  u32_t gainL;
  u32_t gainR;
};

class responseBase
{
public :
  responseBase(Client * pClient);
  ~responseBase();
  virtual void sendResponse() = 0;

protected :
  Client * vcClient;
};


class reponseHelo : public responseBase {

public :
  reponseHelo(Client * pClient, const uint8_t pMac[6]);
  void sendResponse();

private :
  struct __attribute__((packed)) stResponse
  {
    byte command[4];       // HELO
    u32_t sizeResponse;
    char  diviceID;
    char  firmwareRevision;
    byte  macAddr[6];
    byte  uuid[16];
    byte  wlanChannel[2];
    byte  receivedData[8];
    byte  language[2];
    char  capabilites[92];
  };

  stResponse vcResponse;
};


class reponseSTAT : public responseBase {

public :
  reponseSTAT(Client * pClient);
  void sendResponse();

  struct __attribute__((packed)) STAT_packet {
    char  opcode[4];       // STAT
    u32_t sizeResponse;
    char  event[4];
    u8_t  num_crlf;
    u8_t  mas_initialized;
    u8_t  mas_mode;
    u32_t stream_buffer_size;
    u32_t stream_buffer_fullness;
    u32_t bytes_received_H;
    u32_t bytes_received_L;
    u16_t signal_strength;
    u32_t jiffies;
    u32_t output_buffer_size;
    u32_t output_buffer_fullness;
    u32_t elapsed_seconds;
    u16_t voltage;
    u32_t elapsed_milliseconds;
    u32_t server_timestamp;
    u16_t error_code;
  };

  STAT_packet vcResponse;
};

typedef struct StrmStructDef StrmStruct;
typedef struct audg_packet AudgStruct;


class slimproto
{
public:
  slimproto(String pAdrLMS, Client * pClient);
  ~slimproto();

  /**
   * Read message from socket and handle commands
   **/
  int HandleMessages();

  /**
   * Drive the audio decoder / I2S output
   **/
  int HandleAudio();

private:
  void HandleCommand(byte pCommand [], int pSize);
  void HandleStrmQCmd(byte pCommand [], int pSize);
  void HandleStrmTCmd(byte pCommand [], int pSize);
  void HandleStrmSCmd(byte pCommand [], int pSize);
  void HandleStrmPCmd(byte pCommand [], int pSize);
  void HandleStrmUCmd(byte pCommand [], int pSize);
  void HandleAudgCmd(byte pCommand [], int pSize);

  void ByteArrayCpy(byte * pDst, byte * pSrv, int pSize);
  void PrintByteArray(byte * psrc, int pSize);

  u32_t unpackN(u32_t *src);

  void StopDacPlayback();
  void ApplyVolume(u32_t pVolume);

  String vcAdrLMS;

  int vcCommandSize;

  unsigned long StartTimeCurrentSong = 0;
  unsigned long EndTimeCurrentSong = 0;
  uint32_t      ByteReceivedCurrentSong = 0;

  unsigned long LastStatMsg = 0;

  Client * vcClient;            // Client to handle control messages

  // Decoder chain : HTTP stream -> jitter buffer -> generator -> I2S out
  AudioGenerator *           vcDacAudioGen = 0;
  AudioFileSourceICYStream * vcDacFile = 0;
  AudioFileSourceBuffer *    vcDacBuff = 0;
  AudioOutputI2S *           vcDacOut = 0;

  // Current software volume gain (0.0 .. 1.0), PCM5102A has no register
  float vcVolumeGain = 1.0f;

  enum player_status {
    StopStatus,
    PlayStatus,
    PauseStatus
  };

  player_status vcPlayerStat = StopStatus;    /* 0 = stop , 1 = play , 2 = pause */
};

#endif
