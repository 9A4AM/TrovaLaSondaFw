#ifndef __RS41_HELTECV3_H__
#define __RS41_HELTECV3_H__
#include "sx126x.h"
#include "sx126x_regs.h"
#include "sx126x_hal.h"
#include "sx126x_long_pkt.h"

#define PACKET_LENGTH RS41AUX_PACKET_LENGTH //longest packet length
#define SERIAL_LENGTH 12
#define SYNCWORD_SIZE  8

typedef struct Sonde_s {
  const char *name;
  int bitRate,
    frequencyDeviation;
  unsigned bandwidthHz;
  int packetLength, 
    partialPacketLength; //Minimum # of bytes to be read to determine actual packet length
  int preambleLengthBytes;
  int syncWordLen;
  bool flipBytes;
  uint8_t syncWord[SYNCWORD_SIZE];
  
  int (*processPartialPacket)(uint8_t buf[]); //To be called after partialPacketLength bytes 
                                              //have been read to determine the actual packet length
  bool (*processPacket)(uint8_t buf[]);
} Sonde;

extern const uint8_t flipByte[];
extern Sonde *sondes[];
extern uint32_t freq;
extern int frame, currentSonde;
extern const uint8_t flipByte[];//TODO:
extern struct sx126x_long_pkt_rx_state pktRxState;
extern int rssi, mute, batt;
extern bool encrypted, connected;
extern char serial[SERIAL_LENGTH + 1];
extern double lat, lng;
extern float alt, vel;
extern char version[];
extern uint8_t bkStatus;
extern uint16_t bkTime;
extern bool otaRunning;
extern int otaLength, otaErr, otaProgress;

sx126x_gfsk_preamble_detector_t getPreambleLength(unsigned lengthInBytes);
sx126x_gfsk_bw_t getBandwidth(unsigned bandwidth);
void dump(uint8_t buf[], int size);
void savePrefs();
void bip(int duration, int freq);
#endif