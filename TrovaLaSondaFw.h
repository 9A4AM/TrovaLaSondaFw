#ifndef __RS41_HELTECV3_H__
#define __RS41_HELTECV3_H__

#define PACKET_LENGTH 312
#define SERIAL_LENGTH 12
#define SYNCWORD_SIZE  8

typedef struct Sonde_s {
  const char *name;
  int bitRate;
  int frequencyDeviation;
  sx126x_gfsk_bw_t bandWidth;
  int packetLength;
  sx126x_gfsk_preamble_detector_t preambleLength;
  int syncWordLen;
  bool flipBytes;
  uint8_t syncWord[SYNCWORD_SIZE];
  
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
extern float lat, lng, alt;

void dump(uint8_t buf[], int size);
void savePrefs();
void bip(int duration, int freq);
#endif