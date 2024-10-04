#include <arduino.h>
#include "sx126x.h"
#include "TrovaLaSondaFw.h"
#include "dfm.h"

static void processPacket(uint8_t buf[]);

Sonde dfm={
  .name="DFM",
  //TODO:
  .bitRate=4800,
  .frequencyDeviation= 3600,//?
  .bandWidth=SX126X_GFSK_BW_9700,
  .packetLength=0,///////////////////////////////
  .preambleLength=SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS,
  .syncWordLen=64,
  .flipBytes=true,
  .syncWord={ 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 },
  .processPacket=processPacket
};

 void processPacket(uint8_t buf[]) {
 }