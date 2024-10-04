#include <arduino.h>
#include "sx126x.h"
#include "TrovaLaSondaFw.h"
#include "m20.h"

static void processPacket(uint8_t buf[]);

Sonde m20={
  .name="M20",
    //TODO:
  .bitRate=9600,
  .frequencyDeviation= 3600,//?
  .bandWidth=SX126X_GFSK_BW_14600,
  .packetLength=M20_PACKET_LENGTH,
  .preambleLength=SX126X_GFSK_PREAMBLE_DETECTOR_OFF,
  .syncWordLen= 48,
  .flipBytes=false,
  .syncWord={ 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 },
  .processPacket=processPacket
};

 void processPacket(uint8_t buf[]) {
 }