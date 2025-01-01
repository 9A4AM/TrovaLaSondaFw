//Portions from https://github.com/dbdexter-dev/sondedump/tree/master/sonde/dfm09
//under MIT license
#include <arduino.h>
#include "TrovaLaSondaFw.h"
#include "radio.h"
#include "m10.h"
#include "m20.h"

static bool processPacket(uint8_t buf[]);

Sonde m20 = {
  .name = "M20",
  .bitRate = 9600,
  .afcBandWidth = 50000,
  .frequencyDeviation = 12500,  //?
  .bandwidthHz = 14600,
  .packetLength = M20_PACKET_LENGTH,
  .partialPacketLength = 0,
  .preambleLengthBytes = 0,
  .syncWordLen = 48,
  .flipBytes = false,
  .syncWord = { 0x66, 0x66, 0x66, 0x66, 0xB3, 0x66 },
  .processPartialPacket = NULL,
  .processPacket = processPacket
};

static void descramble(uint8_t* data, uint8_t* out) {
  uint8_t topbit = 0x80, tmp;
  for (int i = 0; i < M20_PACKET_LENGTH / 2; i++) {
    tmp = data[i] << 7;
    out[i] = data[i] ^ 0xFF ^ (topbit | data[i] >> 1);
    topbit = tmp;
  }
}

static bool checkCrc(uint8_t* data) {
  uint16_t crcCalc = 0, crc = data[0x44] << 8 | data[0x45];

  for (int i = 0; i < 0x44; i++)
    crcCalc = m10CrcStep(crcCalc, data[i]);
  return crc == crcCalc;
}

static void decodeFrame(uint8_t* a) {
  uint8_t b0 = a[0x12];
  uint16_t s2 = (a[0x14] << 8) | a[0x13];
  uint8_t ym = 0x7F & b0;
  uint8_t y = int(ym / 12);
  uint8_t m = ym % 12 + 1;

  frame = a[0x15];
  //serial = f'{y}{m:02}-{ (s2&0x3)+2}-{(s2>>(2+13))&0x1}{ (s2>>2)&0x1FFF:04}'
  snprintf(serial, sizeof serial, "%d%02d-%d-%d%04d", y, m, s2 & 0x3 + 2, (s2 >> (2 + 13)) & 0x1, (s2 >> 2) & 0x1FFF);
  serial[sizeof serial - 1] = '\0';
  lat = (a[0x1C + 0] << 24 | a[0x1C + 1] << 16 | a[0x1C + 2] << 8 | a[0x1C + 3]) / 1e6;
  lng = (a[0x20 + 0] << 24 | a[0x20 + 1] << 16 | a[0x20 + 2] << 8 | a[0x20 + 3]) / 1e6;
  alt = (a[8] << 16 | a[9] << 8 | a[10]) / 1e2;
  Serial.printf("Serial:%s\n", serial);
}

static bool processPacket(uint8_t buf[]) {
  uint8_t frame[M20_PACKET_LENGTH / 2], out[M20_PACKET_LENGTH / 2];
  if (manchesterDecode(buf, frame, M20_PACKET_LENGTH)) {
    for (int i = 0; i < M20_PACKET_LENGTH / 2; i++) frame[i] = ~frame[i];
    //Serial.println("manchester");
    //dump(frame,M20_PACKET_LENGTH/2);
    descramble(frame, out);
    //Serial.println("descramble");
    //dump(out, M20_PACKET_LENGTH/2);
    if (checkCrc(out)) {
      decodeFrame(out);
      return true;
    }
  }
  return false;
}