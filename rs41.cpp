#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <CRC.h>
#include <RS-FEC.h>
#include "sx126x.h"
#include "TrovaLaSondaFw.h"
#include "rs41.h"

static void processPacket(uint8_t buf[]);

Sonde rs41={
  .name="RS41",
  .bitRate=4800,
  .frequencyDeviation= 3600,//?
  .bandWidth=SX126X_GFSK_BW_9700,
  .packetLength=RS41_PACKET_LENGTH,
  .preambleLength=SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS,
  .syncWordLen=64,
  .flipBytes=true,
  .syncWord={ 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 },
  .processPacket=processPacket
};

RS::ReedSolomon<99 + (RS41_PACKET_LENGTH - 48) / 2, 24> rs;

// clang-format off
const uint8_t   whitening[] = {
  0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26, 0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1, 
  0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1, 0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C, 
  0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61, 0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23, 
  0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1, 0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98 
};
// clang-format on

static bool correctErrors(uint8_t data[]) {
  static uint8_t buf[256], dec[256];
  int i;

  //prima parte
  memset(buf, 0, 256);
  for (i = 0; i < (RS41_PACKET_LENGTH - 48) / 2; i++)
    buf[99 + i] = data[RS41_PACKET_LENGTH - 1 - 2 * i];
  for (i = 0; i < 24; i++)
    buf[254 - i] = data[24 + i];

  if (0 != rs.Decode(buf, dec)) return false;

  for (i = 0; i < (RS41_PACKET_LENGTH - 48) / 2; i++)
    data[311 - 2 * i] = dec[99 + i];

  //seconda parte
  memset(buf, 0, 256);
  for (i = 0; i < (RS41_PACKET_LENGTH - 48) / 2; i++)
    buf[99 + i] = data[RS41_PACKET_LENGTH - 1 - 2 * i - 1];
  for (i = 0; i < 24; i++)
    buf[254 - i] = data[i];

  if (0 != rs.Decode(buf, dec)) return false;

  for (i = 0; i < (RS41_PACKET_LENGTH - 48) / 2; i++)
    data[RS41_PACKET_LENGTH - 1 - 2 * i - 1] = dec[99 + i];

  return true;
}

//https://gis.stackexchange.com/questions/265909/converting-from-ecef-to-geodetic-coordinates
static void ecef2wgs84(float x, float y, float z, float& lat, float& lng, float& height) {
  // WGS84 constants
  float a = 6378137.0,
        f = 1.0 / 298.257223563;
  // derived constants
  float b = a - f * a,
        e = sqrt(pow(a, 2.0) - pow(b, 2.0)) / a,
        clambda = atan2(y, x),
        p = sqrt(pow(x, 2.0) + pow(y, 2)),
        h_old = 0.0;
  //first guess with h=0 meters
  float theta = atan2(z, p * (1.0 - pow(e, 2.0))),
        cs = cos(theta),
        sn = sin(theta),
        N = pow(a, 2.0) / sqrt(pow(a * cs, 2.0) + pow(b * sn, 2.0)),
        h = p / cs - N;
  int nMaxLoops = 100;
  while (abs(h - h_old) > 1.0e-6 && nMaxLoops-- > 0) {
    h_old = h;
    theta = atan2(z, p * (1.0 - pow(e, 2.0) * N / (N + h)));
    cs = cos(theta);
    sn = sin(theta);
    N = pow(a, 2.0) / sqrt(pow(a * cs, 2.0) + pow(b * sn, 2.0));
    h = p / cs - N;
  }
  lng = clambda / M_PI * 180;
  lat = theta / M_PI * 180;
  height = h;
}

static void processPacket(uint8_t buf[]) {
  float x, y, z;
  int n = 48 + 1;

  frame = 0;
  strcpy(serial, "????????");
  encrypted = false;

  for (int i = 0; i < RS41_PACKET_LENGTH; i++)
    buf[i] = whitening[i % sizeof whitening] ^ flipByte[buf[i]];

  if (correctErrors(buf) && buf[48] == 0x0F) {
    while (n < RS41_PACKET_LENGTH) {
      int blockType = buf[n];
      int blockLength = buf[n + 1];
      uint16_t crc = calcCRC16(buf + n + 2, blockLength, CRC16_CCITT_FALSE_POLYNOME, CRC16_CCITT_FALSE_INITIAL, CRC16_CCITT_FALSE_XOR_OUT, CRC16_CCITT_FALSE_REV_IN, CRC16_CCITT_FALSE_REV_OUT);

      //Serial.printf("Blocco 0x%02X, lunghezza %d, CRC: %02X%02X/%02X%02X\n", blockType, blockLength, buf[n + blockLength + 3], buf[n + blockLength + 2], crc >> 8, crc & 0xFF);
      if ((crc & 0xFF) == buf[n + blockLength + 2] && (crc >> 8) == buf[n + blockLength + 3]) {  //CRC OK
        switch (blockType) {
          case 0x79:  //status
            frame = buf[n + 2] + (buf[n + 3] << 8);
            Serial.printf(" frame: %d [%.8s]", frame, buf + n + 4);

            strncpy(serial, (char*)buf + n + 4, sizeof serial - 1);
            serial[sizeof serial - 1] = 0;
            break;
          case 0x7B:  //GPSPOS
            x = buf[n + 2] + 256 * (buf[n + 3] + 256 * (buf[n + 4] + 256 * buf[n + 5])) / 100.0;
            y = buf[n + 6] + 256 * (buf[n + 7] + 256 * (buf[n + 8] + 256 * buf[n + 9])) / 100.0;
            z = buf[n + 10] + 256 * (buf[n + 11] + 256 * (buf[n + 12] + 256 * buf[n + 13])) / 100.0;
            ecef2wgs84(x, y, z, lat, lng, alt);
            Serial.printf(" lat:%f lon:%f h:%f", lat, lng, alt);

            break;
          case 0x80:  //CRYPTO
            encrypted = true;
            break;
        }
      }
      n += blockLength + 4;
    }
  }
  Serial.println();
}

