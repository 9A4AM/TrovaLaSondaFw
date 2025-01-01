#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <CRC.h>
#include <RS-FEC.h>
#include "TrovaLaSondaFw.h"
#include "radio.h"
#include "rs41.h"

static bool processPacket(uint8_t buf[]);
static int processPartialPacket(uint8_t buf[]);

Sonde rs41 = {
  .name = "RS41",
  .bitRate = 4800,
  .afcBandWidth = 12500,
  .frequencyDeviation = 6300,
  .bandwidthHz = 9700,
  .packetLength = RS41_PACKET_LENGTH,
  .partialPacketLength = 49,
  .preambleLengthBytes = 3,
  .syncWordLen = 64,
  .flipBytes = true,
  .syncWord = { 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 },
  .processPartialPacket = processPartialPacket,
  .processPacket = processPacket
};

RS::ReedSolomon<99 + (RS41_PACKET_LENGTH - 48) / 2, 24> rs;
RS::ReedSolomon<99 + (RS41AUX_PACKET_LENGTH - 48) / 2, 24> rsAux;
static int actualPacketLength = RS41_PACKET_LENGTH;

// clang-format off
const uint8_t   whitening[] = {
  0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26, 0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1, 
  0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1, 0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C, 
  0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61, 0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23, 
  0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1, 0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98 
};
// clang-format on

static bool correctErrors(uint8_t data[], int length) {
  static uint8_t buf[256], dec[256];
  int i;

  //prima parte
  memset(buf, 0, 256);
  for (i = 0; i < (length - 48) / 2; i++)
    buf[99 + i] = data[length - 1 - 2 * i];
  for (i = 0; i < 24; i++)
    buf[254 - i] = data[24 + i];

  if (length == RS41_PACKET_LENGTH) {
    if (0 != rs.Decode(buf, dec)) return false;
  } else if (0 != rsAux.Decode(buf, dec)) return false;

  for (i = 0; i < (length - 48) / 2; i++)
    data[311 - 2 * i] = dec[99 + i];

  //seconda parte
  memset(buf, 0, 256);
  for (i = 0; i < (length - 48) / 2; i++)
    buf[99 + i] = data[length - 1 - 2 * i - 1];
  for (i = 0; i < 24; i++)
    buf[254 - i] = data[i];

  if (length == RS41_PACKET_LENGTH) {
    if (0 != rs.Decode(buf, dec)) return false;
  } else if (0 != rsAux.Decode(buf, dec)) return false;

  for (i = 0; i < (length - 48) / 2; i++)
    data[length - 1 - 2 * i - 1] = dec[99 + i];

  return true;
}

//Accurate Conversion of Earth-Fixed Earth-Centered
//Coordinates to Geodetic Coordinates
//Karl Osen
#define WGS84_INVAA +2.45817225764733181057e-0014    /* 1/(a^2) */
#define WGS84_P1MEEDAA +2.44171631847341700642e-0014 /* (1-(e^2))/(a^2) */
#define WGS84_EEEE +4.48147234524044602618e-0005     /* e^4 */
#define WGS84_INVCBRT2 +7.93700525984099737380e-0001 /* 1/(2^(1/3)) */
#define WGS84_INV3 +3.33333333333333333333e-0001     /* 1/3 */
#define WGS84_INV6 +1.66666666666666666667e-0001     /* 1/6 */
#define WGS84_EEEED4 +1.12036808631011150655e-0005   /* (e^4)/4 */
#define WGS84_EED2 +3.34718999507065852867e-0003     /* (e^2)/2 */
#define WGS84_P1MEE +9.93305620009858682943e-0001    /* 1-(e^2) */

void ecef2wgs84(double x, double y, double z, double &lat, double &lon, float &alt) {
  double latitude, longitude, altitude;

  // The variables below correspond to symbols used in the paper
  // "Accurate Conversion of Earth-Centered, Earth-Fixed Coordinates
  // to Geodetic Coordinates"
  double beta, C, dFdt, dt, dw, dz, F, G, H, i, k, m, n, p, P, t, u, v, w;

  // Intermediate variables
  double j, ww, mpn, g, tt, ttt, tttt, zu, wv, invuv, da;
  double t1, t2, t3, t4, t5, t6, t7;

  ww = x * x + y * y;
  m = ww * WGS84_INVAA;
  n = z * z * WGS84_P1MEEDAA;
  mpn = m + n;
  p = WGS84_INV6 * (mpn - WGS84_EEEE);
  G = m * n * WGS84_EEEED4;
  H = 2 * p * p * p + G;

  C = pow(H + G + 2 * sqrt(H * G), WGS84_INV3) * WGS84_INVCBRT2;
  i = -WGS84_EEEED4 - 0.5 * mpn;
  P = p * p;
  beta = WGS84_INV3 * i - C - P / C;
  k = WGS84_EEEED4 * (WGS84_EEEED4 - mpn);

  // Compute left part of t
  t1 = beta * beta - k;
  t2 = sqrt(t1);
  t3 = t2 - 0.5 * (beta + i);
  t4 = sqrt(t3);

  // Compute right part of t
  t5 = 0.5 * (beta - i);

  // t5 may accidentally drop just below zero due to numeric turbulence
  // This only occurs at latitudes close to +- 45.3 degrees
  t5 = fabs(t5);
  t6 = sqrt(t5);
  t7 = (m < n) ? t6 : -t6;

  // Add left and right parts
  t = t4 + t7;

  // Use Newton-Raphson's method to compute t correction
  j = WGS84_EED2 * (m - n);
  g = 2 * j;
  tt = t * t;
  ttt = tt * t;
  tttt = tt * tt;
  F = tttt + 2 * i * tt + g * t + k;
  dFdt = 4 * ttt + 4 * i * t + g;
  dt = -F / dFdt;

  // Compute latitude (range -PI/2..PI/2)
  u = t + dt + WGS84_EED2;
  v = t + dt - WGS84_EED2;
  w = sqrt(ww);
  zu = z * u;
  wv = w * v;
  latitude = atan2(zu, wv);

  // Compute altitude
  invuv = 1 / (u * v);
  dw = w - wv * invuv;
  dz = z - zu * WGS84_P1MEE * invuv;
  da = sqrt(dw * dw + dz * dz);
  altitude = (u < 1) ? -da : da;

  // Compute longitude (range -PI..PI)
  longitude = atan2(y, x);

  // Convert from radians to degrees
  lat = latitude * 180.0 / M_PI;
  lon = longitude * 180.0 / M_PI;
  alt = altitude;
}

static int processPartialPacket(uint8_t buf[]) {
  return actualPacketLength = buf[48] == 0xF0 ? RS41AUX_PACKET_LENGTH : RS41_PACKET_LENGTH;
}

static bool processPacket(uint8_t buf[]) {
  //TODO: testare AUX
  double x, y, z, vx, vy, vz, vn, ve, vu;
  int svs, n = 48 + 1;

  // frame = 0;
  // encrypted = false;
  // bkStatus = 0;
  // lat = lng = NAN;
  // alt = NAN;
  if (bkTime != 0xFFFFU) bkTime--;

  for (int i = 0; i < actualPacketLength; i++)
    buf[i] = whitening[i % sizeof whitening] ^ flipByte[buf[i]];

  if (!correctErrors(buf, actualPacketLength) && buf[48] == 0x0F) return false;
  while (n < actualPacketLength) {
    int blockType = buf[n];
    int blockLength = buf[n + 1];
    int subframeNumber;
    uint16_t crc = calcCRC16(buf + n + 2, blockLength, CRC16_CCITT_FALSE_POLYNOME, CRC16_CCITT_FALSE_INITIAL, CRC16_CCITT_FALSE_XOR_OUT, CRC16_CCITT_FALSE_REV_IN, CRC16_CCITT_FALSE_REV_OUT);

    //Serial.printf("Blocco 0x%02X, lunghezza %d, CRC: %02X%02X/%02X%02X\n", blockType, blockLength, buf[n + blockLength + 3], buf[n + blockLength + 2], crc >> 8, crc & 0xFF);
    if ((crc & 0xFF) == buf[n + blockLength + 2] && (crc >> 8) == buf[n + blockLength + 3]) {  //CRC OK
      switch (blockType) {
        case 0x79:  //status
          frame = buf[n + 2] + (buf[n + 3] << 8);
          Serial.printf(" frame: %d [%.8s] (subframe:%d)", frame, buf + n + 4, buf[n + 2 + 0x17]);

          strncpy(serial, (char *)buf + n + 4, 8);
          serial[8] = 0;

          subframeNumber = buf[n + 2 + 0x17];
          switch (subframeNumber) {
            case 0x02:
              bkStatus = buf[2 + n + 0x18 + 0x0B];
              Serial.printf("BkStatus: %d\n", bkStatus);
              break;
            case 0x32:
              bkTime = buf[2 + n + 0x18] + 256 * buf[2 + n + 0x18 + 1];
              cpuTemp = (int8_t)buf[2 + n + 0x18 + 8];
              radioTemp = (int8_t)buf[2 + n + 0x18 + 9];
              Serial.printf("BkTime: %d CPU: %d° radio: %d°\n", bkTime, cpuTemp, radioTemp);
              break;
          }
          break;
        case 0x7B:  //GPSPOS
          svs = buf[n + 0x14];
          if (svs >= 3) {
            x = (int32_t)(buf[n + 2] + 256 * (buf[n + 3] + 256 * (buf[n + 4] + 256 * buf[n + 5]))) / 100.0;
            y = (int32_t)(buf[n + 6] + 256 * (buf[n + 7] + 256 * (buf[n + 8] + 256 * buf[n + 9]))) / 100.0;
            z = (int32_t)(buf[n + 10] + 256 * (buf[n + 11] + 256 * (buf[n + 12] + 256 * buf[n + 13]))) / 100.0;
            ecef2wgs84(x, y, z, lat, lng, alt);

            vx = (int16_t)(buf[n + 2 + 0x0C] + 256 * buf[n + 2 + 0x0D]) / 100.0;
            vy = (int16_t)(buf[n + 2 + 0x0E] + 256 * buf[n + 2 + 0x0F]) / 100.0;
            vz = (int16_t)(buf[n + 2 + 0x10] + 256 * buf[n + 2 + 0x11]) / 100.0;
            vn = (-(vx * sin(lat) * cos(lng)) - vy * sin(lat) * sin(lng)) + vz * cos(lat);
            ve = -(vx * sin(lng)) + vy * cos(lng);
            vu = vx * cos(lat) * cos(lng) + vy * cos(lat) * sin(lng) + vz * sin(lat);
            vel = sqrt(pow(vn, 2) + pow(ve, 2));
            Serial.printf(" lat:%f lon:%f h:%f svs:%d vel:%fm/s vup:%fm/s", lat, lng, alt, svs, vel, vu);
          }
          break;
        case 0x80:  //CRYPTO
          encrypted = true;
          break;
      }
    }
    n += blockLength + 4;
  }
  Serial.println();
  return true;
}
