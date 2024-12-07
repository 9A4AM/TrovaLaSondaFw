#include <arduino.h>
#include "TrovaLaSondaFw.h"
#include "dfm.h"
#include "m10.h"

static bool processPacket(uint8_t buf[]);

Sonde dfm = {
  .name = "DFM",
  .bitRate = 2500,
  .frequencyDeviation = 10400,  //?
  .bandWidth = SX126X_GFSK_BW_11700,
  .packetLength = DFM_PACKET_LENGTH * NPACKETS + 4 * (NPACKETS - 1),  //HACK!
  .partialPacketLength = 0,
  .preambleLength = SX126X_GFSK_PREAMBLE_DETECTOR_OFF,
  .syncWordLen = 16,
  .flipBytes = false,
  .syncWord = { /*0x9A, 0x99,*/ 0x5A, 0x55 },
  .processPartialPacket=NULL,
  .processPacket = processPacket
};

static void deinterleave(uint8_t* in, uint8_t* out, int len) {
  uint8_t b, dataIn[8 * len], dataOut[8 * len];
  int i, j;

  for (i = 0; i < len; i++) {  //spread the bits
    b = in[i];
    for (j = 0; j < 8; j++) {
      dataIn[8 * i + j] = b >> 7;
      b <<= 1;
    }
  }

  for (j = 0; j < 8; j++)
    for (i = 0; i < len; i++)
      dataOut[8 * i + j] = dataIn[len * j + i];

  for (i = 0; i < len; i++) {  //squeeze back the bits
    b = 0;
    for (j = 0; j < 8; j++) {
      b <<= 1;
      b |= dataOut[8 * i + j];
    }
    out[i] = b;
  }
}

static int parity(uint8_t x) {
  int ret;

  for (ret = 0; x; ret++)
    x &= x - 1;

  return ret & 1;
}

static int hamming(uint8_t* data, int len) {
  const uint8_t hamming_bitmasks[] = { 0xaa, 0x66, 0x1e, 0xff };
  int errpos, errcount = 0;
  int i, j;

  for (i = 0; i < len; i++) {
    errpos = 0;
    for (j = 0; j < (int)sizeof(hamming_bitmasks); j++)
      errpos += (1 << j) * parity(data[i] & hamming_bitmasks[j]);

    if (errpos > 7) return -1;

    if (errpos) {
      errcount++;
      data[i] ^= 1 << (8 - errpos);
    }
  }

  return errcount;
}

static void processConf(uint8_t type, uint8_t* data) {
  static int serialNumberConfType = -1;
  static uint64_t raw_serial = 0;
  bool valid = false;
  uint32_t ch = data[0] << 16 | data[1] << 8 | data[2];

  for (int i = 0; i < 7 / 2; i++)
    if (data[i] != 0) {
      valid = true;
      break;
    }
  //if (!valid) return;

  if ((ch & 0xFFFF) == 0) {
    serialNumberConfType = type + 1;
    return;
  }

  if (type == serialNumberConfType) {
    int serial_idx = 3 - (ch & 0xF);
    uint16_t serial_shard = (ch >> 4) & 0xFFFF;
    raw_serial &= ~((uint64_t)((1 << 16) - 1) << (16 * serial_idx));
    raw_serial |= (uint64_t)serial_shard << (16 * serial_idx);
    if ((ch & 0xF) == 0) {
      while (raw_serial && !(raw_serial & 0xFFFF)) raw_serial >>= 16;
      Serial.printf("\tSerial: %08ld=============================\n", raw_serial);
      snprintf(serial, sizeof serial - 1, "%08ld", raw_serial);
      serial[sizeof serial - 1] = '\0';
      raw_serial = 0;
    }
  }
}

static void processDat(uint8_t type, uint8_t* data) {
  uint16_t sec;
  bool valid = false;

  for (int i = 0; i < 13 / 2; i++)
    if (data[i] != 0) {
      valid = true;
      break;
    }
  if (!valid) return;

  switch (type) {
    case TIME:
      sec = data[4] << 8 | data[5];
      Serial.printf("\t\t\tSec: %d\n", sec / 1000);
      break;
    case LAT:
      lat = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) / 1e7;
      Serial.printf("\t\t\tLat: %f\n", lat);
      break;
    case LON:
      lng = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) / 1e7;
      Serial.printf("\t\t\tLon: %f\n", lng);
      break;
    case ALT:
      alt = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) / 1e2;
      Serial.printf("\t\t\tAlt: %f\n", alt);
      break;
  }
}

static bool processPacket(uint8_t buf[]) {
  uint8_t out[(DFM_PACKET_LENGTH * NPACKETS + (NPACKETS - 1) * 4) / 2];
  bool valid=true;

  if (manchesterDecode(buf, out, DFM_PACKET_LENGTH * NPACKETS + (NPACKETS - 1) * 4)) {
    Serial.println("----------------------------------------------");
    for (int k = 0; k < NPACKETS; k++) {
      /////////////////////////////////////////////
      for (int i = 0; i < 33; i++) {
        uint8_t b = out[i + k * 35];
        for (int j = 0; j < 8; j++) {
          putchar('0' + (1 & (b >> 7)));
          b <<= 1;
        }
      }
      putchar('\n');
      /////////////////////////////////////////////

      if (k > 0)
        if (out[k * 35 - 2] != 0x45 || out[k * 35 - 1] != 0xCF) {
          Serial.printf("Syncword errata %02X %02X\n", out[k * 35 - 2], out[k * 35 - 1]);
          continue;
        }

      uint8_t conf[7], dat1[13], dat2[13];
      deinterleave(out + k * 35, conf, 7);
      deinterleave(out + k * 35 + 7, dat1, 13);
      deinterleave(out + k * 35 + 7 + 13, dat2, 13);
      //dump(conf, 7);

      int err = hamming(conf, 7);
      if (err < 0) {
        Serial.println("ECC CONF");
        valid=false;
      }
      err = hamming(dat1, 13);
      if (err < 0) {
        Serial.println("ECC DAT1");
        valid=false;
      }
      err = hamming(dat2, 13);
      if (err < 0) {
        Serial.println("ECC DAT2 ");
        valid=false;
      }

      if (!valid) return false;

      uint8_t confType = conf[0] >> 4;
      Serial.printf("conf type: %1X, data: ", confType);
      for (int i = 0; i < 7 / 2; i++) conf[i] = conf[2 * i + 1] & 0xF0 | conf[2 * i + 2] >> 4;
      dump(conf, 7 / 2);

      uint8_t dat1Type = dat1[12] >> 4;
      //Serial.printf("dat1 type: %1X, data: ", dat1Type);
      for (int i = 0; i < 13 / 2; i++) dat1[i] = dat1[2 * i] & 0xF0 | dat1[2 * i + 1] >> 4;
      //dump(dat1, 13 / 2);

      uint8_t dat2Type = dat2[12] >> 4;
      //Serial.printf("dat2 type: %1X, data: ", dat2Type);
      for (int i = 0; i < 13 / 2; i++) dat2[i] = dat2[2 * i] & 0xF0 | dat2[2 * i + 1] >> 4;
      //dump(dat2, 13 / 2);

      processConf(confType, conf);
      processDat(dat1Type, dat1);
      processDat(dat2Type, dat2);
      return true;
    }
  } else
    Serial.println("Errore codifica manchester");
  return false;
}