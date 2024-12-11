//Portions from https://github.com/dbdexter-dev/sondedump/tree/master/sonde/dfm09
//under MIT license
#include <arduino.h>
#include "TrovaLaSondaFw.h"
#include "m10.h"

static bool processPacket(uint8_t buf[]);

Sonde m10={
  .name="M10",
  .bitRate=9615,
  .frequencyDeviation= 12500,//?
  .bandwidthHz=14600,
  .packetLength=M10_PACKET_LENGTH,
  .partialPacketLength = 0,
  .preambleLengthBytes=0,
  .syncWordLen=48,
  .flipBytes=false,
  .syncWord={ 0x66, 0x66, 0x66, 0x66, 0xB3, 0x66 },
  .processPartialPacket=NULL,
  .processPacket=processPacket
};

bool manchesterDecode(uint8_t* data, uint8_t* out, int len) {
  uint8_t t;
  uint16_t w;

  for (int i = 0; i < len / 2; i++) {
    out[i] = 0;
    w = (data[2 * i] << 8) | data[2 * i + 1];
    for (int j = 0; j < 8; j++) {
      out[i] <<= 1;
      t = (w >> 8) & 0b11000000;
      if (t == 0b01000000)
        out[i] |= 1;
      else if (t != 0b10000000) {
        Serial.printf("Err manchester @ bit # %d\n", i);
        return false;
      }
      w <<= 2;
    }
  }
  return true;
}

uint16_t m10CrcStep(uint16_t c, uint8_t b) {
  int c1 = c & 0xFF;
  // B
  b = (b >> 1) | ((b & 1) << 7);
  b ^= (b >> 2) & 0xFF;
  // A1
  int t6 = (c & 1) ^ ((c >> 2) & 1) ^ ((c >> 4) & 1);
  int t7 = ((c >> 1) & 1) ^ ((c >> 3) & 1) ^ ((c >> 5) & 1);
  int t = (c & 0x3F) | (t6 << 6) | (t7 << 7);
  // A2
  int s = (c >> 7) & 0xFF;
  s ^= (s >> 2) & 0xFF;
  int c0 = b ^ t ^ s;
  return ((c1 << 8) | c0) & 0xFFFF;
}

static void m10_frame_descramble(M10Frame *frame) {
	uint8_t *raw_frame = (uint8_t*)frame;
	uint8_t tmp, topbit;
	int i;

	topbit = 0;
	for (i=0; i<(int)sizeof(*frame); i++) {
		tmp = raw_frame[i] << 7;
		raw_frame[i] ^= 0xFF ^ (topbit | raw_frame[i] >> 1);
		topbit = tmp;
	}
}

static float m10_9f_lat(const M10Frame_9f* f) {
	int32_t lat =  f->lat[0] << 24 | f->lat[1] << 16 | f->lat[2] << 8 | f->lat[3];
	return lat * 360.0 / ((uint64_t)1UL << 32);
}

static float m10_9f_lon(const M10Frame_9f* f) {
	int32_t lon = f->lon[0] << 24 | f->lon[1] << 16 | f->lon[2] << 8 | f->lon[3];
	return lon * 360.0 / ((uint64_t)1UL << 32);
}

static float m10_9f_alt(const M10Frame_9f* f) {
	int32_t alt =  f->alt[0] << 24 | f->alt[1] << 16 | f->alt[2] << 8 | f->alt[3];
	return alt / 1e3;
}

static void m10_9f_serial(char *dst, const M10Frame_9f *frame) {
	const uint32_t serial_0 = (frame->serial[2] >> 4) * 100 + (frame->serial[2] & 0xF);
	const uint32_t serial_1 = frame->serial[0];
	const uint32_t serial_2 = frame->serial[3] | frame->serial[4] << 8;

	sprintf(dst, "%03d-%d-%1d%04d", serial_0, serial_1, serial_2 >> 13, serial_2 & 0x1FFF);
}

static int m10_frame_correct(M10Frame_9f *frame) {
	const uint8_t *raw_frame = (uint8_t*)&frame->len;
	const uint8_t *crc_ptr = (uint8_t*)&frame->len + frame->len - 1;
	const uint16_t expected = crc_ptr[0] << 8 | crc_ptr[1];
	uint16_t crc;

	crc = 0;
	for (; raw_frame < crc_ptr; raw_frame++)
		crc = m10CrcStep(crc, *raw_frame);

	return (crc == expected) ? 0 : -1;
}

 static bool processPacket(uint8_t buf[]) {
  static M10Frame_9f frame;
  frame.sync_mark[0]=0x55;
  frame.sync_mark[1]=0x55;
  frame.sync_mark[2]=0x85;

  if (manchesterDecode(buf, &frame.len, M10_PACKET_LENGTH)) {
    for (int i=0;i<M10_PACKET_LENGTH/2;i++) ((uint8_t*)&frame.len)[i]^=0xFF;
    Serial.println("manchester");
    dump((uint8_t*)&frame,sizeof frame);
    m10_frame_descramble((M10Frame*)&frame);
    //Serial.println("descramble");
    //dump(out, M10_PACKET_LENGTH/2);
    if (m10_frame_correct(&frame)==0) {
      m10_9f_serial(serial,&frame);
      lat=m10_9f_lat(&frame);
      lng=m10_9f_lon(&frame);
      alt=m10_9f_alt(&frame);
      //TODO: frame #
      return true;
    }
    else
      Serial.println("Errore CRC");
  }
  return false;
}