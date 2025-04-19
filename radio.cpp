#include "sx1278.h"
#include <arduino.h>
#include <SPI.h>
#include "TrovaLaSondaFw.h"
#include "radio.h"
#include "rs41.h"
#include "Ble.h"

uint8_t buf[RS41AUX_PACKET_LENGTH];
int nBytesRead = 0;

#ifdef SX126X
struct sx126x_long_pkt_rx_state pktRxState;

sx126x_gfsk_preamble_detector_t getPreambleLength(unsigned lengthInBytes) {
  sx126x_gfsk_preamble_detector_t tab[] = {
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS
  };
  if (lengthInBytes <= 0 || lengthInBytes - 1 > sizeof tab / sizeof(*tab))
    return SX126X_GFSK_PREAMBLE_DETECTOR_OFF;
  return tab[lengthInBytes - 1];
}

sx126x_gfsk_bw_t getBandwidth(unsigned bandwidth) {
  // clang-format off
  sx126x_gfsk_bw_t tab[] = {
    SX126X_GFSK_BW_4800,SX126X_GFSK_BW_5800,SX126X_GFSK_BW_7300,
    SX126X_GFSK_BW_9700,SX126X_GFSK_BW_11700,SX126X_GFSK_BW_14600,
    SX126X_GFSK_BW_19500,SX126X_GFSK_BW_23400,SX126X_GFSK_BW_29300,
    SX126X_GFSK_BW_39000,SX126X_GFSK_BW_46900,SX126X_GFSK_BW_58600,
    SX126X_GFSK_BW_78200,SX126X_GFSK_BW_93800,SX126X_GFSK_BW_117300,
    SX126X_GFSK_BW_156200,SX126X_GFSK_BW_187200,SX126X_GFSK_BW_234300,
    SX126X_GFSK_BW_312000,SX126X_GFSK_BW_373600,SX126X_GFSK_BW_467000,
  };
  unsigned  limits[] = {
    4800,5800,7300,9700,11700,14600,19500,23400,29300,39000,46900,
    58600,78200,93800,117300,156200,187200,234300,312000,373600,467000,
  };
  // clang-format on
  for (int i = 0; i < sizeof limits / sizeof *limits - 1; i++)
    if (bandwidth < (limits[i] + limits[i + 1]) / 2) return tab[i];
  return SX126X_GFSK_BW_467000;
}
#endif

#ifdef SX1278
uint8_t calcMantExp(uint16_t bw) {
  uint8_t exp = 1;
  bw = SX127X_CRYSTAL_FREQ / bw / 8;
  while (bw > 31) {
    exp++;
    bw /= 2;
  }
  uint8_t mant = bw < 17 ? 0 : bw < 21 ? 1
                                       : 2;
  return (mant << 3) | exp;
}

void dumpRegisters(void) {
  for (int j = 0; j < 16; j++) {
    for (int i = 0; i < 7; i++)
      Serial.printf("%02X:%02X ", i * 16 + j, readRegister(i * 16 + j));
    Serial.println();
  }
}
#endif

void initRadio() {
  char s[20];

  *packet.serial = '\0';
  packet.lat = packet.lng = packet.alt = 0;

  pinMode(RADIO_NSS, OUTPUT);
  digitalWrite(RADIO_NSS, HIGH);
  pinMode(RADIO_RESET, OUTPUT);
  digitalWrite(RADIO_RESET, HIGH);

  SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI);

  uint8_t syncWord[SYNCWORD_SIZE];
  for (int i = 0; i < SYNCWORD_SIZE; i++)
    syncWord[i] = sondes[currentSonde]->flipBytes ? flipByte[sondes[currentSonde]->syncWord[i]] : sondes[currentSonde]->syncWord[i];

#ifdef SX126X
  pinMode(RADIO_DIO_1, INPUT);
  pinMode(RADIO_BUSY, INPUT);

  sx126x_mod_params_gfsk_t modParams = {
    .br_in_bps = sondes[currentSonde]->bitRate,
    .fdev_in_hz = sondes[currentSonde]->frequencyDeviation,
    .pulse_shape = SX126X_GFSK_PULSE_SHAPE_OFF,
    .bw_dsb_param = getBandwidth(sondes[currentSonde]->bandwidthHz),
  };
  sx126x_pkt_params_gfsk_t pktParams = {
    .preamble_len_in_bits = 0,
    .preamble_detector = getPreambleLength(sondes[currentSonde]->preambleLengthBytes),
    .sync_word_len_in_bits = sondes[currentSonde]->syncWordLen,
    .address_filtering = SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
    .header_type = SX126X_GFSK_PKT_FIX_LEN,
    .pld_len_in_bytes = min(255, sondes[currentSonde]->packetLength),
    .crc_type = SX126X_GFSK_CRC_OFF,
    .dc_free = SX126X_GFSK_DC_FREE_OFF
  };
  sx126x_status_t res = sx126x_reset(NULL);
  res = sx126x_set_dio3_as_tcxo_ctrl(NULL, SX126X_TCXO_CTRL_1_6V, 128);  //2ms
  delay(1);
  res = sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);
  Serial.printf("sx126x_set_standby %d\n", res);
  while (digitalRead(RADIO_BUSY) == HIGH)
    ;
  res = sx126x_set_reg_mode(NULL, SX126X_REG_MODE_DCDC);
  Serial.printf("sx126x_set_reg_mode %d\n", res);

  res = sx126x_set_pkt_type(NULL, SX126X_PKT_TYPE_GFSK);
  Serial.printf("sx126x_set_pkt_type %d\n", res);
  res = sx126x_set_rf_freq(NULL, freq * 1000UL);
  Serial.printf("sx126x_set_rf_freq %d\n", res);
  res = sx126x_set_gfsk_mod_params(NULL, &modParams);
  Serial.printf("sx126x_set_gfsk_mod_params %d\n", res);

  if (sondes[currentSonde]->packetLength > 255) {
    res = sx126x_long_pkt_rx_set_gfsk_pkt_params(NULL, &pktParams);
    Serial.printf("sx126x_long_pkt_rx_set_gfsk_pkt_params %d\n", res);
    res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    Serial.printf("sx126x_set_dio_irq_params %d\n", res);
  } else {
    res = sx126x_set_gfsk_pkt_params(NULL, &pktParams);
    Serial.printf("sx126x_rx_set_gfsk_pkt_params %d\n", res);
    res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_RX_DONE, SX126X_IRQ_RX_DONE, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    Serial.printf("sx126x_set_dio_irq_params %d\n", res);
  }

  res = sx126x_set_gfsk_sync_word(NULL, syncWord, sizeof syncWord);
  Serial.printf("sx126x_set_gfsk_sync_word %d\n", res);

  res = sx126x_cal_img(NULL, 0x6B, 0x6F);
  uint8_t val = 0x96;
  res = sx126x_write_register(NULL, SX126X_REG_RXGAIN, &val, 1);

  res = sx126x_clear_device_errors(NULL);

  if (sondes[currentSonde]->packetLength > 255) {
    res = sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
    Serial.printf("sx126x_long_pkt_set_rx %d\n", res);
  } else {
    res = sx126x_set_rx_with_timeout_in_rtc_step(NULL, 0);  //SX126X_RX_CONTINUOUS);
    Serial.printf("sx126x_set_rx %d\n", res);
  }
#endif
#ifdef SX1278
  Serial.println("Initializing SX1278");
  uint8_t mode;

  pinMode(RADIO_DIO_0, INPUT);
  ///////////////////////////////////////////////////////////
  // pinMode(15, INPUT);//DIO1
  // pinMode(13, INPUT);//DIO2
  ///////////////////////////////////////////////////////////

  //reset SX1278
  delay(10);
  digitalWrite(RADIO_RESET, LOW);
  delay(1);
  digitalWrite(RADIO_RESET, HIGH);
  delay(5);

  writeRegister(RegOcp, 0x3B);  //max current //TODO:

  writeRegister(RegOpMode, SLEEP_MODE);
  writeRegister(RegOpMode, SLEEP_MODE);
  writeRegister(RegOpMode, STANDBY_MODE);
  delay(100);

  mode = readRegister(RegOpMode);
  if (mode != STANDBY_MODE)
    Serial.printf("Error, not in standby: %d\n", mode);

  uint16_t bps = sondes[currentSonde]->bitRate,
           bitRate = (SX127X_CRYSTAL_FREQ * 1.0) / bps,
           fracRate = (SX127X_CRYSTAL_FREQ * 16.0) / bps - bitRate * 16 + 0.5;
  writeRegister(RegBitRateMsb, bitRate >> 8);
  writeRegister(RegBitRateLsb, bitRate);
  writeRegister(RegBitRateFrac, fracRate);

  writeRegister(RegRxBw, calcMantExp(sondes[currentSonde]->bandwidthHz));
  writeRegister(RegAfcBw, calcMantExp(sondes[currentSonde]->afcBandWidth));
  writeRegister(RegFdevLsb, sondes[currentSonde]->frequencyDeviation / 61);
  writeRegister(RegFdevMsb, (sondes[currentSonde]->frequencyDeviation / 61) >> 8);

  writeRegister(RegRxConfig, 1 << 7 | 1 << 6 | 1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1);  //7);  //RestartRxOnCollision, RestartRxWithoutPllLock, AfcAutoOn, AgcAutoOn, RxTrigger: Rssi and preamble detect, Interrupt&PreambleDetect both AGC and AFC

  writeRegister(RegSyncConfig, 1 << 6 | 1 << 5 | 1 << 4 | (sondes[currentSonde]->syncWordLen / 8 - 1));  //autorestart w/o PLL wait, sync on, sync on, n bytes sync word
  if (sondes[currentSonde]->preambleLengthBytes > 0) {
    writeRegister(RegPreambleDetect, 1u << 7 | (((sondes[currentSonde]->preambleLengthBytes - 1) / 8) - 1) << 4 | 0xA);
    writeRegister(RegDioMapping2, 1);
  } else {
    writeRegister(RegPreambleDetect, 0);  //disabled
    writeRegister(RegDioMapping2, 0);     //Preamble detectrq disabled
  }

  for (int i = 0; i < sondes[currentSonde]->syncWordLen / 8; i++)
    writeRegister(RegSyncValue1 + i, syncWord[i]);
  writeRegister(RegPacketConfig1, 0x08);  //fixed length, no DC-free,no CRC,no filtering

  // writeRegister(RegPacketConfig2,0);//continuous mode/////////////////////////////
  // writeRegister(RegOokPeak,1<<5);//bit synchronizer on//////////////////////////////////
  // writeRegister(RegSyncConfig,0);/////////////////////////////////////////////////////
  writeRegister(RegPacketConfig2, 1u << 6 | ((sondes[currentSonde]->packetLength >> 8) & 7));  //packet mode,payload length msb
  writeRegister(RegPayloadLength, sondes[currentSonde]->packetLength);                         //payload length lsb

  writeRegister(RegDioMapping1, 0 << 6);  //DIO0:Payload ready

  writeRegister(RegFifoThresh, 48);

  //~ writeRegister(RegLna,0b110u<<5); //-48dB

  //~ writeRegister(RegOsc,1<<3);	//OSC calibration
  //~ while (readRegister(RegOsc)&1<<3) ;

  delay(1);

  uint32_t f = freq * (1UL << 11);
  f /= (SX127X_CRYSTAL_FREQ / 1000) / (1UL << 8);
  writeRegister(RegFreqMsb, f >> 16);
  writeRegister(RegFreqMid, f >> 8);
  writeRegister(RegFreqLsb, f);

  writeRegister(RegOpMode, 1 << 3 | RX_MODE);  //Low frequency
  delay(1);

  // writeRegister(RegIrqFlags1, 0xFF);
  // writeRegister(RegIrqFlags2, 0xFF);

  // dumpRegisters();
#endif
}

bool loopRadio() {
  static uint64_t tLastRead = 0, tLastPacket = 0, tLastRSSI = 0;
  static int16_t actualPacketLength = 0;
  bool validPacket = false;

#ifdef SX126X
  sx126x_status_t res;
  sx126x_pkt_status_gfsk_t pktStatus;
  sx126x_rx_buffer_status_t bufStatus;

  if (sondes[currentSonde]->packetLength > 255) {
    if (digitalRead(RADIO_DIO_1) == HIGH) {
      //Serial.println("SYNC");
      tLastPacket = tLastRead = millis();
      nBytesRead = 0;
      actualPacketLength = sondes[currentSonde]->packetLength;
      res = sx126x_clear_irq_status(NULL, SX126X_IRQ_SYNC_WORD_VALID);
      res = sx126x_get_gfsk_pkt_status(NULL, &pktStatus);
      rssi = pktStatus.rssi_sync;
    }
    if (tLastRead != 0 && millis() - tLastRead > 1000) {
      res = sx126x_long_pkt_rx_prepare_for_last(NULL, &pktRxState, 0);
      tLastRead = 0;
      nBytesRead = 0;
    }
    if (tLastRead != 0 && millis() - tLastRead > 300) {
      tLastRead = millis();
      uint8_t read;
      res = sx126x_long_pkt_rx_get_partial_payload(NULL, &pktRxState, buf + nBytesRead, sizeof buf - nBytesRead, &read);
      if (read == 0) {
        tLastRead = 0;
        nBytesRead = 0;
        validPacket = false;
      }
      //Serial.printf("  READ %d\n", read);
      if (sondes[currentSonde]->processPartialPacket != NULL && nBytesRead < sondes[currentSonde]->partialPacketLength && nBytesRead + read >= sondes[currentSonde]->partialPacketLength)
        actualPacketLength = sondes[currentSonde]->processPartialPacket(buf);
      nBytesRead += read;
      if (actualPacketLength - nBytesRead <= 255)
        res = sx126x_long_pkt_rx_prepare_for_last(NULL, &pktRxState, actualPacketLength - nBytesRead);

      if (nBytesRead == actualPacketLength) {
        Serial.printf("fine\n");
        sx126x_long_pkt_rx_complete(NULL);
        sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
        tLastRead = 0;
        nBytesRead = 0;

        //dump(buf, actualPacketLength);
        validPacket = sondes[currentSonde]->processPacket(buf);
      }
    }
  } else {
    if (digitalRead(RADIO_DIO_1) == HIGH) {
      tLastPacket = millis();
      res = sx126x_clear_irq_status(NULL, SX126X_IRQ_RX_DONE);
      res = sx126x_set_rx_with_timeout_in_rtc_step(NULL, 0);
      res = sx126x_get_rx_buffer_status(NULL, &bufStatus);
      res = sx126x_read_buffer(NULL, bufStatus.buffer_start_pointer, buf, bufStatus.pld_len_in_bytes);
      res = sx126x_get_gfsk_pkt_status(NULL, &pktStatus);
      rssi = pktStatus.rssi_sync;
      //Serial.printf("PKT %d bytes\n", bufStatus.pld_len_in_bytes);
      //dump(buf, PACKET_LENGTH);
      validPacket = sondes[currentSonde]->processPacket(buf);
    }
  }
#endif

#ifdef SX1278
  //TODO: TESTARE AUX  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  static uint16_t nCurByte = 0;
  static uint8_t oldIrq1, oldIrq2;
  uint8_t irq1 = readRegister(RegIrqFlags1),
          irq2 = readRegister(RegIrqFlags2);

  ///////////////////////////////////////////////////////////////
  if (tLastPacket != 0 && millis() - tLastPacket > 3000) {
    tLastPacket = 0;
    // Serial.println("\n---------------");
    // dumpRegisters();
    // Serial.println("---------------");
    initRadio();  //HACKHACK
  }
  ///////////////////////////////////////////////////////////////

  if ((irq2 & 0x10) != 0) {  //FIFO overrun
    Serial.println("OVERRUN");
    writeRegister(RegIrqFlags2, 0x10);  //reset bit
    nCurByte = 0;
  }
  if ((irq2 & 4) != 0) {  //digitalRead(RADIO_DIO_0) == HIGH) {  //DIO0: payload ready
    Serial.printf("PKT (len=%d)\n",actualPacketLength);
    while (nCurByte < actualPacketLength) {
      buf[nCurByte] = readRegister(RegFIFO);
      nCurByte++;
    }
    tLastPacket = millis();
    validPacket = sondes[currentSonde]->processPacket(buf);
    nCurByte = 0;
  }
  if ((oldIrq1 & 0x01) == 0 && (irq1 & 0x01) != 0) {
    Serial.println("SYNC");
    nCurByte = 0;
    actualPacketLength = sondes[currentSonde]->packetLength;
    tLastPacket = millis();
  }
  if (irq1 != oldIrq1 || irq2 != oldIrq2) {
    oldIrq1 = irq1;
    oldIrq2 = irq2;
    // Serial.printf("irq1: %02X irq2: %02X\n", irq1, irq2);
  }
  if ((irq2 & 0x20) != 0) {  //fifo level
    if (nCurByte == 0)
      rssi = readRegister(RegRssiValue);
    for (int i = 0; i < 48 && nCurByte < actualPacketLength; i++, nCurByte++) {
      buf[nCurByte] = readRegister(RegFIFO);
      if (sondes[currentSonde]->partialPacketLength > 0 && nCurByte == sondes[currentSonde]->partialPacketLength)
        actualPacketLength = sondes[currentSonde]->processPartialPacket(buf);
      // Serial.printf("nCurByte=%d\n", nCurByte);
    }
  }
#endif

  if (validPacket) {
    Serial.println("invio notifica pacchetto");
    BLENotifyPacket();
  } else {
    if ((tLastPacket == 0 || millis() - tLastPacket > 3000) && (tLastRSSI == 0 || millis() - tLastRSSI > 500)) {
#ifdef SX126X
      int16_t t;
      sx126x_get_rssi_inst(NULL, &t);
      rssi = t;
#else
      rssi = -readRegister(RegRssiValue) / 2;
#endif
      //Serial.printf("rssi: %d\n", rssi);
      tLastRSSI = millis();
    }
  }

  return validPacket;
}

void sleepRadio() {
#ifdef SX126X
  sx126x_set_sleep(NULL, SX126X_SLEEP_CFG_COLD_START);
#endif
}