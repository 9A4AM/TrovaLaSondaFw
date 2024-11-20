#include <arduino.h>
#include <SPI.h>
#include <HT_SSD1306Wire.h>
#include <driver/board-config.h>
#include "radio.h"
#include "TrovaLaSondaFw.h"
#include "rs41.h"
#include "Ble.h"

uint8_t buf[RS41_PACKET_LENGTH];
int nBytesRead = 0;

void initRadio() {
  char s[20];

  pinMode(RADIO_NSS, OUTPUT);
  digitalWrite(RADIO_NSS, HIGH);
  pinMode(RADIO_RESET, OUTPUT);
  digitalWrite(RADIO_RESET, HIGH);
  pinMode(RADIO_BUSY, INPUT);
  pinMode(RADIO_DIO_1, INPUT);

  SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI);

  sx126x_mod_params_gfsk_t modParams = {
    .br_in_bps = sondes[currentSonde]->bitRate,//4800,
    .fdev_in_hz = sondes[currentSonde]->frequencyDeviation,//3600,                          //?
    .pulse_shape = SX126X_GFSK_PULSE_SHAPE_OFF,  //?
    .bw_dsb_param = sondes[currentSonde]->bandWidth,//SX126X_GFSK_BW_9700          //?
  };
  sx126x_pkt_params_gfsk_t pktParams = {
    .preamble_len_in_bits = 0,
    .preamble_detector = sondes[currentSonde]->preambleLength,//SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS,
    .sync_word_len_in_bits = sondes[currentSonde]->syncWordLen,//64,
    .address_filtering = SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
    .header_type = SX126X_GFSK_PKT_FIX_LEN,
    .pld_len_in_bytes = min(255,sondes[currentSonde]->packetLength), //255
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

  if (sondes[currentSonde]->packetLength>255) {
    res = sx126x_long_pkt_rx_set_gfsk_pkt_params(NULL, &pktParams);
    Serial.printf("sx126x_long_pkt_rx_set_gfsk_pkt_params %d\n", res);
    res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    Serial.printf("sx126x_set_dio_irq_params %d\n", res);
  }
  else {
    res = sx126x_set_gfsk_pkt_params(NULL, &pktParams);
    Serial.printf("sx126x_rx_set_gfsk_pkt_params %d\n", res);
    res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_RX_DONE, SX126X_IRQ_RX_DONE, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    Serial.printf("sx126x_set_dio_irq_params %d\n", res);
  }

  uint8_t syncWord[SYNCWORD_SIZE];
  for (int i = 0; i < SYNCWORD_SIZE; i++)
    syncWord[i] = sondes[currentSonde]->flipBytes ? flipByte[sondes[currentSonde]->syncWord[i]] : sondes[currentSonde]->syncWord[i];
  res = sx126x_set_gfsk_sync_word(NULL, syncWord, sizeof syncWord);
  Serial.printf("sx126x_set_gfsk_sync_word %d\n", res);

  res = sx126x_cal_img(NULL, 0x6B, 0x6F);
  uint8_t val = 0x96;
  res = sx126x_write_register(NULL, SX126X_REG_RXGAIN, &val, 1);

  res = sx126x_clear_device_errors(NULL);

  if (sondes[currentSonde]->packetLength>255) {
    res = sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
    Serial.printf("sx126x_long_pkt_set_rx %d\n", res);
  }
  else {
    res = sx126x_set_rx_with_timeout_in_rtc_step(NULL, 0);//SX126X_RX_CONTINUOUS);
    Serial.printf("sx126x_set_rx %d\n", res);
  }
}

bool loopRadio() {
  static uint64_t tLastRead = 0, tLastPacket = 0, tLastRSSI = 0;
  bool validPacket=false;
  sx126x_status_t res;
  sx126x_pkt_status_gfsk_t pktStatus;
  sx126x_rx_buffer_status_t bufStatus;

  if (sondes[currentSonde]->packetLength>255) {
    if (digitalRead(RADIO_DIO_1) == HIGH) {
      //Serial.println("SYNC");
      tLastPacket = tLastRead = millis();
      nBytesRead = 0;
      res = sx126x_clear_irq_status(NULL, SX126X_IRQ_SYNC_WORD_VALID);
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
        validPacket=false;
      }
      //Serial.printf("READ %d\n", read);
      nBytesRead += read;
      if (sizeof buf - nBytesRead <= 255)
        res = sx126x_long_pkt_rx_prepare_for_last(NULL, &pktRxState, sizeof buf - nBytesRead);
      
      if (nBytesRead == sizeof buf) {
        sx126x_long_pkt_rx_complete(NULL);
        sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
        tLastRead = 0;
        nBytesRead = 0;
        
        //dump(buf, sizeof buf);
        sondes[currentSonde]->processPacket(buf);
        validPacket=true;
      }
    } 
  }
  else {
    if (digitalRead(RADIO_DIO_1) == HIGH) {
      tLastPacket=millis();
      res = sx126x_clear_irq_status(NULL, SX126X_IRQ_RX_DONE);
      res = sx126x_set_rx_with_timeout_in_rtc_step(NULL, 0);
      res = sx126x_get_rx_buffer_status(NULL, &bufStatus);
      res = sx126x_read_buffer(NULL, bufStatus.buffer_start_pointer, buf, bufStatus.pld_len_in_bytes);
      res = sx126x_get_gfsk_pkt_status(NULL, &pktStatus);
      rssi = pktStatus.rssi_avg ;
      Serial.printf("PKT %d bytes\n", bufStatus.pld_len_in_bytes);
      //dump(buf, PACKET_LENGTH);
      validPacket = sondes[currentSonde]->processPacket(buf);
    }
  }
  if (validPacket) {
    Serial.println("NOTIFICHE");
    BLENotifyLat();
    BLENotifyLon();
    BLENotifyAlt();
    BLENotifyFrame();
    BLENotifySerial();
  }
  else {
      if ((tLastPacket == 0 || millis() - tLastPacket > 3000) && (tLastRSSI == 0 || millis() - tLastRSSI > 500)) {
        int16_t t;
        sx126x_get_rssi_inst(NULL, &t);
        rssi=t;
        //Serial.printf("rssi: %d\n", rssi);
        tLastRSSI = millis();
      }
  }

  return validPacket;
}

void sleepRadio() {
  sx126x_set_sleep(NULL,SX126X_SLEEP_CFG_COLD_START);
}