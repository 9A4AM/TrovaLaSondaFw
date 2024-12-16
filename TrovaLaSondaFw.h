#ifndef __TROVALASONDAFW_H__
#define __TROVALASONDAFW_H__

#include <stdint.h>
#include <driver/gpio.h>

#ifdef WIFI_LoRa_32_V3
#define SX126X
// #include <driver\board-config.h>
const gpio_num_t BUTTON = GPIO_NUM_0, VBAT_PIN = GPIO_NUM_1, ADC_CTRL_PIN = GPIO_NUM_37, BUZZER = GPIO_NUM_46,
                 RADIO_NSS = GPIO_NUM_8, RADIO_DIO_1 = GPIO_NUM_14, RADIO_BUSY = GPIO_NUM_13, RADIO_RESET = GPIO_NUM_12,
                 LORA_CLK = GPIO_NUM_9, LORA_MISO = GPIO_NUM_11, LORA_MOSI = GPIO_NUM_10;
#else
#ifdef ARDUINO_TTGO_LoRa32_V1
#define SDA_OLED 21
#define SCL_OLED 22
#define RST_OLED 16

#define SX1278
const gpio_num_t BUTTON = GPIO_NUM_NC, VBAT_PIN = GPIO_NUM_35, ADC_CTRL_PIN = GPIO_NUM_NC, BUZZER = GPIO_NUM_4,
                 Vext = GPIO_NUM_NC, RADIO_NSS = (gpio_num_t)SS, RADIO_RESET = GPIO_NUM_23, LORA_CLK = (gpio_num_t)SCK, RADIO_DIO_0 = GPIO_NUM_26;
#else
#error "Board not supported"
#endif
#endif

#define PACKET_LENGTH RS41AUX_PACKET_LENGTH  //longest packet length
#define SERIAL_LENGTH 12
#define SYNCWORD_SIZE 8

typedef struct Sonde_s {
  const char *name;
  unsigned bitRate, afcBandWidth,
    frequencyDeviation, bandwidthHz;
  int packetLength,
    partialPacketLength,  //Minimum # of bytes to be read to determine actual packet length
    preambleLengthBytes,
    syncWordLen;
  bool flipBytes;
  uint8_t syncWord[SYNCWORD_SIZE];

  int (*processPartialPacket)(uint8_t buf[]);  //To be called after partialPacketLength bytes
                                               //have been read to determine the actual packet length
  bool (*processPacket)(uint8_t buf[]);
} Sonde;

extern const uint8_t flipByte[];
extern Sonde *sondes[];
extern uint32_t freq;
extern int frame, currentSonde;
extern const uint8_t flipByte[];
extern int rssi, mute, batt;
extern bool encrypted, connected;
extern char serial[SERIAL_LENGTH + 1];
extern double lat, lng;
extern float alt, vel;
extern char version[];
extern uint8_t bkStatus;
extern uint16_t bkTime;
extern bool otaRunning;
extern int otaLength, otaErr, otaProgress;

void dump(uint8_t buf[], int size);
void savePrefs();
void bip(int duration, int freq);
#endif