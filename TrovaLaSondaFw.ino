//arduino-cli compile --fqbn Heltec-esp32:esp32:heltec_wifi_lora_32_V3
//arduino-cli upload -p COM3 --fqbn Heltec-esp32:esp32:heltec_wifi_lora_32_V3
//or --fqbn esp32:esp32:ttgo-lora32
#include <SPI.h>
#include <Wire.h>
#include <Ticker.h>
#include <MD_KeySwitch.h>
#include <Preferences.h>
// #include <melody_player.h>
// #include <melody_factory.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include "disp.h"
#include "TrovaLaSondaFw.h"
#include "radio.h"
#include "rs41.h"
#include "m10.h"
#include "m20.h"
#include "dfm.h"
#include "Ble.h"

char version[] = "2.09";
#if defined(ARDUINO_TTGO_LoRa32_V1)
char platform[] = "TL32";
#elif defined(WIFI_LoRa_32_V3)
char platform[] = "HL32";
#endif
const int BATTERY_SAMPLES = 20;
uint32_t freq = 403000;
int currentSonde = 0;
int rssi, mute, batt;
bool connected = false;
Packet packet={
  .frame=0,
  .lat=0,
  .lng=0,
  .alt=0,
  .hVel=0, 
  .vVel=0,
  .encrypted=false,
  .serial="",
};

bool otaRunning = false;
int otaLength = 0, otaErr = 0, otaProgress = 0;
// MelodyPlayer player(BUZZER, LOW);
// const int nNotes = 7, timeUnit = 175;
// String notes[nNotes] = { "2f#5", "b#5", "c#6", "d#6", "SILENCE", "b5", "d#6" };

// clang-format off
const uint8_t flipByte[] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
  };
// clang-format on
// Sonde unsupported = {
//   .name = "unsupp",
//   .bitRate = 9600,
//   .processPacket = [](uint8_t *p) -> bool {
//     return false;
//   }
// };
Sonde *sondes[] = { &rs41, &m20, &m10, &dfm09, &dfm17 };
Preferences preferences;
Ticker tickBuzzOff, tickLedOff;
MD_KeySwitch button(BUTTON, LOW);

void dump(uint8_t buf[], int size, int rowLen) {
  for (int i = 0; i < size; i++)
    Serial.printf("%02X%c", buf[i], i % rowLen == (rowLen-1) ? '\n' : ' ');
  if (size % rowLen != 0) Serial.println();
}

void bip(int duration = 200, int freq = 400) {
  if (mute) return;
  analogWriteFrequency(BUZZER, freq);
  analogWrite(BUZZER, 128);
  tickBuzzOff.once_ms(duration, []() {
    analogWrite(BUZZER, 0);
  });
}

void flash(int duration = 100) {
  digitalWrite(LED_BUILTIN, HIGH);
  tickLedOff.once_ms(duration, []() {
    digitalWrite(LED_BUILTIN, LOW);
  });
}

void VBattInit() {
  pinMode(VBAT_PIN, INPUT);
  if (ADC_CTRL_PIN != GPIO_NUM_NC)
    pinMode(ADC_CTRL_PIN, OUTPUT);
}

int getBattLevel() {
  if (ADC_CTRL_PIN != GPIO_NUM_NC) {
    digitalWrite(ADC_CTRL_PIN, LOW);
    delay(10);
  }
  uint32_t raw = 0;
  for (int i = 0; i < BATTERY_SAMPLES; i++)
    raw += analogRead(VBAT_PIN);

  raw /= BATTERY_SAMPLES;

  if (ADC_CTRL_PIN != GPIO_NUM_NC)
    digitalWrite(ADC_CTRL_PIN, HIGH);
  return constrain(map(raw, 670, 950, 0, 100), 0, 100);
}

void savePrefs() {
  preferences.begin("TLS", false);
  preferences.putInt("freq", freq);
  preferences.putShort("type", currentSonde);
  preferences.end();
}

void readPrefs() {
  preferences.begin("TLS", true);
  freq = preferences.getInt("freq", 403000);
  currentSonde = preferences.getShort("type", 0);
  preferences.end();
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  VBattInit();
  if (BUTTON != GPIO_NUM_NC) {
    pinMode(BUTTON, INPUT);
    button.enableRepeat(false);
    button.enableLongPress(true);
    button.setLongPressTime(1000);
  }
  readPrefs();
  bip();
  flash();
  // Melody melody = MelodyFactory.load("Nice Melody", timeUnit, notes, nNotes);
  // player.playAsync(melody);

  initDisplay();
  delay(1000);
  initRadio();
  BLEInit();

  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(running, &ota_state);

  if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
    Serial.println("Confermo partizione OK");
    esp_ota_mark_app_valid_cancel_rollback();
  }
}

void loop() {
  static uint64_t tLastDisplay = 0, tLastBLELoop = 0;

////////////////////////////////////////////////////////
  // static bool oldClk=false;
  // static int n=0;
  // bool clk=digitalRead(15)==HIGH;
  // if (clk) {
  //   if (!oldClk) {
  //     oldClk=true;
  //     Serial.print(digitalRead(13)==HIGH?'1':'0');
  //     if (++n==100) {
  //       n=0;
  //       Serial.println();
  //     }
  //   }
  // }
  // else
  //   oldClk=false;
  // return;
////////////////////////////////////////////////////////

  if (tLastBLELoop == 0 || millis() - tLastBLELoop > 500) {
    tLastBLELoop = millis();
    BLELoop();
  }
  if (loopRadio()) {
    bip(150, constrain(map(packet.alt, 0, 40000, 200, 9000), 200, 9000));
    flash(10);
  }
  if (tLastDisplay == 0 || millis() - tLastDisplay > 1000) {
    if (otaRunning) {
      displayOTA();
    } else {
      tLastDisplay = millis();
      batt = getBattLevel();
      updateDisplay(freq, sondes[currentSonde]->name, mute, connected, packet.serial, batt, rssi, packet.lat, packet.lng, packet.alt);
      BLENotifyBatt();
      BLENotifyRSSI();
    }
  }
  if (BUTTON != GPIO_NUM_NC)
    switch (button.read()) {
      case MD_KeySwitch::KS_PRESS:
        break;
      case MD_KeySwitch::KS_LONGPRESS:
        sleepRadio();
        esp_sleep_enable_ext0_wakeup(BUTTON, 0);
        displayOff();

        while (digitalRead(BUTTON) == LOW)
          ;
        delay(100);
        esp_deep_sleep_start();
        break;
    }
}