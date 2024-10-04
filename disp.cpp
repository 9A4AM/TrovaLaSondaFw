#include <HT_SSD1306Wire.h>
#include "disp.h"

const int BATT_W = 14, BATT_H = 30, BATT_X = 113, BATT_Y = 28,
          BATT_CORNER_RADIUS = 4,
          BATT_PLUS_W = 6, BATT_PLUS_H = 3, BT_X = 50, BT_Y = 0,
          SPEAKER_X = 94, SPEAKER_Y = 44;
const int logo_width = 32, logo_height = 32;
static uint8_t logo_bits[] = {
  0x00, 0xFF, 0x01, 0x00, 0xC0, 0xFF, 0x07, 0x00, 0xE0, 0xFF, 0x1F, 0x00,
  0xF0, 0xFF, 0x3F, 0x00, 0xF8, 0xFF, 0x7F, 0x00, 0xFC, 0xFF, 0xFF, 0x00,
  0xFE, 0xFF, 0xFF, 0x00, 0xFE, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0x01,
  0xEF, 0xCF, 0xCF, 0x01, 0x83, 0x83, 0x07, 0x03, 0x01, 0x03, 0x03, 0x03,
  0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x06, 0x03, 0x81, 0x01,
  0x0C, 0x82, 0xC1, 0x00, 0x18, 0xE6, 0x7F, 0x00, 0x30, 0xF6, 0xFF, 0x07,
  0xE0, 0x1F, 0x00, 0x0C, 0xF0, 0xFC, 0x1F, 0x38, 0x90, 0x8D, 0x3F, 0x60,
  0xF0, 0xFF, 0x63, 0x00, 0xF8, 0xFF, 0xC3, 0x01, 0x08, 0xCA, 0x00, 0x01,
  0xF8, 0xFF, 0x03, 0x00, 0xF0, 0xFF, 0x03, 0x00, 0x30, 0x30, 0x00, 0x00,
  0xE0, 0xFF, 0x03, 0x00, 0xC0, 0xFF, 0x03, 0x00, 0x00, 0x03, 0x00, 0x78,
  0x00, 0xFE, 0xFF, 0x0F, 0x00, 0xFC, 0xFF, 0x01
};
const int bt_width = 8, bt_height = 15;
static uint8_t bt_bits[] = {
  0x08, 0x18, 0x38, 0x69, 0xCB, 0x6E, 0x3C, 0x18, 0x3C, 0x6E, 0xCB, 0x69,
  0x38, 0x18, 0x08
};
const int mute_width = 5, mute_height = 12;
static uint8_t mute_bits[] = {
  0x00, 0x10, 0x18, 0x1C, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1C, 0x18, 0x10
};
const int speaker_width = 11, speaker_height = 13;
static uint8_t speaker_bits[] = {
  0x80, 0x00, 0x10, 0x01, 0x58, 0x02, 0x9C, 0x04, 0x1F, 0x05, 0x5F, 0x05,
  0x5F, 0x05, 0x5F, 0x05, 0x1F, 0x05, 0x9C, 0x04, 0x58, 0x02, 0x10, 0x01,
  0x80, 0x00
};

static SSD1306Wire display(0x3c, 700000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void initDisplay() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, HIGH);
  display.init();
  //display.flipScreenVertically();
  display.invertDisplay();
  display.drawXbm((128 - logo_width) / 2, 1, logo_width, logo_height, logo_bits);
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(65, 32, "TrovaLaSonda");
  display.drawString(64, 32, "TrovaLaSonda");
  display.drawString(64, 47, "0.0");
  display.drawString(65, 47, "0.0");
  display.display();
  vTaskDelay(2000);
}

void displayOff() {
  display.clear();
  display.display();
  digitalWrite(Vext, HIGH);
}

static void drawBattery(int level) {
  display.fillRect(BATT_X + BATT_W / 2 - BATT_PLUS_W / 2, BATT_Y, BATT_PLUS_W, BATT_PLUS_H - 1);
  display.drawLine(BATT_X + BATT_CORNER_RADIUS, BATT_Y + BATT_PLUS_H, BATT_X + BATT_W - BATT_CORNER_RADIUS, BATT_Y + BATT_PLUS_H);
  display.drawLine(BATT_X + BATT_CORNER_RADIUS, BATT_Y + BATT_H, BATT_X + BATT_W - BATT_CORNER_RADIUS, BATT_Y + BATT_H);
  display.drawLine(BATT_X, BATT_Y + BATT_PLUS_H + BATT_CORNER_RADIUS, BATT_X, BATT_Y + BATT_H - BATT_CORNER_RADIUS);
  display.drawLine(BATT_X + BATT_W, BATT_Y + BATT_PLUS_H + BATT_CORNER_RADIUS, BATT_X + BATT_W, BATT_Y + BATT_H - BATT_CORNER_RADIUS);

  display.drawCircleQuads(BATT_X + BATT_CORNER_RADIUS, BATT_Y + BATT_PLUS_H + BATT_CORNER_RADIUS, BATT_CORNER_RADIUS, 0b10);
  display.drawCircleQuads(BATT_X + BATT_W - BATT_CORNER_RADIUS, BATT_Y + BATT_PLUS_H + BATT_CORNER_RADIUS, BATT_CORNER_RADIUS, 0b1);
  display.drawCircleQuads(BATT_X + BATT_CORNER_RADIUS, BATT_Y + BATT_H - BATT_CORNER_RADIUS, BATT_CORNER_RADIUS, 0b100);
  display.drawCircleQuads(BATT_X + BATT_W - BATT_CORNER_RADIUS, BATT_Y + BATT_H - BATT_CORNER_RADIUS, BATT_CORNER_RADIUS, 0b1000);
  if (level == 0) {
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.setFont(ArialMT_Plain_24);
    display.drawString(BATT_X + BATT_W / 2, BATT_Y + BATT_PLUS_H + (BATT_H - BATT_PLUS_H) / 2, "?");
  } else {
    int h = BATT_H - BATT_PLUS_H - 4;
    if (level > 100) level = 100;
    display.fillRect(BATT_X + 2, BATT_Y + BATT_PLUS_H + 2 + h * (100 - level) / 100, BATT_W - 3, h * level / 100);
  }
}

void updateDisplay(uint32_t freq, const char* type, bool mute, bool connected, const char *ser, int bat, int rssi, float lat, float lon, float alt, int frame) {
  char s[1 + 3 + 1 + 5 + 1];  //sign,integer,dot,decimals,null

  display.normalDisplay();
  display.clear();
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, type);
  display.drawString(1, 0, type);
  snprintf(s, sizeof s, "%.3f", freq / 1000.0);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(127, 0, s);
  display.drawString(126, 0, s);
  if (connected)
    display.drawXbm(BT_X, BT_Y, bt_width, bt_height, bt_bits);
  if (mute)
    display.drawXbm(SPEAKER_X, SPEAKER_Y, mute_width, mute_height, mute_bits);
  else
    display.drawXbm(SPEAKER_X, SPEAKER_Y, speaker_width, speaker_height, speaker_bits);

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if (ser != NULL) {
    display.drawString(0, 18, ser);
    display.drawString(1, 18, ser);
  }
  if (millis() % 6000 < 3000) {
    if (isnan(lat)) lat = 0;
    snprintf(s, sizeof s, "%+.5f", lat);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(84, 33, s);
    if (isnan(lon)) lon = 0;
    snprintf(s, sizeof s, "%+.5f", lon);
    display.drawString(84, 49, s);
  } else {
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 41, "H:");
    if (isnan(alt)) alt = 0;
    snprintf(s, sizeof s, "%dm", (int)alt);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(84, 41, s);
  }
  drawBattery(bat);
  display.setColor(INVERSE);
  display.fillRect(0, 0, 128 + rssi - 1, 16);

  display.display();
}
