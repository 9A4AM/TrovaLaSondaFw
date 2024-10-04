#ifndef __DISP_H__
#define __DISP_H__
void initDisplay();
void displayOff();
void updateDisplay(uint32_t freq, const char* type, bool mute, bool connected, const char *ser = NULL, int bat = 0, int rssi = 0, float lat = 0, float lon = 0, float alt = 0, int frame = 0);
#endif