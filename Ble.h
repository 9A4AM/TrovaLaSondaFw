#ifndef __BLE_H__
#define __BLE_H__

void BLELoop();
void BLEInit();

void BLENotifyLat();
void BLENotifyLon();
void BLENotifyAlt();
void BLENotifyBatt();
void BLENotifyRSSI();
void BLENotifySerial();
void BLENotifyBurstKill();
#endif