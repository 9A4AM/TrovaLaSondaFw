#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <esp_mac.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <arduino_base64.hpp>
#include "Ble.h"
#include "TrovaLaSondaFw.h"
#include "radio.h"

#define SERVICE_UUID      "177fba78-7843-40a6-801b-a4cd8d7f5c11"

#define PACKET_UUID       "4dee4a71-2e7e-4018-9656-b60f1e562047"
#define BATT_UUID         "4578ee77-f50f-4584-b59c-46264c56d949"
#define RSSI_UUID         "e482dfeb-774f-4f8b-8eea-87a752326fbd"
#define TYPEFREQ_UUID     "66bf4d7f-2b21-468d-8dce-b241c7447cc6"
#define MUTE_UUID         "a8b47819-eb1a-4b5c-8873-6258ddfe8055"
#define VERSION_UUID      "2bc3ed96-a00a-4c9a-84af-7e1283835d71"

#define OTA_SERVICE_UUID "0410c8a6-2c9c-4d6a-9f0e-4bc0ff7e0f7e"

#define OTA_TX_UUID       "63fa4cbe-3a81-463f-aa84-049dea77a209"
#define OTA_RX_UUID       "4f0227ff-dca1-4484-99f9-155cba7f3d86"

BLEServer *pServer = NULL;
BLECharacteristic *pPacketChar, *pBattChar, *pRSSIChar, *pTypeFreqChar, *pMuteChar, *pVersionChr, *pOtaTxChar, *pOtaRxChar;
bool wasConnected = false;
esp_ota_handle_t handleOta;
esp_bd_addr_t connected_addr;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    connected = true;
    memcpy(connected_addr, param->connect.remote_bda, sizeof connected_addr);
    BLEAddress a(param->connect.remote_bda);
    Serial.printf("connected to %s\n", a.toString().c_str());
  };

  void onDisconnect(BLEServer *pServer) {
    connected = false;
    Serial.println("disconnected");
  }

  void onMtuChanged(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    Serial.printf("onMtuChanged %d\n", param->mtu.mtu);
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    bool restart = false;
    if (pCharacteristic->getUUID().equals(BLEUUID(OTA_TX_UUID))) {
      int nLen = pCharacteristic->getLength();
      if (!otaRunning) {
        if (nLen != 8) return;
        pServer->updateConnParams(connected_addr, 9, 9, 0, 50);
        uint8_t *p = pCharacteristic->getData();
        if (p[0] != 0x53 || p[1] != 0x48 || p[2] != 0 || p[3] != 0)
          Serial.printf("Numero magico non corrisponde! (%02X %02X %02X %02X)\n", p[0], p[1], p[2], p[3]);
        otaLength = p[4] + 256 * (p[5] + 256 * (p[6] + 256 * p[7]));
        otaProgress = 0;
        Serial.printf("Lunghezza nuovo firmware : %d bytes\n", otaLength);
        otaRunning = true;
        const esp_partition_t *running = esp_ota_get_running_partition();
        const esp_partition_t *next = esp_ota_get_next_update_partition(running);
        esp_err_t err = esp_ota_begin(next, otaLength, &handleOta);
        if (err != ESP_OK) {
          Serial.printf("Errore esp_ota_begin %d\n", err);
          otaErr = err;
        }
      } else {
        if (nLen == 0) {
          restart = true;
        } else {
          esp_err_t err = esp_ota_write(handleOta, pCharacteristic->getData(), nLen);
          otaProgress += nLen;
          if (err != ESP_OK) {
            Serial.printf("Errore esp_ota_write %s\n", esp_err_to_name(err));
            otaErr = err;
          } else {
            Serial.printf("Ota progress: (%d) %d%%\n", nLen, (100 * otaProgress) / otaLength);

            if (otaProgress == otaLength) {
              esp_ota_end(handleOta);
              const esp_partition_t *running = esp_ota_get_running_partition(),
                                    *next = esp_ota_get_next_update_partition(running);

              esp_ota_set_boot_partition(next);
              restart = true;
            }
          }
        }
      }

      pOtaRxChar->setValue(otaErr);
      pOtaRxChar->notify();

      if (restart) {
        delay(1000);
        ESP.restart();
      }
    } else if (pCharacteristic == pTypeFreqChar) {
      uint8_t *pVal = pCharacteristic->getData();
      currentSonde = pVal[0];
      freq = pVal[1] + 256 * (pVal[2] + 256 * (pVal[3] + 256 * pVal[4]));

      Serial.printf("TypeFreq: %d %d\n", currentSonde, freq);
      savePrefs();
      initRadio();
    } else if (pCharacteristic == pMuteChar) {
      Serial.printf("Mute: %d\n", mute = *(int *)pCharacteristic->getData());
      if (!mute) bip(80, 440);
    }
  }
};

MyCallbacks myCallbacks;
MyServerCallbacks myServerCallbacks;

void createCharacteristic(BLEService *pService, const char *desc, const char *uuid, BLECharacteristic **ppChar, bool writable = false, bool notify = true) {
  uint32_t flags = BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_INDICATE;
  if (writable) flags |= BLECharacteristic::PROPERTY_WRITE;
  if (notify) flags |= BLECharacteristic::PROPERTY_NOTIFY;
  *ppChar = pService->createCharacteristic(uuid, flags);

  // Creates BLE Descriptor 0x2902: Client Characteristic Configuration Descriptor (CCCD)
  (*ppChar)->addDescriptor(new BLE2902());
  // Adds also the Characteristic User Description - 0x2901 descriptor
  BLE2901 *descriptor_2901 = new BLE2901();
  descriptor_2901->setDescription(desc);
  if (!writable)
    descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ);  // enforce read only - default is Read|Write
  (*ppChar)->addDescriptor(descriptor_2901);
  if (writable)
    (*ppChar)->setCallbacks(&myCallbacks);
}

void BLEInit() {
  char s[32], b64[13];
  uint8_t add[8];
  esp_efuse_mac_get_default(add);
  base64::encode(add,8,b64);
  snprintf(s, sizeof s, "TrovaLaSonda%.11s", b64); //removed trailing b64 padding
  //snprintf(s, sizeof s, "TrovaLaSonda%02X%02X%02X%02X%02X%02X", add[0], add[1], add[2], add[3], add[4], add[5]);
  Serial.printf("MAC address: %02X%02X%02X%02X%02X%02X%02X%02X\n", add[0], add[1], add[2], add[3], add[4], add[5], add[6], add[7]);
  Serial.printf("Nome: %s\n", s);
  BLEDevice::init(s);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&myServerCallbacks);

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 90);

  createCharacteristic(pService, "Packet", PACKET_UUID, &pPacketChar);
  createCharacteristic(pService, "Battery", BATT_UUID, &pBattChar);
  createCharacteristic(pService, "RSSI", RSSI_UUID, &pRSSIChar);
  createCharacteristic(pService, "TypeFreq", TYPEFREQ_UUID, &pTypeFreqChar, true, false);
  createCharacteristic(pService, "Mute", MUTE_UUID, &pMuteChar, true, false);
  createCharacteristic(pService, "Version", VERSION_UUID, &pVersionChr, false, false);

  uint8_t buffer[] = { currentSonde, freq, freq >> 8, freq >> 16, freq >> 24 };
  pTypeFreqChar->setValue(buffer, sizeof buffer);

  pMuteChar->setValue(mute);
  pVersionChr->setValue(String(platform)+"/"+String(version));
  pService->start();

  pService = pServer->createService(BLEUUID(OTA_SERVICE_UUID), 90);
  createCharacteristic(pService, "OTA_TX", OTA_TX_UUID, &pOtaTxChar, true, false);
  createCharacteristic(pService, "OTA_RX", OTA_RX_UUID, &pOtaRxChar);
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->addServiceUUID(OTA_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
}

void BLENotifyPacket() {
  if (!connected || otaRunning) return;
  pPacketChar->setValue((uint8_t*)&packet,sizeof packet);
  pPacketChar->notify();
}

void BLENotifyBatt() {
  if (!connected || otaRunning) return;
  pBattChar->setValue(batt);
  pBattChar->notify();
}

void BLENotifyRSSI() {
  if (!connected || otaRunning) return;
  pRSSIChar->setValue(rssi);
  pRSSIChar->notify();
}

void BLELoop() {
  static bool startAdvertising = false;
  if (startAdvertising)
    pServer->startAdvertising();

  if (connected && !wasConnected)
    wasConnected = connected;
  if (!connected && wasConnected) {
    startAdvertising = true;
    wasConnected = connected;
  }
}
