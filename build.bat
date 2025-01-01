@echo off
echo [1m************ compilazione per Heltec ************[0m
arduino-cli compile -e --fqbn Heltec-esp32:esp32:heltec_wifi_lora_32_V3
echo [1m************ compilazione per TTGO ************[0m
arduino-cli compile -e --fqbn esp32:esp32:ttgo-lora32
echo .
echo .
echo .
echo ---------------------------------------------
echo [92mAggiorna i file JSON!![0m
echo [96mUsa 'ftp -s:ftp.txt' per caricare i bin[0m
echo ---------------------------------------------
