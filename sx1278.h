#ifndef __SX1278_H__
#define __SX1278_H__
#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

#define  RegFIFO                0x00
#define  RegOpMode              0x01
#define  RegBitRateMsb      	0x02
#define  RegBitRateLsb      	0x03
#define  RegFdevMsb             0x04
#define  RegFdevLsb             0x05
#define  RegFreqMsb             0x06
#define  RegFreqMid             0x07
#define  RegFreqLsb         	0x08
#define  RegPaConfig            0x09
#define  RegPaRamp              0x0a
#define  RegOcp                 0x0b
#define  RegLna                 0x0c
#define  RegRxConfig            0x0d
#define  RegRssiConfig      	0x0e
#define  RegRssiCollision 	0x0f
#define  RegRssiThresh      	0x10
#define  RegRssiValue           0x11
#define  RegRxBw                0x12
#define  RegAfcBw               0x13
#define  RegOokPeak             0x14
#define  RegOokFix              0x15
#define  RegOokAvg              0x16
#define  RegAfcFei              0x1a
#define  RegAfcMsb              0x1b
#define  RegAfcLsb              0x1c
#define  RegFeiMsb              0x1d
#define  RegFeiLsb              0x1e
#define  RegPreambleDetect  	0x1f
#define  RegRxTimeout1      	0x20
#define  RegRxTimeout2      	0x21
#define  RegRxTimeout3      	0x22
#define  RegRxDelay             0x23
#define  RegOsc                 0x24
#define  RegPreambleMsb     	0x25
#define  RegPreambleLsb     	0x26
#define  RegSyncConfig      	0x27
#define  RegSyncValue1      	0x28
#define  RegSyncValue2      	0x29
#define  RegSyncValue3      	0x2a
#define  RegSyncValue4      	0x2b
#define  RegSyncValue5      	0x2c
#define  RegSyncValue6      	0x2d
#define  RegSyncValue7      	0x2e
#define  RegSyncValue8      	0x2f
#define  RegPacketConfig1       0x30
#define  RegPacketConfig2       0x31
#define  RegPayloadLength       0x32
#define  RegNodeAdrs            0x33
#define  RegBroadcastAdrs       0x34
#define  RegFifoThresh      	0x35
#define  RegSeqConfig1      	0x36
#define  RegSeqConfig2      	0x37
#define  RegTimerResol      	0x38
#define  RegTimer1Coef      	0x39
#define  RegSyncWord		0x39
#define  RegTimer2Coef      	0x3a
#define  RegImageCal            0x3b
#define  RegTemp                0x3c
#define  RegLowBat              0x3d
#define  RegIrqFlags1           0x3e
#define  RegIrqFlags2           0x3f
#define  RegDioMapping1		0x40
#define  RegDioMapping2		0x41
#define  RegVersion		0x42
#define  RegPllHop		0x44
#define  RegTcxo		0x4b
#define  RegPaDac		0x4d
#define  RegBitRateFrac		0x5d

typedef enum OpMode {
	SLEEP_MODE = 0x00,
	STANDBY_MODE = 0x01,
	TX_MODE = 0x03,
	RX_MODE = 0x05
};

uint8_t readRegister(uint8_t r);
void writeRegister(uint8_t r, uint8_t value);
#endif