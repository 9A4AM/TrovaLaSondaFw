#ifndef __RADIO_H__
#define __RADIO_H__
#ifdef SX126X
#include "sx126x.h"
#include "sx126x_regs.h"
#include "sx126x_hal.h"
#include "sx126x_long_pkt.h"

sx126x_gfsk_preamble_detector_t getPreambleLength(unsigned lengthInBytes);
sx126x_gfsk_bw_t getBandwidth(unsigned bandwidth);
#else
#ifdef SX1278
#include <sx1278.h>
#define SX127X_CRYSTAL_FREQ 32000000UL
#endif
#endif
void initRadio();
bool loopRadio();
void sleepRadio();
#endif