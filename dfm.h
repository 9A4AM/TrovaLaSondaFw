#ifndef __DFM_H___
#define __DFM_H___

#define NPACKETS 3
#define DFM_PACKET_LENGTH 66

enum datType { SEQ = 0,
               TIME = 1,
               LAT = 2,
               LON = 3,
               ALT = 4,
               DATE = 8 };

extern Sonde dfm;

 #endif