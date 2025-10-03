//#include <main.h>
#ifndef LIBAPRSESP_H
#define LIBAPRSESP_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include "FIFO.h"
#include "CRC-CCIT.h"
#include "HDLC.h"
#include "AX25.h"

#include "../../include/config.h"

bool APRS_init(Configuration *cfg);
#ifdef RF2
bool APRS_init2(Configuration *cfg);
#endif
bool APRS_poll(void);

// void APRS_setCallsign(char *call, int ssid);
// void APRS_setDestination(char *call, int ssid);
// void APRS_setMessageDestination(char *call, int ssid);
// void APRS_setPath1(char *call, int ssid);
// void APRS_setPath2(char *call, int ssid);

// void APRS_setPreamble(unsigned long pre);
// void APRS_setTail(unsigned long tail);
// void APRS_useAlternateSymbolTable(bool use);
// void APRS_setSymbol(char sym);

// void APRS_setLat(char *lat);
// void APRS_setLon(char *lon);
// void APRS_setPower(int s);
// void APRS_setHeight(int s);
// void APRS_setGain(int s);
// void APRS_setDirectivity(int s);

// void APRS_sendPkt(void *_buffer, size_t length);
// void APRS_sendLoc(void *_buffer, size_t length);
// void APRS_sendMsg(void *_buffer, size_t length);
// void APRS_msgRetry();

//void APRS_printSettings();
void APRS_sendTNC2Pkt(uint8_t *raw,size_t length);

//void APRS_setFreq(float freq);

// void base91encode(long ltemp,char *s);
// long semicircles(char *degstr, char hemi);

int APRS_getTNC2Pkt(uint8_t *raw,String info);

int freeMemory();
void radioSleep();

void taskADDFifo(void *pvParameters);

#endif