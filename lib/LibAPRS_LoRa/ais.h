#ifndef PROTOCOL_AIS_H
#define PROTOCOL_AIS_H

#include <Arduino.h>

void ais_to_nmea (unsigned char *ais, int ais_len, char *nmea, int nmea_size);

int ais_parse (char *sentence, int quiet, char *descr, int descr_size, char *mssi, int mssi_size, double *odlat, double *odlon,
			float *ofknots, float *ofcourse, float *ofalt_m, char *symtab, char *symbol, char *comment, int comment_size);

int ais_check_length (int type, int length);
String ais2aprs(char *info);

#endif