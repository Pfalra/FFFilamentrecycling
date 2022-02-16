#ifndef FFF_WIFI_H
#define FFF_WIFI_H

#include <FFF_Types.h>

void FFF_initializeWiFi(const char* ssid, const char* pwd);

FFF_ModStatus FFF_getWiFiStatus();

bool FFF_reconnectWiFi();

void FFF_reportWiFiEvent();

#endif