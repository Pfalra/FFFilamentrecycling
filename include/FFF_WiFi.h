#ifndef FFF_WIFI_H
#define FFF_WIFI_H

#include <FFF_Types.h>
#include <Arduino.h>


TaskHandle_t* FFF_WiFi_getInitTaskHandle();
TaskHandle_t* FFF_WiFi_getUdpTaskHandle();

void FFF_initializeWiFi(const char* ssid, const char* pwd);

FFF_ModStatus FFF_getWiFiStatus();

bool FFF_reconnectWiFi();

void FFF_reportWiFiEvent();

/******************************************/
/* UDP */
/******************************************/
void FFF_Udp_init();

#endif