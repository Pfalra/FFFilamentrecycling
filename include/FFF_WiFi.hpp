#ifndef FFF_WIFI_H
#define FFF_WIFI_H

#include <FFF_Types.hpp>
#include <Arduino.h>


/* TASK HANDLES */
extern TaskHandle_t UdpTaskHandle;
extern TaskHandle_t InitWiFiTaskHandle;

void FFF_initializeWiFi(const char* ssid, const char* pwd);

FFF_ModStatus FFF_getWiFiStatus();

bool FFF_reconnectWiFi();

void FFF_reportWiFiEvent();

/******************************************/
/* UDP */
/******************************************/
void FFF_Udp_init();

#endif