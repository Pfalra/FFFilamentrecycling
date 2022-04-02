#ifndef FFF_WIFI_H
#define FFF_WIFI_H

#include <FFF_Types.h>
#include <FreeRTOS.h>

/* TASK HANDLES */
TaskHandle_t UdpTaskHandle;
TaskHandle_t initWiFiHandle;


void FFF_initializeWiFi(const char* ssid, const char* pwd);

FFF_ModStatus FFF_getWiFiStatus();

bool FFF_reconnectWiFi();

void FFF_reportWiFiEvent();

/******************************************/
/* UDP */
/******************************************/
void FFF_Udp_init();

#endif