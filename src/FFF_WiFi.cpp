#include <FFF_WiFi.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <FFF_Types.h>

// Prototypes 
void FFF_onWiFiEvent(WiFiEvent_t event);



void FFF_initializeWiFi(const char* ssid, const char* pwd)
{
    /* Initialize WiFi */
    WiFi.mode(WIFI_MODE_AP);
    WiFi.onEvent(FFF_onWiFiEvent);
    WiFi.softAP(ssid, pwd, 1, false, 1); // Only one connection
    Serial.print("WiFi SSID: ");
    Serial.println(ssid);
    Serial.print("WiFi PWD: ");
    Serial.println(pwd);

    Serial.println('\n');
    Serial.println("> SoftAP up and running!");
    Serial.print("IP address: \t");
    Serial.println(WiFi.softAPIP());
}



FFF_ModStatus FFF_getWiFiStatus()
{
    /* Check if the modem is in sleep mode */
    bool sleep = WiFi.getSleep();

    /* Check RSSI */
    int32_t rssi = WiFi.RSSI();

    if (Serial.available())
    {
        Serial.printf("WiFi sleep status: %i", sleep);
        Serial.printf("WiFi RSSI: %d", rssi);
    }

    return MOD_FUNCTIONAL;
}

bool FFF_reconnectWiFi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.reconnect();
    }
    return true;
}

void FFF_reportWiFiEvent()
{

}


void FFF_onWiFiEvent(WiFiEvent_t event)
{
  switch (event) {
 
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 Connected to WiFi Network");
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.println("ESP32 soft AP started");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("Station connected to ESP32 soft AP");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("Station disconnected from ESP32 soft AP");
      break;
    default: break;
  }
}
