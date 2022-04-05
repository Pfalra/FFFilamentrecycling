#include <FFF_WiFi.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <FFF_Types.h>
#include <FFF_Settings.h>
#include <FFF_Credentials.h>
#include <FFF_Rtos.h>
#include <FFF_Uart.h>


/* TASK HANDLES */
TaskHandle_t UdpTaskHandle;
TaskHandle_t InitWiFiTaskHandle;

// Prototypes 
void FFF_onWiFiEvent(WiFiEvent_t event);

//UDP
WiFiUDP udpConn;
char udpRecPBuf[UDP_PBUF_SIZE];
char udpSndPBuf[UDP_PBUF_SIZE];

void FFF_WiFi_init()
{
  /* Start WiFi */
  Serial.println("I>Starting WiFi.");
  FFF_initializeWiFi(MY_SSID, MY_PWD);
  Serial.println("I>WiFi init done");

  FFF_Udp_init();

  xTaskCreate(
    TASK_handleUdp,     // Function that should be called
    "UDP Handler", // Name of the task (for debugging)
    16384,          // Stack size (bytes)
    NULL,          // Parameter to pass
    UDP_TASK_PRIO, // Task priority
    &UdpTaskHandle);

}


void TASK_InitializeWiFi(void *param)
{
  FFF_WiFi_init();
  /* Make task only execute once */
  vTaskDelete(InitWiFiTaskHandle);
}


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
      FFF_Uart_activateInterruptRX_Uart2();
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("Station connected to ESP32 soft AP");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("Station disconnected from ESP32 soft AP");
      FFF_Uart_deactivateInterruptRX_Uart2();
      break;
    default: break;
  }
}


/******************************************/
/* UDP */
/******************************************/
void FFF_Udp_init()
{
  /* Maybe use AsyncUdp in the future */
  udpConn.begin(UDP_PORT);
}


void TASK_handleUdp(void *param)
{
  Serial.println("Reading UDP packets...");
  while (1)
  {
    int packetSize = udpConn.parsePacket();

    if (packetSize)
    {
      int len = udpConn.read(udpRecPBuf, UDP_PBUF_SIZE);

      String udpStr = String(udpRecPBuf);

      if (len > 0)
      {
        udpRecPBuf[len] = 0;
      }

      Serial.println("Received:");
      Serial.println(udpRecPBuf);

      if (udpStr == UDP_APP_START_CMD)
      {
        FFF_Rtos_startApp();
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_START_CONFIRM);
      } 
      else if (udpStr == UDP_APP_STOP_CMD)
      {
        FFF_Rtos_stopApp();
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_STOP_CONFIRM);
      } 
      else if (udpStr == UDP_APP_PAUSE_CMD)
      {
        FFF_Rtos_pauseApp();
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_PAUSE_CONFIRM);
      } 
      else if (udpStr == UDP_APP_LIST_CMD)
      {
        udpConn.println(UDP_CONFIRM);
        char fileStr[MAX_OUT_STRING_LEN];
        // FFF_SD_getFileList(fileStr, MAX_OUT_STRING_LEN);

        const char heading[] = "--- FILE LIST ----";
        Serial.println(heading);
        Serial.println(fileStr);
        udpConn.println(heading);
        udpConn.println(fileStr);

      } 
      else if (udpStr == UDP_SET_EXTRUDER_SPEED_CMD)
      {
        

      } 
      else 
      {
        Serial.println("I>Unknown command");

      }

      // Confirm reception
      udpConn.beginPacket(udpConn.remoteIP(), udpConn.remotePort());
      udpConn.println(UDP_CONFIRM);
      udpConn.endPacket();
    }

    vTaskDelay(UDP_HANDLE_INTERVAL_MS);
  }
}


TaskHandle_t* FFF_WiFi_getInitTaskHandle()
{
  return &InitWiFiTaskHandle;
}

TaskHandle_t* FFF_WiFi_getUdpTaskHandle()
{
  return &UdpTaskHandle;
}
