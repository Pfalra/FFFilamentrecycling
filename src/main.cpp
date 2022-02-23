#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <PID_v1.h>
#include <WiFiUDP.h>

/* FFF includes */
#include <FFF_Types.h>
#include <FFF_WiFi.h>
#include <FFF_Sd.h>
#include <FFF_Adc.h>
#include <FFF_Oled.h>
#include <FFF_Graphics.h>
#include <FFF_Credentials.h>
#include <FFF_Settings.h>

#pragma region TaskControlFunctions
void CreateAppInitTasks();
void CreateAppTasks();

void ResumeAppTasks();
void DeleteAppTasks();
void SuspendAppTasks();
#pragma endregion

#pragma region InitFunctions
void InitializeWiFi(void *param);
void InitializeOther(void *param);
void InitializeSD(void *param);
#pragma endregion

#pragma region HandlerFunctions
void handleUdp(void *param);
void handleOled(void *param);
void handleSdLog(void *param);
void handleMotorPID(void *param);
void handleTempPID(void *param);
void handleADC(void *param);
void handleFPGA(void *param);
#pragma endregion

/* TASK HANDLES */
#pragma region TaskHandles
TaskHandle_t initWiFiHandle;
TaskHandle_t initSDHandle;
TaskHandle_t initOtherHandle;
TaskHandle_t UdpTaskHandle;
TaskHandle_t OledTaskHandle;
TaskHandle_t SdLogTaskHandle;
TaskHandle_t PIDDiameterTaskHandle;
TaskHandle_t PIDTemperatureTaskHandle;
TaskHandle_t ADCTaskHandle;
TaskHandle_t StepperTaskHandle;
TaskHandle_t FPGATaskHandle;
#pragma endregion

// Prototypes
#pragma region TempFunctionPrototypes
void FFF_Stepper_init();
void FFF_Stepper_disable();
void FFF_Stepper_enable();

void FFF_Udp_init();
#pragma endregion

// Stepper
#pragma region Stepper
typedef struct
{
  uint8_t id;
  uint8_t stepPin;
  uint8_t microsteps;
  double targetSpeed;
} FFF_Stepper;

FFF_Stepper ExtruderStepper;
FFF_Stepper PullStepper;
FFF_Stepper WinchStepper;
#pragma endregion

#pragma region PID
double filDiameterMm = 0.0;
double targetDiameterMm = 1.75;
double pwmFrequency = 0.0;

double kp_mot = 0.0;
double ki_mot = 0.0;
double kd_mot = 0.0;

PID pidMotControl(&filDiameterMm, &pwmFrequency, &targetDiameterMm, kp_mot, ki_mot, kd_mot, AUTOMATIC);

// Heater
double hotendTemp = 0.0;
double pwmDutyCycle = 0.0;
double targetTemp = 215.0;

double kp_temp = 24.4;
double ki_temp = 1.4;
double kd_temp = 106.8;

PID pidTempControl(&hotendTemp, &pwmDutyCycle, &targetTemp, kp_temp, ki_temp, kd_temp, AUTOMATIC);
#pragma endregion

#pragma region UDP

WiFiUDP udpConn;
char udpRecPBuf[UDP_PBUF_SIZE];
char udpSndPBuf[UDP_PBUF_SIZE];

#pragma endregion

typedef enum
{
  APP_STOPPED,
  APP_RUNNING,
  APP_PAUSED
} AppStatus;

AppStatus gAppStatus = APP_STOPPED;
bool startApp = false;
bool stopApp = false;
bool pauseApp = false;
uint8_t countApp = 0;



void setup()
{
  /* Start Serial communication */
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println();
  Serial.println("FFF Device starting up...");
  vTaskDelay(200);

  /* Initialize OLED */
  FFF_Oled_init();
  vTaskDelay(1000);

  /* Create tasks for initialization */
  CreateAppInitTasks();

  /* Create Tasks for application */
  CreateAppTasks();
}

void loop()
{
  if (stopApp)
  {
    // Stop Logging

    // Disable Heater

    DeleteAppTasks();
    gAppStatus = APP_STOPPED;
    stopApp = false;
  }

  if (startApp)
  {
    if (gAppStatus == APP_STOPPED)
    {
      CreateAppTasks();
    }
    else
    {
      ResumeAppTasks();
    }

    gAppStatus = APP_RUNNING;
    startApp = false;
  }

  if (pauseApp)
  {
    SuspendAppTasks();
    pauseApp = false;
  }
}

/******************************************/
/* Task Control */
/******************************************/

void ResumeAppTasks()
{
  vTaskResume(SdLogTaskHandle);
  vTaskResume(PIDDiameterTaskHandle);
}

void DeleteAppTasks()
{
  vTaskDelete(SdLogTaskHandle);
  vTaskDelete(PIDDiameterTaskHandle);
  vTaskDelete(PIDTemperatureTaskHandle);
  vTaskDelete(ADCTaskHandle);
}

void SuspendAppTasks()
{
  vTaskSuspend(SdLogTaskHandle);
  vTaskSuspend(PIDDiameterTaskHandle);
}

void CreateAppTasks()
{
  xTaskCreate(
      handleOled,     // Function that should be called
      "OLED Handler", // Name of the task (for debugging)
      8192,           // Stack size (bytes)
      NULL,           // Parameter to pass
      OLED_TASK_PRIO, // Task priority
      &OledTaskHandle);

  xTaskCreate(
      handleSdLog,         // Function that should be called
      "SD Logger Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      SD_LOG_TASK_PRIO,    // Task priority
      &SdLogTaskHandle);

  xTaskCreate(
      handleMotorPID,      // Function that should be called
      "PID Motor Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      3,                   // Task priority
      &PIDDiameterTaskHandle);

  xTaskCreate(
      handleTempPID,       // Function that should be called
      "PID Motor Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      3,                   // Task priority
      &PIDTemperatureTaskHandle);

  xTaskCreate(
      handleADC,     // Function that should be called
      "ADC Handler", // Name of the task (for debugging)
      8192,          // Stack size (bytes)
      NULL,          // Parameter to pass
      3,             // Task priority
      &ADCTaskHandle);
}

void CreateAppInitTasks()
{
  /* Create tasks for further initializations */
  xTaskCreate(
      InitializeWiFi,    // Function that should be called
      "Initialize WiFi", // Name of the task (for debugging)
      16384,              // Stack size (bytes)
      NULL,              // Parameter to pass
      3,                 // Task priority
      &initWiFiHandle);

  xTaskCreate(
      InitializeOther,    // Function that should be called
      "Initialize Other", // Name of the task (for debugging)
      8192,               // Stack size (bytes)
      NULL,               // Parameter to pass
      2,                  // Task priority
      &initOtherHandle);

  xTaskCreate(
      InitializeSD,    // Function that should be called
      "Initialize SD", // Name of the task (for debugging)
      16384,            // Stack size (bytes)
      NULL,            // Parameter to pass
      2,               // Task priority
      &initSDHandle);

  xTaskCreate(
      handleUdp,     // Function that should be called
      "UDP Handler", // Name of the task (for debugging)
      16384,          // Stack size (bytes)
      NULL,          // Parameter to pass
      UDP_TASK_PRIO, // Task priority
      &UdpTaskHandle);
}

/******************************************/
/* Init Task Functions */
/******************************************/
void InitializeWiFi(void *param)
{
  /* Start WiFi */
  Serial.println("I>Starting WiFi.");
  FFF_initializeWiFi(MY_SSID, MY_PWD);
  Serial.println("I>WiFi init done");
  /* Make task only execute once */
  vTaskDelete(initWiFiHandle);
}

void InitializeOther(void *param)
{
  Serial.println("\r\nI>Initializing other hardware/software");
  /* Initialize ADC */
  FFF_Adc_init();

  // Take first value from ADC
  hotendTemp = FFF_Adc_readTemp();

  /* Initialize Steppers */
  FFF_Stepper_init();

  /* Initialize UDP */
  FFF_Udp_init();

  Serial.println("\r\nI>Init done");
  vTaskDelete(initOtherHandle);
}

void InitializeSD(void *param)
{
  /* Initialize SD */
  while (!FFF_SD_Init(SD_OVER_SPI))
  {
    vTaskDelay(500);
  };

  Serial.println("I>SD init done");
  vTaskDelete(initSDHandle);
}

/******************************************/
/* Application Task Functions */
/******************************************/

void handleUdp(void *param)
{
  while (1)
  {
    Serial.println("Reading Packet:");
    int packetSize = udpConn.parsePacket();

    if (packetSize)
    {
      int len = udpConn.read(udpRecPBuf, UDP_PBUF_SIZE);
      if (len > 0)
      {
        udpRecPBuf[len] = 0;
      }

      Serial.println("Received:");
      Serial.println(udpRecPBuf);

      if (strncmp(udpRecPBuf, UDP_APP_START_CMD, sizeof(UDP_APP_START_CMD)) == 0)
      {
        startApp = TRUE;
        udpConn.write((uint8_t*) UDP_CONFIRM, (size_t) sizeof(UDP_CONFIRM));
        udpConn.write((uint8_t*) UDP_START_CONFIRM, (size_t) sizeof(UDP_START_CONFIRM));
      } else if (strncmp(udpRecPBuf, UDP_APP_STOP_CMD, sizeof(UDP_APP_STOP_CMD)) == 0)
      {
        stopApp = TRUE;
        udpConn.write((uint8_t*) UDP_CONFIRM, (size_t) sizeof(UDP_CONFIRM));
        udpConn.write((uint8_t*) UDP_STOP_CONFIRM, (size_t) sizeof(UDP_STOP_CONFIRM));
      } else if (strncmp(udpRecPBuf, UDP_APP_PAUSE_CMD, sizeof(UDP_APP_PAUSE_CMD)) == 0)
      {
        pauseApp = TRUE;
        udpConn.write((uint8_t*) UDP_CONFIRM, (size_t) sizeof(UDP_CONFIRM));
        udpConn.write((uint8_t*) UDP_PAUSE_CONFIRM, (size_t) sizeof(UDP_PAUSE_CONFIRM));
      } else 
      {

      }

      // Confirm reception
      udpConn.beginPacket(udpConn.remoteIP(), udpConn.remotePort());
      udpConn.write((uint8_t*) UDP_CONFIRM, (size_t) sizeof(UDP_CONFIRM));
      udpConn.endPacket();
    }

    vTaskDelay(UDP_HANDLE_INTERVAL_MS);
  }
}

void handleOled(void *param)
{
  while (1)
  {
    
    vTaskDelay(OLED_UPDATE_INTERVAL_MS);
  }
}

void handleSdLog(void *param)
{

  while (1)
  {

    vTaskDelay(SD_LOG_INTERVAL_MS);
  }
}

void handleTempPID(void *param)
{
  while (1)
  {

    vTaskDelay(PID_TEMP_INTERVAL_MS);
  }
}

void handleMotorPID(void *param)
{
  while (1)
  {

    vTaskDelay(PID_DIAMETER_INTERVAL_MS);
  }
}

void handleADC(void *param)
{
  while (1)
  {
    hotendTemp = FFF_Adc_readTemp();
    vTaskDelay(ADC_SAMPLE_INTERVAL_MS);
  }
}

void handleFPGA(void *param)
{
  while (1)
  {

    vTaskDelay(FPGA_CALCULATE_DIAMETER_INTERVAL_MS);
  }
}

/******************************************/
/* Stepper */
/******************************************/
void FFF_Stepper_init()
{
  pinMode(STEPPER_EN_PIN, OUTPUT);
  FFF_Stepper_disable();
  pinMode(EXTRUDER_STEP_PIN, OUTPUT);
  pinMode(PULLER_STEP_PIN, OUTPUT);
  pinMode(WINCH_STEP_PIN, OUTPUT);
  ExtruderStepper.id = 0;
  ExtruderStepper.targetSpeed = 0;
  ExtruderStepper.stepPin = EXTRUDER_STEP_PIN;
  ExtruderStepper.microsteps = 16;

  PullStepper.id = 1;
  PullStepper.targetSpeed = 0;
  PullStepper.stepPin = PULLER_STEP_PIN;
  PullStepper.microsteps = 16;

  WinchStepper.id = 2;
  WinchStepper.targetSpeed = 0;
  WinchStepper.stepPin = WINCH_STEP_PIN;
  WinchStepper.microsteps = 16;

  ledcAttachPin(EXTRUDER_STEP_PIN, ExtruderStepper.id);
  ledcAttachPin(PULLER_STEP_PIN, PullStepper.id);
  ledcAttachPin(WINCH_STEP_PIN, WinchStepper.id);
}

void FFF_Stepper_enable()
{
  digitalWrite(STEPPER_EN_PIN, HIGH);
}

void FFF_Stepper_disable()
{
  digitalWrite(STEPPER_EN_PIN, LOW);
}

/******************************************/
/* UDP */
/******************************************/
void FFF_Udp_init()
{
  /* Maybe use AsyncUdp in the future */
  udpConn.begin(UDP_PORT);
}
