/* Framework and external libs */
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <PID_v1.h>
#include <WiFiUDP.h>

/* FFF includes */
#include <FFF_Types.h>
#include <FFF_Settings.h>
#include <FFF_WiFi.h>
#include <FFF_Stepper.h>
#include <FFF_Sd.h>
#include <FFF_Adc.h>
#include <FFF_Oled.h>
#include <FFF_Graphics.h>
#include <FFF_Credentials.h>
#include <FFF_Heater.h>
#include <FFF_Thermistors.h>


//TaskControlFunctions
void CreateAppInitTasks();
void CreateAppTasks();

void ResumeAppTasks();
void DeleteAppTasks();
void SuspendAppTasks();


//InitFunctions
void InitializeWiFi(void *param);
void InitializeOther(void *param);
void InitializeSD(void *param);


//HandlerFunctions
void handleUdp(void *param);
void handleOled(void *param);
void handleLog(void *param);
void handleDiameterMotorPID(void *param);
void handleTempPID(void *param);
void handleADC(void *param);
void handleFPGA(void *param);


/* TASK HANDLES */
//TaskHandles
TaskHandle_t initWiFiHandle;
TaskHandle_t initSDHandle;
TaskHandle_t initOtherHandle;
TaskHandle_t UdpTaskHandle;
TaskHandle_t OledTaskHandle;
TaskHandle_t LogTaskHandle;
TaskHandle_t PIDDiameterTaskHandle;
TaskHandle_t PIDTemperatureTaskHandle;
TaskHandle_t ADCTaskHandle;
TaskHandle_t StepperTaskHandle;
TaskHandle_t FPGATaskHandle;


// Prototypes
void FFF_Udp_init();

double LookupTemperature(double volts, FFF_Lut* lutPtr); 
//Steinhart-Hart

//PID
double filDiameterMm = 0.0;
double targetDiameterMm = TARGET_DIAMETER;
double pwmFrequency = 0.0;

double kp_mot = KP_PULL_MOT_DEFAULT; 
double ki_mot = KI_PULL_MOT_DEFAULT;
double kd_mot = KP_PULL_MOT_DEFAULT;

PID pidMotControl(&filDiameterMm, &pwmFrequency, &targetDiameterMm, kp_mot, ki_mot, kd_mot, AUTOMATIC);

// Heater
double hotendTemp = 0.0;
double pwmDutyCycle = 0.0;
double targetTemp = 215.0;

double kp_temp = 24.4;
double ki_temp = 1.4;
double kd_temp = 106.8;

PID pidTempControl(&hotendTemp, &pwmDutyCycle, &targetTemp, kp_temp, ki_temp, kd_temp, AUTOMATIC);


//UDP
WiFiUDP udpConn;
char udpRecPBuf[UDP_PBUF_SIZE];
char udpSndPBuf[UDP_PBUF_SIZE];


// Thermistor
extern FFF_Lut thermistor0Lut;

const char paramArr[] = "TEMP" DELIMITER \
              "DIAMETER" DELIMITER \
              "PULLMOT_SPD" DELIMITER \
              "EXTMOT_SPD" DELIMITER;

// Log for our application
FFF_Log appLog;


FFF_AppStatus gAppStatus = APP_STOPPED;
bool startApp = false;
bool stopApp = false;
bool pauseApp = false;
uint8_t countApp = 0;

uint32_t logCount = 0;


void setup()
{
  /* Start Serial communication */
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println();
  Serial.println("FFF Device starting up...");
  vTaskDelay(200);

  filDiameterMm = 3.5;

  appLog.paramStr = paramArr;
  appLog.name = "Application Log";
  appLog.isProtected = false;
  appLog.isActive = false;
  
#if DEBUG_LUT_HANDLING==TRUE
    
    // Print the LUT for Debug
    Serial.print("D> First LUT Entries: ");

    for (int i = 0; i < 10; i++)
    {
      Serial.print("TEMP: ");
      Serial.print(thermistor0Lut.tempPtr[i]);
      Serial.print("\t\t");
      Serial.print("RES: ");
      Serial.print(thermistor0Lut.resPtr[i]);
      Serial.println();
    }
    Serial.println("...");
#endif 

  /* Initialize OLED */
  FFF_Oled_init();
  vTaskDelay(500);

  /* Create tasks for initialization */
  CreateAppInitTasks();

  /* Create Tasks for application */
  CreateAppTasks();
}


void loop()
{
  if (stopApp)
  {
    // Disable Heater

    // Disable Steppers


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

    Serial.println("App started");
    gAppStatus = APP_RUNNING;
    startApp = false;
  }

  if (pauseApp)
  {
    SuspendAppTasks();
    Serial.println("App Suspended");
    gAppStatus = APP_PAUSED;
    pauseApp = false;
  }
}

/******************************************/
/* Task Control */
/******************************************/

void ResumeAppTasks()
{
  vTaskResume(PIDDiameterTaskHandle);
}

void DeleteAppTasks()
{
  vTaskDelete(PIDDiameterTaskHandle);
  vTaskDelete(PIDTemperatureTaskHandle);
  vTaskDelete(ADCTaskHandle);
}

void SuspendAppTasks()
{
  vTaskSuspend(LogTaskHandle);
  vTaskSuspend(PIDDiameterTaskHandle);
}

void CreateAppTasks()
{
  xTaskCreate(
      handleLog,         // Function that should be called
      "Logger Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      &appLog,                // Parameter to pass
      LOG_TASK_PRIO,    // Task priority
      &LogTaskHandle);

  xTaskCreate(
      handleDiameterMotorPID,      // Function that should be called
      "PID Motor Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      3,                   // Task priority
      &PIDDiameterTaskHandle);

  xTaskCreate(
      handleTempPID,       // Function that should be called
      "PID Temperature Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      3,                   // Task priority
      &PIDTemperatureTaskHandle);

  xTaskCreate(
      handleADC,     // Function that should be called
      "ADC Handler", // Name of the task (for debugging)
      16384,          // Stack size (bytes)
      NULL,          // Parameter to pass
      3,             // Task priority
      &ADCTaskHandle);

  // FPGA Readout missing
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
  hotendTemp = FFF_Adc_readVolt(EXT_ADC_TEMP_CHANNEL);

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
  Serial.println("Reading UDP packets...");
  while (1)
  {
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
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_START_CONFIRM);
      } 
      else if (strncmp(udpRecPBuf, UDP_APP_STOP_CMD, sizeof(UDP_APP_STOP_CMD)) == 0)
      {
        stopApp = TRUE;
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_STOP_CONFIRM);
      } 
      else if (strncmp(udpRecPBuf, UDP_APP_PAUSE_CMD, sizeof(UDP_APP_PAUSE_CMD)) == 0)
      {
        pauseApp = TRUE;
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_PAUSE_CONFIRM);
      } 
      else if (strncmp(udpRecPBuf, UDP_APP_LIST_CMD, sizeof(UDP_APP_LIST_CMD)) == 0)
      {
        udpConn.println(UDP_CONFIRM);
        char fileStr[MAX_OUT_STRING_LEN];
        FFF_SD_getFileList(fileStr, MAX_OUT_STRING_LEN);

        const char heading[] = "--- FILE LIST ----";
        Serial.println(heading);
        Serial.println(fileStr);
        udpConn.println(heading);
        udpConn.println(fileStr);

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


void handleLog(void *param)
{
  // The logging that should be executed will be passed as param 
  FFF_Log* logPtr = (FFF_Log*) param;
  Serial.print("I>Configured for log: ");
  Serial.println(logPtr->name);
  while (1)
  {
    if (logPtr == NULL)
    {
      Serial.println("E>Log not initialized");
      continue;
    }

    if (gAppStatus == APP_RUNNING || gAppStatus == APP_PAUSED) // Also log during pause
    {
      String tmpStr;
      tmpStr += hotendTemp;
      tmpStr += DELIMITER;
      tmpStr += filDiameterMm;
      tmpStr += DELIMITER;
      tmpStr += PullStepper.targetSpeed;
      tmpStr += DELIMITER;
      tmpStr += ExtruderStepper.targetSpeed;

      tmpStr.toCharArray(logPtr->dataStr, sizeof(logPtr->dataStr));
      logCount++;

      uint32_t time = logCount * LOG_INTERVAL_MS;
      if (logPtr->isActive)
      {
        // Logging is running so the params were already written
        // FFF_SD_writeToFile((File*) logPtr->logFilePtr, logPtr->dataStr);
        udpConn.println(logPtr->dataStr);
        Serial.println(logPtr->dataStr);
      }
      else 
      {
        // Logging was not active until now, so create file
        // Write params into file and into output
        for (int i = 0; i < 500; i++)
        {
          if (!logPtr->isProtected)
          {
            break;
          }
        }
        logPtr->isProtected = true;

        udpConn.println(logPtr->paramStr);
        Serial.println(logPtr->paramStr);
        
        // if (FFF_SD_attachLogFile(logPtr))
        // {
        //   // File successfully attached
        //   Serial.println("I>File successfully attached");
        //   FFF_SD_writeToFile((File*) logPtr->logFilePtr, logPtr->paramStr);
        // }
        // Serial.println("E>File attachment failed");

        logPtr->isProtected = false;
        logPtr->isActive = true;
      }

    }
    else 
    {
      // App should be stopped so stop logging
      if (logPtr->isActive)
      {
        Serial.println("I>Detaching file");
        // FFF_SD_detachLogFile(logPtr);
      }

    }


    vTaskDelay(LOG_INTERVAL_MS);
  }
}


void handleTempPID(void *param)
{
  while (1)
  {
    pidTempControl.Compute();
    // Provide the values to the heaters 

    vTaskDelay(PID_TEMP_INTERVAL_MS);
  }
}


void handleDiameterMotorPID(void *param)
{
  while (1)
  {
    pidMotControl.Compute();
    // Provide the values to the steppers 
    vTaskDelay(PID_DIAMETER_INTERVAL_MS);
  }
}


void handleADC(void *param)
{
  while (1)
  {
    volatile double oldTemp;
    if (FFF_Adc_isReady())
    {
      hotendTemp = LookupTemperature((double) FFF_Adc_readVolt(EXT_ADC_TEMP_CHANNEL), &thermistor0Lut);
      // Serial.print("I> TH0: ");
      // Serial.print(hotendTemp, 1);
      // Serial.println(" deg C");
    }

    double diff = hotendTemp - oldTemp; 
    if (diff < 0.0)
    {
      diff *= -1;
    }
    Serial.print("DeltaT: ");
    Serial.println(diff, 4);

    if (diff > MAX_TEMP_DELTA_DEG)
    {
      // Some erroneous reading so take the previous
      hotendTemp = oldTemp; 
    } 
    else 
    {
      oldTemp = hotendTemp;
    }

    FFF_Oled_updateTemperature(hotendTemp); 
    
    vTaskDelay(ADC_SAMPLE_INTERVAL_MS);
  }
}


void handleFPGA(void *param)
{
  while (1)
  {
    // Read from Serial2 and analyze the stream
    vTaskDelay(FPGA_CALCULATE_DIAMETER_INTERVAL_MS);
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



/* TODO: Implement something better like Steinhart-Hart method */
double LookupTemperature(double volts, FFF_Lut* lutPtr)
{
  // Traverse the lut and search for the point that comes nearest
  double voltDiff = FFF_DEVICE_SUPPLY - volts;
  double scaling = lutPtr->scalingFac;
  double measuredRes = (volts * THERMISTOR_PULL_UP_VAL) / (voltDiff);

#if DEBUG_LUT_HANDLING==TRUE
  double pupVal = THERMISTOR_PULL_UP_VAL;
  Serial.print("D> DIFFVOLTS: ");
  Serial.println(voltDiff, 4);
  Serial.print("D> PUPVAL: ");
  Serial.println(pupVal, 4);
  Serial.print("D> MEASRES: ");
  Serial.println(measuredRes, 4);
#endif


  if (volts <= 0)
  {
    return -999; // Something bad happened or call is erroneous
  }

  bool ntcThermistor = false;

  if (lutPtr->resPtr[0] > lutPtr->resPtr[1])
  {
    ntcThermistor = true;
  }

  uint16_t foundIndex = 0;

  // NOTE: TRUE = positive, FALSE = negative and zero
  for (int i = 0; lutPtr->resPtr[i] > 0.0 && lutPtr->tempPtr[i] != END_TEMPS; i++)
  {
    // Current values
    double currRes = lutPtr->resPtr[i] * scaling;
    

#if DEBUG_LUT_HANDLING==TRUE
    Serial.print("D> Resistor value in LUT: ");
    Serial.println(currRes);
#endif 

    if (ntcThermistor && measuredRes > currRes)
    {
      // For NTC
      
#if DEBUG_LUT_HANDLING==TRUE
      Serial.println("D> NTC value found");
#endif 
      foundIndex = i;
      break;
    }
    else if (!ntcThermistor && measuredRes < currRes)
    {
      // For PTC
      
#if DEBUG_LUT_HANDLING==TRUE
      Serial.println("D> PTC value found");
#endif 
      foundIndex = i;
      break;      
    } 
  }
  
  
#if DEBUG_LUT_HANDLING==TRUE
    Serial.print("D> Index in LUT: ");
    Serial.println(foundIndex);
#endif 

  if (foundIndex > 1)
  {
    // Interpolate between the current value and the old value
    int absTempDiff = lutPtr->tempPtr[foundIndex] - lutPtr->tempPtr[foundIndex - 1];
    if (absTempDiff < 0.0)
    {
      absTempDiff *= -1.0;
    }

    double absResDiff = lutPtr->resPtr[foundIndex] - lutPtr->resPtr[foundIndex - 1];
    if (absResDiff < 0.0)
    {
      absResDiff *= -1.0;
    } 

    int foundTempVal1 = lutPtr->tempPtr[foundIndex-1];
    int foundTempVal2 = lutPtr->tempPtr[foundIndex];
    double foundResVal1 = lutPtr->resPtr[foundIndex-1];
    double foundResVal2 = lutPtr->resPtr[foundIndex];

    if (foundTempVal1 < 0.0)
    {
      foundTempVal1 *= -1.0;
    }

    if (foundTempVal2 < 0.0)
    {
      foundTempVal2 *= -1.0;
    }

    // Calculate two coefficients
    double coeff1 = foundResVal1 / (double) foundTempVal1;
    double coeff2 = foundResVal2 / (double) foundTempVal2;

    // Take their mean and add it to the first one
    double resCoeff = coeff1 + ((coeff2 - coeff1) / 2);
    
    // Now divide our measured resistance with this value
    return (double)(measuredRes / resCoeff) / (double) 1000;
  }

#if DEBUG_LUT_HANDLING==TRUE
  Serial.println("E> Couldn't find entry in LUT");
  Serial.print("I> Measured voltage: ");
  Serial.println(volts);
  Serial.print("I> Calculated resistance: ");
  Serial.println(measuredRes, 4);
#endif

  return -999;
}

// > Insert Steinhart Hart here
