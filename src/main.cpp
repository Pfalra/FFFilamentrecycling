/* Framework and external libs */
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <PID_v1.h>
#include <WiFiUDP.h>
#include <math.h>
#include <driver/uart.h>

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

/* Sempahore Handles */
SemaphoreHandle_t i2CSemaphoreHandle;

static QueueHandle_t uart2_queue;
volatile bool uart2_rx_complete;

uint8_t diaMeasPoints[MEASUREMENT_LENGTH];

FFF_Measurement diameterMeasurement = {
  .outputVal = 0.0,
  .scalingCoeff = DIAMETER_SCALING_COEFF,
  .mean = 0,
  .len = MEASUREMENT_LENGTH,
  .startIndex = SYNC_LENGTH + FIRST_DEAD_PIX,
  .endIndex = MEASUREMENT_LENGTH - LAST_DEAD_PIX,
  .maxIndex = 0,
  .minIndex = 0,
  .firstLimPass = 0,
  .lastLimPass = 0,
  .passHyst = DIA_MEAS_HYST,
  .passWidth = 0,
  .maxVal = 0,
  .minVal = 0,
  .analyzed = false,
  .protectFlag = false,
  .dataPoints = diaMeasPoints
};



// Prototypes
void FFF_Udp_init();

double LookupTemperature(double volts, FFF_Lut* lutPtr, double measuredRes); 

//Steinhart-Hart
double steinhartCoeff_A = ALPHA_COEFF;
double steinhartCoeff_B = BETA_COEFF;
double steinhartCoeff_C = 0.0;

// void getCoefficients(&thermistor0Lut, &steinhartCoeff_A, &steinhartCoeff_B, &steinhartCoeff_C); //

void calculateCoeffsSteinhartHart(FFF_Lut* lutPtr, double* aCoeffPtr, double* bCoeffPtr, int16_t t1, int16_t t2);

double calculateTempSteinhartHart(double alpha, double beta, double c, double normTemp, double resistance, FFF_Lut* lutPtr);
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
double pwmDutyCycle = 20.0;
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


static void IRAM_ATTR uart2_isr_handler(void *arg)
{
  uint16_t rx_fifo_len, status;
  uint16_t index = 0;

  status = UART2.int_st.val;
  rx_fifo_len = UART2.status.rxfifo_cnt;
  uart2_rx_complete = false;

  while(rx_fifo_len)
  {
    
  }
}



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

  // Serial 2
  uart_config_t uart2_config = 
  {
    .baud_rate = 500000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  }; 


   uart_param_config(UART_NUM_2, &uart2_config);

   uart_set_pin(UART_NUM_2, TX_TO_FPGA_PIN, RX_TO_FPGA_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
   uart_driver_install(UART_NUM_2, MEASUREMENT_LENGTH, 0, 8, &uart2_queue, 0);
  
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
  // vTaskDelay(500);

  /* Create tasks for initialization */
  CreateAppInitTasks();

  /* Create Tasks for application */
  // CreateAppTasks();
}


void loop()
{
  if (stopApp)
  {
    // Disable Heater
    FFF_Heater_stop();
    // Disable Steppers
    FFF_Stepper_disableAll();

    DeleteAppTasks();
    gAppStatus = APP_STOPPED;
    stopApp = false;
  }

  if (startApp)
  {
    FFF_Heater_heat(pwmDutyCycle);
    FFF_Stepper_enableAll();
    FFF_Stepper_runStepsPerSecond(&extruderStepper, EXTRUDE_RATE_STEPS_PS);

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
    FFF_Stepper_stopAll();
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
  // vTaskDelete(ADCTaskHandle);
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
      PID_DIAMETER_TASK_PRIO,                   // Task priority
      &PIDDiameterTaskHandle);

  xTaskCreate(
      handleTempPID,       // Function that should be called
      "PID Temperature Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      PID_TEMP_TASK_PRIO,                   // Task priority
      &PIDTemperatureTaskHandle);
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

  /* ESP and SD don't like each other somehow. So deactivate it for now. */
  // xTaskCreate(
  //     InitializeSD,    // Function that should be called
  //     "Initialize SD", // Name of the task (for debugging)
  //     16384,            // Stack size (bytes)
  //     NULL,            // Parameter to pass
  //     2,               // Task priority
  //     &initSDHandle);
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

  FFF_Udp_init();

  xTaskCreate(
    handleUdp,     // Function that should be called
    "UDP Handler", // Name of the task (for debugging)
    16384,          // Stack size (bytes)
    NULL,          // Parameter to pass
    UDP_TASK_PRIO, // Task priority
    &UdpTaskHandle);

  /* Make task only execute once */
  vTaskDelete(initWiFiHandle);
}


void InitializeOther(void *param)
{
  Serial.println("\r\nI>Initializing other hardware/software");
  /* Initialize ADC */
  FFF_Adc_init();

  // set outputlimits for PID
  pidMotControl.SetOutputLimits(0.0, 1000.0);
  pidTempControl.SetOutputLimits(0.0, 100.0);

  /* Initialize Steppers */
  FFF_Stepper_init();

  Serial.println("\r\nI>Init done");

  xTaskCreate(
    handleADC,     // Function that should be called
    "ADC Handler", // Name of the task (for debugging)
    16384,          // Stack size (bytes)
    NULL,          // Parameter to pass
    ADC_TASK_PRIO, // Task priority
    &ADCTaskHandle);

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

      String udpStr = String(udpRecPBuf);

      if (len > 0)
      {
        udpRecPBuf[len] = 0;
      }

      Serial.println("Received:");
      Serial.println(udpRecPBuf);

      if (udpStr == UDP_APP_START_CMD)
      {
        startApp = TRUE;
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_START_CONFIRM);
      } 
      else if (udpStr == UDP_APP_STOP_CMD)
      {
        stopApp = TRUE;
        udpConn.println(UDP_CONFIRM);
        udpConn.println(UDP_STOP_CONFIRM);
      } 
      else if (udpStr == UDP_APP_PAUSE_CMD)
      {
        pauseApp = TRUE;
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
      tmpStr += pullStepper.pwmPtr->freq;
      tmpStr += DELIMITER;
      tmpStr += extruderStepper.pwmPtr->freq;

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
        // Serial.println("I>Detaching file");
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
    // pidMotControl.Compute();
    // Provide the values to the steppers 
    
    // send data only when you receive data:
  //if (Serial2.available() > 0) {
    // read the incoming byte (should be the FPGA input?):
    //incomingByte = Serial2.read();

    if (pwmFrequency <= 0.0)
    {
      pwmFrequency = 50;
    }
    FFF_Stepper_runStepsPerSecond(&pullStepper, pwmFrequency);
    FFF_Stepper_runStepsPerSecond(&winchStepper, pwmFrequency);
    vTaskDelay(PID_DIAMETER_INTERVAL_MS);
  }
}


void handleADC(void *param)
{
#if DEBUG_ADC == TRUE
    Serial.println("Started ADC task");
#endif  

  while (1)
  {
    double adcRawRead = 0.0;
    volatile double oldTemp;
    if (FFF_Adc_isReady())
    {

      adcRawRead = FFF_Adc_readVolt(EXT_ADC_TEMP_CHANNEL);
      // Traverse the lut and search for the point that comes nearest
      double voltDiff = FFF_DEVICE_SUPPLY - adcRawRead;
      double measuredRes = (adcRawRead * THERMISTOR_PULL_UP_VAL) / (voltDiff);
      #if DEBUG_LUT_HANDLING==TRUE
        double pupVal = THERMISTOR_PULL_UP_VAL;
        Serial.print("D> DIFFVOLTS: ");
        Serial.println(voltDiff, 4);
        Serial.print("D> PUPVAL: ");
        Serial.println(pupVal, 4);
        Serial.print("D> MEASRES: ");
        Serial.println(measuredRes, 4);
      #endif

      #if TEMPERATURE_CALC_METHOD==STEINHART_HART_METHOD
      // Use Steinhart-Hart
      // Serial.println("Calculating Steinhart-Hart:");
      hotendTemp = calculateTempSteinhartHart(steinhartCoeff_A, 
                                              steinhartCoeff_B, 
                                              steinhartCoeff_C, 
                                              NORM_TEMP, 
                                              measuredRes, 
                                              &thermistor0Lut); 
      #else
      hotendTemp = LookupTemperature(adcRawRead, &thermistor0Lut, measuredRes);
      #endif

      FFF_Oled_updateTemperature(hotendTemp);
      FFF_Oled_updatePullMotSpeed(pwmFrequency);
      FFF_Oled_updateExtruderMotSpeed(EXTRUDE_RATE_STEPS_PS);

    }

    double diff = hotendTemp - oldTemp; 
    if (diff < 0.0)
    {
      diff *= -1;
    }

#if DEBUG_ADC == TRUE
    Serial.print("I> ADC raw: ");
    Serial.println(adcRawRead, 4);
    Serial.print("I> TH0: ");
    Serial.print(hotendTemp, 1);
    Serial.println(" deg C");
    Serial.print("DeltaT: ");
    Serial.println(diff, 4);
#endif

    if (diff > MAX_TEMP_DELTA_DEG)
    {
      // Some erroneous reading so take the previous
      hotendTemp = oldTemp; 
    } 
    else 
    {
      oldTemp = hotendTemp;
    }


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



/******************************************/
/* TEMPERATURE CALCULATION */
/******************************************/
double LookupTemperature(double volts, FFF_Lut* lutPtr, double measuredRes)
{
  double scaling = lutPtr->scalingFac;

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


double FindResistance(int16_t temperature, FFF_Lut* lutPtr)
{
  uint16_t foundIndex = 0;

  for (int i = 0; lutPtr->resPtr[i] > 0.0 && lutPtr->tempPtr[i] != END_TEMPS; i++)
  {
    if (temperature <= lutPtr->tempPtr[i])
    {
      Serial.print("Found Temp in LUT at pos ");
      Serial.println(i);
      return i;
    }
  }

  return NULL;

}


void calculateCoeffsSteinhartHart(FFF_Lut* lutPtr, double* aCoeffPtr, double* bCoeffPtr, int16_t t1, int16_t t2)
{
  const uint16_t maximumLutSize = 256; 
  int16_t temp1, temp2;
  temp1 = t1 - 273.15;
  temp2 = t2 - 273.15;

Serial.println("Calculating coefficients for ");
Serial.print("T1 = ");
Serial.println(t1);
Serial.print("T2 = ");
Serial.println(t2);

  double t1Res = 0.0;
  double t2Res = 0.0;

  for (int i = 0; i < maximumLutSize && lutPtr->tempPtr[i] != NULL; i++)
  {
    uint16_t currTemp = lutPtr->tempPtr[i]; 
    uint16_t currRes = lutPtr->resPtr[i];

    if (currTemp >= t1 && t1 > -999)
    {
      t1Res = currRes;
      t1 = -1000; // invalidates t1 
    } else if (currTemp >= t2 && t2 > -999)
    {
      t2Res = currRes;
      t2 = -1000; // invalidates t1 
    }

    if (t1 == t2)
    {
      break;
    }
  }

    Serial.print("R1 = ");
    Serial.println(t1Res);
    Serial.print("R2 = ");
    Serial.println(t2Res);

    // T1Res, T2Res, T3Res are now available
    // Calculate coeffs
    *bCoeffPtr = 1/((1/(double) temp1 - 1/(double) temp2)*log(t1Res/t2Res));
    *aCoeffPtr = ((t2Res - t1Res) / (double) (t1Res*(temp2-temp1))) * pow(10.0,-6.0);
}


double calculateTempSteinhartHart(double alpha, double beta, double c, double normTemp, double resistance, FFF_Lut* lutPtr)
{
  // Serial.println("--------- SH ---------");
  // Serial.print("Coefficients alpha, beta, c: ");
  // Serial.println(alpha, 5);
  // Serial.println(beta, 5);
  // Serial.println(c, 5);

  double normRes = 0.0;
#ifdef NORM_RES
  if (NORM_RES <= 0.0)
  {
    uint16_t index = FindResistance(NORM_TEMP, lutPtr);
    normRes = lutPtr->scalingFac * lutPtr->tempPtr[index];
  } 
  else 
  {
    normRes = NORM_RES;
  }
#else 
  // We will serach for the norm resistance everytime this function gets called and itÄs alway the same
  uint16_t index = FindResistance(NORM_TEMP, lutPtr);
  normRes = lutPtr->scalingFac * lutPtr->tempPtr[index];
#endif

  // We convert to Kelvin
  normTemp += 273.15;
  double subTerm = log(normRes/resistance);
  double dividend = (normTemp * beta) / (subTerm);
  double divisor = (beta / subTerm) - normTemp;
  double quotient = dividend/divisor;
  double outputTemp = quotient - 273.15; // Convert back

  // Serial.print("Subterm: ");
  // Serial.println(subTerm, 5);
  // Serial.print("Dividend: ");
  // Serial.println(dividend, 5);
  // Serial.print("Divisor: ");
  // Serial.println(divisor, 5);
  return outputTemp;
}
