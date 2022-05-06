#include <stdint.h>
#include <stdbool.h>

#include <FFF_Rtos.h>
#include <FFF_Settings.h>
#include <FFF_WiFi.h>
#include <FFF_Pid.h>
#include <FFF_Heater.h>
#include <FFF_Stepper.h>
#include <FFF_Log.h>
#include <FFF_DiaAnalyzer.h>
#include <FFF_Oled.h>
#include <FFF_Temperature.h>
#include <FFF_Uart.h>

#define MAX_CORES 2
#define KBYTE 1024

bool startApp = false;
bool stopApp = false;
bool pauseApp = false;

FFF_AppStatus gAppStatus = APP_STOPPED;

/******************************************/
/* Task Control */
/******************************************/
void ResumeAppTasks()
{
  /* Resume the diameter control */
  vTaskResume(FFF_Pid_getDiameterTaskHandle());
}


void DeleteAppTasks()
{
  /* Stop the PID tasks for hotend temperature and diameter */

  if (*FFF_Pid_getDiameterTaskHandle() != NULL)
  {
    vTaskDelete(*FFF_Pid_getDiameterTaskHandle());
  }

  if (*FFF_Pid_getTemperatureTaskHandle() != NULL)
  {
    vTaskDelete(*FFF_Pid_getTemperatureTaskHandle());
  }
}


void SuspendAppTasks()
{
  /* Suspend the task for Logging and pause the PID for diameter */
  vTaskSuspend(*FFF_Log_getTaskHandle());
  vTaskSuspend(*FFF_Pid_getDiameterTaskHandle());
}

/******************************************/
/* Task Creation */
/******************************************/
void CreateAppTasks()
{
  /* PID Task for the actual diameter control via motor speed */
  xTaskCreate(
      TASK_handleDiameterMotorPID,      // Function that should be called
      "TASK: PID Motor Handler", // Name of the task (for debugging)
      4 * KBYTE,                // Stack size (bytes)
      NULL,                // Parameter to pass
      PID_DIAMETER_TASK_PRIO,                   // Task priority
      FFF_Pid_getDiameterTaskHandle());

  /* PID Task to control the temperature of the hotend */
  xTaskCreate(
      TASK_handleTempPID,       // Function that should be called
      "TASK: PID Temperature Handler", // Name of the task (for debugging)
      4 * KBYTE,                // Stack size (bytes)
      NULL,                // Parameter to pass
      PID_TEMP_TASK_PRIO,                   // Task priority
      FFF_Pid_getTemperatureTaskHandle());

  /* Task to analyse the diameter. This will be triggered in ISR */
  xTaskCreatePinnedToCore(
      TASK_handleDiaAnalysis,         // Function that should be called
      "TASK: Diameter Analysis",  // Name of the task (for debugging)
      10 * KBYTE,                 // Stack size (bytes)
      NULL,                       // Parameter to pass
      DIA_CALC_TASK_PRIO,         // Task priority
      FFF_DiaAn_getTaskHandle(),
      1 // CoreId
      );
}


void CreateAppInitTasks()
{
  // /* WiFi init is currently done in setup. If in STA mode, do the connection in task context */
  // xTaskCreate(
  //     TASK_InitializeWiFi,    // Function that should be called
  //     "Initialize WiFi",      // Name of the task (for debugging)
  //     32 + KBYTE,             // Stack size (bytes)
  //     NULL,               // Parameter to pass
  //     WIFI_INIT_PRIO,     // Task priority
  //     FFF_WiFi_getInitTaskHandle());

  /* Temperature tracking Task (read ADC and calc temperature) */
  xTaskCreatePinnedToCore(
      TASK_handleTemperature,    // Function that should be called
      "TASK: Track Temperature", // Name of the task (for debugging)
      4 * KBYTE,              // Stack size (bytes)
      NULL,              // Parameter to pass
      ADC_TASK_PRIO,                 // Task priority
      FFF_Temp_getTaskHandle(),
      1);      

  /* Display info on the OLED periodically */
  xTaskCreatePinnedToCore(
      TASK_handleOled,       // Function that should be called
      "TASK: OLED Handler", // Name of the task (for debugging)
      4 * KBYTE,                // Stack size (bytes)
      NULL,                // Parameter to pass
      OLED_TASK_PRIO,                   // Task priority
      FFF_Oled_getTaskHandle(),
      1);

  /* ESP and SD don't like each other somehow. So deactivate it for now. (GPIO12)*/
  /* Initialize the SD. In task context to not interfere with the rest of the inits */
  // xTaskCreate(
  //     InitializeSD,    // Function that should be called
  //     "Initialize SD", // Name of the task (for debugging)
  //     16384,            // Stack size (bytes)
  //     NULL,            // Parameter to pass
  //     2,               // Task priority
  //     &initSDHandle);
}



void FFF_Rtos_StartOS()
{
  /* Create the tasks that initialize the rest of the hardware. 
  * They themselves also create their corresponding handling tasks
  */
  CreateAppInitTasks();
}


void TASK_mainControlApp()
{

  /* STOP command was received */
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


  /* START command was received */
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

  /* PAUSE command was received */
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
/* App Control */
/******************************************/
void FFF_Rtos_startApp()
{
  FFF_Uart_activateInterruptRX_Uart2();
  FFF_Stepper_enableAll();
  FFF_Stepper_runStepsPerSecond(&winchStepper, 50);
  FFF_Stepper_runStepsPerSecond(&extruderStepper, 50);
  FFF_Stepper_runStepsPerSecond(&pullStepper, 50);
  FFF_Heater_changeHeatDC(75.00f);
  startApp = true;
}


void FFF_Rtos_stopApp()
{
  if (FFF_Rtos_getAppStatus() != APP_STOPPED)
  {
    FFF_Uart_deactivateInterruptRX_Uart2();
    stopApp = true;
    FFF_Heater_stop();
  }
}


void FFF_Rtos_pauseApp()
{
  pauseApp = true;
}


FFF_AppStatus FFF_Rtos_getAppStatus()
{
  return gAppStatus;
}

