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
  vTaskResume(FFF_Pid_getDiameterTaskHandle());
}


void DeleteAppTasks()
{
  vTaskDelete(*FFF_Pid_getDiameterTaskHandle());
  vTaskDelete(*FFF_Pid_getTemperatureTaskHandle());
}


void SuspendAppTasks()
{
  vTaskSuspend(*FFF_Log_getTaskHandle());
  vTaskSuspend(*FFF_Pid_getDiameterTaskHandle());
}


void CreateAppTasks()
{
  xTaskCreate(
      TASK_handleDiameterMotorPID,      // Function that should be called
      "TASK: PID Motor Handler", // Name of the task (for debugging)
      4 * KBYTE,                // Stack size (bytes)
      NULL,                // Parameter to pass
      PID_DIAMETER_TASK_PRIO,                   // Task priority
      FFF_Pid_getDiameterTaskHandle());

  xTaskCreate(
      TASK_handleTempPID,       // Function that should be called
      "TASK: PID Temperature Handler", // Name of the task (for debugging)
      4 * KBYTE,                // Stack size (bytes)
      NULL,                // Parameter to pass
      PID_TEMP_TASK_PRIO,                   // Task priority
      FFF_Pid_getTemperatureTaskHandle());

  xTaskCreatePinnedToCore(
      TASK_handleDiaAnalysis,         // Function that should be called
      "TASK: Diameter Analysis",  // Name of the task (for debugging)
      10 * KBYTE,                 // Stack size (bytes)
      NULL,                       // Parameter to pass
      PID_TEMP_TASK_PRIO,         // Task priority
      FFF_DiaAn_getTaskHandle(),
      1 // CoreId
      );
}


void CreateAppInitTasks()
{
  /* SD Mounting and Setting up Wifi can take some time and 
    should not interfere with the other stuff. So we set them up 
    in task context 
  */
  /* Create tasks for further initializations */
  xTaskCreate(
      TASK_InitializeWiFi,    // Function that should be called
      "Initialize WiFi", // Name of the task (for debugging)
      16384,              // Stack size (bytes)
      NULL,              // Parameter to pass
      3,                 // Task priority
      FFF_WiFi_getInitTaskHandle());

  /* ESP and SD don't like each other somehow. So deactivate it for now. (GPIO12)*/
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
  CreateAppInitTasks();
}


void TASK_mainControlApp()
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


void FFF_Rtos_startApp()
{
  startApp = true;
}


void FFF_Rtos_stopApp()
{
  stopApp = true;
}


void FFF_Rtos_pauseApp()
{
  pauseApp = true;
}


FFF_AppStatus FFF_Rtos_getAppStatus()
{
  return gAppStatus;
}

