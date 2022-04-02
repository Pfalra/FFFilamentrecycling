#include <stdint.h>
#include <stdbool.h>

#include <FFF_Rtos.hpp>
#include <FFF_Settings.hpp>
#include <FFF_WiFi.hpp>
#include <FFF_Pid.hpp>
#include <FFF_Heater.hpp>
#include <FFF_Stepper.hpp>
#include <FFF_Log.hpp>

extern TaskHandle_t UdpTaskHandle;
extern TaskHandle_t InitWiFiTaskHandle;

bool startApp = false;
bool stopApp = false;
bool pauseApp = false;

FFF_AppStatus gAppStatus = APP_STOPPED;

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
}


void SuspendAppTasks()
{
  vTaskSuspend(LogTaskHandle);
  vTaskSuspend(PIDDiameterTaskHandle);
}


void CreateAppTasks()
{
  xTaskCreate(
      handleDiameterMotorPID,      // Function that should be called
      "TASK: PID Motor Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      PID_DIAMETER_TASK_PRIO,                   // Task priority
      &PIDDiameterTaskHandle);

  xTaskCreate(
      handleTempPID,       // Function that should be called
      "TASK: PID Temperature Handler", // Name of the task (for debugging)
      8192,                // Stack size (bytes)
      NULL,                // Parameter to pass
      PID_TEMP_TASK_PRIO,                   // Task priority
      &PIDTemperatureTaskHandle);
  // FPGA Readout missing
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
      &InitWiFiTaskHandle);

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


void handleMainLoop()
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