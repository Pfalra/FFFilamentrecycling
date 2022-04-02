#include <FFF_Rtos.h>
#include <FFF_Settings.h>
#include <FFF_WiFi.h>
#include <FFF_Stepper.h>
#include <FFF_Sd.h>
#include <FFF_Adc.h>
#include <FFF_Oled.h>
#include <FFF_Graphics.h>
#include <FFF_Credentials.h>
#include <FFF_Heater.h>
#include <FFF_Temperature.h>


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
      &initWiFiHandle);

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