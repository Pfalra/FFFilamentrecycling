#ifndef FFF_RTOS_H
#define FFF_RTOS_H

#include <FreeRTOS.h>

//TaskControlFunctions
void CreateAppInitTasks();
void CreateAppTasks();

void ResumeAppTasks();
void DeleteAppTasks();
void SuspendAppTasks();


//InitFunctions
void TASK_InitializeWiFi(void *param);
void InitializeOther(void *param);
void InitializeSD(void *param);


//HandlerFunctions
void handleUdp(void *param);
void handleOled(void *param);
void handleLog(void *param);
void handleDiameterMotorPID(void *param);
void handleTempPID(void *param);
void TASK_handleTemperature(void *param);
void handleFPGA(void *param);

void handleMainLoop();


void FFF_Rtos_startApp();
void FFF_Rtos_stopApp();
void FFF_Rtos_pauseApp();

void FFF_Rtos_StartOS();

/* TASK HANDLES */
TaskHandle_t initOtherHandle;
TaskHandle_t OledTaskHandle;
TaskHandle_t LogTaskHandle;
TaskHandle_t PIDDiameterTaskHandle;
TaskHandle_t PIDTemperatureTaskHandle;
TaskHandle_t ADCTaskHandle;
TaskHandle_t StepperTaskHandle;
TaskHandle_t FPGATaskHandle;

/* Sempahore Handles */
SemaphoreHandle_t i2CSemaphoreHandle;

#endif 