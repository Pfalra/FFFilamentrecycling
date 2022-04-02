#ifndef FFF_RTOS_H
#define FFF_RTOS_H

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

void handleMainLoop();


void FFF_Rtos_startApp();
void FFF_Rtos_stopApp();
void FFF_Rtos_pauseApp();

void FFF_Rtos_StartOS();


#endif 