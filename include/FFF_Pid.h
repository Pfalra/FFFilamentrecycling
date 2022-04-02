#ifndef FFF_PID_H
#define FFF_PID_H

#include <PID_v1.h>

void FFF_Pid_init();

TaskHandle_t* FFF_Pid_getTemperatureTaskHandle();
TaskHandle_t* FFF_Pid_getDiameterTaskHandle();
void TASK_handleDiameterMotorPID(void *param);
void TASK_handleTempPID(void *param);

#endif 