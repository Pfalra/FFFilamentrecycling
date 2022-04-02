#ifndef FFF_LOG_H
#define FFF_LOG_H

#include <Arduino.h>

TaskHandle_t* FFF_Log_getTaskHandle();
void TASK_handleLog(void *param);

#endif