#ifndef FFF_PID_H
#define FFF_PID_H

#include <PID_v1.h>

extern TaskHandle_t PIDDiameterTaskHandle;
extern TaskHandle_t PIDTemperatureTaskHandle;

void FFF_Pid_init();


#endif 