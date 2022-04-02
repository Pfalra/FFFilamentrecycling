#ifndef FFF_DIAANALYZER_H
#define FFF_DIAANALYZER_H

#include <Arduino.h>
#include <FFF_Types.h>

extern TaskHandle_t FPGATaskHandle;

void FFF_DiaAn_analyze(FFF_Measurement* meas);
void TASK_handleDiaAnalysis(void* param);
double FFF_DiaAn_getDiameter();

#endif