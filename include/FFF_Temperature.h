#ifndef FFF_TEMPERATURE_H
#define FFF_TEMPERATURE_H

#include <FFF_Types.h>

extern TaskHandle_t ADCTaskHandle;

double LookupTemperature(double volts, FFF_Lut* lutPtr, double measuredRes);
double FindResistance(int16_t temperature, FFF_Lut* lutPtr);
void calculateCoeffsSteinhartHart(FFF_Lut* lutPtr, double* aCoeffPtr, double* bCoeffPtr, int16_t t1, int16_t t2);
double calculateTempSteinhartHart(double alpha, double beta, double c, double normTemp, double resistance, FFF_Lut* lutPtr);
double FFF_Temp_getTemperature();
#endif