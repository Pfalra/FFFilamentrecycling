#ifndef FFF_HEATER_H
#define FFF_HEATER_H

#include <stdint.h>

void FFF_Heater_startHeating();

void FFF_Heater_changeHeatDC(double dutycycle);

void FFF_Heater_stop();

#endif