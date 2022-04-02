#ifndef FFF_HEATER_H
#define FFF_HEATER_H

#include <stdint.h>

void FFF_Heater_init();

void FFF_Heater_heat(double dutycycle);

void FFF_Heater_stop();

#endif