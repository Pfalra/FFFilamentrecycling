#ifndef FFF_STEPPER_H
#define FFF_STEPPER_H

#include <FFF_Types.h>

extern FFF_Stepper ExtruderStepper;
extern FFF_Stepper PullStepper;
extern FFF_Stepper WinchStepper;

void FFF_Stepper_init();

void FFF_Stepper_enable();

void FFF_Stepper_disable();

#endif