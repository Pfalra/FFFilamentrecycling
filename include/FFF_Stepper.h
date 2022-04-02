#ifndef FFF_STEPPER_H
#define FFF_STEPPER_H

#include <FFF_Types.h>

extern FFF_Stepper extruderStepper;
extern FFF_Stepper winchStepper;
extern FFF_Stepper pullStepper;

void FFF_Stepper_init();
void FFF_Stepper_enableAll();
void FFF_Stepper_disableAll();
void FFF_Stepper_stopAll();
void FFF_Stepper_stop(FFF_Stepper* stepperPtr);
void FFF_Stepper_runStepsPerSecond(FFF_Stepper* stepperPtr, double stepsPerSecond);
double FFF_Stepper_getWinchStepperSpeed();
double FFF_Stepper_getPullStepperSpeed();
double FFF_Stepper_getExtruderStepperSpeed();
#endif