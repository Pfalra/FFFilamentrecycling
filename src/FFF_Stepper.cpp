#include <Arduino.h>
#include <FFF_Stepper.h>
#include <FFF_Settings.h>
#include <FFF_Types.h>
#include <FFF_Pwm.h>


FFF_PwmHandle extruderPwm = 
{
  0,                  // id
  EXTRUDER_STEP_PIN,  // outputPin
  0,                  // channel
  FFF_PWM_RES_1BIT,   // resolution
  50.0,                // dutyCycle in percentage
  0.0                 // frequency
};

FFF_Stepper extruderStepper = 
{
  0,                  // id
  EXTRUDER_STEP_PIN,  // stepPin
  STEPPER_MICROSTEPS, // microsteps
  &extruderPwm        // pwmPtr
};


FFF_PwmHandle pullPwm = 
{
  1,                  // id
  PULLER_STEP_PIN,    // outputPin
  0,                  // channel
  FFF_PWM_RES_1BIT,   // resolution
  50.0,                // dutyCycle in percentage
  0.0                 // frequency
};

FFF_Stepper pullStepper = 
{
  1,                  // id
  PULLER_STEP_PIN,    // stepPin
  STEPPER_MICROSTEPS, // microsteps
  &pullPwm            // pwmPtr
};


FFF_PwmHandle winchPwm = 
{
  1,                  // id
  PULLER_STEP_PIN,    // outputPin
  0,                  // channel
  FFF_PWM_RES_1BIT,   // resolution
  50.0,                // dutyCycle in percentage
  0.0                 // frequency
};

FFF_Stepper winchStepper = 
{
  2,                  // id
  WINCH_STEP_PIN,     // stepPin
  STEPPER_MICROSTEPS, // microsteps
  &winchPwm           // pwmPtr
};


void FFF_Stepper_init()
{
  pinMode(STEPPER_EN_PIN, OUTPUT);

  FFF_Stepper_disableAll();

  pinMode(EXTRUDER_STEP_PIN, OUTPUT);
  pinMode(PULLER_STEP_PIN, OUTPUT);
  pinMode(WINCH_STEP_PIN, OUTPUT);

  FFF_Pwm_init(&extruderPwm);
  FFF_Pwm_stopOutput(&extruderPwm);
  FFF_Pwm_init(&pullPwm);
  FFF_Pwm_stopOutput(&pullPwm);
  FFF_Pwm_init(&winchPwm);
  FFF_Pwm_stopOutput(&winchPwm);

  FFF_Stepper_disableAll();
}


void FFF_Stepper_enableAll()
{
  digitalWrite(STEPPER_EN_PIN, LOW);
}


void FFF_Stepper_disableAll()
{
  digitalWrite(STEPPER_EN_PIN, HIGH);
}


void FFF_Stepper_runStepsPerSecond(FFF_Stepper* stepperPtr, double stepsPerSecond)
{
  stepperPtr->pwmPtr->freq = stepsPerSecond;
  FFF_Pwm_changeFrequency(stepperPtr->pwmPtr);
}


void FFF_Stepper_stop(FFF_Stepper* stepperPtr)
{
  FFF_Pwm_stopOutput(stepperPtr->pwmPtr);
}


void FFF_Stepper_stopAll()
{
  FFF_Stepper_stop(&extruderStepper);
  FFF_Stepper_stop(&pullStepper);
  FFF_Stepper_stop(&winchStepper);
}