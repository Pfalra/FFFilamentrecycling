#include <Arduino.h>
#include <FFF_Stepper.h>
#include <FFF_Settings.h>
#include <FFF_Types.h>

FFF_Stepper ExtruderStepper;
FFF_Stepper PullStepper;
FFF_Stepper WinchStepper;

void FFF_Stepper_init()
{
  pinMode(STEPPER_EN_PIN, OUTPUT);
  FFF_Stepper_disable();
  pinMode(EXTRUDER_STEP_PIN, OUTPUT);
  pinMode(PULLER_STEP_PIN, OUTPUT);
  pinMode(WINCH_STEP_PIN, OUTPUT);
  ExtruderStepper.id = 0;
  ExtruderStepper.targetSpeed = EXTRUDE_RATE_STEPS_PS;
  ExtruderStepper.stepPin = EXTRUDER_STEP_PIN;
  ExtruderStepper.microsteps = 16;

  PullStepper.id = 1;
  PullStepper.targetSpeed = 50;
  PullStepper.stepPin = PULLER_STEP_PIN;
  PullStepper.microsteps = 16;

  WinchStepper.id = 2;
  WinchStepper.targetSpeed = 56;
  WinchStepper.stepPin = WINCH_STEP_PIN;
  WinchStepper.microsteps = 16;

  ledcAttachPin(EXTRUDER_STEP_PIN, ExtruderStepper.id);
  ledcAttachPin(PULLER_STEP_PIN, PullStepper.id);
  ledcAttachPin(WINCH_STEP_PIN, WinchStepper.id);
  FFF_Stepper_disable();

}

void FFF_Stepper_enable()
{
  digitalWrite(STEPPER_EN_PIN, LOW);
}

void FFF_Stepper_disable()
{
  digitalWrite(STEPPER_EN_PIN, HIGH);
}
