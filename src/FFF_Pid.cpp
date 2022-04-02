#include <Arduino.h>
#include <FFF_Pid.hpp>
#include <FFF_Settings.hpp>
#include <FFF_Temperature.hpp>
#include <FFF_Stepper.hpp>


TaskHandle_t PIDDiameterTaskHandle;
TaskHandle_t PIDTemperatureTaskHandle;

/* Filament control */
double filDiameterMm = 0.0;
double targetDiameterMm = TARGET_DIAMETER;
double pwmFrequency = 0.0;

double kp_mot = KP_PULL_MOT_DEFAULT; 
double ki_mot = KI_PULL_MOT_DEFAULT;
double kd_mot = KP_PULL_MOT_DEFAULT;

PID pidMotControl(&filDiameterMm, &pwmFrequency, &targetDiameterMm, kp_mot, ki_mot, kd_mot, AUTOMATIC);

/* Temperature control */
double hotendTemp = 0.0;
double pwmDutyCycle = 20.0;
double targetTemp = 215.0;

double kp_temp = 24.4;
double ki_temp = 1.4;
double kd_temp = 106.8;

PID pidTempControl(&hotendTemp, &pwmDutyCycle, &targetTemp, kp_temp, ki_temp, kd_temp, AUTOMATIC);





void FFF_Pid_init()
{
  // set outputlimits for PID
  pidMotControl.SetOutputLimits(0.0, 1000.0);
  pidTempControl.SetOutputLimits(0.0, 100.0);
}

void handleDiameterMotorPID(void *param)
{
  while (1)
  {
    // pidMotControl.Compute();
    // Provide the values to the steppers 
    
    // send data only when you receive data:
  //if (Serial2.available() > 0) {
    // read the incoming byte (should be the FPGA input?):
    //incomingByte = Serial2.read();

    if (pwmFrequency <= 0.0)
    {
      pwmFrequency = 50;
    }
    FFF_Stepper_runStepsPerSecond(&pullStepper, pwmFrequency);
    FFF_Stepper_runStepsPerSecond(&winchStepper, pwmFrequency);
    vTaskDelay(PID_DIAMETER_INTERVAL_MS);
  }
}


void handleTempPID(void *param)
{
  while (1)
  {
    pidTempControl.Compute();
    // Provide the values to the heaters 

    vTaskDelay(PID_TEMP_INTERVAL_MS);
  }
}

