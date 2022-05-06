#include <FFF_Heater.h>
#include <stdint.h>
#include <FFF_Settings.h>
#include <FFF_Pwm.h>
#include <Arduino.h>

FFF_PwmHandle heaterHandle = 
{
    9,                    // id (here char is used for simplification)
    HEATER_OUTPUT_PIN,      // outputPin
    2,                      // channel < TODO: Specify in settings
    FFF_PWM_RES_12BIT,      // resolution of 12 bit should be sufficient
    0.0,                    // dutyCycle must be zero for now  
    PWM_HEATER_FREQUENCY    // frequency
};


void FFF_Heater_startHeating()
{
    pinMode(HEATER_OUTPUT_PIN, OUTPUT);
    FFF_Pwm_init(&heaterHandle);
    FFF_Pwm_stopOutput(&heaterHandle);
}


void FFF_Heater_changeHeatDC(double dutycycle)
{
    // FUCK IT Pwm is somehow not working properly
    //FFF_Pwm_init(&heaterHandle);
    heaterHandle.dutyCycle = dutycycle;
    FFF_Pwm_startOutput(&heaterHandle);
}


void FFF_Heater_stop()
{   
    FFF_Pwm_stopOutput(&heaterHandle);
}



