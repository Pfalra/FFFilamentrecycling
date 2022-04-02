#include <FFF_Heater.hpp>
#include <stdint.h>
#include <FFF_Settings.hpp>
#include <FFF_Pwm.hpp>
#include <Arduino.h>

FFF_PwmHandle heaterHandle = 
{
    'h',                    // id (here char is used for simplification)
    HEATER_OUTPUT_PIN,      // outputPin
    4,                      // channel < TODO: Specify in settings
    FFF_PWM_RES_15BIT,      // resolution of 15 bit should be sufficient
    0.0,                    // dutyCycle must be zero for now  
    PWM_HEATER_FREQUENCY    // frequency
};


void FFF_Heater_init()
{
    pinMode(HEATER_OUTPUT_PIN, OUTPUT);
    digitalWrite(HEATER_OUTPUT_PIN, LOW);

    FFF_Pwm_init(&heaterHandle);
    FFF_Pwm_stopOutput(&heaterHandle);
}


void FFF_Heater_heat(double dutycycle)
{
    FFF_Pwm_startOutput(&heaterHandle);
}


void FFF_Heater_stop()
{   
    FFF_Pwm_stopOutput(&heaterHandle);
}



