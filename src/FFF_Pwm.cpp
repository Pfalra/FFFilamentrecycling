#include <stdint.h>
#include <FFF_Types.hpp>
#include <FFF_Pwm.hpp>
#include <Arduino.h>


void FFF_Pwm_init(FFF_PwmHandle* handle)
{
    pinMode(handle->outputPin, OUTPUT);
    digitalWrite(handle->outputPin, LOW);
    ledcSetup(handle->channel, handle->freq, handle->resolution);
    ledcAttachPin(handle->outputPin, handle->channel);
    ledcWrite(handle->channel, 0);
}


void FFF_Pwm_changeFrequency(FFF_PwmHandle* handle)
{
    ledcWriteTone(handle->channel, handle->freq);
}


void FFF_Pwm_changeDutyCycle(FFF_PwmHandle* handle)
{
    FFF_Pwm_startOutput(handle);
}


void FFF_Pwm_startOutput(FFF_PwmHandle* handle)
{
    if (handle->dutyCycle > 100.0)
    {
        handle->dutyCycle = 100.0; // Clip it
    }

    double percFac = handle->dutyCycle / 100.0f;
    uint32_t llDuty = percFac * UINT32_MAX;
    ledcWrite(handle->channel, llDuty);
}


void FFF_Pwm_stopOutput(FFF_PwmHandle* handle)
{
    ledcWrite(handle->channel, 0);    
}
