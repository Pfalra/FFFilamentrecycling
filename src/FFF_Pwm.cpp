#include <stdint.h>
#include <FFF_Types.h>
#include <FFF_Pwm.h>
#include <Arduino.h>


void FFF_Pwm_init(FFF_PwmHandle* handle)
{
    pinMode(handle->outputPin, OUTPUT);
    digitalWrite(handle->outputPin, LOW);
    Serial.print("LEDC Setup for ");
    Serial.print(handle->id);
    Serial.print(" returned: ");
    Serial.println(ledcSetup(handle->channel, handle->freq, handle->resolution));
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
    if (handle->dutyCycle > 100.0f)
    {
        handle->dutyCycle = 100.0f; // Clip it
    }

    double percFac = handle->dutyCycle / 100.0f;
    uint32_t llDuty = percFac * (1 << handle->resolution);

    Serial.printf("LLDuty: %d @ %d\n", llDuty, handle->resolution);
    ledcWrite(handle->channel, llDuty);
}


void FFF_Pwm_stopOutput(FFF_PwmHandle* handle)
{
    ledcWrite(handle->channel, 0);    
}
