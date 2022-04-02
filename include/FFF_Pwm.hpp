#ifndef FFF_PWM_H
#define FFF_PWM_H

#include <FFF_Types.hpp>

void FFF_Pwm_init(FFF_PwmHandle* handle);

void FFF_Pwm_changeFrequency(FFF_PwmHandle* handle);

void FFF_Pwm_startOutput(FFF_PwmHandle* handle);

void FFF_Pwm_stopOutput(FFF_PwmHandle* handle);
#endif