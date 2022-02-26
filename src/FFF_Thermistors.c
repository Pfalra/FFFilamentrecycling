#include <FFF_Thermistors.h>
#include <FFF_Types.h>
#include <Arduino.h>

#if FFF_THERMISTOR0 == NTC_3950
const int thermistor0_temps[] = NTC3950_TEMPERATURES;
const double thermistor0_resistances[] = NTC3950_RESISTANCE; 
#elif 
const int thermistor0_temps[] = DUMMY_THERMISTOR;
const double thermistor0_resistances[] = DUMMY_RESISTANCE;

#endif

FFF_Lut thermistor0Lut = 
{
    .tempPtr = thermistor0_temps,
    .resPtr = thermistor0_resistances
};