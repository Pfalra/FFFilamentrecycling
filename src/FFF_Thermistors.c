#include <FFF_Thermistors.h>
#include <FFF_Types.h>
#include <Arduino.h>
#include <math.h>

#if FFF_THERMISTOR0 == NTC_3950
#define THERM_SCALING NTC3950_BASE
int thermistor0_temps[] = NTC3950_TEMPERATURES;
float thermistor0_resistances[] = NTC3950_RESISTANCE; 
#elif FFF_THERMISTOR0 == DUMMY_THERMISTOR
const int thermistor0_temps[] = DUMMY_THERMISTOR;
const float thermistor0_resistances[] = DUMMY_RESISTANCE;

#endif

FFF_Lut thermistor0Lut = 
{
    .tempPtr = thermistor0_temps,
    .resPtr = thermistor0_resistances,
    .scalingFac = THERM_SCALING
};

// //Steinhart-Hart
// T = 1 / {A + B[ln(NTC3950_RESISTANCE)] + C[ln(NTC3950_RESISTANCE)]^3}

