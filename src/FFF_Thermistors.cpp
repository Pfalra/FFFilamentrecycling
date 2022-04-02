#include <FFF_Settings.h>
#include <FFF_Thermistors.h>
#include <FFF_Types.h>
#include <Arduino.h>
#include <math.h>

#if FFF_THERMISTOR0 == NTC_3950
#define THERM_SCALING NTC3950_BASE
const int thermistor0_temps[] = NTC3950_TEMPERATURES;
const float thermistor0_resistances[] = NTC3950_RESISTANCE; 
#else
const int thermistor0_temps[] = DUMMY_TEMPERATURES;
const float thermistor0_resistances[] = DUMMY_RESISTANCE;
#endif

FFF_Lut thermistor0Lut = 
{
    .tempPtr = thermistor0_temps,
    .resPtr = thermistor0_resistances,
    .scalingFac = THERM_SCALING
};


void FFF_Therm_printLut()
{
    // Print the LUT for Debug
    Serial.print("D> First LUT Entries: ");

    for (int i = 0; i < 10; i++)
    {
      Serial.print("TEMP: ");
      Serial.print(thermistor0Lut.tempPtr[i]);
      Serial.print("\t\t");
      Serial.print("RES: ");
      Serial.print(thermistor0Lut.resPtr[i]);
      Serial.println();
    }
    Serial.println("...");
}


FFF_Lut* FFF_Therm_getLut()
{
  return &thermistor0Lut;
}


