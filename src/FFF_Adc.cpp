#include <ADS1X15.h>
#include <Arduino.h>
#include <FFF_Adc.h>
#include <FFF_Settings.h>

ADS1115 ExtADC(EXT_ADC_ADDR);


void FFF_Adc_init()
{
    /* Initialize ADC */
    Serial.print("ADS1x15 LIB: ");
    Serial.println(ADS1X15_LIB_VERSION);

    ExtADC.begin();

    Serial.println(F("\nSET\tACTUAL\n=================="));
    for (uint32_t speed = 50000; speed <= 1000000; speed += 50000)
    {
        ExtADC.setWireClock(speed);
        Serial.print(speed); 
        Serial.print("\t");
        Serial.println(ExtADC.getWireClock());
    }
    ExtADC.setWireClock(1000000);
    Serial.println();
    ExtADC.setGain(0);
}


double FFF_Adc_readTemp()
{
    const double vcc = 3.3;
    const uint32_t steps = 65536;
    double res = vcc/(double) 65536; // resolution in volts at 0 Gain
    uint16_t rawTemp = FFF_Adc_readTempRaw();
    return rawTemp * res;
}

uint16_t FFF_Adc_readTempRaw()
{
    return ExtADC.readADC(EXT_ADC_TEMP_PIN);
}