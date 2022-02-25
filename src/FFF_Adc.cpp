#include <ADS1X15.h>
#include <Arduino.h>
#include <FFF_Adc.h>
#include <FFF_Settings.h>
#include <FFF_Types.h>

#define ADC_RECONNECTION_ATTEMPTS 10

ADS1115 ExtADC(EXT_ADC_ADDR);



void FFF_Adc_init()
{
    /* Initialize ADC */
    ExtADC.begin();

    // Serial.println(F("\nSET\tACTUAL\n=================="));
    // for (uint32_t speed = 50000; speed <= 1000000; speed += 50000)
    // {
    //     ExtADC.setWireClock(speed);
    //     Serial.print(speed); 
    //     Serial.print("\t");
    //     Serial.println(ExtADC.getWireClock());
    // }
    ExtADC.setWireClock(1000000);
    Serial.println();
    ExtADC.setGain(0);

    // Check connection
    bool conn = false;
    for (int i = 0; i < ADC_RECONNECTION_ATTEMPTS && conn == false; i++)
    {
        conn = ExtADC.isConnected();
    }

    if (!conn)
    {
        Serial.println("E> ADC not connected");
        return;
    }

    Serial.println("I> ADC connected");
    
    // Check ready state
    bool rdy = false;
    for (int i = 0; i < ADC_RECONNECTION_ATTEMPTS && rdy == false; i++)
    {
        rdy = ExtADC.isReady();
    }
    
    if (!rdy)
    {
        Serial.println("E> ADC not getting ready");
        return;
    }

    Serial.println("I> ADC Ready");

    // Success
    Serial.print("I> ADC initialized for channel: ");
    Serial.println(EXT_ADC_TEMP_CHANNEL);
}


double FFF_Adc_readVolt(uint8_t channel)
{
    double scale = ExtADC.toVoltage();
    uint16_t rawTemp = FFF_Adc_readTempRaw(channel);
    return rawTemp * scale;
}


uint16_t FFF_Adc_readTempRaw(uint8_t channel)
{
    return ExtADC.readADC(EXT_ADC_TEMP_CHANNEL);
}


double FFF_Adc_lookupTemperature(double val, FFF_Lut* lutPtr, double oldVal)
{
    // Traverse the lut and search for the point that comes nearest
    
    // NOTE: TRUE = positive, FALSE = negative and zero


    for (int i = 0; lutPtr->resPtr[i] != NULL; i++)
    {
        for (int j = 0; lutPtr->tempPtr[j] != NULL; j++)
        {

        }
    }

    return -999;
}

