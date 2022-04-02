#include <FFF_Types.h>
#include <FFF_Thermistors.h>
#include <FFF_Temperature.h>
#include <FFF_Adc.h>
#include <Arduino.h>


/******************************************/
/* GLOBAL TEMPERATURES  */
/******************************************/
double hotendTemperature = 0.0;


/******************************************/
/* TEMPERATURE CALCULATION */
/******************************************/
double LookupTemperature(double volts, FFF_Lut* lutPtr, double measuredRes)
{
  double scaling = lutPtr->scalingFac;

  if (volts <= 0)
  {
    return -999; // Something bad happened or call is erroneous
  }

  bool ntcThermistor = false;

  if (lutPtr->resPtr[0] > lutPtr->resPtr[1])
  {
    ntcThermistor = true;
  }

  uint16_t foundIndex = 0;

  // NOTE: TRUE = positive, FALSE = negative and zero
  for (int i = 0; lutPtr->resPtr[i] > 0.0 && lutPtr->tempPtr[i] != END_TEMPS; i++)
  {
    // Current values
    double currRes = lutPtr->resPtr[i] * scaling;
    

#if DEBUG_LUT_HANDLING==TRUE
    Serial.print("D> Resistor value in LUT: ");
    Serial.println(currRes);
#endif 

    if (ntcThermistor && measuredRes > currRes)
    {
      // For NTC
      
#if DEBUG_LUT_HANDLING==TRUE
      Serial.println("D> NTC value found");
#endif 
      foundIndex = i;
      break;
    }
    else if (!ntcThermistor && measuredRes < currRes)
    {
      // For PTC
      
#if DEBUG_LUT_HANDLING==TRUE
      Serial.println("D> PTC value found");
#endif 
      foundIndex = i;
      break;      
    } 
  }
  
  
#if DEBUG_LUT_HANDLING==TRUE
    Serial.print("D> Index in LUT: ");
    Serial.println(foundIndex);
#endif 

  if (foundIndex > 1)
  {
    // Interpolate between the current value and the old value
    int absTempDiff = lutPtr->tempPtr[foundIndex] - lutPtr->tempPtr[foundIndex - 1];
    if (absTempDiff < 0.0)
    {
      absTempDiff *= -1.0;
    }

    double absResDiff = lutPtr->resPtr[foundIndex] - lutPtr->resPtr[foundIndex - 1];
    if (absResDiff < 0.0)
    {
      absResDiff *= -1.0;
    } 

    int foundTempVal1 = lutPtr->tempPtr[foundIndex-1];
    int foundTempVal2 = lutPtr->tempPtr[foundIndex];
    double foundResVal1 = lutPtr->resPtr[foundIndex-1];
    double foundResVal2 = lutPtr->resPtr[foundIndex];

    if (foundTempVal1 < 0.0)
    {
      foundTempVal1 *= -1.0;
    }

    if (foundTempVal2 < 0.0)
    {
      foundTempVal2 *= -1.0;
    }

    // Calculate two coefficients
    double coeff1 = foundResVal1 / (double) foundTempVal1;
    double coeff2 = foundResVal2 / (double) foundTempVal2;

    // Take their mean and add it to the first one
    double resCoeff = coeff1 + ((coeff2 - coeff1) / 2);
    
    // Now divide our measured resistance with this value
    return (double)(measuredRes / resCoeff) / (double) 1000;
  }

#if DEBUG_LUT_HANDLING==TRUE
  Serial.println("E> Couldn't find entry in LUT");
  Serial.print("I> Measured voltage: ");
  Serial.println(volts);
  Serial.print("I> Calculated resistance: ");
  Serial.println(measuredRes, 4);
#endif

  return -999;
}


double FindResistance(int16_t temperature, FFF_Lut* lutPtr)
{
  for (int i = 0; lutPtr->resPtr[i] > 0.0 && lutPtr->tempPtr[i] != END_TEMPS; i++)
  {
    if (temperature <= lutPtr->tempPtr[i])
    {
    //   Serial.print("Found Temp in LUT at pos ");
    //   Serial.println(i);
      return i;
    }
  }

  return 0;

}


void calculateCoeffsSteinhartHart(FFF_Lut* lutPtr, double* aCoeffPtr, double* bCoeffPtr, int16_t t1, int16_t t2)
{
  const uint16_t maximumLutSize = 256; 
  int16_t temp1, temp2;
  temp1 = t1 - 273.15;
  temp2 = t2 - 273.15;

// Serial.println("Calculating coefficients for ");
// Serial.print("T1 = ");
// Serial.println(t1);
// Serial.print("T2 = ");
// Serial.println(t2);

  double t1Res = 0.0;
  double t2Res = 0.0;

  for (int i = 0; i < maximumLutSize && lutPtr->tempPtr[i] != NULL; i++)
  {
    uint16_t currTemp = lutPtr->tempPtr[i]; 
    uint16_t currRes = lutPtr->resPtr[i];

    if (currTemp >= t1 && t1 > -999)
    {
      t1Res = currRes;
      t1 = -1000; // invalidates t1 
    } else if (currTemp >= t2 && t2 > -999)
    {
      t2Res = currRes;
      t2 = -1000; // invalidates t1 
    }

    if (t1 == t2)
    {
      break;
    }
  }

    // Serial.print("R1 = ");
    // Serial.println(t1Res);
    // Serial.print("R2 = ");
    // Serial.println(t2Res);

    // T1Res, T2Res, T3Res are now available
    // Calculate coeffs
    *bCoeffPtr = 1/((1/(double) temp1 - 1/(double) temp2)*log(t1Res/t2Res));
    *aCoeffPtr = ((t2Res - t1Res) / (double) (t1Res*(temp2-temp1))) * pow(10.0,-6.0);
}


double calculateTempSteinhartHart(double alpha, double beta, double c, double normTemp, double resistance, FFF_Lut* lutPtr)
{
  // Serial.println("--------- SH ---------");
  // Serial.print("Coefficients alpha, beta, c: ");
  // Serial.println(alpha, 5);
  // Serial.println(beta, 5);
  // Serial.println(c, 5);

  double normRes = 0.0;
#ifdef NORM_RES
  if (NORM_RES <= 0.0)
  {
    uint16_t index = FindResistance(NORM_TEMP, lutPtr);
    normRes = lutPtr->scalingFac * lutPtr->tempPtr[index];
  } 
  else 
  {
    normRes = NORM_RES;
  }
#else 
  // We will serach for the norm resistance everytime this function gets called and itÄs alway the same
  uint16_t index = FindResistance(NORM_TEMP, lutPtr);
  normRes = lutPtr->scalingFac * lutPtr->tempPtr[index];
#endif

  // We convert to Kelvin
  normTemp += 273.15;
  double subTerm = log(normRes/resistance);
  double dividend = (normTemp * beta) / (subTerm);
  double divisor = (beta / subTerm) - normTemp;
  double quotient = dividend/divisor;
  double outputTemp = quotient - 273.15; // Convert back

  // Serial.print("Subterm: ");
  // Serial.println(subTerm, 5);
  // Serial.print("Dividend: ");
  // Serial.println(dividend, 5);
  // Serial.print("Divisor: ");
  // Serial.println(divisor, 5);
  return outputTemp;
}



void TASK_handleTemperature(void *param)
{
#if DEBUG_ADC == TRUE
    Serial.println("Started ADC task");
#endif  

  while (1)
  {
    double adcRawRead = 0.0;
    volatile double oldTemp;
    if (FFF_Adc_isReady())
    {
      FFF_Lut* lutPtr = FFF_Therm_getLut();

      adcRawRead = FFF_Adc_readVolt(EXT_ADC_TEMP_CHANNEL);
      // Traverse the lut and search for the point that comes nearest
      double voltDiff = FFF_DEVICE_SUPPLY - adcRawRead;
      double measuredRes = (adcRawRead * THERMISTOR_PULL_UP_VAL) / (voltDiff);
      #if DEBUG_LUT_HANDLING==TRUE
        double pupVal = THERMISTOR_PULL_UP_VAL;
        Serial.print("D> DIFFVOLTS: ");
        Serial.println(voltDiff, 4);
        Serial.print("D> PUPVAL: ");
        Serial.println(pupVal, 4);
        Serial.print("D> MEASRES: ");
        Serial.println(measuredRes, 4);
      #endif

      #if TEMPERATURE_CALC_METHOD==STEINHART_HART_METHOD
      // Use Steinhart-Hart
      // Serial.println("Calculating Steinhart-Hart:");
      //Steinhart-Hart
      const double steinhartCoeff_A = ALPHA_COEFF;
      const double steinhartCoeff_B = BETA_COEFF;
      const double steinhartCoeff_C = 0.0;

      hotendTemperature = calculateTempSteinhartHart(steinhartCoeff_A, 
                                              steinhartCoeff_B, 
                                              steinhartCoeff_C, 
                                              NORM_TEMP, 
                                              measuredRes, 
                                              lutPtr); 
      #else
      hotendTemperature = LookupTemperature(adcRawRead, lutPtr, measuredRes);
      #endif
    }

    double diff = hotendTemperature - oldTemp; 
    if (diff < 0.0)
    {
      diff *= -1;
    }

#if DEBUG_ADC == TRUE
    Serial.print("I> ADC raw: ");
    Serial.println(adcRawRead, 4);
    Serial.print("I> TH0: ");
    Serial.print(hotendTemp, 1);
    Serial.println(" deg C");
    Serial.print("DeltaT: ");
    Serial.println(diff, 4);
#endif

    if (diff > MAX_TEMP_DELTA_DEG)
    {
      // Some erroneous reading so take the previous
      hotendTemperature = oldTemp; 
    } 
    else 
    {
      oldTemp = hotendTemperature;
    }
    vTaskDelay(ADC_SAMPLE_INTERVAL_MS);
  }
}


double FFF_Temp_getTemperature()
{
  return hotendTemperature;
}