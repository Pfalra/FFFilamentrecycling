#include <FFF_DiaAnalyzer.h>
#include <FFF_Settings.h>
#include <stdint.h>

#define MAX(a, b) ((a) < (b) ? (b) : (a))

static inline void evalMinMax(FFF_Measurement *meas, uint16_t index)
{
    if (meas->maxVal < meas->dataPoints[index])
    {
        meas->maxVal = meas->dataPoints[index];
        meas->maxIndex = index;
    }

    if (meas->dataPoints[index] < meas->minVal)
    {
        meas->minVal = meas->dataPoints[index];
        meas->minIndex = index;
    }
}


static uint16_t evalLimitPass(FFF_Measurement *meas, uint16_t index)
{
    uint16_t hystVal = meas->passHyst;
    uint16_t currMean = meas->mean;

    uint16_t tempHystIndex0, tempHystIndex1;

    /* FIXME: Only working if we traverse the measurement data in forward direction */
    if (index - hystVal < meas->startIndex)
    {
        tempHystIndex0 = meas->startIndex;
    } 
    else 
    {
        tempHystIndex0 = index - hystVal;
    }


    if (index + hystVal > meas->endIndex)
    {
        tempHystIndex1 = meas->endIndex;
    } 
    else 
    {
        tempHystIndex1 = index + hystVal;
    }


    if (meas->dataPoints[tempHystIndex0] > currMean && meas->dataPoints[tempHystIndex1] < currMean)
    {
        // °°°\...
        // Find the correct index where the passing happens
        for (uint16_t i = tempHystIndex0; i <= tempHystIndex1; i++)
        {
            if (meas->dataPoints[i] < currMean)
            {
                return i;
            }
        }
    }
    else if (meas->dataPoints[tempHystIndex0] < currMean && meas->dataPoints[tempHystIndex1] > currMean)
    {
        // .../°°°
        for (uint16_t i = tempHystIndex0; i <= tempHystIndex1; i++)
        {
            if (meas->dataPoints[i] > currMean)
            {
                return i;
            }
        }
    }

    return 0;
}


static uint16_t weightDataProperty(uint16_t oldVal, uint16_t currVal, uint16_t weigthCoeff)
{
    currVal = ((uint16_t) MEANFILTER_WEIGHT_KOEFF * ((uint16_t) oldVal) + (uint16_t) currVal ) / ((uint16_t) MEANFILTER_WEIGHT_KOEFF + 1);
    return currVal;
}


void FFF_DiaAn_calcOutput(FFF_Measurement *meas, uint16_t inVal)
{
    meas->outputVal = meas->scalingCoeff * ((float)inVal);
}

// TODO: Implement some hysteresis and test this thing with real data
/* FIXME?: Check if we should base the mean limit passage on the previous mean or on the current!
        CURRENTLY WE TAKE THE PREVIOUS */

void FFF_DiaAn_analyze(FFF_Measurement *meas)
{
    if (meas->analyzed)
    {
        return; // Measurement analysis was already done
    }

    if (meas->protectFlag)
    {
        return; // Measurement is protected
    }

    if (meas->dataPoints == NULL_PTR)
    {
        return;
    }


    // uint8_t oldMin = meas->minVal;
    // uint8_t oldMax = meas->maxVal;
    // uint16_t oldMinIndex = meas->minIndex;
    // uint16_t oldMaxIndex = meas->maxIndex;
    // uint16_t oldFirstLimPass = meas->firstLimPass;
    // uint16_t oldLastLimPass = meas->lastLimPass;
    uint16_t oldMean = meas->mean;
    uint16_t oldPassWidth = meas->passWidth;
    uint8_t firstPassFound = FALSE;

    if (meas->mean <= 0.0)
    {
        // Seems like first iteration
        // So don't take the mean into account

        if (meas->endIndex > meas->startIndex)
        {
            // Traverse forwards
            for (uint16_t i = meas->startIndex; i <= meas->endIndex; i++)
            {
                evalMinMax(meas, i);
            }
        }
        else
        {
            // Traverse backwards
            // TODO: Implement if needed
        }

        meas->mean = (meas->maxVal - meas->minVal) / 2;
        meas->analyzed = TRUE;
        return;
    }
    else
    {
        // Consecutive analysis
        if (meas->endIndex > meas->startIndex)
        {
            // Traverse forwards
            for (uint16_t i = meas->startIndex; i <= meas->endIndex; i++)
            {
                uint16_t passIndex = evalLimitPass(meas, i);

                if (passIndex > 0)
                {
                    if (!firstPassFound)
                    {
                        // First pass was detected
                        firstPassFound = TRUE;
                        meas->firstLimPass = passIndex;
                    }
                    else
                    {
                        // Second pass
                        meas->lastLimPass = passIndex;
                    }

                    // A pass was found so jump over the points within the hysteresis
                    i = passIndex + meas->passHyst;
                }

                evalMinMax(meas, i);
            }
        }
        else
        {
            // Traverse backwards
            // TODO: Implement if needed
        }
    }

    // Calculate the other parameters
    if (MEANFILTER_ACTIVE)
    {
        // Don't do filtering if mean is zero as this would corrupt the measurement for a few analyzations
        if (oldMean > 0)
        {
            meas->mean = weightDataProperty(oldMean, meas->mean, MEANFILTER_WEIGHT_KOEFF);
            meas->passWidth = weightDataProperty(oldPassWidth, meas->passWidth, MEANFILTER_WEIGHT_KOEFF);
        }
    }
    else 
    {
        meas->mean = (meas->maxVal - meas->minVal) / 2;
        meas->passWidth = meas->lastLimPass - meas->firstLimPass;
    }

    return;
}
