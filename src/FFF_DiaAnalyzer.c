#include <FFF_DiaAnalyzer.h>
#include <FFF_Settings.h>
#include <stdint.h>

#define MAX(a, b) ((a) < (b) ? (b) : (a))


static inline void evalMinMax(FFF_Measurement* meas, uint16_t index)
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

static inline void evalLimitPass(FFF_Measurement* meas, uint16_t index, uint8_t backTraverse)
{
    uint8_t currVal = meas->dataPoints[index];
    uint16_t hystVal = meas->passHyst;
    uint16_t currMean = meas->mean;

    // FIXME: Handle cases where index and hysteresis limit dataBuf bounds
    if (backTraverse)
    {

    }
    else
    {
        if (meas->dataPoints[index - hystVal] > currMean && meas->dataPoints[index + hystVal] < currMean)
        {
            // °°°\...
            

        }
        else 
        {
            // .../°°°


        }

    }

}

static inline void weightDataPoint(FFF_Measurement* meas, uint16_t index)
{
    // TODO: Implement!
}




void FFF_DiaAn_calcOutput(FFF_Measurement* meas, uint16_t inVal)
{
    meas->outputVal = meas->scalingCoeff * ((float) inVal);
}


// TODO: Implement some hysteresis and test this thing with real data
/* FIXME?: Check if we should base the mean limit passage on the previous mean or on the current! 
        CURRENTLY WE TAKE THE PREVIOUS */

void FFF_DiaAn_analyze(FFF_Measurement* meas)
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

    if (meas->mean <= 0.0)
    {
        // Seems like first iteration
        // So don't take the mean into account 

        if (meas->endIndex > meas->startIndex)
        {
            // Traverse forwards
            uint16_t range = meas->endIndex - meas->startIndex;

            for (uint16_t i = meas->startIndex; i <= meas->endIndex; i++)
            {
                // Take the max and min values and their indices
                evalMinMax(meas, i);
            }
        }
        else 
        {
            // Traverse backwards
            uint16_t range = meas->startIndex - meas->endIndex;

            for (uint16_t i = meas->endIndex; i >= meas->startIndex; i--)
            {                
                // Take the max and min values and their indices
                evalMinMax(meas, i);
            }
        }
    }
    else 
    {
        uint8_t oldMin = meas->minVal;
        uint8_t oldMax = meas->maxVal;
        uint16_t oldMinIndex = meas->minIndex;
        uint16_t oldMaxIndex = meas->maxIndex;
        uint16_t oldMean = meas->mean;
        uint16_t oldFirstLimPass = meas->firstLimPass;
        uint16_t oldLastLimPass = meas->lastLimPass;


        // Consecutive analysis
        if (MEANFILTER_ACTIVE)
        {
            if (meas->endIndex > meas->startIndex)
            {
                // Traverse forwards
                uint16_t range = meas->endIndex - meas->startIndex;

                for (uint16_t i = meas->startIndex; i <= meas->endIndex; i++)
                {
                    

                    evalMinMax(meas, i);
                }
            }
            else 
            {
                // Traverse backwards
                uint16_t range = meas->startIndex - meas->endIndex;

                for (uint16_t i = meas->endIndex; i >= meas->startIndex; i--)
                {            


                    evalMinMax(meas, i);
                }
            }
        }
        else 
        {
            // Don't use filtering  
            if (meas->endIndex > meas->startIndex)
            {
                // Traverse forwards
                uint16_t range = meas->endIndex - meas->startIndex;

                for (uint16_t i = meas->startIndex; i <= meas->endIndex; i++)
                {
                    

                    evalMinMax(meas, i);
                }
            }
            else 
            {
                // Traverse backwards
                uint16_t range = meas->startIndex - meas->endIndex;

                for (uint16_t i = meas->endIndex; i >= meas->startIndex; i--)
                {                


                    evalMinMax(meas, i);
                }
            }
        }
    }


    // Calculate the other parameters
    meas->mean = meas->maxVal - meas->minVal;


    meas->analyzed = TRUE;
    return;
}
