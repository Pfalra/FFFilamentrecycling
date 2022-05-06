#include <FFF_DiaAnalyzer.h>
#include <FFF_Settings.h>
#include <stdint.h>
#include <FFF_Uart.h>
#include <FFF_Rtos.h>

#define MAX(a, b) ((a) < (b) ? (b) : (a))

SemaphoreHandle_t syncUartSemaphore = NULL;

TaskHandle_t DiaReadTaskHandle;

bool diaBuffer0Ready = false;
// bool diaBuffer1Ready = false;

uint8_t diaMeasPoints[MEASUREMENT_LENGTH];

FFF_Measurement diameterMeasurement = {
  .outputVal = 0.0,
  .scalingCoeff = DIAMETER_SCALING_COEFF,
  .mean = 0,
  .len = MEASUREMENT_LENGTH,
  .startIndex = SYNC_LENGTH + FIRST_DEAD_PIX,
  .endIndex = MEASUREMENT_LENGTH - LAST_DEAD_PIX,
  .maxIndex = 0,
  .minIndex = 0,
  .firstLimPass = 0,
  .lastLimPass = 0,
  .passHyst = DIA_MEAS_HYST,
  .passWidth = 0,
  .maxVal = 0,
  .minVal = 255,
  .maxPosEdge = 0,
  .maxNegEdge = 0,
  .posEdgeInd = 0,
  .negEdgeInd = 0,
  .analyzed = false,
  .dataBuffer = NULL
};


static inline void evalMinMax(FFF_Measurement *meas, uint16_t index)
{
    if (meas->maxVal < meas->dataBuffer->dataPtr[index])
    {
        meas->maxVal = meas->dataBuffer->dataPtr[index];
        meas->maxIndex = index;
    }

    if (meas->dataBuffer->dataPtr[index] < meas->minVal)
    {
        meas->minVal = meas->dataBuffer->dataPtr[index];
        meas->minIndex = index;
    }
}


static inline void evalDiffEdges(FFF_Measurement* meas, int16_t index)
{
    int16_t diff = meas->dataBuffer->dataPtr[index] - meas->dataBuffer->dataPtr[index - meas->passHyst];
    if (diff >= 0)
    {
        // Positive edge
        if (meas->maxPosEdge < diff)
        {
            meas->maxPosEdge = diff;
            meas->posEdgeInd = index;
        }
    }
    else 
    {
        // Negative edge

        if (meas->maxNegEdge > diff)
        {
            meas->maxNegEdge = diff;
            meas->negEdgeInd = index;
        }
    }
}


static uint16_t evalLimitPass(FFF_Measurement *meas, uint16_t index)
{
    uint16_t hystVal = meas->passHyst;
    uint16_t currMean = meas->mean;

    uint16_t tempHystIndex0, tempHystIndex1;

    /* FIXIT: Only working if we traverse the measurement data in forward direction */
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


    if (meas->dataBuffer->dataPtr[tempHystIndex0] > currMean && meas->dataBuffer->dataPtr[tempHystIndex1] < currMean)
    {
        // °°°\...
        // Find the correct index where the passing happens
        for (uint16_t i = tempHystIndex0; i <= tempHystIndex1; i++)
        {
            if (meas->dataBuffer->dataPtr[i] < currMean)
            {
                return i;
            }
        }
    }
    else if (meas->dataBuffer->dataPtr[tempHystIndex0] < currMean && meas->dataBuffer->dataPtr[tempHystIndex1] > currMean)
    {
        // .../°°°
        for (uint16_t i = tempHystIndex0; i <= tempHystIndex1; i++)
        {
            if (meas->dataBuffer->dataPtr[i] > currMean)
            {
                return i;
            }
        }
    }

    return 0;
}

static uint16_t weightDataProperty(uint16_t oldVal, uint16_t currVal, uint16_t weigthCoeff)
{
    currVal = ((uint16_t) MEANFILTER_WEIGHT_COEFF * ((uint16_t) oldVal) + (uint16_t) currVal ) / ((uint16_t) MEANFILTER_WEIGHT_COEFF + 1);
    return currVal;
}


static inline void findMinMax(FFF_Measurement *meas)
{
    // Traverse forwards within the bounds we specified
    meas->maxVal = 0;
    meas->minVal = 255;

    for (int16_t i = meas->startIndex; i <= meas->endIndex; i++)
    {
        evalMinMax(meas, i);
        evalDiffEdges(meas, i);
    }
    
}


static inline void findPassWidth(FFF_Measurement *meas)
{
    // First iteration or error
    if (meas->minIndex < meas->startIndex)
    {
        return;
    }
    
    meas->lastLimPass = 0;
    meas->firstLimPass = 0;

    // Traverse from minIndex to right
    for (int16_t i = meas->minIndex; (i <= meas->endIndex) && (meas->dataBuffer->dataPtr[i] <= meas->mean); i++)
    {
        meas->lastLimPass = i;

        // Error case
        if (i >= meas->endIndex)
        {
            // We didnt find a pass somehow
            meas->lastLimPass = 0;
            meas->firstLimPass = 0;
            Serial.println("E> No pass found!");
            return;
        }
    }

    // Traverse from minIndex to left
    for (int16_t i = meas->minIndex; (i >= meas->startIndex) && (meas->dataBuffer->dataPtr[i] <= meas->mean); i--)
    {
        meas->firstLimPass = i;

        // Error case
        if (i <= meas->startIndex)
        {
            // We didnt find a pass somehow
            meas->lastLimPass = 0;
            meas->firstLimPass = 0;
            Serial.println("E> No pass found!");
            return;
        }
    }

    // Everything went fine, so we can calc the passWdith
    meas->passWidth = meas->lastLimPass - meas->firstLimPass;
}


void FFF_DiaAn_calcOutput(FFF_Measurement *meas, uint16_t inVal)
{
    meas->outputVal = meas->scalingCoeff * ((float)inVal);
}


void FFF_DiaAn_analyze(FFF_Measurement *meas)
{
    /* Check if analysis was already done */
    if (meas->analyzed)
    {
        return; // Measurement analysis was already done
    }

    /* Check if the dataBuffer is NULL */
    if (!meas->dataBuffer)
    {
        return;
    }

    meas->endIndex = meas->dataBuffer->len - LAST_DEAD_PIX;
    Serial.print("Start:");
    Serial.println(diameterMeasurement.startIndex);
    Serial.print("End:");
    Serial.println(diameterMeasurement.endIndex);

    if (!(meas->endIndex > meas->startIndex))
    {
        Serial.println("E> Index Error. EI <= SI");
        Serial.println(meas->endIndex);
        Serial.println(meas->startIndex);
        return;
    }

    /* Check if the definition of the measurement is erroneous */
    if(meas->maxIndex > meas->dataBuffer->len)
    {
        Serial.println("E> Error in measurement definition. Check bounds!");
        return;
    }

    if (meas->mean <= 0.0)
    {
        // Seems like first iteration
        // So don't take the mean into account
        // Serial.println("First iteration");
        findMinMax(meas);
        meas->mean = ((meas->maxVal - meas->minVal) / 2) + meas->minVal;
        // findPassWidth(meas);
        meas->passWidth = abs(meas->posEdgeInd - meas->negEdgeInd);
        FFF_DiaAn_calcOutput(meas, meas->passWidth);
        meas->analyzed = TRUE;
        return;
    }
    else
    {
        if (MEANFILTER_ACTIVE == true)
        {
            findMinMax(meas);
            meas->mean = ((((meas->maxVal - meas->minVal) / 2) + meas->minVal) + (meas->mean * MEANFILTER_WEIGHT_COEFF)) / (MEANFILTER_WEIGHT_COEFF + 1);
            // findPassWidth(meas);
            meas->passWidth = abs(meas->posEdgeInd - meas->negEdgeInd);
            float oldOutVal = meas->outputVal;
            FFF_DiaAn_calcOutput(meas, meas->passWidth);
            meas->outputVal = (MEANFILTER_WEIGHT_COEFF * oldOutVal + meas->outputVal) / (MEANFILTER_WEIGHT_COEFF + 1);
            meas->analyzed = TRUE;
        }
        else 
        {
            findMinMax(meas);
            meas->mean = ((meas->maxVal - meas->minVal) / 2) + meas->minVal;
            meas->passWidth = abs(meas->posEdgeInd - meas->negEdgeInd);
            // findPassWidth(meas);
            FFF_DiaAn_calcOutput(meas, meas->passWidth);
            meas->analyzed = TRUE;
        }
    }

       
    Serial.print("MaDInd: ");
    Serial.println(meas->posEdgeInd);
    Serial.print("MiDInd: ");
    Serial.println(meas->negEdgeInd);
    Serial.print("Dia [mm]: ");
    Serial.println(diameterMeasurement.outputVal);

    return;
}


double FFF_DiaAn_getDiameter()
{
    return diameterMeasurement.outputVal;
}


TaskHandle_t* FFF_DiaAn_getTaskHandle()
{
    return &DiaReadTaskHandle;
}


void FFF_DiaAn_giveBackSemaphoreFromISR()
{
    xSemaphoreGiveFromISR(syncUartSemaphore, NULL);
}

/***************************************/
/* TASK */
/***************************************/
void TASK_handleDiaAnalysis(void* param)
{

    while(1)
    {
        if (diaBuffer0Ready)
        {
            diaBuffer0Ready = false;
            FFF_Buffer* bufPtr = FFF_Uart_getFilledBufferUart2();

            if (bufPtr)
            {
                diameterMeasurement.dataBuffer = bufPtr;
                diameterMeasurement.analyzed = false;
                diameterMeasurement.maxIndex = bufPtr->len;
                FFF_DiaAn_analyze(&diameterMeasurement);
                diameterMeasurement.dataBuffer = NULL;
            }
        }

        vTaskDelay(FPGA_CALCULATE_DIAMETER_INTERVAL_MS);
    }

    // // This task will get activated from ISR of UART2
    // syncUartSemaphore = xSemaphoreCreateBinary();
    // while(1)
    // {
    //     if (xSemaphoreTake(syncUartSemaphore, portMAX_DELAY) == pdPASS)
    //     {
    //         // Serial.println("I>DAN");
    //         // After suspension we have to get the current filled up buffer for analysis and protect it
    //         FFF_Buffer* bufPtr = FFF_Uart_getFilledBufferUart2();

    //         if (bufPtr)
    //         {
    //             FFF_Uart_protectBufferUart2(bufPtr);
    //             diameterMeasurement.dataBuffer = bufPtr;
    //             diameterMeasurement.analyzed = false;
    //             diameterMeasurement.maxIndex = bufPtr->len;

    //             FFF_DiaAn_analyze(&diameterMeasurement);


    //             FFF_Uart_unprotectBufferUart2(bufPtr);
    //             diameterMeasurement.dataBuffer = NULL;
    //         } 
    //     }
    // }
}