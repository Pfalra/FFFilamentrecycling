#include <FFF_Log.h>

TaskHandle_t LogTaskHandle;



TaskHandle_t* FFF_Log_getTaskHandle()
{
    return &LogTaskHandle;
}