#ifndef FFF_SD_H
#define FFF_SD_H

#include <FFF_Types.h>

typedef enum FFF_Sdmode_e
{
    SD_OVER_SPI,
    SD_OVER_SDIO
} FFF_Sdmode;

extern bool FFF_SDReader_connected;

bool FFF_SD_Init(FFF_Sdmode mode);

FFF_ModStatus FFF_getSDStatus();

bool FFF_reconnectSD();

void FFF_reportSDEvent();

#endif