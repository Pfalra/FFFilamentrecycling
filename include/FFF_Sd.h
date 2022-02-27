#ifndef FFF_SD_H
#define FFF_SD_H

#include <FFF_Types.h>
#include <FS.h>
#include <Arduino.h>

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

void FFF_SD_openLogFile(String fileName, File* fptr);

void FFF_SD_appendLineToLogFile(String fileName, String line);

bool FFF_SD_getFileList(char outputArr[], uint16_t len);

bool FFF_SD_attachLogFile(FFF_Log* logPtr);

void FFF_SD_detachLogFile(FFF_Log* logPtr);

int8_t FFF_SD_getFileCount();


bool FFF_SD_writeToFile(File* fptr, const char line[]);
bool FFF_SD_writeToFile(File* fptr, char line[]);
#endif