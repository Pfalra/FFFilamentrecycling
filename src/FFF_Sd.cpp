#include <FFF_Sd.h>
#include <SPI.h>
#include <SD_MMC.h>
#include <SD.h>
#include <FS.h>
#include <Arduino.h>
#include <FFF_Types.h>
#include <FFF_Settings.h>
#include <string.h>

bool FFF_SDReader_connected = false;

bool FFF_SD_Init(FFF_Sdmode mode)
{
  bool sdInserted = false;
  if (mode == SD_OVER_SPI)
  {
    /* Configure basic pins */
    pinMode(SPI_CS_PIN_SD, OUTPUT);

    if (!SD.begin(SPI_CS_PIN_SD))
    {
      Serial.println("ERROR: Was not able to mount SD!");
      /* Show error on screen and neopixel */
      while (!SD.begin(SPI_CS_PIN_SD))
      {
        vTaskDelay(500);
      }
    }

    FFF_SDReader_connected = true;

    uint8_t cardType = SD.cardType();

    Serial.print("SD Type: ");
    switch (cardType)
    {
    case CARD_SD:
      Serial.println("SD");
      sdInserted = true;
      break;
    case CARD_SDHC:
      Serial.println("SDHC");
      sdInserted = true;
      break;
    case CARD_MMC:
      Serial.println("MMC");
      sdInserted = true;
      break;
    case CARD_UNKNOWN:
      Serial.println("Unknown type");
      sdInserted = true;
      break;
    case CARD_NONE:
      Serial.println("No card inserted");
      sdInserted = false;
      break;
    }

    if (sdInserted)
    {
      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("SD Card Size: %lluMB\n", cardSize);
    }
  } 
  else if(mode == SD_OVER_SDIO)
  {
    // TODO
  } 
}


FFF_ModStatus FFF_getSDStatus()
{
  sdcard_type_t cardType = SD.cardType();
  
  if (cardType == CARD_NONE || cardType == CARD_UNKNOWN)
  {
    return MOD_FAULT;
  }

  if (!FFF_SDReader_connected)
  {
    return MOD_HW_NOT_DETECTED;
  }

  return MOD_FUNCTIONAL;
}


bool FFF_reconnectSD()
{
  if (!SD.begin(SPI_CS_PIN_SD))
  {
    Serial.println("ERROR: Was not able to mount SD!");
    return false;
  }

  return true;
}


void FFF_reportSDEvent()
{

}


/* PrintFuncPtr for optional future use for printing to another interface */
void FFF_SD_listFiles(void* (printFuncPtr)(const char*))
{
  const char* dir = "/";
  File root = SD.open(dir);

  if(!root){
    printFuncPtr("Failed to open directory");
    return;
  }

  if(!root.isDirectory()){
    printFuncPtr("Could not open root dir");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(!file.isDirectory()){
      printFuncPtr("  FILE: ");
      printFuncPtr(file.name());
    }
    file = root.openNextFile();
  }
}


void FFF_SD_openLogFile(String fileName)
{
  File file = SD.open("/" + fileName, FILE_WRITE);
  
  if(!file){
    Serial.println("Failed to open files for writing");
    return;
  }
  file.close();
}


void FFF_SD_appendLineToLogFile(String fileName, String line)
{
  char lineArr[64];

  File file = SD.open("/" + fileName, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }

  line += NL;
  line.toCharArray(lineArr, line.length());

  if (file.print(lineArr))
  {
    Serial.println("Line appended");
  } 
  else 
  {
    Serial.println("Failed to append line");
  }

  file.close();
}
