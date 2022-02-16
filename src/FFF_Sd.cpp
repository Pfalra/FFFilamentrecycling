#include <FFF_Sd.h>
#include <SPI.h>
#include <SD_MMC.h>
#include <SD.h>
#include <FS.h>
#include <Arduino.h>
#include <FFF_Types.h>
#include <FFF_Settings.h>

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


bool FFF_SD_listFiles(fs::FS &fs, void* (printFuncPtr)(const char*))
{
  const char* dir = "/";
  File root = fs.open(dir);

  if(!root){
    printFuncPtr("Failed to open directory");
    return false;
  }

  if(!root.isDirectory()){
    printFuncPtr("Could not open root dir");
    return false;
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

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}


void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}


void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}


void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.println("SD test successful!");
  file.close();
}