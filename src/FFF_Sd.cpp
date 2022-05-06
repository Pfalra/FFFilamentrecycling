// #include <FFF_Sd.h>
// #include <SPI.h>
// #include <SD_MMC.h>
// #include <SD.h>
// #include <FS.h>
// #include <Arduino.h>
// #include <FFF_Types.h>
// #include <FFF_Settings.h>

// bool reader_connected = false;

// SPIClass sdSpi(HSPI);

// bool FFF_SD_Init(FFF_Sdmode mode)
// {
//   bool sdInserted = false;
//   if (mode == SD_OVER_SPI)
//   {
//     /* Configure basic pins */
//     pinMode(SPI_CS_PIN_SD, OUTPUT);
//     sdSpi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN_SD);

//     if (!SD.begin(SPI_CS_PIN_SD, sdSpi))
//     {
//       /* Show error on screen and neopixel */
//       while (!SD.begin(SPI_CS_PIN_SD))
//       {
//         Serial.println("ERROR: Was not able to mount SD!");
//         vTaskDelay(500);
//       }
//     }

//     reader_connected = true;

//     uint8_t cardType = SD.cardType();

//     Serial.print("SD Type: ");
//     switch (cardType)
//     {
//     case CARD_SD:
//       Serial.println("SD");
//       sdInserted = true;
//       break;
//     case CARD_SDHC:
//       Serial.println("SDHC");
//       sdInserted = true;
//       break;
//     case CARD_MMC:
//       Serial.println("MMC");
//       sdInserted = true;
//       break;
//     case CARD_UNKNOWN:
//       Serial.println("Unknown type");
//       sdInserted = true;
//       break;
//     case CARD_NONE:
//       Serial.println("No card inserted");
//       sdInserted = false;
//       break;
//     }

//     if (sdInserted)
//     {
//       uint64_t cardSize = SD.cardSize() / (1024 * 1024);
//       Serial.printf("SD Card Size: %lluMB\n", cardSize);
//     }
//   } 
//   else if(mode == SD_OVER_SDIO)
//   {
//     // TODO
//   }

//   return true; 
// }


// FFF_ModStatus FFF_getSDStatus()
// {
//   sdcard_type_t cardType = SD.cardType();
  
//   if (cardType == CARD_NONE || cardType == CARD_UNKNOWN)
//   {
//     return MOD_FAULT;
//   }

//   if (!reader_connected)
//   {
//     return MOD_HW_NOT_DETECTED;
//   }

//   return MOD_FUNCTIONAL;
// }


// bool FFF_reconnectSD()
// {
//   if (!SD.begin(SPI_CS_PIN_SD))
//   {
//     Serial.println("ERROR: Was not able to mount SD!");
//     return false;
//   }

//   return true;
// }


// void FFF_reportSDEvent()
// {

// }


// void FFF_SD_openLogFile(String fileName, File* fptr)
// {
//   *fptr = SD.open("/" + fileName, FILE_WRITE);
  
//   if(!*fptr){
//     Serial.println("Failed to open files for writing");
//     return;
//   }
// }


// bool FFF_SD_writeToFile(File* fptr, char line[])
// {
//   if (fptr->availableForWrite())
//   {
//     fptr->println(line);
//     return true;
//   }
//   else 
//   {
//     Serial.println("E> Error in file access");
//     return false;
//   }

//   return false;
// }


// bool FFF_SD_writeToFile(File* fptr, const char line[])
// {
//   if (fptr->availableForWrite())
//   {
//     fptr->println(line);
//     return true;
//   }
//   else 
//   {
//     Serial.println("E> Error in file access");
//     return false;
//   }

//   return false;
// }


// void FFF_SD_appendLineToLogFile(String fileName, String line)
// {
//   char lineArr[64];

//   File file = SD.open("/" + fileName, FILE_APPEND);
//   if(!file){
//     Serial.println("Failed to open file for appending");
//     return;
//   }

//   line += NL;
//   line.toCharArray(lineArr, line.length());

//   if (file.print(lineArr))
//   {
//     Serial.println("Line appended");
//   } 
//   else 
//   {
//     Serial.println("Failed to append line");
//   }

//   file.close();
// }


// bool FFF_SD_getFileList(char outputArr[], uint16_t len)
// {
//   String outStr = "";
//   const char* dir = "/";
//   File root = SD.open(dir);

//   memset(outputArr, '\0', len);
//   if (!root && !root.isDirectory())
//   {
//     const char errAccessMsg[] = "E>ROOT NOT FOUND OR NOT ACCESSIBLE" NL;
//     strncpy(outputArr, errAccessMsg, sizeof(errAccessMsg));
//     return false;
//   }

//   File file = root.openNextFile();
//   uint8_t fileCount = 0;
//   while(file)
//   {
//     if (!file.isDirectory())
//     {
//       outStr += file.name();
//       outStr += NL;
//     }
//     file = root.openNextFile();
//     fileCount++;
//   }

//   if (fileCount > MAX_NUM_LOG_FILES)
//   {
//     const char errFileMsg[] = "E>Too many log files";
//     strncpy(outputArr, errFileMsg, sizeof(errFileMsg));
//   }

//   outStr.toCharArray(outputArr, len);
//   return true;
// }


// int8_t FFF_SD_getFileCount()
// {
//   const char* dir = "/";
//   File root = SD.open(dir);

//   if(!root) 
//   {
//     Serial.println("E>Failed to open directory");
//     return -1;
//   }

//   if(!root.isDirectory())
//   {
//     Serial.println("E>Could not open root dir");
//     return -1;
//   }

//   File file = root.openNextFile();
//   uint8_t counter = 0;
//   while(file)
//   {
//     if(!file.isDirectory()){
//       counter++;
//     }
//     file = root.openNextFile();
//   }

//   return counter;
// }


// bool FFF_SD_attachLogFile(FFF_Log* logPtr)
// {
//   String filename = "log_" + FFF_SD_getFileCount();
//   filename += FFF_LOG_FILE_FORMAT;

//   FFF_SD_openLogFile(filename, (File*) logPtr->logFilePtr);

//   if (logPtr->logFilePtr)
//   {
//     return true;
//   }

//   Serial.println("E>Could not attach file to log.");
//   return false;
// }


// void FFF_SD_detachLogFile(FFF_Log* logPtr)
// {
//   File* filePtr = (File*) logPtr->logFilePtr;
//   filePtr->close();
//   logPtr->logFilePtr = NULL;
// }


// bool FFF_SD_isReaderConnected()
// {
//   return reader_connected;
// } 