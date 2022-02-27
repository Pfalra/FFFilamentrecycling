#ifndef FFF_TYPES_H
#define FFF_TYPES_H

#include <Arduino.h>
#include <stdint.h>

// #define NULL 0x0

/* LOGGING */
#define MAX_NUM_LOG_FILES 32
#define MAX_LOG_PARAM_LEN 64
#define MAX_LOG_DATA_LEN 64

#define MAX_OUT_STRING_LEN 512
#define FALSE 0
#define TRUE 1

typedef enum FFF_AppStatus_e
{
  APP_STOPPED,
  APP_RUNNING,
  APP_PAUSED
} FFF_AppStatus;

typedef enum FFF_ModStatus_e
{
    MOD_HW_NOT_DETECTED,
    MOD_FAULT,
    MOD_NOT_INITIALIZED,
    MOD_INIT_RUNNING,
    MOD_FUNCTIONAL
} FFF_ModStatus;


typedef struct FFF_Measurement_s
{
    float outputVal; 
    float scalingCoeff;   
    uint16_t mean;
    uint16_t len;
    uint16_t startIndex;
    uint16_t endIndex;
    uint16_t maxIndex;
    uint16_t minIndex;
    uint16_t firstLimPass;
    uint16_t lastLimPass;
    uint16_t passHyst;
    uint16_t passWidth;
    uint8_t maxVal;
    uint8_t minVal;
    uint8_t analyzed;
    uint8_t protectFlag;
    uint8_t* dataPoints;
} FFF_Measurement;


typedef struct FFF_Lut_s
{
    int* tempPtr;
    float* resPtr; 
    float scalingFac;
    float volts[100];
} FFF_Lut;


typedef struct FFF_Stepper_s
{
  uint8_t id;
  uint8_t stepPin;
  uint8_t microsteps;
  double targetSpeed;
} FFF_Stepper;


typedef struct FFF_Log_s
{
  const char* name;
  const char* paramStr;
  char dataStr[MAX_LOG_DATA_LEN];
  bool isProtected;
  bool isActive;
  void* logFilePtr;
} FFF_Log;


#endif