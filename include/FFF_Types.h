#ifndef FFF_TYPES_H
#define FFF_TYPES_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// #define NULL 0x0

/* LOGGING */
#define MAX_NUM_LOG_FILES 32
#define MAX_LOG_PARAM_LEN 64
#define MAX_LOG_DATA_LEN 64

#define MAX_OUT_STRING_LEN 512
#define FALSE 0
#define TRUE 1

typedef enum FFF_PwmResolution_s
{
  FFF_PWM_RES_1BIT = 1,
  FFF_PWM_RES_2BIT = 2,
  FFF_PWM_RES_3BIT = 3,
  FFF_PWM_RES_4BIT = 4,
  FFF_PWM_RES_5BIT = 5,
  FFF_PWM_RES_6BIT = 6,
  FFF_PWM_RES_7BIT = 7,
  FFF_PWM_RES_8BIT = 8,
  FFF_PWM_RES_9BIT = 9,
  FFF_PWM_RES_10BIT = 10,
  FFF_PWM_RES_11BIT = 11,
  FFF_PWM_RES_12BIT = 12,
  FFF_PWM_RES_13BIT = 13,
  FFF_PWM_RES_14BIT = 14,
  FFF_PWM_RES_15BIT = 15,
  FFF_PWM_RES_16BIT = 16,
  FFF_PWM_RES_17BIT = 17,
  FFF_PWM_RES_18BIT = 18,
  FFF_PWM_RES_19BIT = 19,
  FFF_PWM_RES_20BIT = 20
} FFF_PwmResolution;

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
    const int* tempPtr;
    const float* resPtr; 
    float scalingFac;
    float volts[100];
} FFF_Lut;


typedef struct FFF_PwmHandle_s
{
  uint8_t id;
  uint8_t outputPin;
  uint8_t channel;
  FFF_PwmResolution resolution;
  double dutyCycle; // Duty cycle in percentage
  double freq;
} FFF_PwmHandle;

typedef struct FFF_Stepper_s
{
  uint8_t id;
  uint8_t stepPin;
  uint8_t microsteps;
  FFF_PwmHandle* pwmPtr;
  float gearRatio;
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


typedef struct FFF_Buffer_s
{
  uint16_t len;
  uint8_t* dataPtr;
  bool protect;
} FFF_Buffer;

#endif