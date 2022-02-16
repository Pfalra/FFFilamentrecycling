#ifndef FFF_TYPES_H
#define FFF_TYPES_H

#include <stdint.h>

// #define NULL 0x0

#define MAX_NUM_MODULES 10
#define MAX_MODNAME_LEN 24


typedef enum FFF_ModStatus_e
{
    MOD_HW_NOT_DETECTED,
    MOD_FAULT,
    MOD_NOT_INITIALIZED,
    MOD_INIT_RUNNING,
    MOD_FUNCTIONAL
} FFF_ModStatus;



#endif