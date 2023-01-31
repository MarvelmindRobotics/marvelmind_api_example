#ifndef __MARVELMIND_POS_H_
#define __MARVELMIND_POS_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "marvelmind_api.h"

#define MARVELMIND_POS_READ_RATE 20

#define RAW_DISTANCES_MODE_OBSOLETE 0
#define RAW_DISTANCES_MODE_STREAMED 1

typedef enum {
    notRead, readSuccess, readFail
} MMPosReadStatus;
MMPosReadStatus marvelmindLocationsReadIfNeeded();

void setRawDistancesMode(uint8_t v);
uint8_t getRawDistancesMode(void);

void initMarvelmindPos();

#endif // __MARVELMIND_POS_H_
