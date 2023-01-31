#ifndef __MARVELMIND_DEVICES_H_
#define __MARVELMIND_DEVICES_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "marvelmind_api.h"
#include "marvelmind_utils.h"

typedef struct {
    uint32_t distance_mm;
} MarvelmindDevDistance;

typedef struct {
    MarvelmindDeviceInfo info;
    MarvelmindDeviceVersion version;

    bool devConnected;
    MMDeviceType deviceType;

    #if MM_LOCATIONS_VERSION==1
    MarvelmindDeviceLocation pos;
    #endif // MM_LOCATIONS_VERSION

    #if MM_LOCATIONS_VERSION==2
    MarvelmindDeviceLocation2 pos;
    #endif // MM_LOCATIONS_VERSION

    MarvelmindDevDistance distances[MM_MAX_DEVICES_COUNT];
} MarvelmindDevice;

// Read Marvelmind devices list from modem
void marvelmindDevicesReadIfNeeded();

// Get device data structure
MarvelmindDevice *getMarvelmindDevice(uint8_t address);

#if MM_LOCATIONS_VERSION==1
// Update device location
MarvelmindDevice *marvelmindUpdateLocation(uint8_t address, MarvelmindDeviceLocation *ppos);
#endif

#if MM_LOCATIONS_VERSION==2
// Update device location
MarvelmindDevice *marvelmindUpdateLocation(uint8_t address, MarvelmindDeviceLocation2 *ppos);
#endif

// Update distance to device
MarvelmindDevice *marvelmindUpdateDistance(uint8_t addressRx, uint8_t addressTx, uint32_t d);

// Initialize structure of Marvelmind devices
void initMarvelmindDevicesList();

#endif // __MARVELMIND_DEVICES_H_
