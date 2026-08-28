#ifndef __LIDAR_LINK_H
#define __LIDAR_LINK_H

#include <stdint.h>
#include "rplidar.h"

void LidarLink_SendScan(const LidarPoint_t *points, uint16_t count, uint32_t revolution);

#endif /* __LIDAR_LINK_H */
