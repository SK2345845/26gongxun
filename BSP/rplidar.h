#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#include "stm32f4xx.h"
#include <stdint.h>

#define LIDAR_MAX_POINTS 720

typedef struct
{
	float angle_deg;
	float dist_mm;
	uint8_t quality;
} LidarPoint_t;

void Lidar_Init(void);
void Lidar_Process(void);
void Lidar_Stop(void);
uint16_t Lidar_GetScan(LidarPoint_t **points);
uint32_t Lidar_GetRevCount(void);
void Lidar_Test(void);

#endif /* __RPLIDAR_H */
