#include "lidar_link.h"
#include "usart.h"
#include <string.h>

#define LIDAR_LINK_SYNC0 0xAA
#define LIDAR_LINK_SYNC1 0x55
#define LIDAR_LINK_TYPE_SCAN 0x01
#define LIDAR_LINK_HEADER_LEN 7
#define LIDAR_LINK_FRAME_MAX (LIDAR_LINK_HEADER_LEN + LIDAR_MAX_POINTS * 9 + 2)

static uint8_t lidar_link_frame[LIDAR_LINK_FRAME_MAX];

static uint16_t lidar_link_crc16(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	uint8_t bit;
	while(len-- > 0)
	{
		crc ^= *data++;
		for(bit = 0; bit < 8; bit++)
		{
			crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
		}
	}
	return crc;
}

void LidarLink_SendScan(const LidarPoint_t *points, uint16_t count, uint32_t revolution)
{
	uint16_t offset = 0;
	uint16_t crc;
	uint16_t i;
	uint32_t value;

	if(count > LIDAR_MAX_POINTS) { count = LIDAR_MAX_POINTS; }
	lidar_link_frame[offset++] = LIDAR_LINK_SYNC0;
	lidar_link_frame[offset++] = LIDAR_LINK_SYNC1;
	lidar_link_frame[offset++] = LIDAR_LINK_TYPE_SCAN;
	lidar_link_frame[offset++] = (uint8_t)revolution;
	lidar_link_frame[offset++] = (uint8_t)(revolution >> 8);
	lidar_link_frame[offset++] = (uint8_t)count;
	lidar_link_frame[offset++] = (uint8_t)(count >> 8);
	for(i = 0; i < count; i++)
	{
		memcpy(&value, &points[i].angle_deg, sizeof(value));
		lidar_link_frame[offset++] = (uint8_t)value;
		lidar_link_frame[offset++] = (uint8_t)(value >> 8);
		lidar_link_frame[offset++] = (uint8_t)(value >> 16);
		lidar_link_frame[offset++] = (uint8_t)(value >> 24);
		memcpy(&value, &points[i].dist_mm, sizeof(value));
		lidar_link_frame[offset++] = (uint8_t)value;
		lidar_link_frame[offset++] = (uint8_t)(value >> 8);
		lidar_link_frame[offset++] = (uint8_t)(value >> 16);
		lidar_link_frame[offset++] = (uint8_t)(value >> 24);
		lidar_link_frame[offset++] = points[i].quality;
	}
	crc = lidar_link_crc16(&lidar_link_frame[2], offset - 2);
	lidar_link_frame[offset++] = (uint8_t)crc;
	lidar_link_frame[offset++] = (uint8_t)(crc >> 8);
	usart6_SendData(lidar_link_frame, offset);
}
