#include "rplidar.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define LIDAR_DMA_BUF_SIZE 2048
#define LIDAR_CMD_STOP 0x25
#define LIDAR_CMD_SCAN 0x20
#define LIDAR_CMD_RESET 0x40

static uint8_t lidar_dma_buf[LIDAR_DMA_BUF_SIZE];
static uint32_t lidar_read_index = 0;
static uint16_t lidar_current_count = 0;
static uint16_t lidar_done_count = 0;
static uint32_t lidar_rev_count = 0;
static LidarPoint_t lidar_current[LIDAR_MAX_POINTS];
static LidarPoint_t lidar_done[LIDAR_MAX_POINTS];
static DMA_InitTypeDef lidar_dma_init;
static uint8_t lidar_node[5];
static uint8_t lidar_node_index = 0;
static uint8_t lidar_parser_state = 0;
static uint8_t lidar_header[5];
static uint8_t lidar_header_index = 0;
static uint32_t lidar_payload_left = 0;
static uint8_t lidar_scan_started = 0;

static void lidar_send(uint8_t command)
{
	uint8_t frame[2] = {0xA5, command};
	uart4_SendData(frame, 2);
}

static void lidar_parser_reset(void)
{
	lidar_parser_state = 0;
	lidar_header_index = 0;
	lidar_node_index = 0;
	lidar_payload_left = 0;
}

static void lidar_emit_node(void)
{
	uint8_t sync_quality = lidar_node[0];
	uint16_t angle_raw = (uint16_t)(lidar_node[1] | ((uint16_t)lidar_node[2] << 8));
	uint16_t distance_raw = (uint16_t)(lidar_node[3] | ((uint16_t)lidar_node[4] << 8));
	uint8_t start = sync_quality & 0x01;
	uint8_t quality = (sync_quality >> 2) & 0x3F;

	if(start && lidar_current_count > 0)
	{
		memcpy(lidar_done, lidar_current, lidar_current_count * sizeof(LidarPoint_t));
		lidar_done_count = lidar_current_count;
		lidar_current_count = 0;
		lidar_rev_count++;
	}
	if(lidar_current_count < LIDAR_MAX_POINTS)
	{
		lidar_current[lidar_current_count].angle_deg = (angle_raw >> 1) / 64.0f;
		lidar_current[lidar_current_count].dist_mm = distance_raw / 4.0f;
		lidar_current[lidar_current_count].quality = quality;
		lidar_current_count++;
	}
}

static void lidar_parse_byte(uint8_t data)
{
	uint32_t length;
	uint8_t loop;

	if(lidar_parser_state == 0)
	{
		if(data == 0xA5) { lidar_parser_state = 1; }
	}
	else if(lidar_parser_state == 1)
	{
		if(data == 0x5A)
		{
			lidar_parser_state = 2;
			lidar_header_index = 0;
		}
		else { lidar_parser_state = 0; }
	}
	else if(lidar_parser_state == 2)
	{
		lidar_header[lidar_header_index++] = data;
		if(lidar_header_index >= 5)
		{
			length = (uint32_t)lidar_header[0] |
			         ((uint32_t)lidar_header[1] << 8) |
			         ((uint32_t)lidar_header[2] << 16) |
			         ((uint32_t)lidar_header[3] << 24);
			loop = (uint8_t)((length >> 30) & 1);
			lidar_payload_left = length & 0x3FFFFFFF;
			if(loop && lidar_header[4] == 0x81)
			{
				lidar_parser_state = 4;
				lidar_node_index = 0;
			}
			else
			{
				lidar_parser_state = 3;
			}
		}
	}
	else if(lidar_parser_state == 3)
	{
		if(lidar_payload_left > 0) { lidar_payload_left--; }
		if(lidar_payload_left == 0) { lidar_parser_state = 0; }
	}
	else
	{
		if(lidar_node_index == 0 && (((data >> 1) ^ data) & 1) == 0) { return; }
		if(lidar_node_index == 1 && (data & 1) == 0)
		{
			lidar_node_index = 0;
			return;
		}
		lidar_node[lidar_node_index++] = data;
		if(lidar_node_index >= 5)
		{
			lidar_emit_node();
			lidar_node_index = 0;
		}
	}
}

void Lidar_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Stream2);
	while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE) {}
	DMA_StructInit(&lidar_dma_init);
	lidar_dma_init.DMA_Channel = DMA_Channel_4;
	lidar_dma_init.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	lidar_dma_init.DMA_Memory0BaseAddr = (uint32_t)lidar_dma_buf;
	lidar_dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;
	lidar_dma_init.DMA_BufferSize = LIDAR_DMA_BUF_SIZE;
	lidar_dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	lidar_dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	lidar_dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	lidar_dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	lidar_dma_init.DMA_Mode = DMA_Mode_Circular;
	lidar_dma_init.DMA_Priority = DMA_Priority_High;
	lidar_dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(DMA1_Stream2, &lidar_dma_init);
	DMA_Cmd(DMA1_Stream2, ENABLE);
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	lidar_read_index = 0;
	lidar_current_count = 0;
	lidar_done_count = 0;
	lidar_rev_count = 0;
	lidar_parser_reset();
	lidar_send(LIDAR_CMD_RESET);
	vTaskDelay(pdMS_TO_TICKS(50));
	lidar_send(LIDAR_CMD_SCAN);
	lidar_scan_started = 1;
}

void Lidar_Process(void)
{
	uint32_t write_index;
	uint32_t remaining;
	uint32_t available;

	if(!lidar_scan_started) { return; }
	remaining = DMA_GetCurrDataCounter(DMA1_Stream2);
	write_index = (LIDAR_DMA_BUF_SIZE - remaining) % LIDAR_DMA_BUF_SIZE;
	available = (write_index - lidar_read_index + LIDAR_DMA_BUF_SIZE) % LIDAR_DMA_BUF_SIZE;
	while(available-- > 0)
	{
		lidar_parse_byte(lidar_dma_buf[lidar_read_index]);
		lidar_read_index = (lidar_read_index + 1) % LIDAR_DMA_BUF_SIZE;
	}
}

void Lidar_Stop(void)
{
	lidar_send(LIDAR_CMD_STOP);
	lidar_scan_started = 0;
	lidar_parser_reset();
}

uint16_t Lidar_GetScan(LidarPoint_t **points)
{
	if(points != 0) { *points = lidar_done; }
	return lidar_done_count;
}

uint32_t Lidar_GetRevCount(void)
{
	return lidar_rev_count;
}

void Lidar_Test(void)
{
	usart1_SendString("[LIDAR] UART4 scan test\r\n");
}
