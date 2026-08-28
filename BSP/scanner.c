#include "scanner.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define GM65_HEAD1              0x7E
#define GM65_HEAD2              0x00
#define GM65_CMD_WRITE         0x08
#define GM65_CMD_SAVE          0x09
#define GM65_REG_MODE          0x0000
#define GM65_REG_TRIG          0x0002
#define SCANNER_QUEUE_LEN      4

static Scanner_Code_t scanner_queue[SCANNER_QUEUE_LEN];
static uint8_t scanner_queue_head = 0;
static uint8_t scanner_queue_tail = 0;
static Scanner_Side_t scanner_side = SCANNER_SIDE_LEFT;
static uint8_t scanner_line[SCANNER_CODE_MAX + 1];
static uint16_t scanner_line_len = 0;

static uint16_t scanner_crc16(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0;
	uint8_t bit;
	while(len-- > 0)
	{
		for(bit = 0x80; bit != 0; bit >>= 1)
		{
			crc <<= 1;
			if(crc & 0x10000) { crc ^= 0x1021; }
			if(*data & bit) { crc ^= 0x1021; }
		}
		data++;
	}
	return crc;
}

static uint8_t scanner_send_frame(uint8_t type, uint16_t addr,
	                              const uint8_t *data, uint8_t len)
{
	uint8_t frame[16];
	uint16_t index = 0;
	uint16_t crc;
	uint8_t i;

	if(len > 6) { return 0; }
	frame[index++] = GM65_HEAD1;
	frame[index++] = GM65_HEAD2;
	frame[index++] = type;
	frame[index++] = len;
	frame[index++] = (uint8_t)(addr >> 8);
	frame[index++] = (uint8_t)addr;
	for(i = 0; i < len; i++) { frame[index++] = data[i]; }
#if SCANNER_CMD_USE_CRC
	crc = scanner_crc16(&frame[2], index - 2);
	frame[index++] = (uint8_t)(crc >> 8);
	frame[index++] = (uint8_t)crc;
#else
	frame[index++] = 0xAB;
	frame[index++] = 0xCD;
#endif
	uart5_SendData(frame, index);
	return 1;
}

static void scanner_queue_push(const uint8_t *data, uint16_t len)
{
	uint8_t next;
	if(len > SCANNER_CODE_MAX) { len = SCANNER_CODE_MAX; }
	if(len == 0) { return; }
	next = (uint8_t)((scanner_queue_head + 1) % SCANNER_QUEUE_LEN);
	if(next == scanner_queue_tail)
	{
		scanner_queue_tail = (uint8_t)((scanner_queue_tail + 1) % SCANNER_QUEUE_LEN);
	}
	memcpy(scanner_queue[scanner_queue_head].code, data, len);
	scanner_queue[scanner_queue_head].code[len] = '\0';
	scanner_queue[scanner_queue_head].len = len;
	scanner_queue[scanner_queue_head].tick = xTaskGetTickCount();
	scanner_queue_head = next;
}

void Scanner_Init(void)
{
	scanner_queue_head = scanner_queue_tail = 0;
	scanner_line_len = 0;
	scanner_side = SCANNER_SIDE_LEFT;
}

void Scanner_SelectSide(Scanner_Side_t side)
{
	uint8_t data;
	/* 当前硬件没有左右切换脚，仅记录目标侧，禁止两路TX直接并联。 */
	scanner_side = side;
	scanner_line_len = 0;
	while(uart5_ReadByte(&data)) {}
}

Scanner_Side_t Scanner_GetSide(void)
{
	return scanner_side;
}

void Scanner_SetMode(uint8_t mode)
{
	static const uint8_t save = 0;
	scanner_send_frame(GM65_CMD_WRITE, GM65_REG_MODE, &mode, 1);
	scanner_send_frame(GM65_CMD_SAVE, 0, &save, 1);
}

uint8_t Scanner_SetModeRuntime(uint8_t mode)
{
	return scanner_send_frame(GM65_CMD_WRITE, GM65_REG_MODE, &mode, 1);
}

uint8_t Scanner_Trigger(void)
{
	static const uint8_t trigger = 1;
	return scanner_send_frame(GM65_CMD_WRITE, GM65_REG_TRIG, &trigger, 1);
}

void Scanner_Process(void)
{
	uint8_t data;
	while(uart5_ReadByte(&data))
	{
		if(data == '\r' || data == '\n')
		{
			scanner_queue_push(scanner_line, scanner_line_len);
			scanner_line_len = 0;
		}
		else if(scanner_line_len < SCANNER_CODE_MAX)
		{
			scanner_line[scanner_line_len++] = data;
		}
	}
}

uint8_t Scanner_GetCode(Scanner_Code_t *code)
{
	if(scanner_queue_head == scanner_queue_tail) { return 0; }
	if(code != 0) { *code = scanner_queue[scanner_queue_tail]; }
	scanner_queue_tail = (uint8_t)((scanner_queue_tail + 1) % SCANNER_QUEUE_LEN);
	return 1;
}
