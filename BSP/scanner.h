#ifndef __SCANNER_H
#define __SCANNER_H

#include "stm32f4xx.h"
#include <stdint.h>

#define SCANNER_CODE_MAX              64
#define SCANNER_MODE_CONTINUOUS      0x22
#define SCANNER_MODE_SENSE           0x63
#define SCANNER_MODE_TRIGGER         0x99
#define SCANNER_CMD_USE_CRC           0

typedef enum
{
	SCANNER_SIDE_LEFT = 0,
	SCANNER_SIDE_RIGHT = 1
} Scanner_Side_t;

typedef struct
{
	char code[SCANNER_CODE_MAX + 1];
	uint16_t len;
	uint32_t tick;
} Scanner_Code_t;

void Scanner_Init(void);
void Scanner_SelectSide(Scanner_Side_t side);
Scanner_Side_t Scanner_GetSide(void);
void Scanner_SetMode(uint8_t mode);
uint8_t Scanner_SetModeRuntime(uint8_t mode);
uint8_t Scanner_Trigger(void);
void Scanner_Process(void);
uint8_t Scanner_GetCode(Scanner_Code_t *code);

#endif /* __SCANNER_H */
