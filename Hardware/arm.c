#include "arm.h"
#include "moving.h"

/**********************************************************
*** 机械臂步进电机控制
**********************************************************/

//6号伸缩结构总行程3.5圈，7号丝杆机构最上到最下一共80圈
__IO int32_t Arm_Lift_Total_Turns = 0;
__IO int32_t Arm_Screw_Total_Turns = 0;
__IO int32_t Arm_Base_Total_Degrees = 0;

static void Arm_Lift_PrintInt32(int32_t value)
{
	char text[12];
	uint8_t i = 0;
	uint32_t number;

	if(value < 0)
	{
		usart1_SendByte('-');
		number = (uint32_t)(-(value + 1)) + 1;
	}
	else
	{
		number = (uint32_t)value;
	}
	if(number == 0)
	{
		usart1_SendByte('0');
		return;
	}
	while(number > 0)
	{
		text[i++] = (char)('0' + number % 10);
		number /= 10;
	}
	while(i > 0) { usart1_SendByte((uint16_t)text[--i]); }
}

static void Arm_Lift_PrintTurns(int32_t milli_turns)
{
	int32_t whole = milli_turns / ARM_TURN_SCALE;
	int32_t fraction = milli_turns % ARM_TURN_SCALE;

	if(milli_turns < 0)
	{
		usart1_SendByte('-');
		whole = -whole;
		fraction = -fraction;
	}
	Arm_Lift_PrintInt32(whole);
	if(fraction != 0)
	{
		usart1_SendByte('.');
		if(fraction < 10) { usart1_SendByte('0'); }
		if(fraction < 100) { usart1_SendByte('0'); }
		Arm_Lift_PrintInt32(fraction);
	}
}

static uint8_t Arm_Lift_ParseTurns(const char *text, int32_t max_value, int32_t *milli_turns)
{
	int32_t whole = 0;
	int32_t fraction = 0;
	uint8_t fraction_digits = 0;
	int8_t sign = 1;
	uint8_t has_digit = 0;

	while(*text == ' ' || *text == '\t') { text++; }
	if(*text == '-') { sign = -1; text++; }
	else if(*text == '+') { text++; }
	while(*text >= '0' && *text <= '9')
	{
		has_digit = 1;
		whole = whole * 10 + (*text - '0');
		text++;
	}
	if(*text == '.')
	{
		text++;
		while(*text >= '0' && *text <= '9')
		{
			has_digit = 1;
			if(fraction_digits >= 3) { return 0; }
			fraction = fraction * 10 + (*text - '0');
			fraction_digits++;
			text++;
		}
	}
	while(*text == ' ' || *text == '\t') { text++; }

	while(fraction_digits < 3)
	{
		fraction *= 10;
		fraction_digits++;
	}
	if(!has_digit || *text != '\0' ||
	   whole * ARM_TURN_SCALE + fraction > max_value * ARM_TURN_SCALE)
	{
		return 0;
	}
	*milli_turns = sign * (whole * ARM_TURN_SCALE + fraction);
	return 1;
}

typedef struct
{
	uint8_t motor;
	int32_t value;
} Arm_Command_t;

static Arm_Command_t arm_command_queue[8];
static Arm_Command_t arm_active_command;
static uint8_t arm_lift_queue_head = 0;
static uint8_t arm_lift_queue_tail = 0;
static uint8_t arm_lift_moving = 0;
static TickType_t arm_lift_finish_tick = 0;

static void Arm_Lift_StartNext(void)
{
	int32_t milli_turns;
	uint32_t pulse;
	uint8_t dir;

	if(arm_lift_moving || arm_lift_queue_head == arm_lift_queue_tail)
	{
		return;
	}

	arm_active_command = arm_command_queue[arm_lift_queue_tail];
	arm_lift_queue_tail = (uint8_t)((arm_lift_queue_tail + 1) % 8);
	milli_turns = arm_active_command.value;
	dir = milli_turns > 0 ? ARM_DIR_CW : ARM_DIR_CCW;
	if(arm_active_command.motor == ARM_BASE_MOTOR)
	{
		pulse = (uint32_t)(milli_turns > 0 ? milli_turns : -milli_turns) * ARM_BASE_DEGREE_PULSE /
		        (360 * ARM_DEGREE_SCALE);
	}
	else if(arm_active_command.motor == ARM_LIFT_MOTOR)
	{
		pulse = (uint32_t)(milli_turns > 0 ? milli_turns : -milli_turns) * ARM_LIFT_ONE_TURN_PULSE / ARM_TURN_SCALE;
	}
	else
	{
		pulse = (uint32_t)(milli_turns > 0 ? milli_turns : -milli_turns) * ARM_EXTEND_ONE_TURN_PULSE / ARM_TURN_SCALE;
	}

	usart1_SendString("[ARM] move turns=");
	Arm_Lift_PrintTurns(milli_turns);
	usart1_SendString(" pulse=");
	Arm_Lift_PrintInt32((int32_t)pulse);
	usart1_SendString("\r\n");

	Emm_V5_En_Control(arm_active_command.motor, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));
	Emm_V5_Pos_Control(arm_active_command.motor, dir,
					   arm_active_command.motor == ARM_BASE_MOTOR ? ARM_BASE_MOVE_VEL :
					   (arm_active_command.motor == ARM_LIFT_MOTOR ? ARM_LIFT_MOVE_VEL : ARM_EXTEND_MOVE_VEL),
					   arm_active_command.motor == ARM_BASE_MOTOR ? ARM_BASE_MOVE_ACC :
					   (arm_active_command.motor == ARM_LIFT_MOTOR ? ARM_LIFT_MOVE_ACC : ARM_EXTEND_MOVE_ACC),
	                   pulse, false, false);
	arm_lift_finish_tick = xTaskGetTickCount() +
	                       pdMS_TO_TICKS((arm_active_command.motor == ARM_BASE_MOTOR ?
	                                     ARM_BASE_TURN_TIME_MS * (milli_turns > 0 ? milli_turns : -milli_turns) / 360 :
	                                     (arm_active_command.motor == ARM_LIFT_MOTOR ? ARM_LIFT_TURN_TIME_MS : ARM_EXTEND_TURN_TIME_MS) *
	                                     (milli_turns > 0 ? milli_turns : -milli_turns) / ARM_TURN_SCALE) + 300);
	arm_lift_moving = 1;
}

void Arm_Lift_Debug_Process(void)
{
	static char command[16];
	static uint8_t length = 0;
	uint8_t data;
	int32_t milli_turns;

	if(arm_lift_moving && (int32_t)(xTaskGetTickCount() - arm_lift_finish_tick) >= 0)
	{
		arm_lift_moving = 0;
		if(arm_active_command.motor == ARM_BASE_MOTOR)
		{
			Arm_Base_Total_Degrees += arm_active_command.value;
			usart1_SendString("[ARM] base total degrees=");
			Arm_Lift_PrintTurns(Arm_Base_Total_Degrees);
		}
		else if(arm_active_command.motor == ARM_LIFT_MOTOR)
		{
			Arm_Screw_Total_Turns += arm_active_command.value;
			usart1_SendString("[ARM] 7 total turns=");
			Arm_Lift_PrintTurns(Arm_Screw_Total_Turns);
		}
		else
		{
			Arm_Lift_Total_Turns += arm_active_command.value;
			usart1_SendString("[ARM] 6 total turns=");
			Arm_Lift_PrintTurns(Arm_Lift_Total_Turns);
		}
		usart1_SendString("\r\n> ");
	}

	while(usart1_ReadByte(&data))
	{
		if(data == 'w' || data == 'a' || data == 's' || data == 'd' ||
		   data == 'e' || data == 'r' || data == 'x')
		{
			Chassis_Remote_Command((char)data);
			continue;
		}
		if(data == '\r' || data == '\n')
		{
			if(length == 0) { continue; }
			command[length] = '\0';
			length = 0;
			if(command[0] == '+' && command[1] == '\0')
			{
				Chassis_Remote_SpeedAdjust(1);
				usart1_SendString("[REMOTE] speed +\r\n> ");
				continue;
			}
			if(command[0] == '-' && command[1] == '\0')
			{
				Chassis_Remote_SpeedAdjust(-1);
				usart1_SendString("[REMOTE] speed -\r\n> ");
				continue;
			}
			uint8_t motor = ARM_EXTEND_MOTOR;
			const char *value_text = command;
			int32_t max_value = ARM_EXTEND_MAX_TURNS;
			if((command[0] == '5' || command[0] == '6' || command[0] == '7') &&
			   (command[1] == '/' || command[1] == ':' || command[1] == ' ' || command[1] == '\t'))
			{
				motor = (uint8_t)(command[0] - '0');
				value_text = &command[2];
				while(*value_text == ' ' || *value_text == '\t' || *value_text == ':') { value_text++; }
				max_value = motor == ARM_BASE_MOTOR ? ARM_BASE_MAX_DEGREES : ARM_LIFT_MAX_TURNS;
			}
			if(!Arm_Lift_ParseTurns(value_text, max_value, &milli_turns))
			{
				usart1_SendString("[ARM] invalid turns (-100..100)\r\n> ");
				continue;
			}
			if(milli_turns == 0)
			{
				usart1_SendString("[ARM] zero turns, no movement\r\n> ");
				continue;
			}

			if((uint8_t)((arm_lift_queue_head + 1) % 8) == arm_lift_queue_tail)
			{
				usart1_SendString("[ARM] command queue full\r\n> ");
				continue;
			}
			arm_command_queue[arm_lift_queue_head].motor = motor;
			arm_command_queue[arm_lift_queue_head].value = milli_turns;
			arm_lift_queue_head = (uint8_t)((arm_lift_queue_head + 1) % 8);
			usart1_SendString("[ARM] queued turns=");
			Arm_Lift_PrintTurns(milli_turns);
			usart1_SendString("\r\n");
			Arm_Lift_StartNext();
		}
		else if((data == '\b' || data == 0x7F) && length > 0) { length--; }
		else if(length < sizeof(command) - 1 && data >= 0x20 && data <= 0x7E)
		{
			command[length++] = (char)data;
		}
	}
	Arm_Lift_StartNext();
}

/**
 * @brief   5号电机单圈回零
 * @note    回零参数应在驱动器中预先配置，模式0为单圈回零并清零当前位置
 * @param   无
 * @retval  无
 */
void Arm_Base_Origin_Return(void)
{
	Emm_V5_En_Control(ARM_BASE_MOTOR, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));
	Emm_V5_Origin_Trigger_Return(ARM_BASE_MOTOR, 0, false);
	vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief   7号电机丝杆机构运行一圈
 * @note    按16细分配置，3200脉冲为电机轴一圈；采用相对位置模式
 * @param   无
 * @retval  无
 */
void Arm_Lift_One_Turn_Test(void)
{
	usart1_SendString("[ARM] screw7 one-turn test start\r\n");
	Emm_V5_En_Control(ARM_LIFT_MOTOR, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));

	usart1_SendString("[ARM] addr=7 pulse=3200 vel=4000 acc=250 dir=CW\r\n");
	Emm_V5_Pos_Control(ARM_LIFT_MOTOR, ARM_DIR_CW,
	                   ARM_LIFT_MOVE_VEL, ARM_LIFT_MOVE_ACC,
	                   ARM_LIFT_ONE_TURN_PULSE, false, false);
	vTaskDelay(pdMS_TO_TICKS(3000));
	usart1_SendString("[ARM] screw7 one-turn test done\r\n");
}

/**
 * @brief   配置5号电机内部闭环PID参数
 * @note    svF为false，仅本次上电有效，确认参数后可在驱动器调试工具中保存
 * @param   无
 * @retval  无
 */
void Arm_Base_PID_Configure(void)
{
	Emm_V5_Modify_PID_Params(ARM_BASE_MOTOR, false,
	                         ARM_BASE_PID_KP, ARM_BASE_PID_KI,
	                         ARM_BASE_PID_KD);
	vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief   5号电机正向一圈后反向一圈
 * @note    按16细分配置，3200脉冲为一圈
 * @param   无
 * @retval  无
 */
void Arm_Base_Forward_Reverse_Test(void)
{
	Arm_Base_PID_Configure();
	Emm_V5_En_Control(ARM_BASE_MOTOR, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 正向一圈
	Emm_V5_Pos_Control(ARM_BASE_MOTOR, ARM_DIR_CW,
	                   ARM_BASE_MOVE_VEL, ARM_BASE_MOVE_ACC,
	                   ARM_BASE_ONE_TURN_PULSE, false, false);
	vTaskDelay(pdMS_TO_TICKS(3000));

	// 反向一圈
	Emm_V5_Pos_Control(ARM_BASE_MOTOR, ARM_DIR_CCW,
	                   ARM_BASE_MOVE_VEL, ARM_BASE_MOVE_ACC,
	                   ARM_BASE_ONE_TURN_PULSE, false, false);
	vTaskDelay(pdMS_TO_TICKS(3000));
}

/**
 * @brief   机械臂控制硬件回归测试
 * @note    该测试会实际驱动5号电机，调用前应确认机械结构处于安全位置
 * @param   无
 * @retval  无
 */
void Arm_Test_Run(void)
{
	// 执行5号底部转盘的单圈回零
	Arm_Base_Origin_Return();
}
