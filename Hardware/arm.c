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
	                                     ARM_BASE_TURN_TIME_MS * (milli_turns > 0 ? milli_turns : -milli_turns) / (360 * ARM_DEGREE_SCALE) :
	                                     (arm_active_command.motor == ARM_LIFT_MOTOR ? ARM_LIFT_TURN_TIME_MS : ARM_EXTEND_TURN_TIME_MS) *
	                                     (milli_turns > 0 ? milli_turns : -milli_turns) / ARM_TURN_SCALE) + 300);
	arm_lift_moving = 1;
}

/**********************************************************
*** 联合机械臂分阶段并行调试流程
**********************************************************/
typedef enum
{
	ARM_SEQ_STAGE_BASE_PICK = 0,
	ARM_SEQ_STAGE_EXTEND_AND_LOWER,
	ARM_SEQ_STAGE_GRAB,
	ARM_SEQ_STAGE_LIFT_AND_HOME,
	ARM_SEQ_STAGE_RETRACT_AND_LOWER,
	ARM_SEQ_STAGE_RELEASE,
	ARM_SEQ_STAGE_DONE
} Arm_Sequence_Stage_t;

static uint8_t arm_sequence_active = 0;
static Arm_Sequence_Stage_t arm_sequence_stage = ARM_SEQ_STAGE_DONE;
static TickType_t arm_sequence_deadline = 0;

static int32_t Arm_Sequence_Abs(int32_t value)
{
	return value < 0 ? -value : value;
}

static uint32_t Arm_Sequence_Move_Time(uint8_t motor, int32_t milli_value)
{
	uint32_t magnitude = (uint32_t)Arm_Sequence_Abs(milli_value);
	uint32_t time_ms;

	if(magnitude == 0) { return 0; }
	if(motor == ARM_BASE_MOTOR)
	{
		time_ms = ARM_BASE_TURN_TIME_MS * magnitude /
		          (360 * ARM_DEGREE_SCALE);
	}
	else if(motor == ARM_LIFT_MOTOR)
	{
		time_ms = ARM_LIFT_TURN_TIME_MS * magnitude / ARM_TURN_SCALE;
	}
	else
	{
		time_ms = ARM_EXTEND_TURN_TIME_MS * magnitude / ARM_TURN_SCALE;
	}
	return time_ms + ARM_SEQ_MOTION_MARGIN_MS;
}

/* 启动一个相对位置动作；多个调用之间只短暂等待帧处理，不等待电机走完。 */
static uint32_t Arm_Sequence_Start_Move(uint8_t motor, int32_t milli_value)
{
	uint32_t pulse;
	uint32_t magnitude;
	uint8_t dir;
	uint16_t vel;
	uint8_t acc;

	if(milli_value == 0) { return 0; }
	magnitude = (uint32_t)Arm_Sequence_Abs(milli_value);
	dir = milli_value > 0 ? ARM_DIR_CW : ARM_DIR_CCW;
	if(motor == ARM_BASE_MOTOR)
	{
		pulse = magnitude * ARM_BASE_DEGREE_PULSE /
		        (360 * ARM_DEGREE_SCALE);
		vel = ARM_BASE_MOVE_VEL;
		acc = ARM_BASE_MOVE_ACC;
	}
	else if(motor == ARM_LIFT_MOTOR)
	{
		pulse = magnitude * ARM_LIFT_ONE_TURN_PULSE / ARM_TURN_SCALE;
		vel = ARM_LIFT_MOVE_VEL;
		acc = ARM_LIFT_MOVE_ACC;
	}
	else
	{
		pulse = magnitude * ARM_EXTEND_ONE_TURN_PULSE / ARM_TURN_SCALE;
		vel = ARM_EXTEND_MOVE_VEL;
		acc = ARM_EXTEND_MOVE_ACC;
	}

	Emm_V5_En_Control(motor, true, false);
	vTaskDelay(pdMS_TO_TICKS(20));
	Emm_V5_Pos_Control(motor, dir, vel, acc, pulse, false, false);
	vTaskDelay(pdMS_TO_TICKS(20));
	return Arm_Sequence_Move_Time(motor, milli_value);
}

static uint32_t Arm_Sequence_Max_Time(uint32_t first, uint32_t second)
{
	return first > second ? first : second;
}

static void Arm_Combined_Sequence_Run_Stage(void)
{
	uint32_t duration = 0;
	int32_t base_delta;
	int32_t lift_up;
	int32_t lift_down;

	lift_up = ARM_SEQ_LIFT_UP_SIGN * ARM_TURN_SCALE;
	lift_down = -lift_up;

	switch(arm_sequence_stage)
	{
	case ARM_SEQ_STAGE_BASE_PICK:
		usart1_SendString("[ARMSEQ] stage1: lift-top + base 180\r\n");
		if(ARM_SEQ_INITIAL_RAISE_TURNS != 0)
		{
			duration = Arm_Sequence_Start_Move(ARM_LIFT_MOTOR,
			                                  lift_up * ARM_SEQ_INITIAL_RAISE_TURNS);
		}
		base_delta = ARM_SEQ_BASE_PICK_DEGREE * ARM_DEGREE_SCALE -
		             Arm_Base_Total_Degrees;
		duration = Arm_Sequence_Max_Time(duration,
		                                  Arm_Sequence_Start_Move(ARM_BASE_MOTOR, base_delta));
		break;

	case ARM_SEQ_STAGE_EXTEND_AND_LOWER:
		usart1_SendString("[ARMSEQ] stage2: extend + lower\r\n");
		duration = Arm_Sequence_Start_Move(ARM_EXTEND_MOTOR,
		                                  ARM_SEQ_EXTEND_TURNS * ARM_TURN_SCALE);
		duration = Arm_Sequence_Max_Time(duration,
		                                  Arm_Sequence_Start_Move(ARM_LIFT_MOTOR,
		                                  lift_down * ARM_SEQ_LOWER_PICK_TURNS));
		break;

	case ARM_SEQ_STAGE_GRAB:
		usart1_SendString("[ARMSEQ] stage3: gripper grab\r\n");
		Arm_Gripper_Grab();
		duration = ARM_SEQ_GRIPPER_WAIT_MS;
		break;

		case ARM_SEQ_STAGE_LIFT_AND_HOME:
			usart1_SendString("[ARMSEQ] stage4: lift + base 0\r\n");
		duration = Arm_Sequence_Start_Move(ARM_LIFT_MOTOR,
		                                  lift_up * ARM_SEQ_RAISE_AFTER_PICK_TURNS);
		base_delta = ARM_SEQ_BASE_HOME_DEGREE * ARM_DEGREE_SCALE -
		             Arm_Base_Total_Degrees;
		duration = Arm_Sequence_Max_Time(duration,
		                                  Arm_Sequence_Start_Move(ARM_BASE_MOTOR, base_delta));
		break;

		case ARM_SEQ_STAGE_RETRACT_AND_LOWER:
			usart1_SendString("[ARMSEQ] stage5: retract + lower\r\n");
		duration = Arm_Sequence_Start_Move(ARM_EXTEND_MOTOR,
		                                  -ARM_SEQ_RETRACT_TURNS * ARM_TURN_SCALE);
		duration = Arm_Sequence_Max_Time(duration,
		                                  Arm_Sequence_Start_Move(ARM_LIFT_MOTOR,
		                                  lift_down * ARM_SEQ_LOWER_RELEASE_TURNS));
		break;

		case ARM_SEQ_STAGE_RELEASE:
			usart1_SendString("[ARMSEQ] stage6: gripper release\r\n");
		Arm_Gripper_Release();
		duration = ARM_SEQ_GRIPPER_WAIT_MS;
		break;

	default:
		arm_sequence_active = 0;
		return;
	}
	arm_sequence_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration);
}

static void Arm_Combined_Sequence_Finish_Stage(void)
{
	int32_t lift_up;

	lift_up = ARM_SEQ_LIFT_UP_SIGN * ARM_TURN_SCALE;
	switch(arm_sequence_stage)
	{
		case ARM_SEQ_STAGE_BASE_PICK:
			if(ARM_SEQ_INITIAL_RAISE_TURNS != 0)
			{
				Arm_Screw_Total_Turns += lift_up * ARM_SEQ_INITIAL_RAISE_TURNS;
			}
			Arm_Base_Total_Degrees = ARM_SEQ_BASE_PICK_DEGREE * ARM_DEGREE_SCALE;
			break;
		case ARM_SEQ_STAGE_EXTEND_AND_LOWER:
			Arm_Lift_Total_Turns += ARM_SEQ_EXTEND_TURNS * ARM_TURN_SCALE;
			Arm_Screw_Total_Turns += lift_up * (-ARM_SEQ_LOWER_PICK_TURNS);
			break;
		case ARM_SEQ_STAGE_LIFT_AND_HOME:
			Arm_Screw_Total_Turns += lift_up * ARM_SEQ_RAISE_AFTER_PICK_TURNS;
		Arm_Base_Total_Degrees = ARM_SEQ_BASE_HOME_DEGREE * ARM_DEGREE_SCALE;
		break;
	case ARM_SEQ_STAGE_RETRACT_AND_LOWER:
		Arm_Lift_Total_Turns -= ARM_SEQ_RETRACT_TURNS * ARM_TURN_SCALE;
			Arm_Screw_Total_Turns += -lift_up * ARM_SEQ_LOWER_RELEASE_TURNS;
		break;
	default:
		break;
	}
}

static void Arm_Combined_Sequence_Process(void)
{
	if(!arm_sequence_active ||
	   (int32_t)(xTaskGetTickCount() - arm_sequence_deadline) < 0)
	{
		return;
	}

	Arm_Combined_Sequence_Finish_Stage();
	arm_sequence_stage = (Arm_Sequence_Stage_t)(arm_sequence_stage + 1);
	if(arm_sequence_stage >= ARM_SEQ_STAGE_DONE)
	{
		arm_sequence_active = 0;
		usart1_SendString("[ARMSEQ] sequence complete\r\n> ");
		return;
	}
	Arm_Combined_Sequence_Run_Stage();
}

void Arm_Gripper_Grab(void)
{
	usart1_SendString("[ARMSEQ] grab (servo TODO)\r\n");
}

void Arm_Gripper_Release(void)
{
	usart1_SendString("[ARMSEQ] release (servo TODO)\r\n");
}

void Arm_Combined_Sequence_Start(void)
{
	if(arm_sequence_active)
	{
		usart1_SendString("[ARMSEQ] already running\r\n> ");
		return;
	}
	if(arm_lift_moving || arm_lift_queue_head != arm_lift_queue_tail)
	{
		usart1_SendString("[ARMSEQ] busy: finish manual motor command first\r\n> ");
		return;
	}

	arm_sequence_active = 1;
	arm_sequence_stage = ARM_SEQ_STAGE_BASE_PICK;
	usart1_SendString("[ARMSEQ] sequence start\r\n");
	Arm_Combined_Sequence_Run_Stage();
}

void Arm_Lift_Debug_Process(void)
{
	static char command[16];
	static uint8_t length = 0;
	uint8_t data;
	int32_t milli_turns;

	Arm_Combined_Sequence_Process();
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
			/* 一次性零点标定：输入 "5o" 回车，把5号电机当前位置存为零点(Flash) */
			if(command[0] == '5' && command[1] == 'o' && command[2] == '\0')
			{
				Arm_Base_Set_Origin_Here();
				continue;
			}
			if(command[0] == 'A' && command[1] == 'R' && command[2] == 'M' &&
			   command[3] == 'S' && command[4] == 'E' && command[5] == 'Q' &&
			   command[6] == '\0')
			{
				Arm_Combined_Sequence_Start();
				continue;
			}
			if(arm_sequence_active)
			{
				usart1_SendString("[ARMSEQ] running: manual motor command rejected\r\n> ");
				continue;
			}
			uint8_t motor = ARM_EXTEND_MOTOR;
			const char *value_text = command;
			int32_t max_value = ARM_EXTEND_MAX_TURNS;
			uint8_t absolute = 0;   /* '=' 分隔表示绝对角度（仅 5 号） */
			if((command[0] == '5' || command[0] == '6' || command[0] == '7') &&
			   (command[1] == '/' || command[1] == ':' || command[1] == ' ' || command[1] == '\t' || command[1] == '='))
			{
				motor = (uint8_t)(command[0] - '0');
				absolute = (command[1] == '=');
				value_text = &command[2];
				while(*value_text == ' ' || *value_text == '\t' || *value_text == ':') { value_text++; }
				max_value = motor == ARM_BASE_MOTOR ? ARM_BASE_MAX_DEGREES : ARM_LIFT_MAX_TURNS;
			}
			if(!Arm_Lift_ParseTurns(value_text, max_value, &milli_turns))
			{
				usart1_SendString("[ARM] invalid turns (-100..100)\r\n> ");
				continue;
			}
			if(absolute && motor == ARM_BASE_MOTOR)
			{
				/* 绝对角度：目标 - 当前累计角度 = 相对增量，后面按相对运动走 */
				usart1_SendString("[ARM] abs target=");
				Arm_Lift_PrintTurns(milli_turns);
				milli_turns -= Arm_Base_Total_Degrees;
				usart1_SendString(" delta=");
				Arm_Lift_PrintTurns(milli_turns);
				usart1_SendString("\r\n");
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
 * @brief   读取5号电机实时位置（脉冲，有符号）
 * @note    应答帧(0x36)：addr 0x36 pos[4字节大端有符号] 0x6B，共7字节。
 *          位置零点即 Origin_Set_O 标定的机械零点，正/负表示偏离方向。
 * @param   pulse_out：输出实时位置脉冲数
 * @retval  1=读取成功，0=无应答或帧格式不符
 */
static uint8_t Arm_Base_Read_Position(int32_t *pulse_out)
{
	fifo_initQueue();
	rxFrameFlag = false;
	Emm_V5_Read_Sys_Params(ARM_BASE_MOTOR, S_CPOS);
	vTaskDelay(pdMS_TO_TICKS(200));              // 等驱动器应答

	if(!rxFrameFlag || rxCount < 7 ||
	   rxCmd[0] != ARM_BASE_MOTOR || rxCmd[1] != 0x36)
	{
		return 0;
	}
	*pulse_out = ((int32_t)rxCmd[2] << 24) | ((int32_t)rxCmd[3] << 16) |
	             ((int32_t)rxCmd[4] << 8)  | ((int32_t)rxCmd[5] << 0);
	return 1;
}


/**
 * @brief   一次性标定：把5号电机当前位置设为回零零点并存入Flash
 * @note    用法：先用调试命令（如 "5/45" 转45度）把转盘转到机械零点位，
 *          再串口输入 "5o" 回车。此后每次上电回零都会回到这个位置。
 *          标定只需做一次（写入Flash），不要放进每次上电的流程。
 * @param   无
 * @retval  无
 */
void Arm_Base_Set_Origin_Here(void)
{
	usart1_SendString("[ARM] set current pos as base origin...\r\n");
	Emm_V5_En_Control(ARM_BASE_MOTOR, true, false);
	vTaskDelay(pdMS_TO_TICKS(30));
	Emm_V5_Origin_Set_O(ARM_BASE_MOTOR, true);   // true=存Flash
	vTaskDelay(pdMS_TO_TICKS(30));
	usart1_SendString("[ARM] origin saved to flash\r\n> ");
}

/**
 * @brief   5号电机上电回零
 * @note    发送驱动器真正的回零命令，回到已保存的机械零点。
 *          不调用 Reset_CurPos_To_Zero，避免把当前位置伪装成零点。
 *          回零参数和单圈零点需提前在驱动器中配置并保存。
 * @param   无
 * @retval  1=回零成功，0=读取位置失败或超时（结果USART1打印）
 */
uint8_t Arm_Base_Origin_Return(void)
{
	int32_t pos = 0;
	uint32_t waited = 0;

	usart1_SendString("[ARM] base origin return start\r\n");

	/* 1. 使能电机，等待驱动器接收后续回零命令 */
	Emm_V5_En_Control(ARM_BASE_MOTOR, true, false);
	vTaskDelay(pdMS_TO_TICKS(100));

	/* 2. 触发驱动器回零（0=单圈就近回零） */
	Emm_V5_Origin_Trigger_Return(ARM_BASE_MOTOR,
	                             ARM_BASE_ORIGIN_MODE, false);
	usart1_SendString("[ARM] origin trigger sent\r\n");

	/* 3. 读取实时位置，确认已回到驱动器保存的机械零点 */
	while(waited < ARM_BASE_ORIGIN_WAIT_MS)
	{
		vTaskDelay(pdMS_TO_TICKS(300));
		waited += 300;

		if(Arm_Base_Read_Position(&pos))
		{
			if(pos >= -ARM_BASE_HOME_TOL_PULSE &&
			   pos <= ARM_BASE_HOME_TOL_PULSE)
			{
				Arm_Base_Total_Degrees = 0;
				usart1_SendString("[ARM] base origin return OK\r\n");
				return 1;
			}
		}
	}

	usart1_SendString("[ARM] base origin return FAILED/timeout\r\n");
	return 0;
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
