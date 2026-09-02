#include "moving.h"

/**********************************************************
***     FreeRTOS 任务句柄
**********************************************************/
TaskHandle_t MovingTask_Handler = NULL;
static uint16_t remote_move_vel = REMOTE_MOVE_VEL;
static char remote_active_command = 'x';

/**********************************************************
***     外部声明 main.c 中的顺序业务逻辑函数
**********************************************************/
extern void User_Sequential_Logic(void);

/**
* @brief   FreeRTOS 任务执行函数（运行 main.c 中的顺序控制逻辑）
* @param   pvParameters: 任务参数
* @retval  无
*/
static void moving_task(void *pvParameters)
{
	// 执行 main.c 中编写的顺序逻辑
	User_Sequential_Logic();

	// 任务执行完毕自销毁
	vTaskDelete(NULL);
}

/**
        * @brief   创建 FreeRTOS 任务
        * @param   无
        * @retval  无
        */
void Moving_Task_Create(void)
{
	xTaskCreate((TaskFunction_t )moving_task,
				(const char*    )"moving_task",
				(uint16_t       )MOVING_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )MOVING_TASK_PRIO,
				(TaskHandle_t*  )&MovingTask_Handler);
}

static void chassis_remote_velocity(uint8_t dir1, uint8_t dir2,uint8_t dir3, uint8_t dir4)
{
	uint8_t dirs[4];
	uint8_t i, addr;

	dirs[0] = dir1; dirs[1] = dir2; dirs[2] = dir3; dirs[3] = dir4;

	// 关键：像"能转的 p 探针"那样，逐个电机"使能→间隔→速度→间隔"发送。
	// 4条指令背靠背连发时，共享总线上的驱动器来不及逐帧处理/按地址过滤会丢指令
	// （表现为 w/a/s/d 不动、只有留了间隔的 p 才动）。因此每帧之间必须留处理间隔。
	for(i = 0; i < 4; i++)
	{
			addr = (uint8_t)(i + 1);
			// 运动路径只发不收，不读USART3——避免总线有噪声时被 while(RXNE) 卡死
			Emm_V5_En_Control(addr, true, false);
			vTaskDelay(pdMS_TO_TICKS(8));
			Emm_V5_Vel_Control(addr, dirs[i], remote_move_vel, REMOTE_MOVE_ACC, false);
			vTaskDelay(pdMS_TO_TICKS(8));
	}
}

/* 斜向移动：dirs[i]=方向位，on[i]=1 该轮出力、0 该轮给 0 速度。
 * 45° 斜向用「对角两轮出力、另两轮停」拼合（mecanum 对角分解），
 * 帧间同样留 8ms 处理间隔。 */
static void chassis_remote_velocity4(const uint8_t dirs[4], const uint8_t on[4])
{
	uint8_t i, addr;

	for(i = 0; i < 4; i++)
	{
			addr = (uint8_t)(i + 1);
			Emm_V5_En_Control(addr, true, false);
			vTaskDelay(pdMS_TO_TICKS(8));
			Emm_V5_Vel_Control(addr, dirs[i], on[i] ? remote_move_vel : 0,
							   REMOTE_MOVE_ACC, false);
			vTaskDelay(pdMS_TO_TICKS(8));
	}
}

void Chassis_Remote_Stop(void)
{
	uint8_t i;

	remote_active_command = 'x';
	// 减速停：发目标速度 0 + 与加速相同的 acc，驱动按 acc 对称减速到 0（不再立即停）
	for(i = 1; i <= 4; i++)
	{
		Emm_V5_Vel_Control(i, 0, 0, REMOTE_MOVE_ACC, false);
		vTaskDelay(pdMS_TO_TICKS(8));
	}
}

void Chassis_Remote_SpeedAdjust(int8_t direction)
{
	if(direction > 0 && remote_move_vel < REMOTE_MOVE_VEL_MAX)
	{
		remote_move_vel += REMOTE_MOVE_VEL_STEP;
	}
	else if(direction < 0 && remote_move_vel > REMOTE_MOVE_VEL_MIN)
	{
		remote_move_vel -= REMOTE_MOVE_VEL_STEP;
	}
	if(remote_active_command != 'x')
	{
		Chassis_Remote_Command(remote_active_command);
	}
}

void Chassis_Remote_Command(char command)
{
	remote_active_command = command;
	switch(command)
	{
		case 'w': chassis_remote_velocity(0, 1, 0, 1); break;
		case 's': chassis_remote_velocity(1, 0, 1, 0); break;
		case 'a': chassis_remote_velocity(0, 0, 1, 1); break;
		case 'd': chassis_remote_velocity(1, 1, 0, 0); break;
		case 'e': chassis_remote_velocity(0, 0, 0, 0); break;
		case 'r': chassis_remote_velocity(1, 1, 1, 1); break;
		/* 斜向 45°（方向位表按下述对角推导，若实测方向反/偏，调这里）：
		 *   前左 q: 轮1 CW + 轮4 CCW        前右 c: 轮2 CCW + 轮3 CW
		 *   后左 z: 轮2 CW + 轮3 CCW        后右 v: 轮1 CCW + 轮4 CW   */
		case 'q': { uint8_t d[4] = {0, 0, 0, 1}; uint8_t o[4] = {1, 0, 0, 1}; chassis_remote_velocity4(d, o); break; }
		case 'c': { uint8_t d[4] = {0, 1, 0, 0}; uint8_t o[4] = {0, 1, 1, 0}; chassis_remote_velocity4(d, o); break; }
		case 'z': { uint8_t d[4] = {0, 0, 1, 0}; uint8_t o[4] = {0, 1, 1, 0}; chassis_remote_velocity4(d, o); break; }
		case 'v': { uint8_t d[4] = {1, 0, 0, 0}; uint8_t o[4] = {1, 0, 0, 1}; chassis_remote_velocity4(d, o); break; }
		case 'x': Chassis_Remote_Stop(); break;
		default: break;
	}
}

/**
* @brief   使能底盘4个电机
* @note    张大头 Emm_V5 速度/位置模式在电机未使能时会忽略运动指令，
*          所以底盘上电后、发送速度指令前必须先逐个使能 1~4 号电机。
* @retval  无
*/
void Chassis_Remote_Enable(void)
{
	Emm_V5_En_Control(1, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));
	Emm_V5_En_Control(2, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));
	Emm_V5_En_Control(3, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));
	Emm_V5_En_Control(4, true, false);
	vTaskDelay(pdMS_TO_TICKS(10));
	usart1_SendString("[REMOTE] motors 1-4 enabled\r\n");
}

/**
* @brief   读取USART1并分发底盘遥控指令（底盘专用）
* @note    w/a/s/d/e/r/x 为单字符即时指令；"+"/"-" 以换行结尾用于调速。
*          每收到一个有效指令都会通过USART1回显，便于判断“串口是否收到”。
* @retval  无
*/
void Chassis_Remote_Process(void)
{
	static char line[16];
	static uint8_t len = 0;

	// 直接轮询USART1数据寄存器（已在RxProbe_Begin中关闭接收中断）。
	while(USART1->SR & USART_FLAG_RXNE)
	{
		uint8_t data = (uint8_t)USART1->DR;

		if(data == 'p')          // 逐个探测1~4号驱动器，打印回包（诊断电机侧）
		{
			Chassis_Motor_Probe();
			continue;
		}
		if(data == 'w' || data == 'a' || data == 's' || data == 'd' ||
		   data == 'e' || data == 'r' || data == 'x' ||
		   data == 'q' || data == 'c' || data == 'z' || data == 'v')
		{
			Chassis_Remote_Command((char)data);
			// 回显，证明USART1确实收到并已分发该指令。
			// 注意：GUI 心跳每200ms重发一次当前指令，其中停止指令'x'占多数，
			// 若'x'也回显，GUI 日志区会被每秒5行的 cmd=x 刷屏，故跳过。
			if(data != 'x')
			{
				usart1_SendString("[REMOTE] cmd=");
				usart1_SendByte((uint16_t)data);
				usart1_SendString("\r\n");
			}
			len = 0;
			continue;
		}
		if(data == '\r' || data == '\n')
		{
			if(len == 0) { continue; }
			line[len] = '\0';
			if(line[0] == '+' && line[1] == '\0')
			{
				Chassis_Remote_SpeedAdjust(1);
				usart1_SendString("[REMOTE] speed +\r\n");
			}
			else if(line[0] == '-' && line[1] == '\0')
			{
				Chassis_Remote_SpeedAdjust(-1);
				usart1_SendString("[REMOTE] speed -\r\n");
			}
			else if(line[0] == 'M' && len >= 3 && line[2] == ' ')
			{
				/* 位置模式定距移动：M<dir> <pulses>，如 "Mw 3200" */
				uint8_t k = 3;
				uint32_t mpulse = 0;
				char mdir = line[1];
				while(k < len && line[k] >= '0' && line[k] <= '9')
				{
					mpulse = mpulse * 10U + (uint32_t)(line[k] - '0');
					k++;
				}
				if(mpulse > 0)
				{
					Chassis_Remote_Move(mdir, mpulse);
				}
			}
			len = 0;
			continue;
		}
		if(len < sizeof(line) - 1 && data >= 0x20 && data <= 0x7E)
		{
			line[len++] = (char)data;
		}
	}
}

/**
* @brief   诊断：打印一个字节的16进制
*/
static void diag_print_hex8(uint8_t v)
{
	static const char hex[] = "0123456789ABCDEF";
	usart1_SendByte((uint16_t)hex[(v >> 4) & 0x0F]);
	usart1_SendByte((uint16_t)hex[v & 0x0F]);
}

/**
* @brief   诊断：有上限地清空USART3接收缓冲
* @note    总线空闲立即返回；即使持续有数据/噪声，读满上限也会退出，绝不死循环。
*/
static void diag_flush_usart3(void)
{
	volatile uint32_t d = 0;
	while((USART3->SR & USART_FLAG_RXNE) && d < 100000UL)
	{
		(void)USART3->DR;
		d++;
	}
}

/**
* @brief   诊断：轮询USART3收集驱动器回包并打印（十六进制）
* @note    双重上限保护：gap为距上一个字节的空转（判断回包结束），total为绝对上限，
*          即使总线噪声/驱动器持续发送也不会卡死（这正是拔掉6/7后卡死的根因）。
*/
static void diag_dump_usart3_reply(void)
{
	uint8_t buf[16];
	uint8_t n = 0;
	uint8_t i;
	volatile uint32_t total = 0;
	volatile uint32_t gap = 0;

	while(gap < 150000UL && total < 3000000UL)
	{
		if(USART3->SR & USART_FLAG_RXNE)
		{
			uint8_t b = (uint8_t)USART3->DR;
			if(n < sizeof(buf)) { buf[n++] = b; }
			else { break; }          // 缓冲满即停
			gap = 0;
		}
		else
		{
			gap++;
		}
		total++;
	}

	if(n == 0) { usart1_SendString(" <none>"); }
	for(i = 0; i < n; i++)
	{
		usart1_SendString(" 0x");
		diag_print_hex8(buf[i]);
	}
	usart1_SendString("\r\n");
}

/**
* @brief   诊断：逐个测试1~4号驱动器（使能→速度0.4秒→停止），打印每步回包
* @note    张大头回包：成功=addr 功能码 02 6B；条件不满足(未使能等)=addr 功能码 E2 6B；
*          命令/地址/校验错=addr 00 EE 6B；无回包=总线/接线/波特率/地址问题。
*          每个电机单独转约0.4秒，可直接看哪个电机动、哪个不动。
*/
void Chassis_Motor_Probe(void)
{
	uint8_t addr;

	usart1_SendString("[PROBE] test drivers 1-4 ...\r\n");
	for(addr = 1; addr <= 4; addr++)
	{
		diag_flush_usart3();
		usart1_SendString("[M");
		usart1_SendByte((uint16_t)('0' + addr));
		usart1_SendString("] en: ");
		Emm_V5_En_Control(addr, true, false);
		diag_dump_usart3_reply();

		diag_flush_usart3();
		usart1_SendString("[M");
		usart1_SendByte((uint16_t)('0' + addr));
		usart1_SendString("] vel:");
		Emm_V5_Vel_Control(addr, 0, remote_move_vel, REMOTE_MOVE_ACC, false);
		diag_dump_usart3_reply();

		vTaskDelay(pdMS_TO_TICKS(400));   // 该电机转约0.4秒

		diag_flush_usart3();
		usart1_SendString("[M");
		usart1_SendByte((uint16_t)('0' + addr));
		usart1_SendString("] stop:");
		Emm_V5_Stop_Now(addr, false);
		diag_dump_usart3_reply();
	}
	usart1_SendString("[PROBE] done\r\n");
}

/**
* @brief   诊断：关闭USART1接收中断，改为主循环直接轮询寄存器
* @note    绕过中断/NVIC/环形缓冲，只留下“UART外设 + 接收线”本身，
*          用于判定 PA10 到底有没有收到数据（RX-OK/RX-ERR/无反应）。
*/
void Chassis_Remote_RxProbe_Begin(void)
{
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	// 电机总线USART3也改用轮询读取回包，关掉其接收/空闲中断避免抢字节
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, DISABLE);
	usart1_SendString("[REMOTE] USART1 RX polling mode ON (IRQ disabled)\r\n");
}

/**
* @brief   诊断：轮询USART1接收寄存器并回报
* @note    收到有效字节打印 RX-OK 并驱动底盘；收到帧/噪声/溢出错误打印 RX-ERR；
*          若一直没有任何打印，说明 PA10 根本没有电平变化（线路/适配器/端口）。
*/
void Chassis_Remote_RxProbe(void)
{
	uint16_t sr = (uint16_t)USART1->SR;

	if(sr & (USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_ORE))
	{
			uint8_t bad = (uint8_t)USART1->DR;   // 读DR清除错误标志
			usart1_SendString("[DIAG] RX-ERR SR=0x");
			diag_print_hex8((uint8_t)sr);
			usart1_SendString(" data=0x");
			diag_print_hex8(bad);
			usart1_SendString("\r\n");
	}
	else if(sr & USART_FLAG_RXNE)
	{
			uint8_t d = (uint8_t)USART1->DR;
			usart1_SendString("[DIAG] RX-OK byte=0x");
			diag_print_hex8(d);
			if(d >= 0x20 && d <= 0x7E)
			{
					usart1_SendString(" '");
					usart1_SendByte((uint16_t)d);
					usart1_SendString("'");
			}
			usart1_SendString("\r\n");
			Chassis_Remote_Command((char)d);     // 若真的收到，顺便驱动底盘
	}
}

/**
* @brief   底盘前进：4 电机多机同步位置控制
* @param   pulse: 旋转脉冲数（16细分下 3200 脉冲 = 1 圈）
* @retval  无
*/
void move_qian(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void move_zuo(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void move_you(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void move_hou(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void zhuan_shun(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void zhuan_ni(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}

/* 位置模式定距移动：dirs[]=方向位，on[]=1 该轮走 pulse 脉冲、0 该轮不动（斜向用） */
static void chassis_pos4(const uint8_t dirs[4], const uint8_t on[4], uint32_t pulse)
{
	uint8_t i, addr;

	for(i = 0; i < 4; i++)
	{
		if(!on[i]) { continue; }
		addr = (uint8_t)(i + 1);
		Emm_V5_Pos_Control(addr, dirs[i], REMOTE_POS_VEL, REMOTE_POS_ACC,
						   pulse, 0, 1);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	Emm_V5_Synchronous_motion(0);           /* 广播触发，多机同步起步 */
	vTaskDelay(pdMS_TO_TICKS(10));
}

/* 斜向位置移动（前6个参数依次为4轮方向位 + 4轮使能位） */
static void chassis_move_diag(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                              uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4,
                              uint32_t pulse)
{
	uint8_t d[4] = {d1, d2, d3, d4};
	uint8_t o[4] = {o1, o2, o3, o4};
	chassis_pos4(d, o, pulse);
}

/* 十进制打印（无 stdio） */
static void print_dec_u32(uint32_t v)
{
	char buf[11];
	uint8_t k = 0;

	if(v == 0) { usart1_SendByte('0'); return; }
	while(v > 0) { buf[k++] = (char)('0' + (v % 10U)); v /= 10U; }
	while(k > 0) { usart1_SendByte((uint16_t)buf[--k]); }
}

/* 位置模式定距移动分发：dir=w/a/s/d/e/r(正交/旋转) 或 q/z/c/v(斜向) */
void Chassis_Remote_Move(char dir, uint32_t pulse)
{
	switch(dir)
	{
		case 'w': move_qian(pulse); break;
		case 's': move_hou(pulse); break;
		case 'a': move_zuo(pulse); break;
		case 'd': move_you(pulse); break;
		case 'e': zhuan_shun(pulse); break;
		case 'r': zhuan_ni(pulse); break;
		case 'q': chassis_move_diag(0, 1, 0, 0, 0, 1, 1, 0, pulse); break;  /* 前左 */
		case 'c': chassis_move_diag(0, 0, 0, 1, 1, 0, 0, 1, pulse); break;  /* 前右 */
		case 'z': chassis_move_diag(1, 0, 0, 0, 1, 0, 0, 1, pulse); break;  /* 后左 */
		case 'v': chassis_move_diag(0, 0, 1, 0, 0, 1, 1, 0, pulse); break;  /* 后右 */
		default: return;
	}

	usart1_SendString("[MOVE] dir=");
	usart1_SendByte((uint16_t)dir);
	usart1_SendString(" pulse=");
	print_dec_u32(pulse);
	usart1_SendString("\r\n");
}
