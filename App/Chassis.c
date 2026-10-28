#include "Chassis.h"
#include <stdint.h>

#define CHASSIS_KEY_W   (1u << 0)  //w = 1
#define CHASSIS_KEY_S 	(1u << 1)  //S = 2
#define CHASSIS_KEY_A 	(1u << 2)  //A = 4
#define CHASSIS_KEY_D 	(1u << 3)  //D = 8
#define CHASSIS_KEY_Q 	(1u << 4)  //Q = 16
#define CHASSIS_KEY_E   (1u << 5)  //E = 32

#define CHASSIS_AXIS_COMMAND 700
#define CHASSIS_MAX_SPEED    1000

static const uint8_t g_motor_reverse[4] = {0u, 1u, 0u, 1u};

//有符号的运动量
static int32_t chassis_axis_value(uint32_t movement, uint32_t positive_key, uint32_t negative_key)
{
	int32_t value = 0;
	
	if((movement & positive_key) != 0u)
	{
		value += CHASSIS_AXIS_COMMAND;
	}

	if((movement & negative_key) != 0u)
	{
		value -= CHASSIS_AXIS_COMMAND;
	}

	return value;
}	

//返回非负数绝对值
static int32_t chassis_abs(int32_t value)
{
	return (value < 0) ? -value : value;
}


//四路电机输出清零
static void chassis_zero_motors(RobotOutput *output)
{
	for(uint32_t i = 0u; i < 4u; i++)
	{
		output->motor[i].duty_permille = 0u;
		output->motor[i].forward = 0u;
	}
}

void Chassis_Update(RobotInput *input, RobotOutput *output)
{
	uint32_t movement;
	int32_t vx;
	int32_t vy;
	int32_t vw;
	int32_t wheel[4];
	int32_t maximum;
	uint32_t i;

	if(output == (RobotOutput *)0)
	{
		return;
	}
	
	if(input == (const RobotInput *)0)
	{
		chassis_zero_motors(output);
		return;
	}
	
	if(input->enabled == 0u)
	{
		chassis_zero_motors(output);
		return;
	}

	movement = (uint32_t)input->movement;
	
	vx = chassis_axis_value(movement, CHASSIS_KEY_W, CHASSIS_KEY_S);
	vy = chassis_axis_value(movement, CHASSIS_KEY_D, CHASSIS_KEY_A);
	vw = chassis_axis_value(movement, CHASSIS_KEY_E, CHASSIS_KEY_Q);

	wheel[0] = vx + vy + vw;    //FL
	wheel[1] = vx - vy - vw;    //FR
	wheel[2] = vx - vy + vw;	//RL 
	wheel[3] = vx + vy - vw;	//RR
	
	maximum = 0;
	
	for(i = 0u; i < 4u; i++)
	{
		int32_t abs_value = chassis_abs(wheel[i]);
		
		if(abs_value > maximum)
		{
			maximum = abs_value;
		}
	}

	if(maximum > CHASSIS_MAX_SPEED)
	{
		for(i = 0u; i < 4u; i++)
		{
			wheel[i] = (wheel[i] * CHASSIS_MAX_SPEED / maximum);
		}
	}
	
	for(i = 0u; i < 4u; i++)
	{
		int32_t logical_target = wheel[i];
		uint8_t forward;
		
		forward = (logical_target >= 0) ? 1u : 0u;
	
		if(g_motor_reverse[i] != 0u)
		{
		forward ^= 1u;
		}
	
	output->motor[i].forward = forward;
	output->motor[i].duty_permille = (uint16_t)chassis_abs(logical_target);
	}
	
}
