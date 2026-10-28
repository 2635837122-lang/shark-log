#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "robot_types.h"
//static int32_t chassis_axis_value(uint32_t movement, uint32_t positive_key, uint32_t negative_key);
//static int32_t chassis_abs(int32_t value);
//static void chassis_zero_motors(RobotOutput *output);
void Chassis_Update(RobotInput *input, RobotOutput *output);

#endif
