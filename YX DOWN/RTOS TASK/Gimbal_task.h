#ifndef GIMBAL_TASK_H__
#define GIMBAL_TASK_H__

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"

extern motor_info_t motor_info_chassis[8]; // 电机信息结构体[4]为云台电机

typedef struct
{
    motor_info_t motor_info;     // 电机信息结构体
    fp32 pid_parameter[3];       // 云台电机的pid参数
    fp32 pid_angle_parameter[3]; // 云台电机的pid参数
    pid_struct_t pid;            // 云台电机的pid结构体
    pid_struct_t pid_angle;      // 云台电机的pid结构体
	//    pid_struct_t pid_pitchangle;      // 云台电机的pid pitch结构体

    fp32 speed_target;           // 云台电机的目标速度
	//  fp32 speed_pitchtarget;           // 云台电机的目标速度2

    fp32 angle_target;           // 云台电机的目标角度
	  //fp32 angle_pitchtarget			//云台电机的目标pitch角度
} gimbal_t;

void Gimbal_task(void const *pvParameters);

#endif // !
