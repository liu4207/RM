#include "Gimbal_task.h"
#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#define MAX_SPEED 200

extern INS_t INS;
extern motor_info_t motor_info_chassis[8]; // 电机信息结构体[4]为云台电机
gimbal_t gimbal;
gimbal_t gimbalpitch;
gimbal_t gimbalshoot1;
gimbal_t gimbalshoot2;

fp32 err_yaw_angle;
fp32 err_pitch_angle;
fp32 err_shoot_speed1;
fp32 err_shoot_speed2;

fp32 shot=3000;//改变发射速度
//gimbal.pid_angle = 0;
uint8_t mode_flag = 0;

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体

// 云台电机的初始化
static void Gimbal_loop_Init();

// 模式选择
static void mode_select();

// 云台电机的任务
static void gimbal_current_give();
static void		pitch_current_give();
static void		shoot_current_give();
//// 遥控器控制云台电机
//static void RC_gimbal_control();

//// 云台旋转固定角度
//static void gimbal_turn_angle(fp32 angle);

// yaw轴控制电机
static void gimbal_yaw_control();
static void detel_calc(fp32 *angle);
static void detel_calc2(fp32 *angle);
static void gimbal_pitch_control();
static void gimbal_shoot_control();

void Gimbal_task(void const *pvParameters)
{
    Gimbal_loop_Init();
    for (;;)
    {
        mode_select();
        gimbal_current_give();
			pitch_current_give();
	shoot_current_give();	

        osDelay(1);
    }
}

// 云台电机的初始化
static void Gimbal_loop_Init()
{
    // 初始化pid参数
    gimbal.pid_parameter[0] = 30;
    gimbal.pid_parameter[1] = 0;
    gimbal.pid_parameter[2] = 0;
	   gimbalpitch.pid_parameter[0] = 30;
    gimbalpitch.pid_parameter[1] = 0;
   gimbalpitch.pid_parameter[2] = 0;
		    gimbalshoot1.pid_parameter[0] = 30;//发射pid
    gimbalshoot1.pid_parameter[1] = 0;
   gimbalshoot1.pid_parameter[2] = 0;
	gimbalshoot2.pid_parameter[0] = 30;
    gimbalshoot2.pid_parameter[1] = 0;
   gimbalshoot2.pid_parameter[2] = 0;
//gimbalpitch gimbalshoot1
    gimbal.pid_angle_parameter[0] = 60;
    gimbal.pid_angle_parameter[1] = 0;
    gimbal.pid_angle_parameter[2] = 0;
   gimbalpitch.pid_angle_parameter[0] = 60;
   gimbalpitch.pid_angle_parameter[1] = 0;
   gimbalpitch.pid_angle_parameter[2] = 0;
    gimbal.motor_info = motor_info_chassis[8]; // 云台电机的信息结构体
	    gimbalpitch.motor_info = motor_info_chassis[7]; // 云台电机的信息结构体
 gimbalshoot1.motor_info = motor_info_chassis[5]; // 云台电机的信息结构体 7 upshooter
 gimbalshoot2.motor_info = motor_info_chassis[6]; // 云台电机的信息结构体 8 downshooter

    gimbal.speed_target = 0;
	    gimbalpitch.speed_target = 0;
	    gimbalshoot1.speed_target = 0;
	    gimbalshoot2.speed_target = 0;

     gimbal.angle_target = 0;
	     gimbalpitch.angle_target = 0;

//    gimbal.angle_target = gimbal.motor_info.real_angle;

    // 初始化pid结构体
    pid_init(&gimbalpitch.pid, gimbalpitch.pid_parameter, 16000, 16000);
		    pid_init(&gimbal.pid, gimbal.pid_parameter, 16000, 16000);
		    pid_init(&gimbalshoot1.pid, gimbal.pid_parameter, 16000, 16000);
		    pid_init(&gimbalshoot2.pid, gimbal.pid_parameter, 16000, 16000);

    pid_init(&gimbalpitch.pid_angle, gimbalpitch.pid_angle_parameter, 15000, 15000);
		    pid_init(&gimbal.pid_angle, gimbal.pid_angle_parameter, 15000, 15000);

}

// 模式选择
static void mode_select()
{
//    if (rc_ctrl.rc.s[0] == 1) // BLUE LED
//    {
//        gimbal_turn_angle(45);
//        mode_flag = 1;
//    }
//    if (rc_ctrl.rc.s[0] == 2) // GREEN LED
//    {
//        RC_gimbal_control();
//        mode_flag = 2;
//    }
//    if (rc_ctrl.rc.s[0] == 3) // RED LED
//    {
        gimbal_yaw_control();
	
	gimbal_pitch_control();

        mode_flag = 3;
//    }
}

// 给电流，CAN1调试用，没板子。。。。。。
static void gimbal_current_give()
{
    gimbal.motor_info = motor_info_chassis[8];//id 5
    gimbal.motor_info.set_current = pid_calc(&gimbal.pid, gimbal.motor_info.rotor_speed, gimbal.speed_target);
	
    set_gimbal_current(1, gimbal.motor_info.set_current, 0, 0, 0);
	//set_pitch_current(1, 0, 0, gimbal.speed_pitchtarget,0);
}
static void pitch_current_give()//逻辑有点问题没改
{
    gimbalpitch.motor_info = motor_info_chassis[7];//id 7
    gimbalpitch.motor_info.set_current = pid_calc(&gimbalpitch.pid, gimbalpitch.motor_info.rotor_speed, gimbalpitch.speed_target);
	
   // set_gimbal_current(1, gimbal.motor_info.set_current, 0, 0, 0);
	set_pitch_current(1, 0, 0,gimbalpitch.motor_info.set_current,0);
}
static void shoot_current_give()
{
    gimbalshoot1.motor_info = motor_info_chassis[5];
    gimbalshoot1.motor_info.set_current = pid_calc(&gimbalshoot1.pid, gimbalshoot1.motor_info.rotor_speed, gimbalshoot1.speed_target);
	 gimbalshoot2.motor_info = motor_info_chassis[6];
    gimbalshoot2.motor_info.set_current = pid_calc(&gimbalshoot2.pid, gimbalshoot2.motor_info.rotor_speed, gimbalshoot2.speed_target);
   // set_gimbal_current(1, gimbal.motor_info.set_current, 0, 0, 0);
set_motor_current_can2(1,  gimbalshoot1.motor_info.set_current,gimbalshoot2.motor_info.set_current, 0,0);//没有调正负
}

// 遥控器控制云台电机
//static void RC_gimbal_control()
//{
//    if (mode_flag != 2)
//    {
//        gimbal.angle_target = gimbal.motor_info.real_angle;
//    }

//    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
//    {
//        // gimbal.speed_target = rc_ctrl.rc.ch[1] / 660.0 * MAX_SPEED;
//        gimbal.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 0.3;
//        detel_calc2(&gimbal.angle_target);
//        gimbal.speed_target = gimbal_PID_calc(&gimbal.pid_angle, gimbal.motor_info.real_angle, gimbal.angle_target);
//    }
//    else
//    {
//        gimbal.speed_target = 0;
//    }
//}

// 云台旋转固定角度
//static void gimbal_turn_angle(fp32 angle)
//{
//    if (mode_flag != 1)
//    {
//        gimbal.angle_target = gimbal.motor_info.real_angle + angle;
//    }

//    detel_calc2(&gimbal.angle_target);
//    gimbal.speed_target = gimbal_PID_calc(&gimbal.pid_angle, gimbal.motor_info.real_angle, gimbal.angle_target);
//}

// yaw轴控制电机
static void gimbal_yaw_control()
{
    if (mode_flag != 3)
    {
        gimbal.angle_target = 0;
    }

    gimbal.motor_info = motor_info_chassis[4];
    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        gimbal.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 0.3;

        detel_calc(&gimbal.angle_target);

        err_yaw_angle = gimbal.angle_target - INS.Yaw;

        detel_calc(&err_yaw_angle);

        gimbal.speed_target = gimbal_PID_calc1(&gimbal.pid_angle, 0, err_yaw_angle);
    }
}

static void detel_calc(fp32 *angle)
{
    if (*angle > 180)
    {
        *angle -= 360;
    }

    else if (*angle < -180)
    {
        *angle += 360;
    }
}

static void detel_calc2(fp32 *angle)
{
    if (*angle > 360)
    {
        *angle -= 360;
    }

    else if (*angle < 0)
    {
        *angle += 360;
    }
}
static void gimbal_pitch_control()
{
	//gimbalpitch.angle_target =0;
	if (rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0] <= 660)
    {
        gimbalpitch.angle_target += rc_ctrl.rc.ch[0] / 660.0 * 0.3;

       // detel_calc(&gimbalpitch.angle_target);

        err_pitch_angle = gimbalpitch.angle_target - INS.Pitch;

      //  detel_calc(&err_pitch_angle);

        gimbalpitch.speed_target = gimbal_PID_calc1(&gimbalpitch.pid_angle, 0, err_pitch_angle);
    }
	
	
}
static void gimbal_shoot_control()
{
	if(1)//遥控器
	{
		err_shoot_speed1=shot-motor_info_chassis[5].rotor_speed ;
		gimbalshoot1.speed_target=pid_calc(&gimbalshoot1.pid, 0, err_shoot_speed1);
		err_shoot_speed2=shot-motor_info_chassis[6].rotor_speed ;
		gimbalshoot2.speed_target=pid_calc(&gimbalshoot2.pid, 0, err_shoot_speed2);

	}
}
