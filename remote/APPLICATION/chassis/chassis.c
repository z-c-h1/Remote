#include "chassis.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "user_lib.h"
#include "robot_def.h"
#include "nac.h"
#include "usb.h" // 引入USB模块
#include "bsp_usart.h"

static DJIMotor_Instance *chassis_lf, *chassis_lb, *chassis_rf, *chassis_rb;
RC_ctrl_t *rc_cmd;
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;                                     // left right forward back
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb; // 6020电机 
static PID_Instance chassis_follow_pid;  // 底盘跟随PID
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static float at_lf, at_rf, at_lb, at_rb; // 底盘的角度解算后的临时输出,待进行限幅

Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;

void ChassisInit()
{
	    USB_Init(); // 初始化USB模块
		rc_cmd = RemoteControlInit(&huart3);
	// 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 4, // 3
                .Ki            = 0.2, // 0.5
                .Kd            = 0.005,   // 0
                .IntegralLimit = 3000,//5000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 10000,
            },
            .current_PID = {
                .Kp            = 1, // 1
                .Ki            = 0.01,   // 0
                .Kd            = 0,
                .IntegralLimit = 3000,//3000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3.;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

	chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
		chassis_motor_config.controller_param_init_config.speed_PID.Kp         =2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    // 6020电机初始化
    Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 12,
                .Ki                = 0.2,
                .Kd                = 0,
                .CoefA             = 5,
                .CoefB             = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 1000,
                .MaxOut            = 16000,
                .Derivative_LPF_RC = 0.001,
                .DeadBand          = 0.5,
            },
            .speed_PID = {
                .Kp            = 40,
                .Ki            = 3,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 4000,
                .MaxOut        = 20000,
                .Output_LPF_RC = 0.03,
            },
//						.angle_PID = {
//                .Kp                = 15,
//                .Ki                = 0.5,
//                .Kd                = 0.1,
//                .CoefA             = 5,
//                .CoefB             = 0.1,
//                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
//                .IntegralLimit     = 1000,
//                .MaxOut            = 16000,
//                .Derivative_LPF_RC = 0.001,
//                .DeadBand          = 0.5,
//            },
//            .speed_PID = {
//                .Kp            = 30,
//                .Ki            = 0.5,
//                .Kd            = 0.001,
//                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
//                .IntegralLimit = 4000,
//                .MaxOut        = 16000,
//                .Output_LPF_RC = 0.03,
//            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
//    chassis_motor_steering_config.can_init_config.tx_id = 4;
//    motor_steering_lf                                   = DJIMotorInit(&chassis_motor_steering_config);
//    chassis_motor_steering_config.can_init_config.tx_id = 1;
//    motor_steering_rf                                   = DJIMotorInit(&chassis_motor_steering_config);
//    chassis_motor_steering_config.can_init_config.tx_id = 3;
//    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
//    chassis_motor_steering_config.can_init_config.tx_id = 2;
//    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);

		PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 100, // 6
        .Ki                = 0.1f,
        .Kd                = 17, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 500, // 200
        .MaxOut            = 20000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
		nac_ctrl = NacInit(&huart1);
       chassis_ctrl_cmd.Chassis_IMU_data = INS_Init();
        chassis_ctrl_cmd.correct_mode =  IMU_CORRECT_HYBRID;
        chassis_ctrl_cmd.imu_enable = 1;                       // 使能IMU校准
        chassis_ctrl_cmd.target_yaw = 0;
        chassis_ctrl_cmd.offset_w = 0;
}



void GetCmd(){


}




void ChassisTask()
{
    // 检查遥控器是否在线，离线自动停转防止失控
    if(!RemoteControlIsOnline())
    {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
        DJIMotorSetRef(motor_lb, 0);
        DJIMotorSetRef(motor_rb, 0);
        return;
    }

    // 读取遥控器摇杆原始数据
    int16_t l_x = rc_cmd->rc.rocker_l_;  
    int16_t l_y = rc_cmd->rc.rocker_l1;  
    int16_t r_x = rc_cmd->rc.rocker_r_;  

    // 过滤摇杆中间位置的微小抖动
    if(abs(l_x) < 10) l_x = 0;
    if(abs(l_y) < 10) l_y = 0;
    if(abs(r_x) < 10) r_x = 0;

    // 摇杆输入映射到底盘速度
    const float MAX_LINEAR_SPEED = 8000.0f; 
    const float MAX_ROTATE_SPEED = 5000.0f;  
    float vx = l_y * MAX_LINEAR_SPEED / 660.0f; 
    float vy = l_x * MAX_LINEAR_SPEED / 660.0f; 
    float vw = r_x * MAX_ROTATE_SPEED / 660.0f; 

    // 麦轮运动学解算
    float speed_lf = vx - vy - vw; 
    float speed_rf = vx + vy + vw; 
    float speed_lb = vx + vy - vw; 
    float speed_rb = vx - vy + vw; 

    
    DJIMotorSetRef(motor_lf, speed_lf
	);
    DJIMotorSetRef(motor_rf, speed_rf);
    DJIMotorSetRef(motor_lb, speed_lb);
    DJIMotorSetRef(motor_rb, speed_rb);
}
