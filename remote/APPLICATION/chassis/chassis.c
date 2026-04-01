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
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb, *motor_1, *motor_2;                                     // left right forward back
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
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);
	
	chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
//		chassis_motor_config.controller_param_init_config.speed_PID.Kp         =2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

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
    chassis_motor_steering_config.can_init_config.tx_id = 4;
    motor_steering_lf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_steering_rf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 3;
    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 2;
    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);

	
//// -------------------------- 原来的旧代码 --------------------------
//    PID_Init_Config_s chassis_follow_pid_conf = {
//        .Kp                = 100, // 6
//        .Ki                = 0.1f,
//        .Kd                = 17, // 0.5
//        .DeadBand          = 0.5,
//        .CoefA             = 0.2,
//        .CoefB             = 0.3,
//        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit,
//        .IntegralLimit     = 500, // 200
//        .MaxOut            = 20000,
//        .Derivative_LPF_RC = 0.01, // 0.01
//    };
//    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
//		nac_ctrl = NacInit(&huart1);
//       chassis_ctrl_cmd.Chassis_IMU_data = INS_Init();
//        chassis_ctrl_cmd.correct_mode =  IMU_CORRECT_HYBRID;
//        chassis_ctrl_cmd.imu_enable = 1;                       // ???IMUУ?
//        chassis_ctrl_cmd.target_yaw = 0;
//        chassis_ctrl_cmd.offset_w = 0;


// -------------------------- 替换成下面的新代码 --------------------------
		// 航向跟随PID参数,针对IMU航向锁优化过的参数
		// 你可以根据自己的车调整:
		// Kp越大,锁得越紧,但是太大了会抖; Ki用来消除稳态误差; Kd用来抑制超调
    PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 80,  // 比例系数,默认80,如果跑偏就调大,如果抖就调小
        .Ki                = 0.05f, // 积分系数,默认0.05,用来消除小的稳态误差
        .Kd                = 10,  // 微分系数,默认10,抑制抖动
        .DeadBand          = 0.1f, // 角度死区,0.1度以内的误差忽略,防止抖
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit,
        .IntegralLimit     = 500, // 积分限幅,防止积分饱和
        .MaxOut            = 1000, // 最大补偿输出,限制最大的修正速度,防止修正太猛
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
		nac_ctrl = NacInit(&huart1);
       
       //初始化IMU,获取姿态数据指针(你原来就有这行,没改)
       chassis_ctrl_cmd.Chassis_IMU_data = INS_Init();
        chassis_ctrl_cmd.correct_mode =  IMU_CORRECT_HYBRID;
        chassis_ctrl_cmd.imu_enable = 1;                       // 开启IMU矫正,默认开
        
        //新加的: 开机的时候,把初始的车头方向设为目标航向,不是硬写0了!
        if(chassis_ctrl_cmd.Chassis_IMU_data != NULL)
        {
            chassis_ctrl_cmd.target_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
        }
        else
        {
            chassis_ctrl_cmd.target_yaw = 0;
        }
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
    int16_t l_x = rc_cmd->rc.rocker_l_;  //左水平
    int16_t l_y = rc_cmd->rc.rocker_l1;  //左竖直
    int16_t r_x = rc_cmd->rc.rocker_r_;  //右水平

    // 过滤摇杆中间位置的微小抖动
    if(abs(l_x) < 10) l_x = 0;
    if(abs(l_y) < 10) l_y = 0;
    if(abs(r_x) < 10) r_x = 0;

    const float MAX_LINEAR_SPEED = 8000.0f; 
    const float MAX_ROTATE_SPEED = 5000.0f;  
    float vx = l_y * MAX_LINEAR_SPEED / 660.0f; 
    float vy = l_x * MAX_LINEAR_SPEED / 660.0f; 
    float vw = r_x * MAX_ROTATE_SPEED / 660.0f; 
	

// -------------------------- 新加的IMU航向锁核心代码 --------------------------
    // ====================== IMU航向锁核心逻辑 ======================
    if(chassis_ctrl_cmd.imu_enable && chassis_ctrl_cmd.Chassis_IMU_data != NULL)
    {
        // 1. 先读IMU已经解好的当前车头方向(Yaw角,单位是度)
        float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
        
        if(abs(r_x) < 10) // 2. 判断你有没有碰右摇杆? 没碰=你现在在平移,不想转车头
        {
            // 3. 算一下: 现在的车头,和我要锁的目标车头,差了多少度?
            float yaw_err = chassis_ctrl_cmd.target_yaw - current_yaw;
            // 4. 处理角度过零点的问题: 比如179度到-179度,差的是2度,不是-358度!
            if(yaw_err > 180.0f)      yaw_err -= 360.0f;
            else if(yaw_err < -180.0f) yaw_err += 360.0f;
            
            // 5. PID计算: 把角度误差,转成一个补偿的旋转速度
            PIDCalculate(&chassis_follow_pid, yaw_err, 0);
            chassis_ctrl_cmd.offset_w = chassis_follow_pid.Output;
            
            // 6. 把这个补偿加到原来的旋转速度vw上! 自动把车头掰回来!
            vw += chassis_ctrl_cmd.offset_w;
        }
        else // 3. 你碰了右摇杆=你在手动转车头
        {
            // 那我就把目标车头,更新成你现在转完的新车头!
            chassis_ctrl_cmd.target_yaw = current_yaw;
            // 重置PID的积分,防止积分饱和
            chassis_follow_pid.Iout = 0;
        }
    }

    // 麦轮运动学解算,和原来一样,只是用了修正后的vw
    float speed_lf = vx - vy - vw; 
    float speed_rf = vx + vy + vw; 
    float speed_lb = vx + vy - vw; 
    float speed_rb = vx - vy + vw; 
    
    DJIMotorSetRef(motor_lf, speed_lf);
    DJIMotorSetRef(motor_rf, speed_rf);
    DJIMotorSetRef(motor_lb, speed_lb);
    DJIMotorSetRef(motor_rb, speed_rb);

}
