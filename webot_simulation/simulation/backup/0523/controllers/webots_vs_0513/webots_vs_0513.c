/*
 * File:          legged_control_0413.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */


#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "webotsInterface.h"
#include "controller.h"


WbDeviceTag motor[4][3];
WbDeviceTag pos_sensor[4][3];
WbDeviceTag touch_sensor[4];
WbDeviceTag IMU;

//-----------------------------------------------------------函数声明
void webots_device_init();
void set_motor_position(motorNameTypeDef motorName, double angle);
void joint_drive(legNameTypeDef legName);
void motorangle_2_jointangle(legNameTypeDef legName);
void jointangle_2_motorangle(legNameTypeDef legName);
void IK_2_jointdrive(legNameTypeDef legName);
void trajectory(legNameTypeDef legName, double steplength, double stephight);
double get_motor_angle(motorNameTypeDef motorName);
bool is_foot_touching(legNameTypeDef legName);
int get_keyboard();
eulerAngleTypeDef get_IMU_Angle();
robotTypeDef robot;
//-----------------------------------------------------------device

/*电机*/
const char* MOTION_NAMES[4][3] = {
   {"LF0", "LF1", "LF2"},
   {"RF0","RF1", "RF2"},
   {"RH0","RH1", "RH2"},
   {"LH0", "LH1","LH2"}
};
/*电机编码器*/
const char* POS_MOTION_NAMES[4][3] = {
  {"LF0 sensor", "LF1 sensor", "LF2 sensor"},
  {"RF0 sensor", "RF1 sensor", "RF2 sensor"},
  {"RH0 sensor", "RH1 sensor", "RH2 sensor"},
  {"LH0 sensor", "LH1 sensor", "LH2 sensor"}
};
/*足底触碰开关*/
const char* TOUCH_SENSOR_NAMES[4] = {
  "LF sensor",
  "RF sensor",
  "RH sensor",
  "LH sensor" };

/*
函数功能：初始化设备，电机+角度+足端传感器+IMU
*/
void webots_device_init()
{
    for (legNameTypeDef i = 0; i < 4; i++)
    {
        for (angleNameTypeDef j = 0; j < 3; j++)
        {
            motor[i][j] = wb_robot_get_device(MOTION_NAMES[i][j]);  //电机
            pos_sensor[i][j] = wb_robot_get_device(POS_MOTION_NAMES[i][j]);  //角位移传感器
            wb_position_sensor_enable(pos_sensor[i][j], TIME_STEP);
        }
        touch_sensor[i] = wb_robot_get_device(TOUCH_SENSOR_NAMES[i]);
        wb_touch_sensor_enable(touch_sensor[i], TIME_STEP);
    }
    IMU = wb_robot_get_device("inertial unit");  //惯性测量单元
    wb_inertial_unit_enable(IMU, TIME_STEP);

}
//-----------------------------------------------------------motor
/*
函数功能：设置电机转角
*/
void set_motor_position(motorNameTypeDef motorName, double angle)
{
    switch (motorName) {
    case LF0: {  wb_motor_set_position(motor[0][0], angle); break;  }
    case LF1: {  wb_motor_set_position(motor[0][1], angle); break;  }
    case LF2: {  wb_motor_set_position(motor[0][2], angle); break;  }

    case RF0: {  wb_motor_set_position(motor[1][0], angle); break;  }
    case RF1: {  wb_motor_set_position(motor[1][1], angle); break;  }
    case RF2: {  wb_motor_set_position(motor[1][2], angle); break;  }

    case RH0: {  wb_motor_set_position(motor[2][0], angle); break;  }
    case RH1: {  wb_motor_set_position(motor[2][1], angle); break;  }
    case RH2: {  wb_motor_set_position(motor[2][2], angle); break;  }

    case LH0: {  wb_motor_set_position(motor[3][0], angle); break;  }
    case LH1: {  wb_motor_set_position(motor[3][1], angle); break;  }
    case LH2: {  wb_motor_set_position(motor[3][2], angle); break;  }

    default:break;
    }
}

//-----------------------------------------------------------sensor
/*
函数功能：获取电机角度,弧度制
*/
double get_motor_angle(motorNameTypeDef motorName)
{
    double angle = 0;
    switch (motorName) {
    case LF0: { angle = wb_position_sensor_get_value(pos_sensor[0][0]); break;  }
    case LF1: { angle = wb_position_sensor_get_value(pos_sensor[0][1]); break;  }
    case LF2: { angle = wb_position_sensor_get_value(pos_sensor[0][2]); break;  }

    case RF0: { angle = wb_position_sensor_get_value(pos_sensor[1][0]); break;  }
    case RF1: { angle = wb_position_sensor_get_value(pos_sensor[1][1]); break;  }
    case RF2: { angle = wb_position_sensor_get_value(pos_sensor[1][2]); break;  }

    case RH0: { angle = wb_position_sensor_get_value(pos_sensor[2][0]); break;  }
    case RH1: { angle = wb_position_sensor_get_value(pos_sensor[2][1]); break;  }
    case RH2: { angle = wb_position_sensor_get_value(pos_sensor[2][2]); break;  }

    case LH0: { angle = wb_position_sensor_get_value(pos_sensor[3][0]); break;  }
    case LH1: { angle = wb_position_sensor_get_value(pos_sensor[3][1]); break;  }
    case LH2: { angle = wb_position_sensor_get_value(pos_sensor[3][2]); break;  }

    default:break;
    }
    return angle;
}
//-----------------------------------------------------------sensor
/*
函数功能：检测足底是否接触地面
*/
bool is_foot_touching(legNameTypeDef legName)
{
    if (legName == LF)
        return wb_touch_sensor_get_value(touch_sensor[0]);
    if (legName == RF)
        return wb_touch_sensor_get_value(touch_sensor[1]);
    if (legName == RH)
        return wb_touch_sensor_get_value(touch_sensor[2]);
    if (legName == LH)
        return wb_touch_sensor_get_value(touch_sensor[3]);
    return true;
}

/*
函数功能：读取IMU数据
*/
eulerAngleTypeDef get_IMU_Angle()
{
    const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);

    eulerAngleTypeDef eulerAngle;
    eulerAngle.roll = data[1] * 180.0f / PI;
    eulerAngle.pitch = data[0] * 180.0f / PI;
    eulerAngle.yaw = data[2] * 180.0f / PI;

    return eulerAngle;
}

/*
机器人参数初始化，在while（1）之前调用
*/
void robot_init()
{
    //------------------------------------------------------------------------控制参数 
    robot.Tf = 25;       //02摆动相持续时间 25*10ms = 0.25s
    robot.Ts = 25;       //02支撑相持续时间
    robot.hd = 0.60;
    robot.zh = -0.35;
    robot.Kp = 4000;
    robot.Kdp = 200;
    robot.Kr = 4000;  // 10000;  //roll增益
    robot.Kdr = 200;  // 800;    //roll增益
    robot.Kyaw = 4000;  // 10000;  //yaw增益
    robot.Kdyaw = 200;  // 800;    //yaw增益
    //支撑相
    robot.Kh = 3000;   //8000
    robot.Kdh = 800;   //800
    robot.Kvx = 500;   //1000
    //摆动相
    robot.kx = 10000;   //100 
    robot.kdx = 1000;   //100
    robot.kz1 = 1000;   //10000
    robot.kz2 = 100;   //1000
    robot.kdz = 100;   //100
    robot.ok = 0;
    robot.dvx = 0.125;  //防止后座速度增益
    robot.vxd_forward = -0.5;  //默认前向期望速度
    robot.vxd = 0;  //默认初始x速度
    robot.vyd_right = 0.3;  //默认右向期望速度
    robot.clockwise = 0.8;  //默认逆时针
    robot.vyd = 0;  //默认初始y速度
    robot.vx = 0;  //机器人实际x速度
    robot.vy = 0;  //机器人实际y速度
    // robot.foot[4][3]   = 0               ;  //机器人各个足端位置
    // robot.dfoot[4][3]  = 0               ;  //机器人各个足端速度
    robot.counter = 0;  //机器人实际y速度
  //------------------------------------------------------------------------状态区

    robot.is_touching[LF] = false;      //足底传感器接触标志量
    robot.is_touching[RF] = false;      //足底传感器接触标志量
    robot.is_touching[RH] = false;      //足底传感器接触标志量
    robot.is_touching[LH] = false;      //足底传感器接触标志量
    robot.pitch = 0;      //pitch
    robot.dpitch = 0;      //pitch'
    robot.roll = 0;      //pitch
    robot.droll = 0;      //pitch'
    robot.yaw = 0;      //pitch
    robot.dyaw = 0;      //pitch'

    robot.l1 = 0.1;
    robot.l2 = 0.1;
}
/*
函数功能：输入关节角度，计算足端位置
单位：弧度制
*/
void forward_kinematic(legNameTypeDef legName) {
    if (legName == LF) {
        robot.foot[0][0] = -0.1 * sin(robot.real_theta[0][0]) * (cos(robot.real_theta[0][1] + robot.real_theta[0][2]) + cos(robot.real_theta[0][1]));
        robot.foot[0][1] = 0.1 * sin(robot.real_theta[0][1] + robot.real_theta[0][2]) + 0.1 * sin(robot.real_theta[0][1]);
        robot.foot[0][2] = -0.1 * cos(robot.real_theta[0][0]) * (cos(robot.real_theta[0][1] + robot.real_theta[0][2]) + cos(robot.real_theta[0][1]));
        printf("%.3lf %.3lf %.3lf\n", robot.foot[0][0], robot.foot[0][1], robot.foot[0][2]);
    }
    if (legName == RF) {
        robot.foot[1][0] = -0.1 * sin(robot.real_theta[1][0]) * (cos(robot.real_theta[1][1] + robot.real_theta[1][2]) + cos(robot.real_theta[1][1]));
        robot.foot[1][1] = 0.1 * sin(robot.real_theta[1][1] + robot.real_theta[1][2]) + 0.1 * sin(robot.real_theta[1][1]);
        robot.foot[1][2] = -0.1 * cos(robot.real_theta[1][0]) * (cos(robot.real_theta[1][1] + robot.real_theta[1][2]) + cos(robot.real_theta[1][1]));
    }
    if (legName == RH) {
        robot.foot[2][0] = -0.1 * sin(robot.real_theta[2][0]) * (cos(robot.real_theta[2][1] + robot.real_theta[2][2]) + cos(robot.real_theta[2][1]));
        robot.foot[2][1] = 0.1 * sin(robot.real_theta[2][1] + robot.real_theta[2][2]) + 0.1 * sin(robot.real_theta[2][1]);
        robot.foot[2][2] = -0.1 * cos(robot.real_theta[2][0]) * (cos(robot.real_theta[2][1] + robot.real_theta[2][2]) + cos(robot.real_theta[2][1]));
    }
    if (legName == LH) {
        robot.foot[3][0] = -0.1 * sin(robot.real_theta[3][0]) * (cos(robot.real_theta[3][1] + robot.real_theta[3][2]) + cos(robot.real_theta[3][1]));
        robot.foot[3][1] = 0.1 * sin(robot.real_theta[3][1] + robot.real_theta[3][2]) + 0.1 * sin(robot.real_theta[3][1]);
        robot.foot[3][2] = -0.1 * cos(robot.real_theta[3][0]) * (cos(robot.real_theta[3][1] + robot.real_theta[3][2]) + cos(robot.real_theta[3][1]));
    }
}
/*
函数功能：通过轨迹给定实时的足端位置
参数：指定腿、步长、抬腿高度
*/
void trajectory(legNameTypeDef legName, double steplength, double stephight) {
    double px, py, pz = 0;
    if (robot.counter < robot.Ts) {
        px = 0;
        py = 0 + steplength * (robot.counter / robot.Ts);
        pz = -0.141 + stephight * sin(PI * robot.counter / robot.Ts);
        printf("pz = %.3lf\n", pz);
        printf("py = %.3lf\n", py);
        robot.counter = robot.counter + 1;
        if (legName == LF) {
            robot.foot[0][0] = px;
            robot.foot[0][1] = py;
            robot.foot[0][2] = pz;
        }
        if (legName == RF) {
            robot.foot[1][0] = px;
            robot.foot[1][1] = py;
            robot.foot[1][2] = pz;
        }
        if (legName == RF) {
            robot.foot[2][0] = px;
            robot.foot[2][1] = py;
            robot.foot[2][2] = pz;
        }
        if (legName == LH) {
            robot.foot[3][0] = px;
            robot.foot[3][1] = py;
            robot.foot[3][2] = pz;
        }
    }
    else
        robot.counter = 0;   //重置counter
}
/*
函数功能：输入足端位置，计算关节角度
参数：各腿的名称 LF、RF、RH、LH
*/


void inverse_kinematic(legNameTypeDef legName) {
    double r = 0;  //中间变量
    if (legName == LF) {
        r = sqrt(robot.foot[0][0] * robot.foot[0][0] + robot.foot[0][1] * robot.foot[0][1] + robot.foot[0][2] * robot.foot[0][2]);  //sqrt(x^2+y^2+z^2)

        robot.real_theta[0][0] = atan(robot.foot[0][0] / robot.foot[0][2]);
        robot.real_theta[0][1] = asin(robot.foot[0][1] / r) + acos((r * r) / (2 * r * 0.1));
        robot.real_theta[0][2] = -(PI - acos((2 * robot.l1 * robot.l2 - r * r) / (2 * robot.l1 * robot.l2)));
        // printf("test =%.5lf\n ",r);
        printf("LF0 = %.3lf LF1 = %.3lf LF1 = %.3lf\n", rad_2_deg(robot.real_theta[0][0]), rad_2_deg(robot.real_theta[0][1]), rad_2_deg(robot.real_theta[0][2]));
    }
    if (legName == RF) {
        r = sqrt(robot.foot[1][0] * robot.foot[1][0] + robot.foot[1][1] * robot.foot[1][1] + robot.foot[1][2] * robot.foot[1][2]);  //sqrt(x^2+y^2+z^2)

        robot.real_theta[1][0] = atan(robot.foot[1][0] / robot.foot[1][2]);
        robot.real_theta[1][1] = asin(robot.foot[1][1] / r) + acos((r * r) / (2 * r * 0.1));
        robot.real_theta[1][2] = -(PI - acos((2 * robot.l1 * robot.l2 - r * r) / (2 * robot.l1 * robot.l2)));
    }
    if (legName == RH) {
        r = sqrt(robot.foot[2][0] * robot.foot[2][0] + robot.foot[2][1] * robot.foot[2][1] + robot.foot[2][2] * robot.foot[2][2]);  //sqrt(x^2+y^2+z^2)

        robot.real_theta[2][0] = atan(robot.foot[2][0] / robot.foot[2][2]);
        robot.real_theta[2][1] = asin(robot.foot[2][1] / r) + acos((r * r) / (2 * r * 0.1));
        robot.real_theta[2][2] = -(PI - acos((2 * robot.l1 * robot.l2 - r * r) / (2 * robot.l1 * robot.l2)));
    }
    if (legName == LH) {
        r = sqrt(robot.foot[3][0] * robot.foot[3][0] + robot.foot[3][1] * robot.foot[3][1] + robot.foot[3][2] * robot.foot[3][2]);  //sqrt(x^2+y^2+z^2)

        robot.real_theta[3][0] = atan(robot.foot[3][0] / robot.foot[3][2]);
        robot.real_theta[3][1] = asin(robot.foot[3][1] / r) + acos((r * r) / (2 * r * 0.1));
        robot.real_theta[3][2] = -(PI - acos((2 * robot.l1 * robot.l2 - r * r) / (2 * robot.l1 * robot.l2)));
    }
}
/*
函数功能：由电机角度计算真实的关节角度
参数：各腿的名称 LF、RF、RH、LH
关节单位：弧度制
*/
void motorangle_2_jointangle(legNameTypeDef legName) {
    if (legName == LF) {
        robot.real_theta[0][0] = robot.theta[0][0];
        robot.real_theta[0][1] = robot.theta[0][1] + PI / 4;
        robot.real_theta[0][2] = robot.theta[0][2] - PI / 2;
    }
    if (legName == RF) {
        robot.real_theta[1][0] = robot.theta[1][0];
        robot.real_theta[1][1] = robot.theta[1][1] + PI / 4;
        robot.real_theta[1][2] = robot.theta[1][2] - PI / 2;
    }
    if (legName == RH) {
        robot.real_theta[2][0] = robot.theta[2][0];
        robot.real_theta[2][1] = robot.theta[2][1] + PI / 4;
        robot.real_theta[2][2] = robot.theta[2][2] - PI / 2;
    }
    if (legName == LH) {
        robot.real_theta[3][0] = robot.theta[3][0];
        robot.real_theta[3][1] = robot.theta[3][1] + PI / 4;
        robot.real_theta[3][2] = robot.theta[3][2] - PI / 2;
    }
}
/*
函数功能：由关节角度计算真实的电机角度
参数：各腿的名称 LF、RF、RH、LH
关节单位：弧度制
使用方法： 轨迹规划——运动学逆解——关节角度——电机角度——电机驱动
*/
void jointangle_2_motorangle(legNameTypeDef legName) {
    if (legName == LF) {
        robot.theta[0][0] = robot.real_theta[0][0];
        robot.theta[0][1] = robot.real_theta[0][1] - PI / 4;
        robot.theta[0][2] = robot.real_theta[0][2] + PI / 2;
    }
    if (legName == RF) {
        robot.theta[1][0] = robot.real_theta[1][0];
        robot.theta[1][1] = robot.real_theta[1][1] - PI / 4;
        robot.theta[1][2] = robot.real_theta[1][2] + PI / 2;
    }
    if (legName == RH) {
        robot.theta[2][0] = robot.real_theta[2][0];
        robot.theta[2][1] = robot.real_theta[2][1] - PI / 4;
        robot.theta[2][2] = robot.real_theta[2][2] + PI / 2;
    }
    if (legName == LH) {
        robot.theta[3][0] = robot.real_theta[3][0];
        robot.theta[3][1] = robot.real_theta[3][1] - PI / 4;
        robot.theta[3][2] = robot.real_theta[3][2] + PI / 2;
    }
}

/*
函数功能：输入电机角度，并驱动各电机运动
参数：各腿的名称 LF、RF、RH、LH
关节单位：弧度制
*/
void joint_drive(legNameTypeDef legName) {
    if (legName == LF) {
        set_motor_position(LF0, -robot.theta[0][0]);
        set_motor_position(LF1, -robot.theta[0][1]);
        set_motor_position(LF2, -robot.theta[0][2]);
    }
    if (legName == RF) {
        set_motor_position(RF0, -robot.theta[1][0]);
        set_motor_position(RF1, -robot.theta[1][1]);
        set_motor_position(RF2, -robot.theta[1][2]);
    }
    if (legName == RH) {
        set_motor_position(RH0, -robot.theta[2][0]);
        set_motor_position(RH1, -robot.theta[2][1]);
        set_motor_position(RH2, -robot.theta[2][2]);
    }
    if (legName == LH) {
        set_motor_position(LH0, -robot.theta[3][0]);
        set_motor_position(LH1, -robot.theta[3][1]);
        set_motor_position(LH2, -robot.theta[3][2]);
        printf("robot.theta[3][1] = %.3lf\n", robot.theta[3][1]);
        printf("robot.theta[3][2] = %.3lf\n", robot.theta[3][2]);
    }
}

/*
函数功能：封装函数，由末端位置计算关节角，并驱动电机
参数：各腿的名称 LF、RF、RH、LH
*/
void IK_2_jointdrive(legNameTypeDef legName) {
    if (legName == LF) {
        inverse_kinematic(LF);
        jointangle_2_motorangle(LF);
        joint_drive(LF);
    }
    if (legName == RF) {
        inverse_kinematic(RF);
        jointangle_2_motorangle(RF);
        joint_drive(RF);
    }
    if (legName == RH) {
        inverse_kinematic(RH);
        jointangle_2_motorangle(RH);
        joint_drive(RH);
    }
    if (legName == LH) {
        inverse_kinematic(LH);
        jointangle_2_motorangle(LH);
        joint_drive(LH);
    }
}

/*
主函数
*/


int main(int argc, char** argv) {
    /* necessary to initialize webots stuff */
    double t = 0;
    double test = 0;
    wb_robot_init();
    webots_device_init();
    robot_init();

    while (wb_robot_step(TIME_STEP) != -1) {
        trajectory(LH, -0.05, 0.03);
        // robot.foot[3][0] = 0.0;
        // robot.foot[3][1] = -0.05;
        // robot.foot[3][2] = -0.141;
        IK_2_jointdrive(LH);

        // forward_kinematic(LF);
        eulerAngleTypeDef eulerAngle = get_IMU_Angle(); //读取IMU角度
        // test = get_motor_angle(LF1);

        // printf("%.3lf %.3lf\n",eulerAngle.pitch,eulerAngle.roll);
         // printf("robot.l1 = %.3lf\n",robot.l1);

        t += (double)TIME_STEP / 10;  // 
       //update_IMU();
           // printf("t = %lf\n",t);

         /*


          * Read the sensors :
          * Enter here functions to read sensor data, like:
          *  double val = wb_distance_sensor_get_value(my_sensor);
          */

          /* Process sensor data here */
    };

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}


