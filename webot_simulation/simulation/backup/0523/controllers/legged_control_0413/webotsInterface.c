 /**********************************************************
**             Email:@qq.com   QQ:1753968393
**---------------------------------------------------------
**  Description: 此文件为 双足机器人 仿真环境或硬件接口 头文件
**  Version    : 
**  Notes      : 
**  Author     : 刘智飞
**********************************************************/
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>
#include <webots/camera.h>

#include "webotsInterface.h"
//定义webots电机设备
WbDeviceTag motor[4][3];
WbDeviceTag pos_sensor[4][3];
WbDeviceTag touch_sensor[4];
WbDeviceTag IMU;

//-----------------------------------------------------------函数
void webots_device_init();
void set_motor_torque(motorNameTypeDef motorName, double torque);
void set_motor_position(motorNameTypeDef motorName, double angle);
double get_motor_angle(motorNameTypeDef motorName);
bool is_foot_touching(legNameTypeDef legName);
int get_keyboard();
eulerAngleTypeDef get_IMU_Angle();
//-----------------------------------------------------------device

/*电机*/
const char *MOTION_NAMES[4][3]= { 
  {"LF0", "LF1", "LF2"},
   {"RF0","RF1", "RF2"},
   {"RH0","RH1", "RH2"},
   {"LH0", "LH1","LH2"}
   };
/*电机编码器*/    
const char *POS_MOTION_NAMES[4][3]= { 
  {"LF0 sensor", "LF1 sensor", "LF2 sensor"},
  {"RF0 sensor", "RF1 sensor", "RF2 sensor"},
  {"RH0 sensor", "RH1 sensor", "RH2 sensor"},
  {"LH0 sensor", "LH1 sensor", "LH2 sensor"}
  };
/*足底触碰开关*/
const char *TOUCH_SENSOR_NAMES[4]= {
  "LF sensor", 
  "RF sensor",  
  "RH sensor",
  "LH sensor"};

/*
函数功能：初始化设备，电机+角度+足端传感器+IMU
*/
void webots_device_init()
{
  for(legNameTypeDef i = 0; i < 4; i++)
  {
      for(angleNameTypeDef j = 0; j < 3; j++)
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
  
  wb_keyboard_enable(TIME_STEP);  //采样周期
}
//-----------------------------------------------------------motor
/*
函数功能：设置电机转角
*/
void set_motor_position(motorNameTypeDef motorName, double angle)
{
  switch (motorName){
  case LF0:  {  wb_motor_set_position(motor[0][0], angle);break;  }
  case LF1:  {  wb_motor_set_position(motor[0][1], angle);break;  }
  case LF2:  {  wb_motor_set_position(motor[0][2], angle);break;  }
  
  case RF0:  {  wb_motor_set_position(motor[1][0], angle);break;  }
  case RF1:  {  wb_motor_set_position(motor[1][1], angle);break;  }
  case RF2:  {  wb_motor_set_position(motor[1][2], angle);break;  }

  case RB0:  {  wb_motor_set_position(motor[2][0], angle);break;  }
  case RB1:  {  wb_motor_set_position(motor[2][1], angle);break;  }
  case RB2:  {  wb_motor_set_position(motor[2][2], angle);break;  }
  
  case LB0:  {  wb_motor_set_position(motor[3][0], angle);break;  }
  case LB1:  {  wb_motor_set_position(motor[3][1], angle);break;  }
  case LB2:  {  wb_motor_set_position(motor[3][2], angle);break;  }
  
  default:break;
  }
}
//-----------------------------------------------------------sensor
/*
函数功能：获取电机角度,角度制
*/
double get_motor_angle(motorNameTypeDef motorName)
{
  double angle = 0;
  switch (motorName){
  case LF0:  { angle = wb_position_sensor_get_value(pos_sensor[0][0]);break;  }
  case LF1:  { angle = wb_position_sensor_get_value(pos_sensor[0][1]);break;  }
  case LF2:  { angle = wb_position_sensor_get_value(pos_sensor[0][2]);break;  }
  
  case RF0:  { angle = wb_position_sensor_get_value(pos_sensor[1][0]);break;  }
  case RF1:  { angle = wb_position_sensor_get_value(pos_sensor[1][1]);break;  }
  case RF2:  { angle = wb_position_sensor_get_value(pos_sensor[1][2]);break;  }

  case RB0:  { angle = wb_position_sensor_get_value(pos_sensor[2][0]);break;  }
  case RB1:  { angle = wb_position_sensor_get_value(pos_sensor[2][1]);break;  }
  case RB2:  { angle = wb_position_sensor_get_value(pos_sensor[2][2]);break;  }
  
  case LB0:  { angle = wb_position_sensor_get_value(pos_sensor[3][0]);break;  }
  case LB1:  { angle = wb_position_sensor_get_value(pos_sensor[3][1]);break;  }
  case LB2:  { angle = wb_position_sensor_get_value(pos_sensor[3][2]);break;  }
  
  default:break;
  }
  return angle*180.0f/PI;
}
/*
函数功能：检测足底是否接触地面
*/
bool is_foot_touching(legNameTypeDef legName)
{
  if(legName == LF)
    return wb_touch_sensor_get_value(touch_sensor[0]);
  if(legName == RF)
    return wb_touch_sensor_get_value(touch_sensor[1]);
  if(legName == RB)
    return wb_touch_sensor_get_value(touch_sensor[2]);
  if(legName == LB)
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
  eulerAngle.roll  = data[1]*180.0f/PI;
  eulerAngle.pitch = data[0]*180.0f/PI;
  eulerAngle.yaw   = data[2]*180.0f/PI;
  
  return eulerAngle;
}

//-----------------------------------------------------------keyboard
/*
函数功能：读取键盘键值
*/
int get_keyboard()
{
  return wb_keyboard_get_key();
}