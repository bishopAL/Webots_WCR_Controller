 /**********************************************************
**             
**---------------------------------------------------------
**  Description: 此文件为 双足机器人 仿真环境或硬件接口 头文件
**  Version    : 
**  Notes      : 
**  Author     : 
**********************************************************/
#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

//-----------------------------------------------------------macro
#define PI           (3.141892654f)
#define TIME_STEP    20                      
#define rad_2_deg(X) ( X / PI * 180.0 )
#define deg_2_rad(X) ( X / 180.0 * PI )
//-----------------------------------------------------------typedef

/*
电机名称集合
*/
typedef enum
{
  LF0 = 0x00,
  LF1 = 0x01,
  LF2 = 0x02,
  
  RF0 = 0x03,
  RF1 = 0x04,
  RF2 = 0x05,
  
  RH0 = 0x06,
  RH1 = 0x07,
  RH2 = 0x08,
  
  LH0 = 0x09,
  LH1 = 0x0A,
  LH2 = 0x0B,
}motorNameTypeDef;

/*
腿名称集合
*/
typedef enum
{
  LF = 0x00,   //左
  RF = 0x01,   //右
  RH = 0x02,
  LH = 0x03,
  
}legNameTypeDef;
/*
腿关节名称
*/
typedef enum
{
  hip = 0x00,     //hip  顺负逆正
  ankle = 0x01,   //ankle  顺负逆正
  knee = 0x02,    //knee  顺负逆正
}angleNameTypeDef;
/*
trot步态腿组名称集合
*/
typedef enum
{
  group1 = 0x00,   //前左 右后
  group2 = 0x01,   //前右 后左
}trotGroupNameTypeDef;
typedef enum
{
  F = 0x00,   //前
  B = 0x01,   //后
}FBNameTypeDef;
typedef enum
{
  L = 0x00,   //左
  R = 0x01,   //右  
}LRNameTypeDef;
typedef enum
{
  x = 0x00,   //x
  y = 0x01,   //y 
  z = 0x02,   //z 
}CoordinateNameTypeDef;

/*
摄像头二维向量
*/
typedef struct
{
  int width;
  int height;
}CameraTypeDef;
/* 
1，陀螺仪数据定义,为了方便调试采用角度制，注意，采用角度制
2，webots IMU模块采用RPY角度制，定系旋转，矩阵左乘，即：
       Rot=RotY(yaw)*RotZ(pitch)*RotX(roll);
3，eulerAngleTypeDef结构体描述了数学模型中的RPY，定系旋转，矩阵左乘，即：
       Rot=RotZ(yaw)*RotY(pitch)*RotX(roll);
4,由于webots默认坐标系和数学模型世界坐标系定义不同，因此二者RPY定义不同，对应关系如下：

==============================
*   数学模型   *   webots    *
------------------------------
*   X(roll)   *   Z(pitch)  *
*   Y(pitch)  *   X(roll)   *
*   Z(yaw)    *   Y(yaw)    *
==============================
5,暂不考虑上述两个坐标系因相乘顺序而引起的误差。
*/
typedef struct
{
  double roll;       //横滚，x轴
  double pitch;      //俯仰，z轴
  double yaw;        //偏航，y轴
}eulerAngleTypeDef;

//-----------------------------------------------------------extern
extern void webots_device_init();
extern void set_motor_torque(motorNameTypeDef motorName, double torque);
extern void set_motor_position(motorNameTypeDef motorName, double angle);
extern double get_motor_angle(motorNameTypeDef motorName);
extern int get_keyboard();
extern eulerAngleTypeDef get_IMU_Angle();
//extern void get_camera();
#endif

