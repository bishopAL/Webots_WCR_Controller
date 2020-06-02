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
#include <webots/accelerometer.h>
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
WbDeviceTag accelerometer;


//-----------------------------------------------------------函数声明
void webots_device_init();

void set_motor_position(motorNameTypeDef motorName, double angle);
void joint_drive(legNameTypeDef legName);
void motorangle_2_jointangle(legNameTypeDef legName);
void jointangle_2_motorangle(legNameTypeDef legName);
void IK_2_jointdrive(legNameTypeDef legName);
void starting_point();
void trajectory1(legNameTypeDef legName,double steplength,double stephight);
void trajectory2(legNameTypeDef legName,double steplength,double stephight);
void trajectory();
double get_motor_angle(motorNameTypeDef motorName);
bool is_foot_touching(legNameTypeDef legName);
void IMU_feedback();
void state_update();
void footforce_update();
void get_IMU_Angle();
void robot_state_update();
void get_accelerometer();
robotTypeDef robot;
eulerAngleTypeDef eulerAngle;
//-----------------------------------------------------------device

int i =0;
double test= 0;
double start_counter =0;
int IMU_counter =0;  
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
  
}
//-----------------------------------------------------------motor
/*
函数功能：设置电机转角
*/
void set_motor_position(motorNameTypeDef motorName, double angle)
{
  wb_motor_set_position(motor[motorName/3][motorName%3], angle);// 电机/3 确定腿  余数确定电机  
}

//-----------------------------------------------------------sensor
/*
函数功能：获取电机角度,弧度制   未测试
*/
double get_motor_angle(motorNameTypeDef motorName)
{
  double angle = 0;
  angle = wb_position_sensor_get_value(pos_sensor[motorName/3][motorName%3]);
  return angle;
}
//-----------------------------------------------------------sensor
/*
函数功能：检测足底是否接触地面
*/
bool is_foot_touching(legNameTypeDef legName)
{
    return wb_touch_sensor_get_value(touch_sensor[legName]);
}

/*
函数功能：读取IMU数据
*/
void get_IMU_Angle()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  robot.IMU[0]  = data[0]*180.0f/PI;    //roll
  robot.IMU[1]  = data[1]*180.0f/PI;    //pitch
  robot.IMU[2]  = data[2]*180.0f/PI;    //yaw
}

/*
函数功能：读取加速度计数据

*/
void get_accelerometer()
{
  const double* data1 =  wb_accelerometer_get_values(accelerometer);
  robot.accelerometer[0]  = data1[0];      //向前 +
  robot.accelerometer[1]  = data1[1];      //向下 +
  robot.accelerometer[2]  = data1[2];      //向左 +
  printf(" %lf ,%lf，%lf \n",robot.accelerometer[0],robot.accelerometer[1],robot.accelerometer[2]);   //roll

}

/*
机器人参数初始化，在while（1）之前调用
*/
void robot_init()
{
  //------------------------------------------------------------------------控制参数 
   robot.Ts      =   16;       //02摆动相持续时间 20*10ms = 0.25s
   robot.hd      =   0.60;
   robot.zh      =  -0.35;
   robot.kpitch  =   4000;
   robot.kdpitch =    200;
   robot.kroll   =   0.0001;  // 10000;  //roll增益
   robot.kdroll  =    200;  // 800;    //roll增益
   robot.kyaw    =   0.1;  // 10000;  //yaw增益
   robot.kdyaw   =    200;  // 800;    //yaw增益
   //支撑相
   robot.Kh     =   3000;   //8000
   robot.Kdh    =    800;   //800
   robot.Kvx    =    500;   //1000
   //摆动相
   robot.kx     =  10000;   //100 
   robot.kdx    =   1000;   //100
   robot.kz1    =   1000;   //10000
   robot.kz2    =    100;   //1000
   robot.kdz    =    100;   //100
   robot.ok = 0;
   robot.dvx          =  0.125          ;  //防止后座速度增益
   robot.vxd_forward  = -0.5            ;  //默认前向期望速度
   robot.vxd          = 0               ;  //默认初始x速度
   robot.vyd_right    = 0.3             ;  //默认右向期望速度
   robot.clockwise    = 0.8             ;  //默认逆时针
   robot.vyd          = 0               ;  //默认初始y速度
   robot.vx           = 0               ;  //机器人实际x速度
   robot.vy           = 0               ;  //机器人实际y速度
   robot.stepLength         = 0.1     ;
   robot.stepHight          = 0.03    ;
   for(int i =0;i<=3;i++){
    robot.stance_endpos[i][0] = 0;
    robot.stance_endpos[i][1] = 0.5*robot.stepLength; 
    robot.stance_endpos[i][2] = -0.141;
    robot.swing_endpos[i][0] = 0;
    robot.swing_endpos[i][1] = -0.5*robot.stepLength; 
    robot.swing_endpos[i][2] = -0.141;
   }
    //robot.foot[4][3]   = 0               ;  //机器人各个足端位置
   // robot.dfoot[4][3]  = 0               ;  //机器人各个足端速度
    // robot.counter[legName]           = 0            ;  //机器人实际y速度
  //------------------------------------------------------------------------状态区

   robot.is_touching[LF]   = false;      //足底传感器接触标志量
   robot.is_touching[RF]   = false;      //足底传感器接触标志量
   robot.is_touching[RH]   = false;      //足底传感器接触标志量
   robot.is_touching[LH]   = false;      //足底传感器接触标志量
   robot.l1                =0.1   ;  
   robot.l2                =0.1   ;
   }
 /*
 函数功能：输入关节角度，计算足端位置   未测试
 单位：弧度制
 */
void forward_kinematic(legNameTypeDef legName){
  robot.foot[legName][0] = -0.1*sin(robot.real_theta[legName][0])*(cos(robot.real_theta[legName][1] +robot.real_theta[legName][2]) + cos(robot.real_theta[legName][1]));
  robot.foot[legName][1] =  0.1*sin(robot.real_theta[legName][1] +robot.real_theta[legName][2]) + 0.1*sin(robot.real_theta[legName][1]);
  robot.foot[legName][2] = -0.1*cos(robot.real_theta[legName][0])*(cos(robot.real_theta[legName][1] +robot.real_theta[legName][2]) + cos(robot.real_theta[legName][1]));
  // printf("%.3lf %.3lf %.3lf\n",robot.foot[legName][0],robot.foot[legName][1],robot.foot[legName][2]);  
}
/*
函数功能：通过轨迹给定实时的足端位置  —— 
最后加入驱动程序，实现机器人的运动控制
02 sw
02 st
参数：指定腿、步长、抬腿高度
*/
void trajectory1(legNameTypeDef legName,double steplength,double stephight){
    // double p[3] = {0};      // x,y,z
        if(robot.counter[legName]<=robot.Ts){ // Ts =20 + 5  
             if(robot.counter[legName]<=(robot.Ts-2)){
             robot.foot[legName][0] =  robot.stance_endpos[legName][0] - robot.stance_endpos[legName][0]*(robot.counter[legName]/(robot.Ts-2)) ;
             robot.foot[legName][1] =  robot.stance_endpos[legName][1] - robot.stepLength*(robot.counter[legName]/(robot.Ts-2));
             robot.foot[legName][2] =  robot.stance_endpos[legName][2] + robot.stepHight*sin(PI*robot.counter[legName]/(robot.Ts-2));  
              robot.counter[legName] = robot.counter[legName] + 1;  
             // printf("%.3lf ,%.3lf ,%.3lf\n",p[1],p[2],robot.counter[legName]);
             }
             else {   //记录摆动相结束位置 检测是否触地,触地则进入支撑相
                 if  (robot.state == 1){     //
                 robot.swing_endpos[legName][0]= robot.foot[legName][0];
                 robot.swing_endpos[legName][1]= robot.foot[legName][1];
                 robot.swing_endpos[legName][2]= robot.foot[legName][2];
                 // printf("%.3lf ,%.3lf,%.3lf ,%.3lf\n",robot.swing_endpos[legName][0],robot.swing_endpos[legName][1],robot.swing_endpos[legName][2],robot.counter[legName]);
                  robot.counter[legName]++;
                 }
                  if (robot.state == 2) {
                    robot.swing_endpos[legName][0]= robot.foot[legName][0];
                    robot.swing_endpos[legName][1]= robot.foot[legName][1];
                    robot.swing_endpos[legName][2]= robot.foot[legName][2];
                    robot.counter[legName] = robot.Ts+1; 
                  }     
             }
         }
    // } 
      if(robot.counter[legName]>robot.Ts && robot.counter[legName]<=2*robot.Ts){   //25<x<50
             if(robot.counter[legName]>=(robot.Ts) && robot.counter[legName]<=(2*robot.Ts-2)){  //25<x<45
             robot.foot[legName][0] = robot.swing_endpos[legName][0] - robot.IMU_position2[0]*((robot.counter[legName]-robot.Ts)/(robot.Ts-2));
              // robot.foot[legName][0] = robot.swing_endpos[legName][0];
             robot.foot[legName][1] = robot.stepLength * ((robot.counter[legName]-robot.Ts)/(robot.Ts-2))- 0.5 * robot.stepLength;
             robot.foot[legName][2]= robot.swing_endpos[legName][2] +0; 
              robot.counter[legName] = robot.counter[legName] + 1; 
              }
             else {   //记录摆动相结束位置 检测是否触地,触地则进入支撑相
                     if  (robot.state == 1){       //not use now
                     robot.stance_endpos[legName][0]= robot.foot[legName][0];
                     robot.stance_endpos[legName][1]= robot.foot[legName][1];
                     robot.stance_endpos[legName][2]= robot.foot[legName][2];
                     // printf("%.3lf ,%.3lf,%.3lf ,%.3lf\n",robot.swing_endpos[legName][0],robot.swing_endpos[legName][1],robot.swing_endpos[legName][2],robot.counter[legName]);
                      robot.counter[legName]++;
                         if (robot.counter[legName] == 2*robot.Ts+1)  robot.counter[legName] =0;
                     }
                     if (robot.state == 2)  {
                     robot.stance_endpos[legName][0]= robot.foot[legName][0];
                     robot.stance_endpos[legName][1]= robot.foot[legName][1];
                     robot.stance_endpos[legName][2]= robot.foot[legName][2];                     
                     robot.counter[legName] = 0; 
                     }               
                  }
       } 
      
       IK_2_jointdrive(legName);  
  }
// 
/*
函数功能：通过轨迹给定实时的足端位置 —— 24
参数：指定腿、步长、抬腿高度

*/
void trajectory2(legNameTypeDef legName,double steplength,double stephight){
    // double p[3] = {0};      // x,y,z
        if(robot.counter[legName]<=robot.Ts){ // Ts =20 + 5  13 st
             if(robot.counter[legName]<=(robot.Ts-2)){              
              robot.foot[legName][0] = robot.swing_endpos[legName][0] - robot.IMU_position2[0]*((robot.counter[legName]-robot.Ts)/(robot.Ts-2));;
              robot.foot[legName][1] = robot.swing_endpos[legName][1] + robot.stepLength*(robot.counter[legName]/(robot.Ts-2)); // 支撑相
              robot.foot[legName][2] = robot.swing_endpos[legName][2];                 
              robot.counter[legName] = robot.counter[legName] + 1;  
             // printf("%.3lf ,%.3lf ,%.3lf\n",p[1],p[2],robot.counter[legName]);
             }
             else {   //记录摆动相结束位置 检测是否触地,触地则进入支撑相
                 if  (robot.state == 1){  //
                 robot.stance_endpos[legName][0]= robot.foot[legName][0];
                 robot.stance_endpos[legName][1]= robot.foot[legName][1];
                 robot.stance_endpos[legName][2]= robot.foot[legName][2];
               // printf("%.3lf ,%.3lf ,%.3lf\n",robot.swing_endpos[legName][0],p[2],robot.counter[legName]);
                  robot.counter[legName]++;
                 }
                 if (robot.state == 2){
                    robot.stance_endpos[legName][0]= robot.foot[legName][0];
                    robot.stance_endpos[legName][1]= robot.foot[legName][1];
                    robot.stance_endpos[legName][2]= robot.foot[legName][2];                
                    robot.counter[legName] = robot.Ts+1;     
                 }  
             }
         }
      if(robot.counter[legName]>robot.Ts && robot.counter[legName]<=2*robot.Ts){   //20<x<40
             if(robot.counter[legName]>=(robot.Ts) && robot.counter[legName]<=(2*robot.Ts-2)){  //20<x<36
             robot.foot[legName][0] =  robot.stance_endpos[legName][0] - robot.stance_endpos[legName][0]*((robot.counter[legName]-robot.Ts)/(robot.Ts-2));
             robot.foot[legName][1] =  robot.stance_endpos[legName][1] -robot.stepLength*((robot.counter[legName]-robot.Ts)/(robot.Ts-2));
             robot.foot[legName][2] =  robot.stance_endpos[legName][2] + robot.stepHight*sin(PI*(robot.counter[legName]-robot.Ts)/(robot.Ts-2));  
              robot.counter[legName] = robot.counter[legName] + 1; 
              }
             else {   //记录摆动相结束位置 检测是否触地,触地则进入支撑相
                     if  (robot.state == 1){  //
                     robot.swing_endpos[legName][0]= robot.foot[legName][0];
                     robot.swing_endpos[legName][1]= robot.foot[legName][1];
                     robot.swing_endpos[legName][2]= robot.foot[legName][2];
                   // printf("%.3lf ,%.3lf ,%.3lf\n",robot.swing_endpos[legName][0],robot.swing_endpos[legName][0],robot.counter[legName]);
                      robot.counter[legName]++;
                         if (robot.counter[legName] == 2*robot.Ts+1)  robot.counter[legName] =0;
                     }
                     if (robot.state == 2)   robot.counter[legName] = 0;               
                  }
                }        
       IK_2_jointdrive(legName);  
  }

/*
函数功能：输入起始位置，默认为步长的一半。
参数：各腿的名称 LF、RF、RH、LH  
*/
void starting_point(){ 
  if(start_counter<=10){
       robot.foot[LF][0] = 0;
       robot.foot[RH][0] = 0;
       robot.foot[LH][0] = 0;;
       robot.foot[RF][0] = 0;
       robot.foot[LF][1] = 0.5*robot.stepLength*(start_counter/10);
       robot.foot[RH][1] = 0.5*robot.stepLength*(start_counter/10);
       robot.foot[LH][1] = -0.5*robot.stepLength*(start_counter/10);
       robot.foot[RF][1] = -0.5*robot.stepLength*(start_counter/10);
       robot.foot[LF][2] = -0.141;
       robot.foot[RH][2] = -0.141;
       robot.foot[LH][2] = -0.141;
       robot.foot[RF][2] = -0.141;
       IK_2_jointdrive(LF);
       IK_2_jointdrive(RH);
       IK_2_jointdrive(LH);
       IK_2_jointdrive(RF);
       start_counter++;
   }
   // if (start_counter==11) robot.state = 1;
}
/*
函数功能：输入足端位置，计算关节角度
参数：各腿的名称 LF、RF、RH、LH  
*/
void inverse_kinematic(legNameTypeDef legName){
  double r = 0;  //中间变量
   // printf("test =%.5lf\n ",r);
  // printf("LF0 = %.3lf LF1 = %.3lf LF1 = %.3lf\n", rad_2_deg(robot.real_theta[0][0]),rad_2_deg(robot.real_theta[0][1]),rad_2_deg(robot.real_theta[0][2]));
   r = sqrt(robot.foot[legName][0]*robot.foot[legName][0]+robot.foot[legName][1]*robot.foot[legName][1]+robot.foot[legName][2]*robot.foot[legName][2]);  //sqrt(x^2+y^2+z^2) 
   robot.real_theta[legName][0] = atan(robot.foot[legName][0]/robot.foot[legName][2]);
   robot.real_theta[legName][1] = asin(robot.foot[legName][1]/r)+acos((r*r)/(2*r*0.1));
   robot.real_theta[legName][2] = -(PI-acos((2*robot.l1*robot.l2-r*r)/(2*robot.l1*robot.l2)));
}
/*
函数功能：由电机角度计算真实的关节角度
参数：各腿的名称 LF、RF、RH、LH
关节单位：弧度制  未测试
*/
void motorangle_2_jointangle(legNameTypeDef legName){
  robot.real_theta[legName][0]=robot.theta[legName][0];
  robot.real_theta[legName][1]=robot.theta[legName][1]+PI/4;
  robot.real_theta[legName][2]=robot.theta[legName][2]-PI/2;    
}
/*
函数功能：由关节角度计算真实的电机角度
参数：各腿的名称 LF、RF、RH、LH
关节单位：弧度制
使用方法： 轨迹规划——运动学逆解——关节角度——电机角度——电机驱动
*/
void jointangle_2_motorangle(legNameTypeDef legName){

  robot.theta[legName][0]=robot.real_theta[legName][0];
  robot.theta[legName][1]=robot.real_theta[legName][1]-PI/4;
  robot.theta[legName][2]=robot.real_theta[legName][2]+PI/2; 
   
}

/*
函数功能：输入电机角度，并驱动各电机运动
参数：各腿的名称 LF、RF、RH、LH
关节单位：弧度制
*/
void joint_drive(legNameTypeDef legName){

  // printf("robot.theta[legName][1] = %.3lf\n",robot.theta[legName][1]);
   // printf("robot.theta[legName][2] = %.3lf\n",robot.theta[legName][2]);
  set_motor_position(legName*3+0,-robot.theta[legName][0]);
  set_motor_position(legName*3+1,-robot.theta[legName][1]);
  set_motor_position(legName*3+2,-robot.theta[legName][2]);
  
}
/*
函数功能：封装函数，由末端位置计算关节角，并驱动电机
参数：各腿的名称 LF、RF、RH、LH
*/
void IK_2_jointdrive(legNameTypeDef legName){

  inverse_kinematic(legName);
  jointangle_2_motorangle(legName);
  joint_drive(legName);
 
}
/*
足端力更新   
Under construction 🚧正在施工🚧
*/
void footforce_update(){
  const double *f0 = wb_touch_sensor_get_values(touch_sensor[0]);
  const double *f1 = wb_touch_sensor_get_values(touch_sensor[1]);
  const double *f2 = wb_touch_sensor_get_values(touch_sensor[2]);
  const double *f3 = wb_touch_sensor_get_values(touch_sensor[3]);
    // printf("force vector: %.2f %.2f %8.2f \n", f0[0],f2[0],robot.counter[0]);

  if (robot.t >=0.3*robot.Ts && robot.t <=1.7*robot.Ts){ //检测02腿的触地状态
      if(f0[0]>10)  robot.is_touching[0] = 1;
      if(f2[0]>10)  robot.is_touching[2] = 1;      
  }
  // else {
  if (robot.t<=0.7*robot.Ts || robot.t >=1.3*robot.Ts){ //检测13腿的触地状态
      if(f1[0]>10)  robot.is_touching[1] = 1;
      if(f3[0]>10)  robot.is_touching[3] = 1;    
  }
  if(robot.t == robot.Ts){    //13支撑相结束时落地状态清零
  robot.is_touching[1] = 0;
  robot.is_touching[3] = 0;
  }
  if(robot.t == 2*robot.Ts){    //02支撑相结束时落地状态清零
  robot.is_touching[0] = 0;
  robot.is_touching[2] = 0;
  }
   robot.t++;
   if(robot.t == robot.Ts+1 ) robot.t =0;
   // printf("t = %lf\n",robot.t);
}


/*
状态机函数,更新机器人的运动状态
其中,初始状态为 0    -  起步状态为 1    -    02落地进入状态2    -   13落地进入状态1    
*/
void robot_state_update(){
  if(robot.is_touching[0] == 1 && robot.is_touching[2] == 1){    //13支撑相结束时落地状态清零
  robot.state = 2;
  }
  if(robot.is_touching[1] == 1 && robot.is_touching[3] == 1){    //13支撑相结束时落地状态清零
  robot.state = 1;
  }
}
/*
状态更新函数
Under construction 🚧正在施工🚧
*/
void state_update(){
   footforce_update();
   robot_state_update();
   get_IMU_Angle();    //读取IMU角度
   IMU_feedback();    //根据IMU反馈的偏差角计算阻断的位移偏差
   // for(i = 0;i<=2;i++){
   // robot.dIMU[i] = (robot.IMU[i]-robot.IMU1[i])/ ((double)TIME_STEP / 1000);                ; 
   // robot.IMU1[i]   = robot.IMU[i];
   // }
   // i = 0;
}
/*
姿态反馈函数
通过姿态角计算足端轨迹的偏移量，
最终需要附加至足端轨迹上。
Under construction 🚧正在施工🚧`
*/
void IMU_feedback(){
   // double pos[3] = {0};
    if(IMU_counter <= robot.Ts){
      robot.IMU_array1[IMU_counter][0] = robot.IMU[0];
      robot.IMU_array1[IMU_counter][1] = robot.IMU[1];
      robot.IMU_array1[IMU_counter][2] = robot.IMU[2];
        if(IMU_counter == robot.Ts){
          for(int i=0;i<=robot.Ts;i++){  //前半周期的IMU_err平均值
            robot.IMU_err1[0] = robot.IMU_err1[0] +  robot.IMU_array1[i][0]/robot.Ts;                 
          }
          robot.IMU_position1[0] = robot.kroll* robot.IMU_err1[0];  //根据IMUerr计算前半周期位置偏差量
        }  
          printf("array1 = %lf,IMU_position= %lf,IMU = %lf,%d\n",robot.IMU_array1[IMU_counter][0],robot.IMU_position1[0],robot.IMU[0],IMU_counter); 
    }
    if(IMU_counter > robot.Ts && IMU_counter <= 2*robot.Ts){
        robot.IMU_array2[IMU_counter-robot.Ts][0] = robot.IMU[0];
        robot.IMU_array2[IMU_counter-robot.Ts][1] = robot.IMU[1];
        robot.IMU_array2[IMU_counter-robot.Ts][2] = robot.IMU[2];
         if(IMU_counter == 2*robot.Ts){
          for(int j=0;j<=robot.Ts;j++){  //前半周期的IMU_err平均值
            robot.IMU_err2[0] = robot.IMU_err2[0] +  robot.IMU_array2[j][0]/robot.Ts; 
          }
          robot.IMU_position2[0] = robot.kroll* robot.IMU_err2[0];  //根据IMUerr计算前半周期位置偏差量
        }  
        printf("array2 = %.3f,IMU_position= %.3f,IMU = %lf,%d\n",robot.IMU_array2[IMU_counter-robot.Ts][0],robot.IMU_position2[0],robot.IMU[0],IMU_counter);

      }
     if (IMU_counter == 2*robot.Ts+1) {
        IMU_counter = 0;
      }
      // printf("array1 = %lf,array2= %lf,IMU = %lf,%d\n",robot.IMU_array1[IMU_counter][0],robot.IMU_array2[IMU_counter-robot.Ts][0],robot.IMU[0],IMU_counter);
      printf(" %lf,%lf,%d\n",robot.IMU_err1[0],robot.IMU_err2[0],IMU_counter);
      IMU_counter++;
}
/*
设置初始位置，通过规划两条两条对角腿的足端轨迹，实现机器人的控制。
*/
void trajectory(){     
     trajectory2(LH,robot.stepLength,robot.stepHight);
     trajectory2(RF,robot.stepLength,robot.stepHight);
     trajectory1(LF,robot.stepLength,robot.stepHight);   
     trajectory1(RH,robot.stepLength,robot.stepHight);
}
/*
主函数
*/
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
double t = 0;  
  wb_robot_init();
  webots_device_init();
  robot_init();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    starting_point();
    state_update();
    // printf("state = %lf\n",robot.state);
   // printf("robot_counter= %lf %lf %lf %lf \n", robot.counter[0],robot.counter[1],robot.counter[2],robot.counter[3]);
    // footforce_update();
  // set_motor_position(3*3+1,-PI/4);
  // set_motor_position(3*3+0,-PI/8);
   // robot.foot[1][0] = 0;
   // robot.foot[1][1] = 0.1;
   // robot.foot[1][2] = -0.141;
   // IK_2_jointdrive(1);
    trajectory();
    // printf("roll = %lf   pitch = %lf  yaw = %lf  \n",robot.IMU[0],robot.IMU[1],robot.IMU[2]);   //roll 
    // trajectory1(RH, 1,robot.stepHight);
    // trajectory1(LF,0.2,robot.stepHight);
    
    // trajectory2(RF,0.2,robot.stepHight);
    // trajectory2(LH,0.2,robot.stepHight);
    // motorangle_2_jointangle(LF);
    // printf(" %.2f %.2f %.2f\n",rad_2_deg(robot.real_theta[0][0]),rad_2_deg(robot.real_theta[0][1]),rad_2_deg(robot.real_theta[0][2]));
   // double f[12]={0};
   // double** f = {0};

   // f[0] = (double*)wb_touch_sensor_get_values(touch_sensor[0]);
     // printf("force vector_LF: %8.2f %8.2f %8.2f\n", f[0][0],f[0][1], f[0][2]);
      // const double *f1 = wb_touch_sensor_get_values(touch_sensor[1]);
     // printf("force vector_RF: %8.2f %8.2f %8.2f\n", f1[0], f1[1], f1[2]);
    
  // footforce_update();
  // forward_kinematic(LF);

   // test = get_motor_angle(LF1);
     // state_update();
    // printf("pitch = %.3lf ,roll = %.3lf,yaw = %.3lf\n",eulerAngle.pitch,eulerAngle.roll,eulerAngle.yaw);   
    t += (double)TIME_STEP / 1000;  // 
        // printf("yaw = %lf   dyaw = %lf \n",robot.IMU[2],robot.dIMU[2]);
       // printf("t = %lf\n",t);   
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}


