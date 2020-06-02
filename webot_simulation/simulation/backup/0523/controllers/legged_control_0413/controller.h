/**********************************************************
**             
**---------------------------------------------------------
**  Description: 此文件为四足机器人 控制器 头文件
**  Version    : 
**  Notes      :
**  Author     : 

                ↑       向上为前进方向                  
        0 LF ******* RF 1                 **************        ↑  Z   2
             *     *                         *       *
             *     *                           *       *        →   Y  1
        3 LH ******* RH 2                    *       *   
                                                                ▪   X  0
         *                                                       
           *                                                       
             *                                                     
           *   theta1 沿杆方向的力与地面夹角为90+real_θ1+real_θ2
      *  *  *        垂直杆方向的力与地面夹角 = -real_θ1-real_θ2
**********************************************************/
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdbool.h>

#include "easyMat.h"
#include "webotsInterface.h"

//-----------------------------------------------------------macro
#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )

#define g        (9.8)
//-----------------------------------------------------------typedef
/*
足底3维向量
*/
typedef struct
{
  double x;
  double y;
  double z;
}vect3TypeDef;
/*
机器人定义,包括机器人状态和机器人参数
*/
typedef struct
{
  //------------------------------------------------------------------------控制参数
  double Tf     ;  // 0.5;
  int Ts     ;  // 0.5;
  double hd     ;  // 0.6;   //机器人期望高度
  double zh     ;  // -0.5; 
  double kpitch     ;  // 10000;  //pitch增益
  double kdpitch    ;  // 800;    //pirtch增益
  double kroll     ;  // 10000;  //roll增益
  double kdroll    ;  // 800;    //roll增益
  double kyaw   ;  // 10000;  //yaw增益
  double kdyaw  ;  // 800;    //roll增益
  double Kh     ;  // 8000;   //机器人高度增益
  double Kdh    ;  // 800;    //机器人高度增益
  double Kvx    ;  // 1000;   //机器人速度增益
  double kx     ;  // 100;    //机器人抬腿前进增益
  double kdx    ;  // 100;    //机器人抬腿前进增益
  double kz1    ;  // 10000;  //机器人抬腿高度增益
  double kz2    ;  // 1000;   //机器人抬腿高度增益
  double kdz    ;  // 100;    //机器人抬腿高度增益
  
  double angular_velocity ; //期望转角
  double clock_angle      ;  //转角
  double dclock_angle     ; //转速
  double clock_angle_exp  ; //期望转速
  double z_st_exp[4]      ; //四足期望高度
  bool   ok               ;
  int    clock_flag       ;
  int    num              ; //转过的圈数
  double dvx              ; //防止后座速度增益
  double vxd_forward      ; //默认前向期望速度
  double vxd_backward     ; //默认后向期望速度
  double vxd              ; //默认初始x速度
  double vyd_right        ; //默认右向期望速度
  double vyd_left         ; //默认左向期望速度
  double vyd              ; //默认初始y速度
  double vx               ;  //机器人实际x速度
  double vy               ;  //机器人实际y速度
  double clockwise        ;
  double cclockwise       ;
  double d_yaw            ; //转弯
  double xT[4]            ; //x落脚点
  double yT[4]            ; //y落脚点
  double F_st[4][3]       ;
  double F_sw[4][3]       ;
  double T_clock          ; 
  double tao_roll         ;
  double stepLength       ;
  double stepHight        ;
  double state            ;
  double            t              ;  // 0;      //单步计时器
  double            clock          ;  // 0;      //从0开始，经过的时间
  double            counter[4]        ;
  trotGroupNameTypeDef    st       ;  // group1;     //前支撑腿
  trotGroupNameTypeDef    sw       ;  // group1;     //前摆动腿
  bool              is_touching[4] ;  // false 0 表示未触底;  //足底传感器接触标志量,下标为L,R
  double           IMU[3]          ;  // 0;      //pitch
  double           IMU1[3]          ;  // 0;      //pitch
  double           dIMU[3]          ;  // 0;      //pitch
  double       accelerometer[4]     ;
  double           IMU_position1[3];   //四条腿的位置反馈控制量
  double           IMU_position2[3];   //四条腿的位置反馈控制量
  double            theta[4][3]    ;  //电机控制角度
  double            real_theta[4][3] ; //真实关节角度
  double            foot[4][3]     ;  //足底坐标
  double            dfoot[4][3]    ;  //足底速度
  double            toed[4]        ;  //摆动相足底期望坐标
  double            dtoed[4]       ;  //摆动相足底期望速度
  double      stop_toed[4]   ;  //摆动相足底提前触底坐标
  double      stop_dtoed[4]  ;  //摆动相足底提前触底速度
  double      toe0[4]        ;  //足底初始位置
  double      dtoe0[4]       ;  // 足底初始速度
  double      footforce[4][3]      ;
  double      real_footforce[4][3] ;
  double      foottheta[4][2]      ;  //
  double      swing_endpos[4][3]   ;
  double      stance_endpos[4][3]  ;
  double      IMU_array1[25][3]    ;  //0-T 的IMU序列
  double      IMU_array2[25][3]    ;  //T-2T 的IMU序列
  double      IMU_err1[3]          ; //0-T 的IMU err
  double      IMU_err2[3]          ; //T-2T 的IMU err

  
   /**************************
 机器人尺寸参数
 ****************************/
  double           l1           ;  //机器人大腿长度
  double           l2            ; //机器人小腿长度
}robotTypeDef;
//-----------------------------------------------------------extern
// extern robotTypeDef robot;

extern void robot_init();
extern void updateRobotState();
extern void robot_control();
extern void update_IMU                 ();
extern void update_foot_touch_sensor   ();
extern void update_vxd                 ();
extern void phase_swap                 ();
extern void update_theta               ();
extern void update_clock_angle         ();
void forwardKinematics          ();
void estimate_vxy               ();
void create_transJ(matTypeDef* transJ, legNameTypeDef leg);
void curve2D(vect3TypeDef* ft, vect3TypeDef* dft, double xT, vect3TypeDef toe0, vect3TypeDef dtoe0);
void curve3D(vect3TypeDef* ft, vect3TypeDef* dft, double xT, double yT, vect3TypeDef toe0, vect3TypeDef dtoe0);
void update_toe0_dtoe0          ();
void robot_trot                 ();
void robot_static               ();
void robot_control              ();
void robot_trot_control         ();

#endif

