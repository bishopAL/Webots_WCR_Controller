/**********************************************************
**             
**---------------------------------------------------------
**  Description: 此文件为四足机器人 控制器 头文件
**  Version    : 
**  Notes      :
**  Author     : 

                ↑       向上为前进方向                  
        0 LF ******* RF 1                 **************        ↑  Z
             *     *                         *       *
             *     *                           *       *        →   Y
        3 LH ******* RH 2                    *       *   
                                                                ▪   X 

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
  double Ts     ;  // 0.5;
  double hd     ;  // 0.6;   //机器人期望高度
  double zh     ;  // -0.5; 
  double Kp     ;  // 10000;  //pitch增益
  double Kdp    ;  // 800;    //pirtch增益
  double Kr     ;  // 10000;  //roll增益
  double Kdr    ;  // 800;    //roll增益
  double Kyaw   ;  // 10000;  //roll增益
  double Kdyaw  ;  // 800;    //roll增益
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

  double            t              ;  // 0;      //从一步开始，经过的时间
  double            clock          ;  // 0;      //从0开始，经过的时间
  double            counter        ;
  trotGroupNameTypeDef    st       ;  // group1;     //前支撑腿
  trotGroupNameTypeDef    sw       ;  // group1;     //前摆动腿
  bool              is_touching[4] ;  // false 0 表示未触底;  //足底传感器接触标志量,下标为L,R
  double            pitch          ;  // 0;      //pitch
  double            dpitch         ;  // 0;      //pitch'
  double            roll           ;  // 0;      //pitch
  double            droll          ;  // 0;      //pitch'
  double            yaw            ;  // 0;      //pitch
  double            dyaw           ;  // 0;      //pitch'
  double            yaw_now        ;
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

