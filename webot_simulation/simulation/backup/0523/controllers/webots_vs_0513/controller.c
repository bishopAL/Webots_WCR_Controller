/**********************************************************
**             Email:@qq.com  
**---------------------------------------------------------
**  Description: 此文件为四足机器人 控制器 文件
**  Version    : 
**  Notes      :
**  Author     : 
**********************************************************/
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <webots/keyboard.h>

#include "easyMat.h"
#include "controller.h"


robotTypeDef robot;

/*
机器人参数初始化，在while（1）之前调用
*/
void robot_init()
{
  //------------------------------------------------------------------------控制参数 
   robot.Tf     =   0.25;
   robot.Ts     =   0.25;
   robot.hd     =   0.60;
   robot.zh     =  -0.35;
   robot.Kp     =   4000;
   robot.Kdp    =    200;
   robot.Kr     =   4000;  // 10000;  //roll增益
   robot.Kdr    =    200;  // 800;    //roll增益
   robot.Kyaw   =   4000;  // 10000;  //yaw增益
   robot.Kdyaw  =    200;  // 800;    //yaw增益
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
   robot.angular_velocity           =  0;  //转弯期望角速度
   robot.dclock_angle               =  0;  //转弯实际角速度
   robot.clock_flag                 =  0;
  //------------------------------------------------------------------------模型参数
   robot.body_long  =   0.6 ;
   robot.body_width =   0.48;
   robot.a0         =   0   ;
   robot.a1         =   0.4 ;    //大腿连杆长度
   robot.a2         =   0.4 ;    //小腿连杆长度
   robot.m0         =   0   ;
   robot.m1         =   4   ;    //大腿连杆质量
   robot.m2         =   4   ;    //小腿连杆质量
   robot.M          =   50  ;    //躯干质量
  //------------------------------------------------------------------------状态区
   robot.t                    =  0;      //从一步开始，经过的时间
   robot.clock                =  0;      //从0开始，经过的时间
   robot.st               = group1;      //初始状态支撑腿组
   robot.sw               = group2;      //初始状态摆动腿组
   robot.is_touching[LF]   = false;      //足底传感器接触标志量
   robot.is_touching[RF]   = false;      //足底传感器接触标志量
   robot.is_touching[RB]   = false;      //足底传感器接触标志量
   robot.is_touching[LB]   = false;      //足底传感器接触标志量
   robot.pitch             = 0    ;      //pitch
   robot.dpitch            = 0    ;      //pitch'
   robot.roll              = 0    ;      //pitch
   robot.droll             = 0    ;      //pitch'
   robot.yaw               = 0    ;      //pitch
   robot.dyaw              = 0    ;      //pitch'
   robot.yaw_now           = 0    ;
   for(legNameTypeDef legID = 0; legID < 4; legID ++)
   {
     robot.dtoe[legID].x = 0     ;
     robot.dtoe[legID].y = 0     ;
     robot.dtoe[legID].z = 0     ;
     
     robot.toe0[legID].x = 0     ;
     robot.toe0[legID].y = 0     ;
     robot.toe0[legID].z = -0.64;
   }
   for(trotGroupNameTypeDef groupID = 0; groupID < 2; groupID ++)
   {
     robot.dtoe0[groupID].x         = 0;
     robot.dtoe0[groupID].y         = 0;
     robot.dtoe0[groupID].z         = 0;
   }
}
/*
机器人状态更新，包括读取传感器的数据以及一些状态估计
*/
void updateRobotState()
{
  /*IMU和IMU导数更新*/
  update_IMU();
  /*通过键盘修改期望速度*/
  update_vxd();
  /*更新转弯转角*/
  update_clock_angle();
  /*足底接触传感器更新*/
  update_foot_touch_sensor();
  /*相位切换*/
  phase_swap();
  /*更新关节角*/
  update_theta();
  /*运动学正解*/
  forwardKinematics();
  /*记录相位切换瞬间，摆动足的起始位置和起始速度*/
  update_toe0_dtoe0();
  /*机器人水平速度估计*/
  estimate_vxy();  //在robot.t=0时候特殊处理
  /*时钟更新*/
  robot.t += 0.001*TIME_STEP; 
  robot.clock += 0.001*TIME_STEP;   
}
/*
通过键盘修改期望速度
*/
void update_vxd()
{
   /*读取键盘，获取速度*/
  switch(get_keyboard())
  {
    case WB_KEYBOARD_UP:
    {
      robot.vxd = robot.vxd_forward;
      break;
    }
    case WB_KEYBOARD_DOWN:
    {
      robot.vxd = -robot.vxd_forward;
      break;
    }
    case WB_KEYBOARD_LEFT:
    {
      robot.vyd = -robot.vyd_right;
      break;
    }
    case WB_KEYBOARD_RIGHT:
    {
      robot.vyd = robot.vyd_right;
      break;
    }
    case WB_KEYBOARD_PAGEUP:
    {
      robot.clock_flag = 1;
      robot.angular_velocity = PI/4;
      robot.yaw_now = robot.yaw;
      break;
    }
    case WB_KEYBOARD_PAGEDOWN:
    {
      robot.clock_flag = 2;
      robot.angular_velocity = -PI/4;
      robot.yaw_now = robot.yaw;
      break;
    }
    default:
    {
      robot.vxd              = 0;
      robot.vyd              = 0;
      robot.angular_velocity = 0;
      robot.clock_flag       = 0;
      break;
    }
  } 
}
/*
足底传感器更新
*/
void update_foot_touch_sensor()
{
  for(legNameTypeDef legID = 0; legID < 4; legID ++)
  {
    robot.is_touching[legID] = is_foot_touching(legID);
  }
}
/*
更新关节角
*/
void update_theta()
{
  for(legNameTypeDef legID = 0; legID < 4; legID ++)
  {
    for(angleNameTypeDef angleID = 0; angleID < 3; angleID ++)
    {
      robot.theta[legID][angleID] = get_motor_angle(3*legID + angleID);
//      printf("%lf  ", robot.theta[legID][angleID]);
    }
//    printf("\n");
  }
//   printf("\n");
}
/*
IMU与IMU的导数更新
*/
void update_IMU()
{
  static eulerAngleTypeDef pre_eulerAngle = {0,0,0};
  eulerAngleTypeDef eulerAngle = get_IMU_Angle();
  //IMU
  robot.pitch  = eulerAngle.pitch;
  robot.roll   = eulerAngle.roll;
  robot.yaw    = eulerAngle.yaw;
  printf("%lf  ", robot.yaw);
  printf("\n");
  //IMU'
  static double pre_dpitch = 0;
  static double pre_droll = 0;
  static double pre_dyaw = 0;
  robot.dpitch = (eulerAngle.pitch - pre_eulerAngle.pitch)/((double)TIME_STEP/1000.0);
  robot.droll  = (eulerAngle.roll - pre_eulerAngle.roll)/((double)TIME_STEP/1000.0);
  robot.dyaw   = (eulerAngle.yaw - pre_eulerAngle.yaw)/((double)TIME_STEP/1000.0);
  robot.dpitch = robot.dpitch*0.1 + pre_dpitch*0.9;  //滤波
  robot.droll  = robot.droll*0.1 + pre_droll*0.9;  //滤波
  robot.dyaw   = robot.dyaw*0.1 + pre_dyaw*0.9;  //滤波
  
  //pre IMU  IMU'
  pre_eulerAngle = eulerAngle;
  pre_dpitch     = robot.dpitch;
  pre_droll      = robot.droll;
  pre_dyaw      = robot.dyaw;
//  printf("%lf  \n",robot.roll);
}
/*
运动学正解
*/
void forwardKinematics()
{
  double a1        = robot.a1;
  double a2        = robot.a2;
  
  //足底位置
  for(legNameTypeDef legID = 0; legID < 4; legID++)
  {
    double s0  = sin( robot.theta[legID][0]/180.0*PI );
    double c0  = cos( robot.theta[legID][0]/180.0*PI );
    double s1  = sin( robot.theta[legID][1]/180.0*PI );
    double c1  = cos( robot.theta[legID][1]/180.0*PI );
    double s12 = sin( robot.theta[legID][1]/180.0*PI + robot.theta[legID][2]/180.0*PI);
    double c12 = cos( robot.theta[legID][1]/180.0*PI + robot.theta[legID][2]/180.0*PI);
    
    robot.toe[legID].x =        - a1*s1 - a2*s12;  //前后
    robot.toe[legID].y =    a1*s0*c1 + a2*s0*c12;  //左右
    robot.toe[legID].z =   -a1*c0*c1 - a2*c0*c12;  //高度
  }

//  printf("%lf  %lf\n",robot.toe[RF].x,robot.toe[RF].z);
  //足底速度
  static vect3TypeDef pre_toe[4];
  static bool first_run_flag = true;
  if(first_run_flag == true)
  {
    first_run_flag = false;
  }
  else
  {
    for(legNameTypeDef legId = 0; legId < 4; legId++)
    {
      robot.dtoe[legId].x = (robot.toe[legId].x - pre_toe[legId].x)/(0.001*(double)TIME_STEP);
      robot.dtoe[legId].y = (robot.toe[legId].y - pre_toe[legId].y)/(0.001*(double)TIME_STEP);
      robot.dtoe[legId].z = (robot.toe[legId].z - pre_toe[legId].z)/(0.001*(double)TIME_STEP);
    }
  }
  for(legNameTypeDef legId = 0; legId < 4; legId++)
  {
    pre_toe[legId] = robot.toe[legId];
  }
}
/*
机器人水平速度估计
*/
void estimate_vxy()
{
  static vect3TypeDef pre_toe[2];
         vect3TypeDef     toe[2];
  
  toe[0] = robot.toe[robot.st]; //支撑相
  toe[1] = robot.toe[robot.st + 2]; //支撑相
  if(robot.t != 0)
  {
    robot.vx = -((toe[0].x - pre_toe[0].x) + (toe[1].x - pre_toe[1].x))/(2*0.001*(double)TIME_STEP);
    robot.vy = -((toe[0].y - pre_toe[0].y) + (toe[1].y - pre_toe[1].y))/(2*0.001*(double)TIME_STEP);
  }
  static double pre_vx = 0, pre_vy = 0;
  robot.vx = robot.vx*0.1 + pre_vx*0.9; //滤波
  robot.vy = robot.vy*0.1 + pre_vy*0.9; //滤波
  
  pre_vx = robot.vx;
  pre_vy = robot.vy;
  pre_toe[0] = toe[0];
  pre_toe[1] = toe[1];
  
//  printf("%lf  \n",robot.vz);
}
/*
更新摆动足的起始位置和起始速度
*/
void update_toe0_dtoe0()
{
  static vect3TypeDef pre_st_toe[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  static vect3TypeDef st_dtoe[4]    = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  if(robot.t == 0) //刚刚切换过对角腿，记录此时的摆动足即可
  {
    for(legNameTypeDef legId = 0; legId <= 3; legId++ )
    {
      //记录初始位置
      robot.toe0[legId] = robot.toe[legId];
      //记录初始速度
      robot.dtoe0[legId] = st_dtoe[legId];
    }
  }
  else
  {
    for(legNameTypeDef legId = 0; legId <= 3; legId++ )
    {
      st_dtoe[legId].x = (robot.toe[legId].x - pre_st_toe[legId].x)/(0.001*((double)TIME_STEP));
      st_dtoe[legId].y = (robot.toe[legId].y - pre_st_toe[legId].y)/(0.001*((double)TIME_STEP));
      st_dtoe[legId].z = (robot.toe[legId].z - pre_st_toe[legId].z)/(0.001*((double)TIME_STEP));
    }
  }
  for(legNameTypeDef legId = 0; legId <= 3; legId++ )
    pre_st_toe[legId] = robot.toe[legId];
}
/*
三维足底轨迹
*/
void curve3D(vect3TypeDef* ft, vect3TypeDef* dft, double xT,double yT, vect3TypeDef toe0, vect3TypeDef dtoe0)
{
  double t   = robot.t;
  double x0  = toe0.x;
  double y0  = toe0.y;
  double z0  = toe0.z;
  double dx0 = dtoe0.x;
  double dy0 = dtoe0.y;
//  double dz0 = dtoe0.z;
  double Tf  = robot.Tf;
  double zh  = robot.zh;
  //x
  if(t < Tf/4.0)
  {  
    ft->x  = -4*dx0*t*t/Tf + dx0*t + x0;
    dft->x = -8*dx0*t/Tf + dx0;
  }
  else if((t >= Tf/4.0)&&(t < 3.0*Tf/4.0))
  {
    ft->x  = ( -4*Tf*dx0 - 16*xT + 16*x0)*t*t*t/(Tf*Tf*Tf) + 
             (  7*Tf*dx0 + 24*xT - 24*x0)*t*t/(Tf*Tf) + 
             (-15*Tf*dx0 - 36*xT + 36*x0)*t/(4*Tf) + 
             (  9*Tf*dx0 + 16*xT)/16;
            
    dft->x = ( -4*Tf*dx0 - 16*xT + 16*x0)*3*t*t/(Tf*Tf*Tf) + 
             (  7*Tf*dx0 + 24*xT - 24*x0)*2*t/(Tf*Tf) + 
             (-15*Tf*dx0 - 36*xT + 36*x0)/(4*Tf);
  }
  else
  {
    ft->x  = xT;
    dft->x = 0;
  }
  //y
  if(t < Tf/4.0)
  {  
    ft->y  = -4*dy0*t*t/Tf + dy0*t + y0;
    dft->y = -8*dy0*t/Tf + dy0;
  }
  else if((t >= Tf/4.0)&&(t < 3.0*Tf/4.0))
  {
    ft->y  = ( -4*Tf*dy0 - 16*yT + 16*y0)*t*t*t/(Tf*Tf*Tf) + 
             (  7*Tf*dy0 + 24*yT - 24*y0)*t*t/(Tf*Tf) + 
             (-15*Tf*dy0 - 36*yT + 36*y0)*t/(4*Tf) + 
             (  9*Tf*dy0 + 16*yT)/16;
            
    dft->y = ( -4*Tf*dy0 - 16*yT + 16*y0)*3*t*t/(Tf*Tf*Tf) + 
             (  7*Tf*dy0 + 24*yT - 24*y0)*2*t/(Tf*Tf) + 
             (-15*Tf*dy0 - 36*yT + 36*y0)/(4*Tf);
  }
  else
  {
    ft->y  = yT;
    dft->y = 0;
  }
  //z
  if(t < Tf/2.0)
  {
    ft->z  = 16*(z0 - zh)*t*t*t/(Tf*Tf*Tf) + 12*(zh - z0)*t*t/(Tf*Tf) + z0;
    dft->z = 16*(z0 - zh)*3*t*t/(Tf*Tf*Tf) + 12*(zh - z0)*2*t/(Tf*Tf);
  }
  else
  {
    ft->z  = 4*(z0 - zh)*t*t/(Tf*Tf) - 4*(z0 - zh)*t/Tf + z0;
    dft->z = 4*(z0 - zh)*2*t/(Tf*Tf) - 4*(z0 - zh)/Tf;
  }
  // printf("规划：%lf  %lf\n",ft->x, ft->z);
}

/*
相位切换,通过时间和足底传感器确定支撑对角腿和摆动对角腿
*/

void phase_swap()
{
  if(robot.t > 0.75*robot.Tf)
  {
    if(robot.st == group1)  //支撑腿是L
    {
      if(robot.is_touching[RF] && robot.is_touching[LB])
      {
        robot.st = group2;
        robot.sw = group1;
        robot.t  =      0;      //相位切换时候时间归零
      }
    }
    else if(robot.st == group2)  //支撑腿是R
    {
      if(robot.is_touching[LF] && robot.is_touching[RB])
      {
        robot.st = group1;
        robot.sw = group2;
        robot.t  =      0;      //相位切换时候时间归零
      }
    }
  }
}

/*
创建力雅可比
*/
void create_transJ(matTypeDef* transJ, legNameTypeDef leg)
{
  double a0 = robot.a0;
  double a1 = robot.a1;
  double a2 = robot.a2;
  
  double s0  = sin( robot.theta[leg][0]/180.0*PI );
  double c0  = cos( robot.theta[leg][0]/180.0*PI );
  double s1  = sin( robot.theta[leg][1]/180.0*PI );
  double c1  = cos( robot.theta[leg][1]/180.0*PI );
  double s12 = sin( robot.theta[leg][1]/180.0*PI + robot.theta[leg][2]/180.0*PI);
  double c12 = cos( robot.theta[leg][1]/180.0*PI + robot.theta[leg][2]/180.0*PI);
  transJ->data[0][0] =                           0;
  transJ->data[0][1] =    c0*(a0 + a1*c1 + a2*c12); 
  transJ->data[0][2] =    s0*(a0 + a1*c1 + a2*c12); 
  
  transJ->data[1][0] =             -a1*c1 - a2*c12;
  transJ->data[1][1] =        -s0*(a1*s1 + a2*s12); 
  transJ->data[1][2] =         c0*(a1*s1 + a2*s12); 
  
  transJ->data[2][0] =                     -a2*c12;
  transJ->data[2][1] =                  -a2*s0*s12; 
  transJ->data[2][2] =                  a2*c0*s12 ; 
}
/*
机器人static
*/
void robot_static()
{
  vect3TypeDef toed[4], dtoed[4];
  double F_static_x[4], F_static_y[4], F_static_z[4];
  matTypeDef static_f[4];  //摆动腿
  matTypeDef static_transJ[4], static_tau[4];
  for(legNameTypeDef legId = 0; legId <= 3; legId++ )
  {
    toed[legId].x  = 0;
    toed[legId].y  = 0;
    toed[legId].z  = -robot.hd;
    dtoed[legId].x = 0;
    dtoed[legId].y = 0;
    dtoed[legId].z = 0;
    
    F_static_x[legId] = -(robot.kx*(robot.toe[legId].x - toed[legId].x) + robot.kdx*(robot.dtoe[legId].x - dtoed[legId].x));
    F_static_y[legId] = -(robot.kx*(robot.toe[legId].y - toed[legId].y) + robot.kdx*(robot.dtoe[legId].y - dtoed[legId].y));
    F_static_z[legId] = -(robot.kz1*(robot.toe[legId].z - toed[legId].z) + robot.kdz*(robot.dtoe[legId].z - dtoed[legId].z));

    easyMat_create(&static_f[legId], 3,1);
    static_f[legId].data[0][0] =  F_static_x[legId];
    static_f[legId].data[1][0] =  F_static_y[legId];
    static_f[legId].data[2][0] =  F_static_z[legId];

    easyMat_create(&static_tau[legId]   , 3, 1);
    easyMat_create(&static_transJ[legId], 3, 3);
    create_transJ(&static_transJ[legId], legId);
    easyMat_mult(&static_tau[legId], &static_transJ[legId], &static_f[legId]);
    
    //发送关节力到电机
    set_motor_torque(3*legId + 0, static_tau[legId].data[0][0]);
    set_motor_torque(3*legId + 1, static_tau[legId].data[1][0]);
    set_motor_torque(3*legId + 2, static_tau[legId].data[2][0]);
    
    //释放内存
    easyMat_free(&static_f[legId]     );
    easyMat_free(&static_transJ[legId]);
    easyMat_free(&static_tau[legId]   );  
  }
}
/*
机器人trot
*/
void robot_trot()
{
  //-----------------------------------------------------------------支撑相
  
  //创建矩阵
  matTypeDef st_f[3];
  matTypeDef st_transJ[3], st_tau[3];
  easyMat_create(&st_f[F], 3, 1);
  easyMat_create(&st_f[B], 3, 1);
  
  if(robot.st == group1)
  {
    for(CoordinateNameTypeDef CoordinateID = 0; CoordinateID < 3; CoordinateID ++)
    {
      st_f[F].data[CoordinateID][0] =  robot.F_st[LF][CoordinateID];
      st_f[B].data[CoordinateID][0] =  robot.F_st[RB][CoordinateID];
    }
  }
  else  //(robot.st == R)
  {
    for(CoordinateNameTypeDef CoordinateID = 0; CoordinateID < 3; CoordinateID ++)
    {
      st_f[F].data[CoordinateID][0] =  robot.F_st[RF][CoordinateID];
      st_f[B].data[CoordinateID][0] =  robot.F_st[LB][CoordinateID];
    }
  }
  //矩阵计算
  for(FBNameTypeDef FBId = 0; FBId <= 1; FBId++)
  {
    easyMat_create(&st_tau[FBId]   , 3, 1);
    easyMat_create(&st_transJ[FBId], 3, 3);
    create_transJ(&st_transJ[FBId], (2*FBId + robot.st));
    easyMat_mult(&st_tau[FBId], &st_transJ[FBId], &st_f[FBId]);
    easyMat_mult_k(-1.0, &st_tau[FBId]); 
  }
  //发送关节力到电机
  if(robot.st == group1)
  {
    set_motor_torque(LF0, st_tau[F].data[0][0] - robot.tao_roll);
    set_motor_torque(LF1, st_tau[F].data[1][0]);
    set_motor_torque(LF2, st_tau[F].data[2][0]);
    set_motor_torque(RB0, st_tau[B].data[0][0] - robot.tao_roll);
    set_motor_torque(RB1, st_tau[B].data[1][0]);
    set_motor_torque(RB2, st_tau[B].data[2][0]);
  }
  else if(robot.st == group2)
  {
    set_motor_torque(RF0, st_tau[F].data[0][0] - robot.tao_roll);
    set_motor_torque(RF1, st_tau[F].data[1][0]);
    set_motor_torque(RF2, st_tau[F].data[2][0]);
    set_motor_torque(LB0, st_tau[B].data[0][0] - robot.tao_roll);
    set_motor_torque(LB1, st_tau[B].data[1][0]);
    set_motor_torque(LB2, st_tau[B].data[2][0]);
  }
  //释放内存
  for(FBNameTypeDef FBId = 0; FBId <= 1; FBId++)
  {
    easyMat_free(&st_f[FBId]     );
    easyMat_free(&st_transJ[FBId]);
    easyMat_free(&st_tau[FBId]   );  
  }

  //-----------------------------------------------------------------摆动相控制
  for(legNameTypeDef legId = 0; legId <= 3; legId++ )
    curve3D(&robot.toed[legId], &robot.dtoed[legId], robot.xT[legId], robot.yT[legId], robot.toe0[legId], robot.dtoe0[legId]);  
  //----------------------------------------------------------------触底停止下落检测
  //LF
  if(robot.is_touching[LF] == 1 && robot.st == group1)
  {
    robot.toed[LF]  = robot.stop_toed[LF];
    robot.dtoed[LF] = robot.stop_dtoed[LF];
  }else
  {
    robot.stop_toed[LF]  = robot.toed[LF];
    robot.stop_dtoed[LF] = robot.dtoed[LF];
  }
  //RB
  if(robot.is_touching[RB] == 1 && robot.st == group1)
  {
    robot.toed[RB]  = robot.stop_toed[RB];
    robot.dtoed[RB] = robot.stop_dtoed[RB];
  }else
  {
    robot.stop_toed[RB]  = robot.toed[RB];
    robot.stop_dtoed[RB] = robot.dtoed[RB];
  }
  //RF
  if(robot.is_touching[RF] == 1 && robot.st == group2)
  {
    robot.toed[RF]  = robot.stop_toed[RF];
    robot.dtoed[RF] = robot.stop_dtoed[RF];
  }else
  {
    robot.stop_toed[RF]  = robot.toed[RF];
    robot.stop_dtoed[RF] = robot.dtoed[RF];
  }
  //LB
  if(robot.is_touching[LB] == 1 && robot.st == group2)
  {
    robot.toed[LB]  = robot.stop_toed[LB];
    robot.dtoed[LB] = robot.stop_dtoed[LB];
  }else
  {
    robot.stop_toed[LB]  = robot.toed[LB];
    robot.stop_dtoed[LB] = robot.dtoed[LB];
  }
  
  matTypeDef sw_f[3];  //摆动腿
  easyMat_create(&sw_f[F], 3,1);
  easyMat_create(&sw_f[B], 3,1);

  if(robot.sw == group1)
  {
    for(CoordinateNameTypeDef CoordinateID = 0; CoordinateID < 3; CoordinateID ++)
    {
      sw_f[F].data[CoordinateID][0] =  robot.F_sw[LF][CoordinateID];
      sw_f[B].data[CoordinateID][0] =  robot.F_sw[RB][CoordinateID];
    }
  }
  else  //(robot.st == R)
  {
    for(CoordinateNameTypeDef CoordinateID = 0; CoordinateID < 3; CoordinateID ++)
    {
      sw_f[F].data[CoordinateID][0] =  robot.F_sw[RF][CoordinateID];
      sw_f[B].data[CoordinateID][0] =  robot.F_sw[LB][CoordinateID];
    }
  }
  //求足底力
  
  matTypeDef sw_transJ[3], sw_tau[3];
  for(FBNameTypeDef FBId = 0; FBId <= 1; FBId++)
  {
    easyMat_create(&sw_tau[FBId]   , 3, 1);
    easyMat_create(&sw_transJ[FBId], 3, 3);
    create_transJ(&sw_transJ[FBId], (2*FBId + robot.sw));
    easyMat_mult(&sw_tau[FBId], &sw_transJ[FBId], &sw_f[FBId]);
  }
  //发送关节力到电机
  if(robot.sw == group1)
  {
    set_motor_torque(LF0, sw_tau[F].data[0][0]);
    set_motor_torque(LF1, sw_tau[F].data[1][0]);
    set_motor_torque(LF2, sw_tau[F].data[2][0]);
    set_motor_torque(RB0, sw_tau[B].data[0][0]);
    set_motor_torque(RB1, sw_tau[B].data[1][0]);
    set_motor_torque(RB2, sw_tau[B].data[2][0]);
  }
  else if(robot.sw == group2)
  {
    set_motor_torque(RF0, sw_tau[F].data[0][0]);
    set_motor_torque(RF1, sw_tau[F].data[1][0]);
    set_motor_torque(RF2, sw_tau[F].data[2][0]);
    set_motor_torque(LB0, sw_tau[B].data[0][0]);
    set_motor_torque(LB1, sw_tau[B].data[1][0]);
    set_motor_torque(LB2, sw_tau[B].data[2][0]);
  }
  //释放内存
  for(FBNameTypeDef FBId = 0; FBId <= 1; FBId++)
  {
    easyMat_free(&sw_f[FBId]     );
    easyMat_free(&sw_transJ[FBId]);
    easyMat_free(&sw_tau[FBId]   );  
  }
}
/*
更新转弯转角
*/
void update_clock_angle()
{
  static double pre_clock_angle = 0;
//  static double cclock_direction_flag = 0;
  //------------------------------------------------------逆时针
  if(robot.clock_flag == 1)
  {
    if(robot.yaw > 0 && robot.yaw < 1 && robot.ok == 0)
    {
      robot.ok = 1;
    }
    if(robot.yaw > -1 && robot.yaw <0  && robot.ok == 1)
    {
      robot.num = robot.num + 1;
      robot.ok = 0;
    }
    if(robot.yaw < 0)
    {
      if(robot.yaw > -1 && robot.yaw <0)
        robot.clock_angle = (robot.num - 1)* 360 + robot.yaw + 360;
      else
        robot.clock_angle = robot.num* 360 + 360 + robot.yaw;
    }
    else 
      robot.clock_angle = robot.num* 360 + robot.yaw;
  }
  //------------------------------------------------------顺时针
  if(robot.clock_flag == 2)
  {
    if(robot.yaw < 0 && robot.yaw > -1 && robot.ok == 0)
    {
      robot.ok = 1;
    }
    if(robot.yaw > 0 && robot.yaw < 1  && robot.ok == 1)
    {
      robot.num = robot.num - 1;
      robot.ok = 0;
    }
    if(robot.yaw > 0)
    {
      if(robot.yaw > 0 && robot.yaw < 1)
        robot.clock_angle = (robot.num + 1)* 360 + robot.yaw - 360;
      else
        robot.clock_angle = robot.num* 360 + robot.yaw - 360;  
    }
    else
        robot.clock_angle = robot.num* 360 + 360 + robot.yaw; 
  }
  robot.dclock_angle = (robot.clock_angle - pre_clock_angle)/((double)TIME_STEP/1000.0);
  pre_clock_angle = robot.clock_angle;
//  printf("%lf  %lf  %d\n", robot.clock_angle, pre_clock_angle, robot.num);
}
/*
机器人trot步态控制
*/
void robot_trot_control()
{
  //----------------------------------------------------平坦路面行走
  double Tx = -(robot.Kr*(robot.roll/180.0*PI - 0) + robot.Kdr*(robot.droll/180.0*PI - 0));
  robot.tao_roll = 0*Tx;
  double Ty = -(robot.Kp*(robot.pitch/180.0*PI - 0) + robot.Kdp*(robot.dpitch/180.0*PI - 0));
  //  double Ty = 0;
  //高度补偿计算
  double dot_pitch_z[2] = {0, 0}, dot_roll_z[2] = {0, 0};
  double K_pitch_z = 0.001, K_roll_z = 0.001;
  dot_pitch_z[F] = dot_pitch_z[F] - K_pitch_z*Ty;
  dot_pitch_z[B] = dot_pitch_z[B] + K_pitch_z*Ty;
  
  dot_roll_z[L] = dot_roll_z[L] + K_roll_z*Tx;
  dot_roll_z[R] = dot_roll_z[R] - K_roll_z*Tx;
//  dot_roll_z[L] = 0;
//  dot_roll_z[R] = 0;

  //-----------------------------------------------------上坡自适应策略
  double d_M_x = robot.M*g*sin(-robot.pitch/180*PI);
  double d_M_z = robot.M*g*cos(-robot.pitch/180*PI);
  static double d_x = 0;  //上坡时调整x方向
  d_x = robot.hd*tan(-robot.pitch/180*PI);
  //-----------------------------------------------------保持方向
  if(robot.yaw_now > 178 && robot.yaw_now <= 180)
    robot.yaw_now = 178;
  if(robot.yaw_now < -178 && robot.yaw_now >= -180)
    robot.yaw_now = -178;
  double F_M_z;
  double M_z = -(1000*(robot.yaw/180.0*PI - robot.yaw_now/180.0*PI) + 100*(robot.dyaw/180.0*PI - 0));
  if(robot.clock_flag == 0)
    F_M_z = M_z/(2*robot.body_width);
  else 
    F_M_z = 0;
//  printf("%lf  %lf\n",robot.yaw_now, robot.yaw);
  //-----------------------------------------------------转弯
  robot.T_clock = -1000*(robot.dclock_angle/180*PI - robot.angular_velocity);
  double F_clock = robot.T_clock/(robot.body_long/2);
  //------------------------------------------------------支撑相
  //期望高度计算
  robot.z_st_exp[LF] = -(robot.hd + dot_pitch_z[F] + dot_roll_z[L]);
  robot.z_st_exp[RF] = -(robot.hd + dot_pitch_z[F] + dot_roll_z[R]);
  robot.z_st_exp[RB] = -(robot.hd + dot_pitch_z[B] + dot_roll_z[R]);
  robot.z_st_exp[LB] = -(robot.hd + dot_pitch_z[B] + dot_roll_z[L]);
  
  //计算group1、group2的脚底分解力
  robot.F_st[LF][y] = robot.Kvx*(robot.dtoe[LF].y - robot.vyd) + F_clock + F_M_z;
  robot.F_st[RF][y] = robot.Kvx*(robot.dtoe[RF].y - robot.vyd) + F_clock + F_M_z;
  robot.F_st[RB][y] = robot.Kvx*(robot.dtoe[RB].y - robot.vyd) - F_clock - F_M_z;
  robot.F_st[LB][y] = robot.Kvx*(robot.dtoe[LB].y - robot.vyd) - F_clock - F_M_z;
  for(legNameTypeDef legId = 0; legId <= 3; legId++ )
    robot.yT[legId] = robot.vy*robot.Ts/2.0 + 0.01*(robot.vx - robot.vxd);
  for(legNameTypeDef legId = 0; legId <= 3; legId++ )
  {
    robot.F_st[legId][x] = robot.Kvx*(robot.dtoe[legId].x - robot.vxd + robot.dvx) - d_M_x/2;
    robot.F_st[legId][z] = robot.Kh*(robot.toe[legId].z - robot.z_st_exp[legId]) + robot.Kdh*(robot.dtoe[legId].z -0)+ d_M_z/2;
  }
  //-------------------------------------------------------摆动相
  //求落足点
  for(legNameTypeDef legId = 0; legId <= 3; legId++ )
  {
    robot.xT[legId] = robot.vx*robot.Ts/2.0 + 0.01*(robot.vx - robot.vxd) - d_x;
    robot.F_sw[legId][x] = -(robot.kx*(robot.toe[legId].x - robot.toed[legId].x) + robot.kdx*(robot.dtoe[legId].x - robot.dtoed[legId].x));
    robot.F_sw[legId][y] = -(robot.kx*(robot.toe[legId].y - robot.toed[legId].y) + robot.kdx*(robot.dtoe[legId].y - robot.dtoed[legId].y));
    robot.F_sw[legId][z] = -(robot.kz1*(robot.toe[legId].z -robot.toed[legId].z) + robot.kdz*(robot.dtoe[legId].z - robot.dtoed[legId].z));
  }
}
/*
机器人控制
*/
void robot_control()
{
  robot_trot_control();
  robot_trot();
  //robot_static();
}

  