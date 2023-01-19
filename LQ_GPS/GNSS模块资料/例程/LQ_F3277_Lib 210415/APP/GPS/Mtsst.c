#include "include.h"
extern short   gyro[3], accel[3], sensors;
extern float   Pitch, Roll; 
extern void solve(void);

volatile float F_ib[3], W_ib[3];        //加速度（载体(b)相对惯性空间(i)的运动加速度在b系的投影）
float C_bt[3][3];
volatile float gx,gy,gz,ax,ay,az;
float pitch0,roll0,yaw0;
float   Pitch2=0.00001;
float   *Pitch3;

void chushijiaozhun(float ya, float rol, float pit)//Z周角， Y轴角 ， X轴角
{
  C_bt[0][0] = cos(ya)*cos(rol);
  C_bt[0][1] = -cos(ya)*sin(rol)*sin(pit) - sin(ya)*cos(pit);
  C_bt[0][2] = -cos(ya)*sin(rol)*cos(pit) + sin(ya)*sin(pit);
  C_bt[1][0] = sin(ya)*cos(rol);
  C_bt[1][1] = -sin(ya)*sin(rol)*sin(pit) + cos(ya)*cos(pit);
  C_bt[1][2] = -sin(ya)*sin(rol)*cos(pit) - cos(ya)*sin(pit);
  C_bt[2][0] = sin(rol);
  C_bt[2][1] = sin(rol)*sin(pit);
  C_bt[2][2] = sin(rol)*cos(pit);
}

void B_Tzubiao(float ax, float ay, float az)
{
  
}
void Test_MY(void)
{
  char  txt[10];
  
  TFTSPI_Init(1);        //LCD初始化  0:横屏  1：竖屏
  SOFT_IIC_Init();  
  LQ_DMP_Init();
  delayms(100);
  while(1)
  {       
    LQ_DMP_Read(); 
    gx = (float)gyro[0];
    gy = (float)gyro[1];
    gz = (float)gyro[2];
    ax = (float)accel[0];
    ay = (float)accel[1];
    az = (float)accel[2];
    pitch0 = (float)Pitch;
    roll0 = (float)Roll;
    yaw0 = (float)Yaw;
//    solve();
     
    sprintf((char*)txt,"Pitch0:%f", pitch0);
    TFTSPI_P8X16Str(0,0,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"Roll:%f", roll0);
    TFTSPI_P8X16Str(0,1,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"YAW:%f", yaw0);
    TFTSPI_P8X16Str(0,2,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"ax:%06d",accel[0]);
    TFTSPI_P8X16Str(0,4,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"ay:%06d",accel[1]);
    TFTSPI_P8X16Str(0,5,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"az:%06d",accel[2]);
    TFTSPI_P8X16Str(0,6,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"gx:%06d",gyro[0]);
    TFTSPI_P8X16Str(0,7,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"gy:%06d",gyro[1]);
    TFTSPI_P8X16Str(0,8,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"gz:%06d",gyro[2]);
    TFTSPI_P8X16Str(0,9,txt,u16RED,u16BLACK);  
    
    //上位机
   // ANO_DT_send_int16((short)Pitch, (short)Roll, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
  }
}