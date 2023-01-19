//�������̽���C���Գ�������Ԫ����������̬����
#include "include.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "LQ_MPU6050_DMP.h"

#define PI      3.1415926               //Pi
#define w_ie    7.29212e-5              //����ÿ��(������ת�ٶ�)
#define g       9.78                    //�������ٶ�
#define R       6371000                 //����ƽ���뾶6371Km
#define RAD     PI/180                  // Pi/180
#define D       180/PI                  //180/Pi
#define T       0.01                    //0.01

void solve_init(void);                  //�ٶȵĳ�ʼֵV_ins[]��Ϊ0
void IMU_sample(void);                  //��ȡ�����Ǽ��ٶ�ֵ
void calw_nie(void);                    //����w_nie
void calw_nen(void);                    //����w_nen
void calw_nin(void);                    //����w_nin
void calc_nb(void);                     //����̬�����ת��
void calw_bin(void);                    //����w_bin

void calw_bnb(volatile float w_bnb[3],volatile float w_bib[3]);  
void updateQ(void);
void updatec_bn(void);                  //�ø��º����Ԫ��������̬����
void cal_angl(void);                    //���������Ƕ�
void convacce(void);                    //�����ٶȼƵĲ���ֵͶӰ������ϵ
void updateV(void);                     //ͨ�����������󵼺�ϵ�µ��ٶ�
void initC(void);
void updateC(void);
void calPOS(void);                      // ����λ��
void calvelocity(void);                 // ��ˮƽ�ٶȣ�����������ٶȵ�ƽ���Ϳ�����
void jiaozheng(void);



 float w_nie[3],w_nen[3],w_nin[3],w_bin[3];
 double L_ins,lamda_ins;
 float h_ins,v_ins[3],angl[3],v_ins_r[3];       //�ٶȳ�ʼֵ
 float w_bib1[3],w_bib2[3],w_bib3[3];           //���������ֵ3��Kʱ�� 2:kʱ�̺�k+1ʱ�����ݵ��м�ֵ
 float w_bnb1[3],w_bnb2[3],w_bnb3[3];
 float c_bn[3][3],c_nb[3][3];
 float q[4];
 float pitch_ins,roll_ins,yaw_ins;
 float f_b[3],f_n[3];                           //b:��ǰʱ�̼��ٶȼƵ����
 float cc[3][3];
 float POS[3];
 float v;
 float v_ins1[3];//����λ�ø���
float tempfhi_cbn[3][3];
float mulitc_bn[3][3];
int I=0, smileP0=0;

extern volatile float gx,gy,gz,ax,ay,az;
//extern volatile double X[4][1];
extern float pitch0,roll0,yaw0;
//extern long smile;
int kalman_flag = 0;
//3946.99716N, 11629.32198E
float L_GPS = 39.7832, lamda_GPS = 116.4887 ,H_GPS = 39.7832;
int jiaozhen = 1;

void cuduizhun(void) //��ʼ��׼����ģ�����������������̬����c_bn�ĳ�ʼֵ
{
  //fai = 
 c_bn[0][0]=cos(pitch0)*cos(yaw0); //������
 c_bn[0][1]=sin(pitch0)*cos(yaw0)*sin(roll0)-sin(yaw0)*cos(roll0);
 c_bn[0][2]=sin(pitch0)*cos(yaw0)*cos(roll0)+sin(yaw0)*sin(roll0);
 c_bn[1][0]=cos(pitch0)*sin(yaw0);
 c_bn[1][1]=sin(pitch0)*sin(yaw0)*sin(roll0)+cos(yaw0)*cos(roll0);
 c_bn[1][2]=sin(pitch0)*sin(yaw0)*cos(roll0)-cos(yaw0)*sin(roll0);
 c_bn[2][0]=-sin(pitch0);
 c_bn[2][1]=cos(pitch0)*sin(roll0);
 c_bn[2][2]=cos(pitch0)*cos(roll0);
 
 q[0]=sqrt(fabs(1+c_bn[0][0]+c_bn[1][1]+c_bn[2][2]))/2; //���ʼ��Ԫ��
  
 if((c_bn[2][1]-c_bn[1][2])>=0)
   q[1]=sqrt(fabs(1+c_bn[0][0]-c_bn[1][1]-c_bn[2][2]))/2;
  else
   q[1]=-sqrt(fabs(1+c_bn[0][0]-c_bn[1][1]-c_bn[2][2]))/2;

 if((c_bn[0][2]-c_bn[2][0])>=0)
   q[2]=sqrt(fabs(1-c_bn[0][0]+c_bn[1][1]-c_bn[2][2]))/2;
  else
   q[2]=-sqrt(fabs(1-c_bn[0][0]+c_bn[1][1]-c_bn[2][2]))/2;

 if((c_bn[1][0]-c_bn[0][1])>=0)
   q[3]=sqrt(fabs(1-c_bn[0][0]-c_bn[1][1]+c_bn[2][2]))/2;
  else
   q[3]=-sqrt(fabs(1-c_bn[0][0]-c_bn[1][1]+c_bn[2][2]))/2; 


  q[0]=q[0]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));   
  q[1]=q[1]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));
  q[2]=q[2]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));
  q[3]=q[3]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])); 

}

extern short   gyro[3], accel[3], sensors;

void test(void)
{
  char  txt[10];
  
  while(1)
  {
    LQ_DMP_Read(); 
    gx = (float)gyro[0]/16.4;
    gy = (float)gyro[1]/16.4;
    gz = (float)gyro[2]/16.4;
    ax = (float)accel[0]/2048*9.8;
    ay = (float)accel[1]/2048*9.8;
    az = (float)accel[2]/2048*9.8;
    pitch0 = (float)Pitch;
    roll0 = (float)Roll;
    yaw0 = (float)Yaw;
    if(I==0) //��һ�������ʼ��������λ�ú��ٶȵĳ�ֵ
    {
      solve_init(); //�����ʼ��
      I=1;
    }
    IMU_sample();         //��ȡ�����Ǻͼ��ٶȼƵ�����
    cuduizhun();          //�����������������̬������Ԫ��
    calw_nie();           //����w_nie
    calw_nen();           //����w_nen
    calw_nin();           //����w_nin
    calc_nb();            //������̬�����c_bn��ת�þ���c_nb
    calw_bin();           //����w_bin
    calw_bnb(w_bnb1,w_bib1); //����kʱ��w_bnb[]��ֵ����w_bnb1[]��ʾ
    calw_bnb(w_bnb2,w_bib2); //����k+0.5ʱ��w_bnb[]��ֵ����w_bnb2[]��ʾ
    calw_bnb(w_bnb3,w_bib3); //����k+1ʱ��w_bnb[]��ֵ,��w_bnb3[]��ʾ
    updateQ();            //������Ԫ��
    updatec_bn();         //������̬����
    convacce();           //���������µļ��ٶ�ֵת�������� ϵ��
    updateV();            //�����ٶ�
    cal_angl();           //����Ƕ�
    calPOS();             //����λ��
    calvelocity();        //����ˮƽ�ٶ�
    
    w_bib1[0]=gx;         //��¼kʱ�����ݵ��������k+1ʱ�̳���ʹ��
    w_bib1[1]=gy;
    w_bib1[2]=gz;
    
    sprintf((char*)txt,"pitch:%f",pitch_ins);
    TFTSPI_P8X16Str(0,4,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"roll:%f",roll_ins);
    TFTSPI_P8X16Str(0,5,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"yaw:%f",yaw_ins);
    TFTSPI_P8X16Str(0,6,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"vins:%f",v_ins[0]);
    TFTSPI_P8X16Str(0,7,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"vins:%f",v_ins[1]);
    TFTSPI_P8X16Str(0,8,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"vins:%f",v_ins[2]);
    TFTSPI_P8X16Str(0,9,txt,u16RED,u16BLACK);
    
    sprintf((char*)txt,"Pitch0:%f", pitch0);
    TFTSPI_P8X16Str(0,0,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"Roll:%f", roll0);
    TFTSPI_P8X16Str(0,1,txt,u16RED,u16BLACK);

    sprintf((char*)txt,"YAW:%f", yaw0);
    TFTSPI_P8X16Str(0,2,txt,u16RED,u16BLACK);
  }
}

void solve(void) //����������
{ 
  char  txt[10];
  if(I==0) //��һ�������ʼ��������λ�ú��ٶȵĳ�ֵ
  {
    solve_init(); //�����ʼ��
    I=1;
  }
  IMU_sample();  //��ȡ�����Ǻͼ��ٶȼƵ�����
//  cuduizhun();
  calw_nie();  //����w_nie
  calw_nen();  //����w_nen
  calw_nin();  //����w_nin
  calc_nb();   //������̬�����c_bn��ת�þ���c_nb
  calw_bin();  //����w_bin
  calw_bnb(w_bnb1,w_bib1); //����kʱ��w_bnb[]��ֵ����w_bnb1[]��ʾ
  calw_bnb(w_bnb2,w_bib2); //����k+0.5ʱ��w_bnb[]��ֵ����w_bnb2[]��ʾ
  calw_bnb(w_bnb3,w_bib3); //����k+1ʱ��w_bnb[]��ֵ,��w_bnb3[]��ʾ
  updateQ();  //������Ԫ��
  updatec_bn();  //������̬����
  convacce();  //���������µļ��ٶ�ֵת�������� ϵ��
  updateV();  //�����ٶ�
  cal_angl();  //����Ƕ�
  calPOS();  //����λ��
  calvelocity();  //����ˮƽ�ٶ�
  
  w_bib1[0]=gx; //��¼kʱ�����ݵ��������k+1ʱ�̳���ʹ��
  w_bib1[1]=gy;
  w_bib1[2]=gz;
  
  sprintf((char*)txt,"pitch:%f",pitch_ins);
  TFTSPI_P8X16Str(0,4,txt,u16RED,u16BLACK);

  sprintf((char*)txt,"roll:%f",roll_ins);
  TFTSPI_P8X16Str(0,5,txt,u16RED,u16BLACK);

  sprintf((char*)txt,"yaw:%f",yaw_ins);
  TFTSPI_P8X16Str(0,6,txt,u16RED,u16BLACK);

  sprintf((char*)txt,"POS:%f",POS[0]);
  TFTSPI_P8X16Str(0,7,txt,u16RED,u16BLACK);

  sprintf((char*)txt,"POS%f",POS[1]);
  TFTSPI_P8X16Str(0,8,txt,u16RED,u16BLACK);

  sprintf((char*)txt,"v:%f",v);
  TFTSPI_P8X16Str(0,9,txt,u16RED,u16BLACK);

}

void solve_init()
{
 v_ins[0]=0;  //�ٶȵĳ�ʼֵ��Ϊ0
 v_ins[1]=0;
 v_ins[2]=0;

 h_ins=H_GPS;

 L_ins=L_GPS*3.14/180;
 lamda_ins=lamda_GPS*3.14/180;

 w_bib1[0]=gx; 
 w_bib1[1]=gy;
 w_bib1[2]=gz;

 v_ins1[0]=v_ins[0];
 v_ins1[1]=v_ins[1];
 v_ins1[2]=v_ins[2];
}

void IMU_sample()
{
  w_bib3[0]=gx;  //��ǰʱ�̵��������
  w_bib3[1]=gy;
  w_bib3[2]=gz;  

  w_bib2[0]=(w_bib1[0]+w_bib3[0])/2;  //kʱ�̺�k+1ʱ�����ݵ��м�ֵ
  w_bib2[1]=(w_bib1[1]+w_bib3[1])/2;
  w_bib2[2]=(w_bib1[2]+w_bib3[2])/2;

  f_b[0]=ax;  //��ǰʱ�̼��ٶȼƵ����
  f_b[1]=ay;
  f_b[2]=az;
}

void calw_nie() //����w_nie
{
  w_nie[0]=w_ie*cos(L_GPS*3.14/180);
  w_nie[1]=0;
  w_nie[2]=w_ie*sin(L_GPS*3.14/180);
}

void calw_nen() //����w_nen
{
  w_nen[0]=-v_ins[1]/R;
  w_nen[1]=v_ins[0]/R;
  w_nen[2]=-v_ins[1]*tan(L_GPS)/R;
}

void calw_nin() //����w_nin
{
  w_nin[0]=w_nen[0]+w_nie[0];
  w_nin[1]=w_nen[1]+w_nie[1];
  w_nin[2]=w_nen[2]+w_nie[2];
}

void calc_nb() //����̬�����ת��
{
  int i,j;

  for(i=0;i<3;i++)
   for(j=0;j<3;j++)
  c_nb[i][j]=c_bn[j][i];
}

void calw_bin()   //����w_bin
{
  w_bin[0]=c_nb[0][0]*w_nin[0]+ c_nb[0][1]*w_nin[1]+c_nb[0][2]*w_nin[2];
  w_bin[1]=c_nb[1][0]*w_nin[0]+ c_nb[1][1]*w_nin[1]+c_nb[1][2]*w_nin[2];
  w_bin[2]=c_nb[2][0]*w_nin[0]+ c_nb[2][1]*w_nin[1]+c_nb[2][2]*w_nin[2];
}

void calw_bnb(volatile float w_bnb[3],volatile float w_bib[3])
{
  int i;
  for(i=0;i<3;i++)
  {
    w_bnb[i]=w_bib[i]-w_bin[i];
  }
}
void updateQ()   
{
  float k1[4],k2[4],k3[4],k4[4];
  float A[4],B[4],C[4];

  k1[0]=(-w_bnb1[0]*q[1]-w_bnb1[1]*q[2]-w_bnb1[2]*q[3])/2;  
  k1[1]=(w_bnb1[0]*q[0]+w_bnb1[2]*q[2]-w_bnb1[1]*q[3])/2;
  k1[2]=(w_bnb1[1]*q[0]-w_bnb1[2]*q[1]+w_bnb1[0]*q[3])/2;
  k1[3]=(w_bnb1[2]*q[0]+w_bnb1[1]*q[1]-w_bnb1[0]*q[2])/2;

  A[0]=q[0]+T*k1[0]/2;
  A[1]=q[1]+T*k1[1]/2;
  A[2]=q[2]+T*k1[2]/2;
  A[3]=q[3]+T*k1[3]/2;

  k2[0]=(-w_bnb2[0]*A[1]-w_bnb2[1]*A[2]-w_bnb2[2]*A[3])/2;
  k2[1]=(w_bnb2[0]*A[0]+w_bnb2[2]*A[2]-w_bnb2[1]*A[3])/2;
  k2[2]=(w_bnb2[1]*A[0]-w_bnb2[2]*A[1]+w_bnb2[0]*A[3])/2;
  k2[3]=(w_bnb2[2]*A[0]+w_bnb2[1]*A[1]-w_bnb2[0]*A[2])/2;

  B[0]=q[0]+T*k2[0]/2;
  B[1]=q[1]+T*k2[1]/2;
  B[2]=q[2]+T*k2[2]/2;
  B[3]=q[3]+T*k2[3]/2;

  k3[0]=(-w_bnb2[0]*B[1]-w_bnb2[1]*B[2]-w_bnb2[2]*B[3])/2;
  k3[1]=(w_bnb2[0]*B[0]+w_bnb2[2]*B[2]-w_bnb2[1]*B[3])/2;
  k3[2]=(w_bnb2[1]*B[0]-w_bnb2[2]*B[1]+w_bnb2[0]*B[3])/2;
  k3[3]=(w_bnb2[2]*B[0]+w_bnb2[1]*B[1]-w_bnb2[0]*B[2])/2;

  C[0]=q[0]+T*k3[0];
  C[1]=q[1]+T*k3[1];
  C[2]=q[2]+T*k3[2];
  C[3]=q[3]+T*k3[3];

  k4[0]=(-w_bnb3[0]*C[1]-w_bnb3[1]*C[2]-w_bnb3[2]*C[3])/2;
  k4[1]=(w_bnb3[0]*C[0]+w_bnb3[2]*C[2]-w_bnb3[1]*C[3])/2;
  k4[2]=(w_bnb3[1]*C[0]-w_bnb3[2]*C[1]+w_bnb3[0]*C[3])/2;
  k4[3]=(w_bnb3[2]*C[0]+w_bnb3[1]*C[1]-w_bnb3[0]*C[2])/2;

  q[0]=q[0]+T*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6; //��Ԫ������
  q[1]=q[1]+T*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6;
  q[2]=q[2]+T*(k1[2]+2*k2[2]+2*k3[2]+k4[2])/6;
  q[3]=q[3]+T*(k1[3]+2*k2[3]+2*k3[3]+k4[3])/6;

  q[0]=q[0]/sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]); //��һ��
  q[1]=q[1]/sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[2]=q[2]/sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[3]=q[3]/sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);


}

void updatec_bn() //�ø��º����Ԫ��������̬����
{
 
  //float tempfhi_cbn[3][3];
  //float mulitc_bn[3][3];
  if(jiaozhen==1)
  {
   c_bn[0][0]=cos(pitch0)*cos(yaw0); //������
   c_bn[0][1]=sin(pitch0)*cos(yaw0)*sin(roll0)-sin(yaw0)*cos(roll0);
   c_bn[0][2]=sin(pitch0)*cos(yaw0)*cos(roll0)+sin(yaw0)*sin(roll0);
   c_bn[1][0]=cos(pitch0)*sin(yaw0);
   c_bn[1][1]=sin(pitch0)*sin(yaw0)*sin(roll0)+cos(yaw0)*cos(roll0);
   c_bn[1][2]=sin(pitch0)*sin(yaw0)*cos(roll0)-cos(yaw0)*sin(roll0);
   c_bn[2][0]=-sin(pitch0);
   c_bn[2][1]=cos(pitch0)*sin(roll0);
   c_bn[2][2]=cos(pitch0)*cos(roll0);

  //ͬʱУ׼��Ԫ�����Ա�������У��������¿�������ģ��ĳ�ʼֵ����Ƕ�

  q[0]=sqrt(fabs(1+c_bn[0][0]+c_bn[1][1]+c_bn[2][2]))/2;

  if((c_bn[2][1]-c_bn[1][2])>=0)
   q[1]=sqrt(fabs(1+c_bn[0][0]-c_bn[1][1]-c_bn[2][2]))/2;
  else
   q[1]=-sqrt(fabs(1+c_bn[0][0]-c_bn[1][1]-c_bn[2][2]))/2;

  if((c_bn[0][2]-c_bn[2][0])>=0)
   q[2]=sqrt(fabs(1-c_bn[0][0]+c_bn[1][1]-c_bn[2][2]))/2;
  else
   q[2]=-sqrt(fabs(1-c_bn[0][0]+c_bn[1][1]-c_bn[2][2]))/2;

  if((c_bn[1][0]-c_bn[0][1])>=0)
   q[3]=sqrt(fabs(1-c_bn[0][0]-c_bn[1][1]+c_bn[2][2]))/2;
  else
   q[3]=-sqrt(fabs(1-c_bn[0][0]-c_bn[1][1]+c_bn[2][2]))/2; 


  q[0]=q[0]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));   
  q[1]=q[1]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));
  q[2]=q[2]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));
  q[3]=q[3]/(sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));

 }

 else
 {
   c_bn[0][0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
   c_bn[0][1]=2*(q[1]*q[2]-q[0]*q[3]);
   c_bn[0][2]=2*(q[1]*q[3]+q[0]*q[2]);

   c_bn[1][0]=2*(q[1]*q[2]+q[0]*q[3]);
   c_bn[1][1]=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
   c_bn[1][2]=2*(q[2]*q[3]-q[0]*q[1]);

   c_bn[2][0]=2*(q[1]*q[3]-q[0]*q[2]);
   c_bn[2][1]=2*(q[2]*q[3]+q[1]*q[0]);
   c_bn[2][2]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];

  if(smileP0==0)  //500��ѭ�����ڣ�һ������10ms��������ģ��У׼��̬���󣬼�ÿ5sУ׼һ��
  {
    cuduizhun();  //������Ա����飬����ҪУ׼�����������ע�͵�
   }
 }
}

void convacce() //�����ٶȼƵĲ���ֵͶӰ������ϵ
{
  f_n[0]=-(c_bn[0][0]*f_b[0]+c_bn[0][1]*f_b[1]+c_bn[0][2]*f_b[2]);
  f_n[1]=-(c_bn[1][0]*f_b[0]+c_bn[1][1]*f_b[1]+c_bn[1][2]*f_b[2]);
  f_n[2]=c_bn[2][0]*f_b[0]+c_bn[2][1]*f_b[1]+c_bn[2][2]*f_b[2];
}

void updateV() //ͨ�����������󵼺�ϵ�µ��ٶ�
{
  volatile float ww[3][3];
  volatile float wwv[3];
  volatile float dv[3];
  int i,j;

  ww[0][0]=0;
  ww[0][1]=-(2*w_nie[2]+w_nen[2]);
  ww[0][2]=2*w_nie[1]+w_nen[1];
  ww[1][0]=2*w_nie[2]+w_nen[2];
  ww[1][1]=0;
  ww[1][2]=-(2*w_nie[0]+w_nen[0]);
  ww[2][0]=-(2*w_nie[1]+w_nen[1]);
  ww[2][1]=2*w_nie[0]+w_nen[0];
  ww[2][2]=0;

  for(i=0;i<3;i++)
   wwv[i]=0;

  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      wwv[i]=wwv[i]+ww[i][j]*v_ins[j];

  wwv[0]=wwv[0]+0;
  wwv[1]=wwv[1]+0;
  wwv[2]=wwv[2]-g;

  dv[0]=f_n[0]-wwv[0];
  dv[1]=f_n[1]-wwv[1];
  dv[2]=f_n[2]-wwv[2];

               
  for(i=0;i<3;i++)    //һ��ŷ�������ٶ�΢�ַ���
  {
    dv[i]=dv[i]*T;
    v_ins[i]=v_ins[i]+dv[i];
  }
    //v_ins[0]=0.01;
  //v_ins[1]=0.01;

//  if(kalman_flag==1) // �������˲�У���ٶ�
//  {
//    v_ins[0]-=X[0][0];
//    v_ins[1]-=X[1][0];
//    // v_ins[2]-=X[2];
//  }

}

void cal_angl() //���������Ƕ�
{
  if(jiaozhen==1)
  {
    pitch_ins=pitch0;
    roll_ins=roll0;
    yaw_ins=yaw0;
  }
  else
  {
      pitch_ins=asin(-c_bn[2][0]);

      roll_ins=atan(c_bn[2][1]/c_bn[2][2]);

      yaw_ins=atan(c_bn[1][0]/c_bn[0][0]);

      if(c_bn[2][2]==0)
         {
           if(c_bn[2][1]>0)
         roll_ins=PI/2;
       if(c_bn[2][1]<0)
         roll_ins=-PI/2;
         }
      if(c_bn[2][2]>0)
         {
          if(c_bn[2][1]>0)
         roll_ins=roll_ins;
       if(c_bn[2][1]<0)
         roll_ins=roll_ins;
         }
      if(c_bn[2][2]<0)
         {
          if(c_bn[2][1]>0)
         roll_ins=roll_ins+PI;
       if(c_bn[2][1]<0)
         roll_ins=roll_ins-PI;
         }

      if(c_bn[0][0]==0)
         {
          if(c_bn[1][0]>0)
         yaw_ins=PI/2;
       if(c_bn[1][0]<0)
         yaw_ins=3*PI/2;
         }
     if(c_bn[0][0]>0)
         {
          if(c_bn[1][0]>0)
          yaw_ins=yaw_ins;
       if(c_bn[1][0]<0)
          yaw_ins=yaw_ins+2*PI;
         }
     if(c_bn[0][0]<0)
         {
          if(c_bn[1][0]>0)
         yaw_ins+=PI;
       if(c_bn[1][0]<0)
         yaw_ins+=PI;
         }
  }
  pitch_ins=pitch_ins*180/3.14; //������ת���ɶ���
  roll_ins=roll_ins*180/3.14;
  yaw_ins=yaw_ins*180/3.14;
}

void calPOS()   //����λ��
{  // v_ins[0]=5;
            // v_ins[1]=5;

  L_ins=L_ins+T*v_ins[0]/R;
  lamda_ins=lamda_ins-T*v_ins[1]/((R+H_GPS)*cos(L_GPS*3.14/180));  

  h_ins+=T*v_ins[2];
 

//if(kalman_flag==1) //�������˲�У��λ��
//{
//    L_ins-=X[2][0];
//   lamda_ins-=X[3][0];
  // h_ins-=X[5];
//}
 
  POS[0]=L_ins*3.14/180;  //������ת���ɶ�
  POS[1]=lamda_ins*3.14/180;
  POS[2]=0;
 
}

void calvelocity() // ��ˮƽ�ٶȣ�����������ٶȵ�ƽ���Ϳ�����
{
 v=sqrt(v_ins[0]*v_ins[0]+v_ins[1]*v_ins[1]);
} 