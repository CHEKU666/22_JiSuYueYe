/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����    ����GPS����ģ��GT-U12
����    д������Ƽ�
��E-mail  ��chiusir@163.com
������汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2021��12��16�գ��������£����ע���°棡
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��IDE��IAR7.8 KEIL5.24�����ϰ汾
��Target �� MM32F3277
��SYS PLL�� 120MHz Ƶ��̫�߿����޷�����system_mm32f327x.c
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/


#ifndef __LQ_GPS_BD_H
#define __LQ_GPS_BD_H

typedef struct SaveData 
{
  char GPS_Buffer[128];         //��������
  char isGetData;		//�Ƿ��ȡ��GPS����
  char isParseData;	        //�Ƿ�������
  char UTCTime[11];		//UTCʱ��
  char latitude[11];		//γ��
  char N_S[2];		        //N/S
  char longitude[12];		//����
  char E_W[2];		        //E/W
  char isUsefull;		//��λ��Ϣ�Ƿ���Ч
} _SaveData;

typedef struct UseData  //ʵ�ʵĲ�ֵ
{
  int lat_err;          //γ�Ȳ�
  int lon_err;          //���Ȳ�
  int lat_centre;       //�����������ĵ�γ��
  int lon_centre;       //�����������ĵ㾭��
  float radius;         //Բ���İ뾶
  float distan_next;    //����һ��Ŀ���ľ���
  float distan_centre;  //��Բ�ĵľ���
  float lat_cent_err;   //�����ĵ�γ�Ȳ�
  float lon_cent_err;   //�����ĵľ��Ȳ�
  float lat_next_err;   //��һ�����γ�Ȳ�
  float lon_next_err;   //��һ����ľ��Ȳ�

} _UseData;

extern _SaveData Save_Data;


void parseGpsBuffer(void);
void Data_Average(int *D1, int* D2);
void Gps_test(void);


#endif









