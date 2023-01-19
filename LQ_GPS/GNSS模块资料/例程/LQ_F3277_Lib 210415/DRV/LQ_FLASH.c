/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技F3277核心板和母板
【编    写】龙邱科技
【E-mail  】chiusir@163.com
【软件版本】V1.0 版权所有，单位使用请先联系授权
【最后更新】2020年12月24日，持续更新，请关注最新版！
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【IDE】IAR7.8 KEIL5.24及以上版本
【Target 】 MM32F3277
【SYS PLL】 120MHz 频率太高可能无法启动system_mm32f327x.c
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "HAL_conf.h"
#include "HAL_device.h"
#include "HAL_flash.h"
#include "LQ_FLASH.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】u16 LQFLASH_ReadByte(u32 faddr)
【功  能】读取指定地址的半字(16位数据)
【参数值】faddr:读地址(此地址必须为2的倍数!!)
【返回值】对应地址的数据.
【作  者】L Q
【最后更新】2021年1月22日 
【软件版本】V1.0
【调用样例】buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
u16 LQFLASH_ReadByte(u32 faddr)
{
  return *(vu16*)faddr; 
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】void LQFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
【功  能】不检查的写入(16位数据)
【参数值】riteAddr:起始地址
【参数值】pBuffer:数据指针
【参数值】NumToWrite:半字(16位)数   
【返回值】无
【作  者】L Q
【最后更新】2021年1月22日 
【软件版本】V1.0
【调用样例】buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
  u16 i;
  for(i=0;i<NumToWrite;i++)
  {
    FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
    WriteAddr+=2;//地址增加2.
  }  
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】void LQFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
【功  能】从指定地址开始写入指定长度的数据
【参数值】WriteAddr:起始地址(此地址必须为2的倍数!!)
【参数值】pBuffer:数据指针
【参数值】NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
【返回值】无
【作  者】L Q
【最后更新】2021年1月22日 
【软件版本】V1.0
【调用样例】buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#if LQ_FLASH_SIZE<256
#define LQ_SECTOR_SIZE 1024 //字节
#else 
#define LQ_SECTOR_SIZE	2048
#endif		 
u16 LQFLASH_BUF[LQ_SECTOR_SIZE/2];//最多是2K字节

void LQFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
  u32 secpos;	   //扇区地址
  u16 secoff;	   //扇区内偏移地址(16位字计算)
  u16 secremain;   //扇区内剩余地址(16位字计算)	   
  u16 i;    
  u32 offaddr;     //去掉0X08000000后的地址
  if(WriteAddr<FLASH_BASE_ADDR||(WriteAddr>=(FLASH_BASE_ADDR+1024*FLASH_SIZE)))return;//非法地址
  FLASH_Unlock();						//解锁
  offaddr=WriteAddr-FLASH_BASE_ADDR;		//实际偏移地址.
  secpos=offaddr/LQ_SECTOR_SIZE;			//扇区地址  0~127 for LQF103RBT6
  secoff=(offaddr%LQ_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
  secremain=LQ_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小   
  if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
  while(1) 
  {	
    LQFLASH_Read(secpos*LQ_SECTOR_SIZE+FLASH_BASE_ADDR,LQFLASH_BUF,LQ_SECTOR_SIZE/2);//读出整个扇区的内容
    for(i=0;i<secremain;i++)//校验数据
    {
      if(LQFLASH_BUF[secoff+i]!=0XFFFF) break;//需要擦除  	  
    }
    if(i<secremain)//需要擦除
    {
      FLASH_ErasePage(secpos*LQ_SECTOR_SIZE+FLASH_BASE_ADDR);//擦除这个扇区
      for(i=0;i<secremain;i++)//复制
      {
        LQFLASH_BUF[i+secoff]=pBuffer[i];	  
      }
      LQFLASH_Write_NoCheck(secpos*LQ_SECTOR_SIZE+FLASH_BASE_ADDR,LQFLASH_BUF,LQ_SECTOR_SIZE/2);//写入整个扇区  
    }else LQFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
    if(NumToWrite==secremain)break;//写入结束了
    else//写入未结束
    {
      secpos++;				//扇区地址增1
      secoff=0;				//偏移位置为0 	 
      pBuffer+=secremain;  	//指针偏移
      WriteAddr+=secremain;	//写地址偏移	   
      NumToWrite-=secremain;	//字节(16位)数递减
      if(NumToWrite>(LQ_SECTOR_SIZE/2))secremain=LQ_SECTOR_SIZE/2;//下一个扇区还是写不完
      else secremain=NumToWrite;//下一个扇区可以写完了
    }	 
  };	
  FLASH_Lock();//上锁
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】void LQFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
【功  能】从指定地址开始读出指定长度的数据
【参数值】ReadAddr:起始地址
【参数值】pBuffer:数据指针
【参数值】NumToWrite:半字(16位)数
【返回值】无
【作  者】L Q
【最后更新】2021年1月22日 
【软件版本】V1.0
【调用样例】buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
  u16 i;
  for(i=0;i<NumToRead;i++)
  {
    pBuffer[i]=LQFLASH_ReadByte(ReadAddr);//读取2个字节.
    ReadAddr+=2;//偏移2个字节.	
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】void Test_EEPROM(void)
【功  能】测试FLASH读写，通过按键进行操作
【参数值】无
【返回值】无
【作  者】L Q
【最后更新】2021年1月22日 
【软件版本】V1.0
【调用样例】
---------------------------------------------------------------
//按下母板上KEY0按键，写入当前变量到指定地址
//按下母板上KEY2按键，读取指定地址数据到屏幕
//复位单片机后，如果仍然能读取到原来写入的数据，说明保存成功
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

u8 TEXT_Buffer[]={"000LQ FLASH TEST"};
#define SIZE sizeof(TEXT_Buffer)	 	//数组长度
#define FLASH_SAVE_ADDR  0X08070000 		//设置FLASH 保存地址(必须为偶数)
u8 datatemp[SIZE];

void Test_EEPROM(void)
{	
  u8 cnt=100;
  char txt[16];  
  
  TFTSPI_Init(0);        //LCD初始化  0:横屏  1：竖屏
  TFTSPI_CLS(u16BLUE);   //蓝色屏幕
  TFTSPI_Show_Logo(0,37);//显示龙邱LOGO
  TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);//字符串显示
  delayms(1000);         //延时等待
  TFTSPI_Init(1);        //LCD初始化  0:横屏  1：竖屏
  TFTSPI_CLS(u16BLUE);   //蓝色屏幕
  GPIO_KEY_Init();
  
  TFTSPI_P8X16Str(1,4,"WriteValue:",u16RED,u16BLUE);       //屏幕显示出来  
  LQFLASH_Read(FLASH_SAVE_ADDR,(u16*)datatemp,SIZE);   //读取指定地址数据到屏幕
  sprintf(txt,"Read Value:%03d",datatemp[0]);          //转换为字符串，以便屏幕显示出来
  TFTSPI_P8X16Str(1,3,txt,u16RED,u16BLUE);             //屏幕显示出读取的数据  
  
  while(1)
  {
    if(KEY_Read(KEY0)==0)                                  //按下母板上KEY0按键是否按下，
    {	
      TEXT_Buffer[0]=++cnt;                                //变动数值，以便观察读写是否成功
      sprintf(txt,"WriteValue:%03d",cnt);                  //转换为字符串，以便屏幕显示出来
      TFTSPI_P8X16Str(1,4,txt,u16RED,u16BLUE);             //屏幕显示出来
      
      LQFLASH_Write(FLASH_SAVE_ADDR,(u16*)TEXT_Buffer,SIZE);//写入当前变量到指定地址		
    }
    if(KEY_Read(KEY2)==0)                                  //按下母板上KEY2按键是否按下，
    {					
      LQFLASH_Read(FLASH_SAVE_ADDR,(u16*)datatemp,SIZE);   //读取指定地址数据到屏幕
      sprintf(txt,"Read Value:%03d",datatemp[0]);          //转换为字符串，以便屏幕显示出来
      TFTSPI_P8X16Str(1,3,txt,u16RED,u16BLUE);             //屏幕显示出读取的数据
    }   
    LED_Ctrl(LED0,RVS);                                    //LED闪烁
    delayms(1000);                                         //延时等待
  }
}


