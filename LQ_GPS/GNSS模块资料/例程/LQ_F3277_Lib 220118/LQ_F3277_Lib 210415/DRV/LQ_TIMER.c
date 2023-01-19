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

const u32 TIMERx[] = {TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, TIM6_BASE, TIM7_BASE, TIM8_BASE};
TIM_TypeDef *TIMERxP[8] = {TIM1,TIM2,TIM3,TIM4,TIM5,TIM6,TIM7,TIM8};

int16_t ECPULSE1 = 0;          // 速度全局变量
int16_t ECPULSE2 = 0;          // 速度全局变量
volatile int32_t RAllPulse = 0;// 速度全局变量
extern volatile uint8_t Game_Over;    // 小车完成全部任务，停车
extern volatile int16_t targetSpeed;

void TIM1_UP_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源  
  }  
}
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
    //用户程序    
    LED_Ctrl(LED1,RVS);//LED翻转闪烁
    
  }  
}
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
    //用户程序    
    LED_Ctrl(LED1,RVS);//LED翻转闪烁
    
  }  
}
void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
    //用户程序    
    LED_Ctrl(LED1,RVS);//LED翻转闪烁
    
  }  
}


void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
    //用户程序    
    LED_Ctrl(LED0,RVS);//LED翻转闪烁     
  }  
}

void TIM6_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
    //用户程序    
    LED_Ctrl(LED1,RVS);//LED翻转闪烁
    ECPULSE1 = Read_Encoder(3); //左电机 母板上编码器1，小车前进为负值
    ECPULSE2 = Read_Encoder(4); //右电机 母板上编码器2，小车前进为正值 
  }  
}
void TIM7_IRQHandler(void)
{
  long sta = TIM1->SR;			// 读取中断状态
  TIM1->SR &= ~sta;					// 清除中断状态
  //用户程序    
  LED_Ctrl(LED1,RVS);//LED翻转闪烁
 
  
}
void TIM8_UP_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
  {
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
    //用户程序    
    LED_Ctrl(LED1,RVS);//LED翻转闪烁
    
  }  
}

/*************************************************************************
*  函数名称：void TIMER_InitConfig(TIMER_Name_t timern, u16 nms)
*  功能说明：电机PWM初始化
*  参数说明：
//  @param      timern      定时器通道
//  @param      nms          定时周期pch,PWM通道所对应的的定时器及管脚

*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：TIMER_InitConfig(TIMER_1, 5); 使用定时器1作为5ms一次的周期中断
*************************************************************************/
void TIMER_InitConfig(TIMER_Name_t timern, u16 nms)
{  
  u32 freq_div = 0;                                          //分频值
  u32 tmperiod;                                             //周期值
  
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  if(TIMER_1 == timern)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);                //时钟使能
  else if(TIMER_2 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  else if(TIMER_3 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    
  else if(TIMER_4 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    
  else if(TIMER_5 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM5, ENABLE);
  else if(TIMER_6 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM6, ENABLE);    
  else if(TIMER_7 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM7, ENABLE);
  else if(TIMER_8 == timern)
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM8, ENABLE);
  
  freq_div = SystemCoreClock/10000;                                       //多少分频 ,设置为10K ,100us计数一次                        
  if(freq_div<1) freq_div=1;
  tmperiod = nms*10;                                                      //周期值 nms*10*100us=nms
  if(tmperiod<1) tmperiod=1;
  
  //定时器初始化
  TIM_TimeBaseStructure.TIM_Period = tmperiod-1;                          //设置自动重装载寄存器周期nms*10*100us=nms
  TIM_TimeBaseStructure.TIM_Prescaler = freq_div-1;                       //设置为100us计数一次
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                 //设置时钟Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;             //向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;                        //重复计数器设置为0
  TIM_TimeBaseInit((TIM_TypeDef *)TIMERx[timern], &TIM_TimeBaseStructure);//初始化TIMx时基
  
  TIM_ITConfig((TIM_TypeDef *)TIMERx[timern],TIM_IT_Update,ENABLE );      //使能TIM中断
  
  //中断优先级设置
  if(TIMER_1 == timern)   nvic_init(TIM1_UP_IRQn, 0, 2, ENABLE);
  else if(TIMER_2 == timern)   nvic_init(TIM2_IRQn, 0, 2, ENABLE);
  else if(TIMER_3 == timern)   nvic_init(TIM3_IRQn, 0, 2, ENABLE);
  else if(TIMER_4 == timern)   nvic_init(TIM4_IRQn, 0, 2, ENABLE);
  else if(TIMER_5 == timern)   nvic_init(TIM5_IRQn, 0, 2, ENABLE);
  else if(TIMER_6 == timern)   nvic_init(TIM6_IRQn, 0, 2, ENABLE);
  else if(TIMER_7 == timern)   nvic_init(TIM7_IRQn, 0, 2, ENABLE);
  else if(TIMER_8 == timern)   nvic_init(TIM8_UP_IRQn, 0, 2, ENABLE);
  
  TIM_Cmd((TIM_TypeDef *)TIMERx[timern], ENABLE);  //使能TIMx  
}



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】 void Test_Timer56(void)
【功  能】 测试定时中断和LED灯闪烁
【参数值】 无
【参数值】 无
【返回值】 无 
【作  者】 L Q
【最后更新】 2021年1月22日 
【软件版本】 V1.0
【调用样例】 Test_GPIO_LED()
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_TimerPIT56(void)
{
  GPIO_LED_Init();
  TIMER_InitConfig(TIMER_5, 50);
  TIMER_InitConfig(TIMER_6, 25);
  while(1)
  {
    LED_Ctrl(LED2,RVS);     //LED翻转闪烁
    delayms(50);
  }
}
