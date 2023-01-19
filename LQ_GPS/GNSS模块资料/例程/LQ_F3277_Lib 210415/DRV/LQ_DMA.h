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

#ifndef _LQ_DMA_H_
#define _LQ_DMA_H_
#include "include.h"
#include "hal_dma.h"
#include "hal_rcc.h"
#include "hal_tim.h"

void DMA_CameraInitConfig(DMA_Channel_TypeDef* dma_ch,u32 src_addr, u32 des_addr, u32 size);
void DMA_CameraTriggerTimerInit(TIMER_Name_t timern, GPIO_Name_t pin);
void DMA_CameraTriggerTimer1PE7Init();
#endif /* 0_APPSW_TRICORE_APP_LQ_GPIO_H_ */
