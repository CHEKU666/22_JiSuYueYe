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
-----------------------------------------------------------------
摄像头接口    龙邱神眼或者OV7725模块
● 数据端口：  PF0-PF7口，共8位，接摄像头的数据端口；
● 时钟像素：  外部中断PE7；
● 行信号：    外部中断PE8/如果用DMA+TIMER1:PE7
● 场信号：    外部中断PG1
● I2C：       PF8 I2C1-SCL
● I2C：       PF9 I2C1-SDA
-----------------------------------------------------------------
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"


// 图像原始数据存放 //
unsigned char Image_Data[IMAGEH][IMAGEW];

// 压缩后之后用于存放屏幕显示数据 
unsigned char Image_Use[LCDH][LCDW];

// 二值化后用于OLED显示的数据 
unsigned char Bin_Image[LCDH][LCDW];

unsigned char Camera_Flag=0;
int16_t OFFSET0 = 0;      //最远处，赛道中心值综合偏移量
int16_t OFFSET1 = 0;      //第二格
int16_t OFFSET2 = 0;      //最近，第三格
int16_t TXV = 0;          //梯形的左高度，右高度


/*************************************************************************
*  函数名称：void Test_CAMERA (void)
*  功能说明：摄像头测试例程
*  参数说明：void
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：注意需要使用  带与非门版（白色）转接座，或者新版摄像头模块
*************************************************************************/
void Test_CAMERA (void)
{
  
  TFTSPI_Init(0);               //TFT1.8初始化0:横屏  1：竖屏
  TFTSPI_CLS(u16BLUE);          //清屏
  
  // 摄像头初始化 //
  CAMERA_Init(50);
  
  while (1)
  {
    if (Camera_Flag == 1)
    {
      // 清除摄像头采集完成标志位  如果不清除，则不会再次采集数据 
      Camera_Flag = 0;       
      
      // 提取部分使用的数据
      Get_Use_Image();    
      
#if 1 //显示原始图像
      //TFT1.8动态显示摄像头图像
      TFTSPI_Road(0, 0, LCDH, LCDW, (unsigned char *)Image_Use);
      
#else //显示二值化图像
      
      // 二值化 //
      Get_Bin_Image(0);
      
      // 显示摄像头图像            
      TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);             
#endif
      
      //上报数据给上位机 串口速度比较慢 注意上位机图像宽高设置为120*188
      //CAMERA_Reprot();  //山外上位机显示  
      
      LED_Ctrl(LED0, RVS);
    }
    LED_Ctrl(LED2, RVS);
  }
}
/*************************************************************************
*  函数名称：void CAMERA_Reprot (void)
*  功能说明：串口上报上位机
*  参数说明：void
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：注意上位机的帧头可能有所区别
*************************************************************************/
void CAMERA_Reprot (void)
{
  short j, i;
  
//  UART_PutChar(UART1,0x01);  //山外上位机帧头0x01
//  UART_PutChar(UART1,0xFE);  //山外上位机帧头0xFE
  
  UART_PutChar(UART1,0xFE);  //开源上位机帧头0xef
  UART_PutChar(UART1,0xEF);  //开源上位机帧头0xfe
  
  for (i = 0; i < IMAGEH; i++)
  {
    for (j = 0; j < IMAGEW; j++)
    {
      if (Image_Data[i][j] == 0xfe)  //防止错误发送帧尾
      {
        Image_Data[i][j] = 0xff;
      }
      UART_PutChar(UART1,Image_Data[i][j]); //发送数据
    }
  }
  UART_PutChar(UART1,0xEF);  //开源上位机帧尾
  UART_PutChar(UART1,0xFE);  //开源上位机帧尾
  
//  UART_PutChar(UART1,0xFE);  //山外上位机帧头0x01
//  UART_PutChar(UART1,0x01);  //山外上位机帧头0xFE
}

/*************************************************************************
* @brief    摄像头MT9V034场中断
* @date     2020/4/8
*************************************************************************/
void MT9V034_DMA_START(void)
{
  MT9V034_DMA_CH->CNDTR = MT9V034_IMAGEH * MT9V034_IMAGEW;     //DMA传输摄像头数据的数量为摄像头分辨率。
  MT9V034_DMA_CH->CCR |= DMA_CCR1_EN;                          //开始
}
/*************************************************************************
* @brief    MT9V03X摄像头DMA完成中断
* @note
* @see
* @date     2020/4/8
*************************************************************************/
void MT9V034_DMA_CLS(void)
{
  MT9V034_DMA_CH->CCR &= (u16)(~DMA_CCR1_EN);   //关闭DMA1
  Camera_Flag = 1;                              //一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)  
}
/*************************************************************************
*  函数名称：void CAMERA_Init (unsigned char fps)
*  功能说明：摄像头初始化
*  参数说明：fps:  帧率
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：
*  摄像头的一些参数，在LQ_MT9V034.c中的宏定义中修改
*  CAMERA_Init(50);   //初始化MT9V034  50帧 V2版的摄像头注意使用白色带与非门版转接座,
*************************************************************************/
void CAMERA_Init (unsigned char fps)
{ 
  // 初始化摄像头 数据IO
  PIN_InitConfig(PF0, GPI, 0,GPIO_Mode_FLOATING);
  PIN_InitConfig(PF1, GPI, 0,GPIO_Mode_FLOATING);
  PIN_InitConfig(PF2, GPI, 0,GPIO_Mode_FLOATING);
  PIN_InitConfig(PF3, GPI, 0,GPIO_Mode_FLOATING);
  PIN_InitConfig(PF4, GPI, 0,GPIO_Mode_FLOATING);
  PIN_InitConfig(PF5, GPI, 0,GPIO_Mode_FLOATING);
  PIN_InitConfig(PF6, GPI, 0,GPIO_Mode_FLOATING);
  PIN_InitConfig(PF7, GPI, 0,GPIO_Mode_FLOATING);
  
  // 神眼初始化
  MT9V034_Init(fps);   
  
  // 摄像头DMA初始化，定时器触发DMA进行数据搬运    
  DMA_CameraInitConfig(MT9V034_DMA_CH,(u32)GPIOF_BYTE0, (u32)Image_Data, IMAGEH*IMAGEW);
  
  // PLCK引脚初始化触发定时器，通过DMA进行数据搬运
  DMA_CameraTriggerTimerInit(TIMER_1, MT9V034_PCLK_PIN); //GPIO触发TIM，只有引脚固定的几个可以选择  
  
  // VSYNC初始化 场中断初始化 G1
  PIN_Exti(MT9V034_VSYNC_PIN, EXTI_Trigger_Falling, GPIO_Mode_FLOATING,0,0); 
  
}
/*************************************************************************
*  函数名称：void Get_Use_Image (void)
*  功能说明：把摄像头采集到原始图像，缩放到赛道识别所需大小
*  参数说明：无
*  函数返回：无
*  修改时间：2020年10月28日
*  备    注：  IMAGEW为原始图像的宽度，神眼为188，OV7725为320
*       IMAGEH为原始图像的高度，神眼为120，OV7725为240
*************************************************************************/
void Get_Use_Image(void)
{
  short i = 0, j = 0, row = 0, line = 0;
  
  for (i = 0; i < LCDH; i ++)          //神眼高 120 
    // for (i = 0; i < IMAGEH; i += 3)       //OV7725高 240 / 3  = 80，
  {
    for (j = 0; j <= LCDW; j ++)     //神眼宽188
      // for (j = 0; j <= IMAGEW; j += 3)  //OV7725宽320 / 3  = 106，
    {
      Image_Use[row][line] = Image_Data[i][j];
      line++;
    }
    line = 0;
    row++;
  }
}

/************************************************************************
*  函数名称：void Get_Bin_Image (unsigned char mode)
*  功能说明：图像二值化到Bin_Image[][]
*  参数说明：mode  ：
*    0：使用大津法阈值
*    1：使用平均阈值
*    2: sobel 算子改进型  手动阈值，同时输出改为提取边沿的图像
*    3：sobel 算子改进型   动态阈值，同时输出改为提取边沿的图像
*  函数返回：无
*  修改时间：2020年10月28日
*  备    注：  Get_Bin_Image(0); //使用大津法二值化
************************************************************************/
void Get_Bin_Image (unsigned char mode)
{
  unsigned short i = 0, j = 0;
  unsigned short Threshold = 0;
  unsigned long tv = 0;
  //char txt[16];
  
  if (mode == 0)
  {
    Threshold = GetOSTU(Image_Use);  //大津法阈值
  }
  if (mode == 1)
  {
    //累加
    for (i = 0; i < LCDH; i++)
    {
      for (j = 0; j < LCDW; j++)
      {
        tv += Image_Use[i][j];   //累加
      }
    }
    Threshold =(unsigned short)(tv / LCDH / LCDW);   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
    Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
  }
  else if (mode == 2)
  {
    Threshold = 80;                          //手动调节阈值
    lq_sobel(Image_Use, Bin_Image, (unsigned char) Threshold);
    
    return;
    
  }
  else if (mode == 3)
  {
    lq_sobelAutoThreshold(Image_Use, Bin_Image);  //动态调节阈值
    return;
  }
  /*
#ifdef USEOLED
  sprintf(txt,"%03d",Threshold);//显示阈值
  OLED_P6x8Str(80,0,txt);
#else
  sprintf(txt, "%03d", Threshold);  //显示阈值
  TFTSPI_P6X8Str(0,7, txt, u16RED, u16BLUE);
#endif
  */
  /* 二值化 */
  for (i = 0; i < LCDH; i++)
  {
    for (j = 0; j < LCDW; j++)
    {
      if (Image_Use[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
        Bin_Image[i][j] = 1;
      else
        Bin_Image[i][j] = 0;
    }
  }
}
/************************************************************************
*  函数名称：short GetOSTU (unsigned char tmImage[LCDH][LCDW])
*  功能说明：大津法求阈值大小
*  参数说明：tmImage ： 图像数据
*  函数返回：无
*  修改时间：2011年10月28日
*  备    注：  GetOSTU(Image_Use);//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
************************************************************************/
short GetOSTU (unsigned char tmImage[LCDH][LCDW])
{
  signed short i, j;
  unsigned long Amount = 0;
  unsigned long PixelBack = 0;
  unsigned long PixelshortegralBack = 0;
  unsigned long Pixelshortegral = 0;
  signed long PixelshortegralFore = 0;
  signed long PixelFore = 0;
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
  signed short MinValue, MaxValue;
  signed short Threshold = 0;
  unsigned char HistoGram[256];              //
  
  for (j = 0; j < 256; j++)
    HistoGram[j] = 0; //初始化灰度直方图
  
  for (j = 0; j < LCDH; j++)
  {
    for (i = 0; i < LCDW; i++)
    {
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    }
  }
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值
  
  if (MaxValue == MinValue)
    return MaxValue;         // 图像中只有一个颜色
  if (MinValue + 1 == MaxValue)
    return MinValue;        // 图像中只有二个颜色
  
  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  像素总数
  
  Pixelshortegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //前景像素点数
    PixelFore = Amount - PixelBack;           //背景像素点数
    OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
    OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
    PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
    PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
    MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
    MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
}
/*************************************************************************
*  函数名称：lq_sobel
*  功能说明：基于soble边沿检测算子的一种边沿检测
* @param    imageIn    输入数组
*           imageOut   输出数组      保存的二值化后的边沿信息
*           Threshold  阈值
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：注意需要使用  带与非门版（白色）转接座，或者新版摄像头模块
*************************************************************************/
void lq_sobel (unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW], unsigned char Threshold)
{
  //* 卷积核大小 //
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd = LCDW - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd = LCDH - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]        // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};
      
      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};
      
      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0
      
      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1
      
      temp[0] = abs(temp[0]);
      temp[1] = abs(temp[1]);
      temp[2] = abs(temp[2]);
      temp[3] = abs(temp[3]);
      
      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }
      
      if (temp[0] > Threshold)
      {
        imageOut[i][j] = 1;
      }
      else
      {
        imageOut[i][j] = 0;
      }
    }
  }
}

/*************************************************************************
*  函数名称：lq_sobel
*  功能说明：基于soble边沿检测算子的一种自动阈值边沿检测
* @param    imageIn    输入数组
*           imageOut   输出数组      保存的二值化后的边沿信息 *
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：注意需要使用  带与非门版（白色）转接座，或者新版摄像头模块
*************************************************************************/
void lq_sobelAutoThreshold (unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW])
{
  //* 卷积核大小 //
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd   = LCDW - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd   = LCDH - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]       // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};
      
      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};
      
      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0
      
      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1
      
      temp[0] = abs(temp[0]);
      temp[1] = abs(temp[1]);
      temp[2] = abs(temp[2]);
      temp[3] = abs(temp[3]);
      
      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }
      
      // 使用像素点邻域内像素点之和的一定比例    作为阈值  //
      temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
        + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
          + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];
      
      if (temp[0] > temp[3] / 12.0f)
      {
        imageOut[i][j] = 1;
      }
      else
      {
        imageOut[i][j] = 0;
      }
      
    }
  }
}

/*---------------------------------------------------------------
【函    数】Bin_Image_Filter
【功    能】过滤噪点
【参    数】无
【返 回 值】无
【注意事项】
----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
  int16_t nr; //行
  int16_t nc; //列
  
  for (nr = 1; nr < LCDH - 1; nr++)
  {
    for (nc = 1; nc < LCDW - 1; nc = nc + 1)
    {
      if ((Bin_Image[nr][nc] == 0)
          && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 2))
      {
        Bin_Image[nr][nc] = 1;
      }
      else if ((Bin_Image[nr][nc] == 1)
               && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] < 2))
      {
        Bin_Image[nr][nc] = 0;
      }
    }
  }
}

/**************************************************************************
*                                                                          *
*  函数名称：Seek_Road(void)                                           *
*  功能说明：寻找白色区域偏差值                                            *
*  参数说明：无                                                            *
*  函数返回：值的大小                                                      *
*  修改时间：2017-07-16                                                    *
*  备    注：以中间为0，左侧减一，右侧加一，数值代表1的面积                *
*            计算区域从第一行开始到倒数第二行结束。                        *
*            如果面积为负数，数值越大说明越偏左边；                        *
*            如果面积为正数，数值越大说明越偏右边。                        *
**************************************************************************/
void Seek_Road (void)
{
    int16_t nr; //行
    int16_t nc; //列
    int16_t temp = 0; //临时数值
    //for(nr=1; nr<MAX_ROW-1; nr++)
    temp = 0;
    for (nr = 8; nr < 30; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET0 = temp;
    temp = 0;
    for (nr = 30; nr < 50; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET1 = temp;
    temp = 0;
    for (nr = 50; nr < 80; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET2 = temp;
    return;
}
