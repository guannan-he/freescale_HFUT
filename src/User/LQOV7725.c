/*******************************************************************************
【平    台】龙邱K66FX智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2018年4月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4及以上
【Target  】K66FX1M0VLQ18
【Crystal 】 50.000Mhz
【busclock】100.000MHz
【pllclock】200.000MHz
20180424采用新的DMA触发方式
******************************************************************************/
#include "include.h"
//#include "Lptmr.h"

u8 Image_Data[IMAGEH][IMAGEW];  //图像原始数据存放

u8 Image_Use[LCDH][LCDW]; //压缩后之后用于存放屏幕显示数据
u16 Pixle[LCDH][LCDW];              //二值化后用于OLED显示的数据
uint8_t Threshold;                  //OSTU大津法计算的图像阈值
u8  Line_Cont=0;          //行计数
u8  Field_Over_Flag=0;    //场标识

int OFFSET0=0;      //最远处，赛道中心值综合偏移量
int OFFSET1=0;      //第二格
int OFFSET2=0;      //最近，第三格
int AUX_1=0;//附加1
int AUX_2=0;//附加2
int start_stop=0;
int TXV=0;          //梯形的左高度，右高度
//int road_data[3];
int center_line[LCDH];

//摄像头图像采集中断处理函数
void PORTD_Interrupt(void)
{     
  //行中断PTD14
  if((PORTD_ISFR & 0x4000))//行中断 (1<<14)
  {    
    PORTD_ISFR |= 0x4000;  //清除中断标识
    // 用户程序            
    DMATransDataStart(DMA_CH4,(uint32_t)(&Image_Data[Line_Cont][0]));   //开启DMA传输 
    if(Line_Cont > IMAGEH)  //采集结束
    { 
      Line_Cont=0; 
      return ;
    } 
    ++Line_Cont;            //行计数
    return ; 
  }
  //场中断PTD15
  if((PORTD_ISFR & 0x8000))//(1<<15)
  {
    PORTD_ISFR |= 0x8000; //清除中断标识
    // 用户程序 
    Line_Cont = 0;         //行计数清零
    Field_Over_Flag=1;     //场结束标识
    GPIO_Reverse (PTA, 25);
  } 
}

/*************************************************************************
*                    北京龙邱智能科技 
*
*  函数名称：void SendPicture()
*  功能说明：摄像头数据发送
*  参数说明：无
*  函数返回：无
*  修改时间：
*  备    注：
*************************************************************************/
void SendPicture(void)
{
  int i = 0, j = 0;
  UART_Put_Char(UART_4,0xff);//发送帧头标志
  for(i=0;i<IMAGEH;i++)      //输出
  {
    for(j=0;j<IMAGEW;j++)    //输出从第0列到列，用户可以选择性的输出合适的列数
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//防止发送标志位
      }
      UART_Put_Char(UART_4,Image_Data[i][j]);
    }
  }
}

//测试主函数
void Test_OV7725(void)
{  
  
  LCD_Show_Frame_custome();      //画图像 LCDW*LCDH 外框
  OV7725_Init();          //摄像头初始化
  int frame_ps=0;
  char frame_psc[4];
  u32 time_count=0;//计时时间
  while(1)
  { 
    pit_time_start(PIT3);//开始计时
    LED_Ctrl(LED1, RVS);   //LED指示程序运行状态
    if(Field_Over_Flag)    //完成一场图像采集后显示并发送数据到上位机
    {                                   
      Field_Over_Flag= 0;            
     //UARTSendPicture(Image_Data);     //发送数据到上位机Demok//使用上位机显示的时候打开此函数，关闭Draw_Road()函数。
     //UARTSendPicture2(Image_Data);
      Get_Use_Image();     //采集显示图像数据存放数组
      Get_01_Value();      //二值化显示图像数据
                  
      Threshold = GetOSTU_IMP(Image_Data);   //OSTU大津法 获取全局阈值，传送原图
      //改进otsu算法
      //BinaryImage(Image_Data,Threshold); //二值化图像数据
      Pixle_Filter();//显示去噪
      Draw_Road();         //龙邱OLED模块显示动态图像//使用OLED显示的时候打开此函数，关闭UARTSendPicture()函数。
      Field_Over_Flag= 0; 
      //time_delay_ms(500);
      time_count=pit_time_get(PIT3);//读取计时
      pit_time_close(PIT3);
      frame_ps=110000000/time_count;
      sprintf(frame_psc,"%02dFPS",frame_ps);
      LCD_P6x8Str(81,2,(u8*)frame_psc);
      
      //time_delay_ms(500);
    }    
  }
}

// OV7725_Init Port Init
void OV7725_Init(void)
{     
    uint8_t id[2];    //存放摄像头id
    uint8_t ack = 0;  //检验所有寄存器操作是否出错   
    uint16_t width = IMAGEW, height = IMAGEH;
    uint16_t hstart, vstart, hsize;  
  //摄像头寄存器设置
    SCCB_Init();                     //两个地址口都拉高，为MT9V034_I2C_ADDR 
    OV7725_SoftwareReset();   //寄存器恢复初始值
    time_delay_ms(500);  
      /*7725最大分辨率 640 * 480*/
    if ((IMAGEW > 640) || (IMAGEH > 480))
    {
        LCD_P6x8Str(2,1,(u8*)"7725 dpi!!");                      //摄像头识分辨率设置过大，停止运行
        while(1);
    }
    ack += SCCB_RegRead(OV7725_SCCB_ADDR,OV7725_PID_REG,&id[0]);  //读取摄像头id高八位
    ack += SCCB_RegRead(OV7725_SCCB_ADDR,OV7725_VER_REG,&id[1]);  //读取摄像头id低八位
    if(OV7725_REVISION != (((uint32_t)id[0] << 8U) | (uint32_t)id[1]))//摄像头版本寄存器 
    {     
        
        LCD_P6x8Str(2,0,(u8*)"OV7725 Failed");                      //摄像头识别失败，停止运行
        while(1); 
    } 
    else                                                   //芯片ID正确
    {
        LCD_P6x8Str(2,0,(u8*)"OV7725 OK");
    }
    ack += OV7725_Init_Regs();     //先按官方的默认初始化 VGA 后面需要再改
    
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM2_REG, 0x03);  //输出驱动能力 Bit[4]: 睡眠模式 Bit[1:0]: 驱动能力00: 1x 01: 2x 10: 3x 11: 4x
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x46);  //  使用QVGA 和 RGB565 格式
//                                                                  /*OV7725_COM7_REG
//                                                                    Bit[7]: 重置寄存器
//                                                                    0: 不重置
//                                                                    1: 将所有寄存器重置为默认值
//                                                                    Bit[6]: 分辨率设置（7725输出支持两种分辨率，其他的任意自定义分辨率相当于截取这两种分辨率其中的一部分，会丢失视野）
//                                                                    0: VGA
//                                                                    1: QVGA
//                                                                    Bit[5]: BT.656协议开/关选择
//                                                                    Bit[4]: 传感器的原始值
//                                                                    Bit[3:2]: RGB输出格式控件
//                                                                    00: GBR4:2:2
//                                                                    01: RGB565
//                                                                    10: RGB555
//                                                                    11: RGB444
//                                                                    Bit[1:0]: 输出格式控制
//                                                                    00: YUV
//                                                                    01: Processed Bayer RAW
//                                                                    10: RGB
//                                                                    11: Bayer RAW */
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM3_REG, 0x00);//Bit[6]: 0水平镜像关 1水平镜像开  注意开启水平镜像时最好开启0x32寄存器的Bit[7]:镜像图像边缘对齐-在镜像模式下应该设置为1
//                                                                 //Bit[4]: 0 UYVY模式  1 YUYV模式  
//                                                                 //Bit[3]: 0 小端      1 大端 
//    hstart = (0x3fU << 2U);   //图像水平开始位置 使用VGA时 为0x23
//    vstart = (0x03U  << 1U) ;   //图像垂直开始位置 使用VGA时 为0x07   
    
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x43);  //  使用QVGA 和 Bayer RAW 格式
                                                                  /*OV7725_COM7_REG
                                                                    Bit[7]: 重置寄存器
                                                                    0: 不重置
                                                                    1: 将所有寄存器重置为默认值
                                                                    Bit[6]: 分辨率设置（7725输出支持两种分辨率，其他的任意自定义分辨率相当于截取这两种分辨率其中的一部分，会丢失视野）
                                                                    0: VGA
                                                                    1: QVGA
                                                                    Bit[5]: BT.656协议开/关选择
                                                                    Bit[4]: 传感器的原始值
                                                                    Bit[3:2]: RGB输出格式控件
                                                                    00: GBR4:2:2
                                                                    01: RGB565
                                                                    10: RGB555
                                                                    11: RGB444
                                                                    Bit[1:0]: 输出格式控制
                                                                    00: YUV
                                                                    01: Processed Bayer RAW
                                                                    10: RGB
                                                                    11: Bayer RAW */
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_DSP_CTRL4_REG, 0x4a);  /*Bit[1:0]: Output selection
                                                                          00: YUV or RGB
                                                                          01: YUV or RGB
                                                                          10: RAW8
                                                                          11: RAW10*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM3_REG, 0x00);//Bit[6]: 0水平镜像关 1水平镜像开  注意开启水平镜像时最好开启0x32寄存器的Bit[7]:镜像图像边缘对齐-在镜像模式下应该设置为1
                                                                 //Bit[4]: 0 UYVY模式  1 YUYV模式  
                                                                 //Bit[3]: 大小端
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);/*Bit[7]: 启用快速AGC/AEC算法
                                                                    Bit[6]: AEC -步长限制
                                                                    0: 步长限于垂直空白
                                                                    1: 无限的步长
                                                                    Bit[5]: 带过滤开/关
                                                                    Bit[4]: 启用低于条带值的AEC
                                                                    0: 将曝光时间限制在1/100或1/120
                                                                    其次在任何光照条件下当启用带通滤波器
                                                                    1: 允许曝光时间小于1/100或1/120
                                                                    其次在强光条件下当启用带通滤波器
                                                                    Bit[3]: 良好的AEC开/关控制
                                                                    0: 限制最小曝光时间为一行
                                                                    1: 允许曝光时间小于一行
                                                                    Bit[2]: 自动增益控制使
                                                                    0: 手动模式
                                                                    1: 自动模式
                                                                    Bit[1]: AWB Enable
                                                                    0: 手动模式
                                                                    1: 自动模式
                                                                    Bit[0]: AEC Enable
                                                                    0: 手动模式
                                                                    1: 自动模式*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_CNST_REG, 0x20);/*对比度设置 归一化值为0x20  越小对比度越大*/
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BRIGHT_REG, 0x20);/*亮度设置*/
//    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SIGN_REG, 0x20);/*亮度设置*/
    hstart = 0x3fU << 2U;   //图像水平开始位置
    vstart = 0x03U << 1U;   //图像垂直开始位置

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM4_REG, 0x41);/* 锁相环4倍频 满窗口
                                                                    Bit[7:6]: 锁相环倍频器控制
                                                                    00: 不用倍频
                                                                    01: 4倍频
                                                                    10: 6倍频
                                                                    11: 8倍频
                                                                    Bit[5:4]: 自动曝光窗口大小
                                                                    00: Full window
                                                                    01: 1/2 window
                                                                    10: 1/4 window
                                                                    11: Low 2/3 window*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_CLKRC_REG, 0x02);/*时钟配置 时钟 = 24M * 4 / （（3+1）*2）= 12M  时钟越高，帧率越快 不过DMA可能接受不了会有噪点 
                                                                    Bit[6]: 直接使用外部时钟(没有时钟预刻度可用)
                                                                    Bit[5:0]: 时钟 = 24M × 锁相环倍频器 /[(CLKRC[5:0] + 1) × 2]*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_DM_LNL_REG, 0x00);/*虚拟行低八位， 增加虚拟行可以降低帧率，适当添加把帧率配置到自己想要的帧率*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_DM_LNH_REG, 0x00); /*虚拟行高八位*/
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_EXHCL_REG, 0x00); //虚拟像素插入LSB用于在水平方向插入虚拟像素的LSB
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_ADVFL_REG, 0x00); //垂直同步插入虚拟行(1位等于1行)的LSB
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR, OV7725_ADVFH_REG, 0x00); //垂直同步插入虚拟行的MSB 
    /* Resolution and timing. */
    hsize = width + 16;

    /* 设置输出图片大小. */
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HSTART_REG, hstart >> 2U);  //水平起始位置高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了10位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HSIZE_REG, hsize >> 2U);    //水平宽度高位      因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了10位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VSTART_REG, vstart >> 1U);  //垂直起始位置高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了9位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VSIZE_REG, height >> 1U);   //垂直高度高位      因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了9位数据，这里是高8位，剩下的两位放在OV7725_HREF_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HOUTSIZE_REG, width >> 2U); //水平输出宽度高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了10位数据，这里是高8位，剩下的两位放在OV7725_EXHCH_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VOUTSIZE_REG, height >> 1U);//垂直输出高度高位  因为寄存器是8位的 最大255，像素最大640 * 480 放不下，用了9位数据，这里是高8位，剩下的两位放在OV7725_EXHCH_REG中
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_HREF_REG,
                    ((vstart & 1U) << 6U) | ((hstart & 3U) << 4U) | ((height & 1U) << 2U) | ((hsize & 3U) << 0U)); //水平宽度和垂直高度的低位
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_EXHCH_REG, ((height & 1U) << 2U) | ((width & 3U) << 0U));  //水平输出和垂直输出的低2位和低1位
    
    //GPIO口初始化
    EXTI_Init(PTD,14,rising_down);   //行中断
    EXTI_Init(PTD,15,falling_up);    //场中断  
    GPIO_Init(PTD,0,GPI,0);          //八位数据输入口      
    GPIO_Init(PTD,1,GPI,0);
    GPIO_Init(PTD,2,GPI,0);
    GPIO_Init(PTD,3,GPI,0);
    GPIO_Init(PTD,4,GPI,0);
    GPIO_Init(PTD,5,GPI,0);
    GPIO_Init(PTD,6,GPI,0);
    GPIO_Init(PTD,7,GPI,0);
    //初始化DMA采集  
//    DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data, PTD13, DMA_BYTE1, (IMAGEW * 2 > 511 ? 511:IMAGEW * 2), DMA_rising); //一次DMA传输最大512个字节 
    DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN,(void*)Image_Data, PTD13, DMA_BYTE1, (IMAGEW ), DMA_rising); //一次DMA传输最大512个字节 
}


// 获取需要的图像数据(60*80)
__ramfunc void Get_Use_Image(void)
{
  int i = 0,j = 0,row = 0,line = 5;
  
  for(i = IMAGEH-1; i > 0; i-=4)  //240行，每4行采集一行，
  {
    for(j = IMAGEW-1;j >0; j-=4)  //320列  每4列采一列
    {        
      Image_Use[row][line] = Image_Data[i][j];         
       
      /*if(Image_Data[i][j]==255)
      {
        LED_Ctrl(LED1, RVS);
      }*/
      line++;
    }      
    line = 0;
    row++;      
  }  
}

//按照均值的比例进行二值化
void Get_01_Value(void)
{
  int i = 0,j = 0;
  u8 GaveValue;
  u32 tv=0;
  
  
  //累加
  for(i = 0; i <LCDH; i++)
  {    
    for(j = 0; j <LCDW; j++)
    {                            
      tv+=Image_Use[i][j];   //累加  
    } 
  }
  GaveValue=tv/LCDH/LCDW;     //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100 
  //sprintf(txt,"%03d-%03d",Threshold,GaveValue);//前者为大津法求得的阈值，后者为平均值  
  //LCD_P6x8Str(81,0,"Ots-Avg");
  //LCD_P6x8Str(81,1,(u8*)txt);
  //按照均值的比例进行二值化
  GaveValue=GaveValue*7/10+10;        //此处阈值设置，根据环境的光线来设定 
  for(i = 0; i < LCDH; i++)           //只是二值化屏显内容
  {
    for(j = 0; j < LCDW; j++)
    {                                
      if(Image_Use[i][j] >GaveValue)//平均值阈值
      //if(Image_Use[i][j] >Threshold) //大津法阈值   数值越大，显示的内容越多，较浅的图像也能显示出来    
        Pixle[i][j] =1;        
      else                                        
        Pixle[i][j] =0;
    }    
  }
}
//显示图像到OLED模块
void Draw_Road(void)
{ 	 
  u8 i = 0, j = 0,temp=0;
  
  //发送帧头标志
  for(i=8;i<56;i+=8)//6*8=48行 
  {
    LCD_Set_Pos(0,i/8+1);//起始位置
    for(j=0;j<LCDW;j++)  //列数
    { 
      temp=0;
      if(Image_Use[0+i][j]) temp|=1;
      if(Image_Use[1+i][j]) temp|=2;
      if(Image_Use[2+i][j]) temp|=4;
      if(Image_Use[3+i][j]) temp|=8;
      if(Image_Use[4+i][j]) temp|=0x10;
      if(Image_Use[5+i][j]) temp|=0x20;
      if(Image_Use[6+i][j]) temp|=0x40;
      if(Image_Use[7+i][j]) temp|=0x80;
      LCD_WrDat(temp); 	  	  	  	  
    }
  }  
}
//三面以上反数围绕清除噪点
void Pixle_Filter(void)
{  
  int nr; //行
  int nc; //列
  
  for(nr=1; nr<LCDH-1; nr++)
  {  	    
    for(nc=1; nc<LCDW-1; nc=nc+1)
    {
      if((Pixle[nr][nc]==0)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]>2))         
      {
        Pixle[nr][nc]=1;
      }	
      else if((Pixle[nr][nc]==1)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]<2))         
      {
        Pixle[nr][nc]=0;
      }	
    }	  
  }  
}

/***************************************************************************
*                                                                          *
*  函数名称：int Seek_Road(void)                                           *
*  功能说明：寻找白色区域偏差值                                            *
*  参数说明：无                                                            *
*  函数返回：值的大小                                                      *
*  修改时间：2017-07-16                                                    *
*  备    注：以中间为0，左侧减一，右侧加一，数值代表1的面积                *
*            计算区域从第一行开始到倒数第二行结束。                        *
*            如果面积为负数，数值越大说明越偏左边；                        *
*            如果面积为正数，数值越大说明越偏右边。                        *
***************************************************************************/ 
void Seek_Road(void)
{  
  int nr; //行
  int nc; //列
  int temp_l=0;//临时数值
  int temp_r=80;
  int temp_avg=0;
  char temp_char[5];
  //for(nr=1; nr<MAX_ROW-1; nr++)
  for(nr=0; nr<LCDH; nr++)
  {  	    
    for(nc=1;nc<LCDW-2;nc++)//左到右扫描
    {
      if(!Image_Use[nr][nc-1]&&!Image_Use[nr][nc]&&Image_Use[nr][nc+1]&&Image_Use[nr][nc+2])
      {
        temp_l=nc;
        break;
      }	
    }
    for(nc=2;nc<LCDW-3;nc++)//右到左扫描
    {
      if(Image_Use[nr][LCDW-nc-2]&&Image_Use[nr][LCDW-nc-1]&&!Image_Use[nr][LCDW-nc]&&!Image_Use[nr][LCDW-nc+1])
      {
        temp_r=LCDW-nc;
        //temp_avg=(temp_l+temp_r)/2+temp_avg;
        break;
      }
    }
    temp_avg=(temp_l+temp_r)/2;
    center_line[nr]=(int)temp_avg;
    temp_l=0;
    temp_r=80;
    temp_avg=40;
  }
  temp_avg=0;
  //temp_avg=(int)(temp_avg/10)-40;
  int i=0;
  for(i=5;i<15;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/10-40);
  OFFSET0=temp_avg;
  sprintf(temp_char,":%03d:",OFFSET0);
  LCD_P6x8Str(90,2,(u8*)temp_char);
  temp_avg=0;
  for(i=25;i<35;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/10-40);
  OFFSET1=temp_avg;
  sprintf(temp_char,":%03d:",OFFSET1);
  LCD_P6x8Str(90,3,(u8*)temp_char);
  temp_avg=0;
  for(i=45;i<55;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/10-40);
  OFFSET2=temp_avg;
  sprintf(temp_char,":%03d:",OFFSET2);
  LCD_P6x8Str(90,4,(u8*)temp_char);
  temp_avg=0;
  for(i=18;i<22;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/4-40);
  AUX_1=temp_avg;
  sprintf(temp_char,":%03d:",AUX_1);
  LCD_P6x8Str(90,5,(u8*)temp_char);
  temp_avg=0;
  for(i=38;i<42;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/4-40);
  AUX_2=temp_avg;
  sprintf(temp_char,":%03d:",AUX_2);
  LCD_P6x8Str(90,6,(u8*)temp_char);
  return;  
}


u8 zb[48],yb[48];
void FindTiXing(void)
{
  int nr; //行
  int nc; //列     
  
  for(nr=0; nr<48; nr++)
  {  	    
    zb[nr]=0;
    yb[nr]=100;   
  }  	
  for(nr=0; nr<48; nr++)
  {  	    
    for(nc=2;nc<MAX_COL-2;nc++)
    {
      if((Pixle[nr+8][nc-1]==0)&&(Pixle[nr+8][nc]==0)&&(Pixle[nr+8][nc+1]==1)&&(Pixle[nr+8][nc+2]==1))
      {
        zb[nr]=nc;//左边沿，越来越大
      }
      if((Pixle[nr+8][nc-1]==1)&&(Pixle[nr+8][nc]==1)&&(Pixle[nr+8][nc+1]==0)&&(Pixle[nr+8][nc+2]==0))
      {
        yb[nr]=nc;//右边沿，越来越小
      }                   
    }	    
  }
  TXV=0;
  for(nr=0; nr<47; nr++)
  {  	    
    if((zb[nr]>=zb[nr+1])&&(zb[nr]>0))   TXV++;          
    if((yb[nr]<=yb[nr+1])&&(yb[nr]<100)) TXV--;          
  }  	   
  return;  
}


/***************************************************************
* 
* 函数名称：SendPicture 
* 功能说明：发送图像到上位机 ，不同的上位机注意修改对应的数据接收协议
* 参数说明： 
* 函数返回：void 
* 修改时间：2018年3月27日 
* 备 注： 
***************************************************************/ 
void UARTSendPicture2(uint8_t  tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  UART_Put_Char(UART_4,0x01); //发送帧头标志 WindowsFormsApplication1.exe
  UART_Put_Char(UART_4,0xfe); //发送帧头标志 WindowsFormsApplication1.exe
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW ;j++) 
    { 
      if(tmImage[i][j]==0xfe) 
      { 
        tmImage[i][j]=0xff; //防止发送标志位 
      } 
      UART_Put_Char(UART_4,tmImage[i][j]); 
    } 
  }
  UART_Put_Char(UART_4,0xfe); //发送帧尾标志 
  UART_Put_Char(UART_4,0x01); //发送帧尾标志 
} 

void UARTSendPicture(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int i = 0, j = 0; 
  UART_Put_Char(UART_4,0xFF); //发送帧头标志 DEMOK上位机  
  for(i=0;i < IMAGEH; i++) 
  { 
    for(j=0;j <IMAGEW;j++) 
    { 
      if(tmImage[i][j]==0xFF) 
      { 
        tmImage[i][j]=0xFE; //防止发送标志位 
      } 
      UART_Put_Char(UART_4,tmImage[i][j]); 
    } 
  }
} 
/*
void SendPicture(void)
{
  int i = 0, j = 0;
  UART_Put_Char(UART_4,0xff);//发送帧头标志
  for(i=0;i<Frame_Height;i++)      //输出
  {
    for(j=0;j<Frame_Width;j++)    
    {
      if(Image_Data[i][j]==0xff)
      {
        Image_Data[i][j]=0xfe;//防止发送标志位
      }
      UART_Put_Char(UART_4,Image_Data[i][j]);
    }
  }
}
*/

/*OTSU************************************************************** 
* 
* 函数名称：uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
* 功能说明：求阈值大小 
* 参数说明： 
* 函数返回：阈值大小 
* 修改时间：2018年3月27日 
* 备 注： 
参考：https://blog.csdn.net/zyzhangyue/article/details/45841255
      https://www.cnblogs.com/moon1992/p/5092726.html
      https://www.cnblogs.com/zhonghuasong/p/7250540.html     
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例w0，并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背景像素) 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7）将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。




***************************************************************/ 
uint8_t GetOSTU(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int16_t i,j; 
  uint32_t Amount = 0; 
  uint32_t PixelBack = 0; 
  uint32_t PixelIntegralBack = 0; 
  uint32_t PixelIntegral = 0; 
  int32_t PixelIntegralFore = 0; 
  int32_t PixelFore = 0; 
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差; 
  int16_t MinValue, MaxValue; 
  uint8_t Threshold = 0;
  uint8_t HistoGram[256];              //  

  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图 
  
  for (j = 0; j < IMAGEH; j++) 
  { 
    for (i = 0; i < IMAGEW; i++) 
    { 
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    } 
  } 
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; MaxValue--) ; //获取最大灰度的值
      
  if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色    
  if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];    //前景像素点数
    PixelFore = Amount - PixelBack;         //背景像素点数
    OmegaBack = (double)PixelBack / Amount;//前景像素百分比
    OmegaFore = (double)PixelFore / Amount;//背景像素百分比
    PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
    MicroBack = (double)PixelIntegralBack / PixelBack;   //前景灰度百分比
    MicroFore = (double)PixelIntegralFore / PixelFore;   //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
} 
/*************************************************************** 
* 
* 函数名称：void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW]) 
* 功能说明：图像数据二值化 
* 参数说明： 
* 函数返回：void 
* 修改时间：2018年3月27日 
* 备 注： 
***************************************************************/ 
void BinaryImage(uint8_t tmImage[LCDH][LCDW],uint8_t ThresholdV) 
{ 
  int i = 0, j = 0; 
  for(i = 0;i < LCDH;i++) 
  { 
    for(j = 0; j< LCDW;j++) 
    { 
      if(tmImage[i][j] >= ThresholdV) 
      { 
        tmImage[i][j] = 255; 
      } 
      else 
      { 
        tmImage[i][j] = 0; 
      } 
    } 
  } 
} 





/***************************************************************************
 *7725摄像头驱动
 * 
 */
/*初始化ov7725  自动曝光 没有特效*/
uint8_t OV7725_Init_Regs(void)
{
    uint8_t ack = 0;
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x3d, 0x03);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x42, 0x7f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x4d, 0x09);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x64, 0xff);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x65, 0x20);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x66, 0x00);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x67, 0x48);   
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x0f, 0xc5);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x13, 0xff);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x63, 0xe0);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x14, 0x11);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x22, 0x3f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x23, 0x07);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x24, 0x40);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x25, 0x30);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x26, 0xa1);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x2b, 0x00);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x6b, 0xaa);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x0d, 0x41);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x90, 0x05);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x91, 0x01);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x92, 0x03);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x93, 0x00);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x94, 0x90);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x95, 0x8a);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x96, 0x06);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x97, 0x0b);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x98, 0x95);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x99, 0xa0);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9a, 0x1e);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9b, 0x08);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9c, 0x20);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x9e, 0x81);

    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0xa6, 0x04);


    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x7e, 0x0c);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x7f, 0x16);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x80, 0x2a);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x81, 0x4e);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x82, 0x61);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x83, 0x6f);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x84, 0x7b);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x85, 0x86);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x86, 0x8e);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x87, 0x97);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x88, 0xa4);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x89, 0xaf);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8a, 0xc5);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8b, 0xd7);
    ack += SCCB_RegWrite(OV7725_SCCB_ADDR,0x8c, 0xe8);
    return ack;
}

/*重置 7725的寄存器，恢复默认值*/
void OV7725_SoftwareReset(void)
{
    SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM7_REG, 0x80);
}

/*白平衡设置 0:自动模式1:晴天2,多云3,办公室4,家里5,夜晚*/
void OV7725_LightModeConfigs(uint8_t mode)
{
    switch(mode)
    {
        case 1:  //晴天
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x5a);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x5c);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 2:  //多云
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x58);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x60);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
        
            break;
        case 3:  //办公室
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x84);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x4c);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 4:   //家里
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xfd);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x96);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x40);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        case 5:   //晚上
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0xe5);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
        default:   //自动模式
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM8_REG, 0xff);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_BLUE_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_RED_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_COM5_REG, 0x65);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFL_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_ADVFH_REG, 0x00);
            break;
    }
}
/*7725支持一些简单的特效0:普通模式 1.黑白 2.复古  3,偏蓝4,偏红5,偏绿 6,负片*/
void OV7725_SpecialEffectConfigs(uint8_t mode)
{

    switch(mode)
    {
        case 1:  //黑白
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x26);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x80);
            break;
        case 2:  //复古
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x40);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0xa0);
            break;
        case 3:  //偏蓝
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0xa0);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x40);
            break;
        case 4:   //偏红
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x40);
            break;
        case 5:   //偏绿
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x1e);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x60);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x60);
            break;
        case 6:   //负片
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x46);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x00);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x00);
            break;
        default:   //普通模式
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_SDE_REG, 0x06);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_UFIX_REG, 0x80);
            SCCB_RegWrite(OV7725_SCCB_ADDR,OV7725_VFIX_REG, 0x80);
            break;
    }
}



/*********************************************************************
 *摄像头SCCB底层驱动
 *
 ***********************************************************************/



/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Init(void)
*  功能说明：配置SCCB所用引脚为GPIO功能，暂时不配置数据方向
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Init(void)
{
  GPIO_Init(PTE, 0,GPO,1);//配置为GPIO功能
  PORTE_BASE_PTR->PCR[0] |= 0x03; //上拉
  GPIO_Init(PTE, 1,GPO,1);//配置为GPIO功能 
  PORTE_BASE_PTR->PCR[1] |= 0x03; //上拉
}

/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Wait(void)
*  功能说明：SCCB等待演示
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Wait(void)
{
  uint16_t i=0;
  for(i=0;i<100;i++)
  { 
    asm ("nop");
  }  
}

/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Star(void)
*  功能说明：启动函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Star(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_High;
  SCL_High; 
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_Low; 
  SCCB_Wait();
}
/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_Stop(void)
*  功能说明：停止函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Stop(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_High; 
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();  
}
/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：uint8 SCCB_SendByte(uint8 Data)
*  功能说明：SCCB发送字节函数
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
uint8_t SCCB_SendByte(uint8_t Data)
{
  uint8_t i;
  uint8_t Ack;
  SDA_Out;
  for( i=0; i<8; i++)
  {
    if(Data & 0x80) SDA_High;
    else            SDA_Low;    
    Data <<= 1;
    SCCB_Wait();
    SCL_High;      
    SCCB_Wait();
    SCL_Low;
    SCCB_Wait();
  }
  SDA_In;
  SCCB_Wait();
  
  SCL_High;
  SCCB_Wait();
  Ack = SDA_Data;
  SCL_Low;
  SCCB_Wait();
  SDA_Out;
  return Ack;
}
/*************************************************************** 

* 
* 函数名称：uint8 SCCB_ReadByte(void) 
* 功能说明：SCCB读取字节函数 
* 参数说明： 
* 函数返回：读取字节 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
uint8_t SCCB_ReadByte(void) 
{ 
  uint8_t i; 
  uint8_t byte = 0; 
  SCL_Out; 
  SDA_In; //使能输入
  for( i=0; i<8; i++) 
  { 
    SCCB_Wait(); 
    SCL_High;
    SCCB_Wait();
    byte = byte<<1;
    if(SDA_Data)
        byte++;
    SCCB_Wait();
    SCL_Low;  
  }
  SDA_Out;
  SCCB_Wait(); 
  return byte; 
} 
/*************************************************************** 

* 
* 函数名称：static void SCCB_Ack(void) 
* 功能说明：IIC有回复信号 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
static void SCCB_Ack(void) 
{ 
  SCL_Out; 
  SDA_Out;
  SCL_Low;
  SDA_Low;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 
/*************************************************************** 

* 
* 函数名称：static void SCCB_NAck(void) 
* 功能说明：IIC无回复信号 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
static void SCCB_NAck(void) 
{ 
  SCL_Out; 
  SDA_Out;
  SCL_Low;
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SCL_Low;
  SCCB_Wait();
} 

/*************************************************************************
* 北京龙邱智能科技 KV58智能车母板           
*
*  函数名称：void SCCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
*  功能说明：向设备写数据 
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
uint8_t SCCB_RegWrite(uint8 Device,uint8 Address,uint8_t Data)
{
  uint8_t Ack = 0;
  
    SCCB_Star();
    Ack = SCCB_SendByte(Device<<1);
   
    Ack = SCCB_SendByte(Address);
    
    Ack = SCCB_SendByte(Data);
    
    SCCB_Stop();
    return Ack;
}
/*************************************************************** 

* 
* 函数名称：uint8_t SCB_RegRead(uint8_t Device,uint8_t Address,uint16_t *Data) 
* 功能说明：读取数据 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
uint8_t SCCB_RegRead(uint8_t Device,uint8_t Address,uint8_t *Data) 
{   
  uint8 Ack = 0;
  Device = Device<<1;
  SCCB_Star();
  Ack += SCCB_SendByte(Device);
  SCCB_Wait();
  Ack += SCCB_SendByte(Address);
  SCCB_Wait();
  SCCB_Stop();
  SCCB_Wait();
  
  SCCB_Star();
  Ack += SCCB_SendByte(Device | 0x01);
  
  *Data = SCCB_ReadByte();
//  SCCB_Ack();
//  *Data = *Data<<8;
  
//  *Data += SCCB_ReadByte();
  SCCB_NAck();
  
  SCCB_Stop();
  
  return  Ack;
} 
/***************************************************************  
* 
* 函数名称：int SCCB_Probe(uint8_t chipAddr) 
* 功能说明：查询该地址的设备是否存在 
* 参数说明： 
* 函数返回：void 
* 修改时间：2017年12月5日 
* 备 注： 
***************************************************************/ 
int SCCB_Probe(uint8_t chipAddr) 
{ 
  uint8_t err;
  err = 0;
  chipAddr <<= 1;
  
  SCCB_Star();
  err = SCCB_SendByte(chipAddr);
  SCCB_Stop();
  
  return err;
}

/*自建函数otsu

改进otsu，像素搜索范围
何冠男：摄像头自动白平衡，otsu阈值一般介于90-120，设置为80-139，节省搜索时间

*/


uint8_t GetOSTU_IMP(uint8_t tmImage[IMAGEH][IMAGEW]) 
{ 
  int16_t i,j; 
  uint32_t Amount = 0; 
  uint32_t PixelBack = 0; 
  uint32_t PixelIntegralBack = 0; 
  uint32_t PixelIntegral = 0; 
  int32_t PixelIntegralFore = 0; 
  int32_t PixelFore = 0; 
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差; 
  int16_t MinValue, MaxValue; 
  uint8_t Threshold = 0;
  uint8_t HistoGram[256];              //  

  for (j = 0; j < 255; j++)  HistoGram[j] = 0; //初始化灰度直方图 
  
  for (j = 0; j < IMAGEH; j++) 
  { 
    for (i = 0; i < IMAGEW; i++) 
    { 
      if (tmImage[j][i] >= 80 && tmImage[j][i] <= 139)
        HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    } 
  } 
  MinValue = 80;
  MaxValue = 139;
  //for (MinValue = 80; MinValue < 140 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
  //for (MaxValue = 139; MaxValue > MinValue && HistoGram[MaxValue] == 0; MaxValue--) ; //获取最大灰度的值
      
  //if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色    
  //if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];    //前景像素点数
    PixelFore = Amount - PixelBack;         //背景像素点数
    OmegaBack = (double)PixelBack / Amount;//前景像素百分比
    OmegaFore = (double)PixelFore / Amount;//背景像素百分比
    PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
    MicroBack = (double)PixelIntegralBack / PixelBack;   //前景灰度百分比
    MicroFore = (double)PixelIntegralFore / PixelFore;   //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
} 






/**********************************************************************
先建立一个测试
void camera(int*,int*,int*,int*);
近中远三个点横向偏差
角度偏差
指针写入到主函数里
采集一帧以后就返回主函数
***********************************************************************/
void camera(int* road_data)
{
  char txt[8];
      while(Field_Over_Flag==0)//等呀等呀等摄像头
    {
      ;
    }
/*
一点想法：
dma采集速度非常快，过来之后可能正在处理这一贞，然后头部就被改掉了
所以各种问题也出来了，帧率也上不去
现在想法是dma区只获取otsu阈值
在lcd范围内做控制
----何冠男20190413
*/
  
  /*图像采集段*/
  //if(Field_Over_Flag)    //完成一场图像采集
    //{                                   
      Field_Over_Flag= 0;  //关标识
      start_stop=0;
      Get_Use_Image();
      //Get_Use_Image();
      //Get_01_Value();
      //Pixle_Filter();//显示去噪
      //Draw_Road();
      Threshold = GetOSTU(Image_Data);   //OSTU大津法 获取全局阈值，传送原图
      //Threshold = 80;
      //改进otsu算法
      sprintf(txt,"OTSU:%03d",Threshold);
      LCD_P6x8Str(80,0,(u8*)txt);
      BinaryImage(Image_Use,Threshold); //二值化图像数据
      //Get_Use_Image();
      //Pixle_Filter();
      if_start();//启停检测
      Seek_Road();//返回位置
      //improved_seek_road();//废弃，无限复位
      course_detect();//中线绘制
      Draw_Road();//画路
      if(start_stop==1)
      {
        LED_Ctrl(LED1,ON);
      }
      else
      {
        LED_Ctrl(LED1,OFF);
      }
      /**/
      //LCD_CLS();
      road_data[0]=OFFSET0;
      road_data[1]=OFFSET1;
      road_data[2]=OFFSET2;
      road_data[3]=start_stop;
      road_data[4]=AUX_1;
      
      
      //Draw_Road();         //龙邱OLED模块显示动态图像
      Field_Over_Flag= 0; //确保关标识
      //time_delay_ms(25);
      
      
      
      //time_delay_ms(500);
    //}
  
  return;
}
/*通用函数，把路径添加到图像中，可以不用改--何冠男
能改的则改，不能改的坚决不改*/
void course_detect (void)
{
  int i=0;
  for(i=0;i<LCDH;i++)
  {
    Image_Use[i][center_line[i]]=0;
  }
}
/*改进看路法---何冠男*/

/*处理时间太长，无限复位
应该是没处理当前帧下一帧就到了
无限嵌套中断或者内存溢出
决定不使用这种方法*/
void improved_seek_road(void)
{
  int nr; //行
  int nc; //列
  char temp_char[5];
  int left_jump[LCDH];
  int cock[LCDH-1];
  /*改进段*/
  
  //获取左侧跳变
  for(nr=0; nr<LCDH; nr++)
  {  	    
    for(nc=1;nc<LCDW-2;nc++)//左到右扫描
    {
      if(!Image_Use[nr][nc-1]&&!Image_Use[nr][nc]&&Image_Use[nr][nc+1]&&Image_Use[nr][nc+2])
      {
        left_jump[nr]=nc;
        break;
      }	
    }
  }
  //计算左侧法线斜率
  for(nr=0; nr<LCDH-1; nr++)
  {
    cock[nr]=left_jump[nr]-left_jump[nr+1];
  }
  int last_x=0;
  int last_y=0;
  int next_x=0;
  int next_y=0;
  int center_x=0;
  int center_y=0;
  //沿法线搜索
  for(nr=0; nr<LCDH-1; nr++)
  {
    last_x=left_jump[nr];
    last_y=nr;
    for(nc=1;nc<LCDW;nc++)
    {
      next_y=last_y+1;
      next_x=last_x+cock[nr];
      if(next_x<0||next_x>=LCDW||next_y>=LCDH||next_y<0)//越界检查
      {
        break;
      }
      if(Image_Use[last_y][last_x]==255&&Image_Use[next_y][next_x]==0)//触碰检查
      {
        break;
      }
      last_y=next_y;
      last_x=next_x;
    }
    center_x=(int)((left_jump[nr]+last_x)/2);
    center_y=(int)((nr+next_y)/2);
    center_line[center_y]=center_x;
  }
  
  
  
  
  
  
  /*改进段*/
  /*获取控制点部分，通用不改----何冠男*/
  int temp_avg=0;
  //temp_avg=(int)(temp_avg/10)-40;
  int i=0;
  for(i=5;i<15;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/10-40);
  OFFSET0=temp_avg;
  sprintf(temp_char,":%03d:",OFFSET0);
  LCD_P6x8Str(90,2,(u8*)temp_char);
  temp_avg=0;
  for(i=25;i<35;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/10-40);
  OFFSET1=temp_avg;
  sprintf(temp_char,":%03d:",OFFSET1);
  LCD_P6x8Str(90,3,(u8*)temp_char);
  temp_avg=0;
  for(i=45;i<55;i++)
  {
    temp_avg+=center_line[i];
  }
  temp_avg=(int)(temp_avg/10-40);
  OFFSET2=temp_avg;
  sprintf(temp_char,":%03d:",OFFSET2);
  LCD_P6x8Str(90,4,(u8*)temp_char);
  return;  
}


void if_start(void)
{
  int nr=0;
  int check=0;
  for(nr=0;nr<LCDW;nr++)
  {
    if(Image_Use[30][nr]!=Image_Use[30][nr+1])
    {
      check+=1;
    }
    if(check>=12)
    {
      start_stop=1;
      //LED_Ctrl(LED1,RVS);
    }
  }
}