/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技MK66FX1M0VLQ18核心板
【编    写】CHIUSIR
【备    注】
【软件版本】V1.0
【最后更新】2016年08月20日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【交流邮箱】chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"

volatile u8   Image_Data[R240][L320];
volatile u8   Image_Use [R120][L160];            //剔除之后的数组
          u8   Pixle[R120][L160]; 
 
u8 control_f=0;
u8 control_f_cross=0;
u8 control_hang;
u8 motor_f=0;
u8 da_S=0;
u8 straight_f=0;  
u8 S_f=0;
u8 hang;
u8 line_route = 0;
u8 Rights[60];
u8 Lefts[60];
u8 center_line[R120];u8 hang; 
s16  Dir_Ctr_Out = 0;
s16  Dir_Error = 0;

u16  ROW_Space = 0;
u16  Line_Cont = 0;

u8   VSY_Flag=0;
u8   Cam_Flag= 0;

int  OFFSET0=0;      //最远处，赛道中心值综合偏移量
int  OFFSET1=0;      //第二格
int  OFFSET2=0;      //最近，第三格
int  TXV=0;//梯形的左高度，右高度

//OV7620摄像头图像采集中断处理函数，（图像有偏移，待解决）
void PORTD_Interrupt(void)
{
  int n;  
  n=14;   //行中断
  if((PORTD_ISFR & (1<<n)))
  {
      PORTD_ISFR |= (1<<n); 
      /* 用户自行添加中断内程序 */          
      if(VSY_Flag==0)
        {
            return;
        }
        //DMA采集，在这里修改采集接口PTD_BYTE0_IN是D0--D7。   PLCK接的是PTD13。
        DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN, (void *)Image_Data[Line_Cont], PTD13, DMA_BYTE1, 320, DMA_rising);
        DMA_EN(DMA_CH4);        
        Line_Cont++;
        
        if(Line_Cont==R240)             //采集结束
        {
           Line_Cont = 0;
           VSY_Flag=0;
           Cam_Flag = 1;
           DMA_DIS(DMA_CH4);
           DisableInterrupts;           
        }       
    }
  n=15;  //场中断
  if((PORTD_ISFR & (1<<n)))
  {
      PORTD_ISFR |= (1<<n); 
      /* 用户自行添加中断内程序 */
      Line_Cont = 0;
      VSY_Flag=1;
  } 
}

//测试主函数
void TEST_LQV034(void)
{  
  while(1)
  { 
    LED_Ctrl(LED1, RVS);       //LED指示程序运行状态
    if(Cam_Flag)
    {
      Get_Use_Image();  
      Draw_Road(); 
      Cam_Flag  = 0;
      EnableInterrupts
    }    
  }
}

// MT9V034 Port Init
void Cam_Init(void)
{
    u16 i=0,j=0;
       
      EXTI_Init(PTD,14,rising_down);   //行中断
      EXTI_Init(PTD,15,falling_up);    //场中断 

       for(i=0; i<R240; i++)
       {
          for(j=0;j< L320;j++)
          {
             Image_Data[i][j] = 0;
          }
        } 
}
void MT9V034_Init(void)
{   
    SCCB_Init();
    SCB_RegWrite(0x42,0x11,0x03);   
    SCB_RegWrite(0x42,0x14,0x24);    
    SCB_RegWrite(0x42,0x28,0x20);   
    SCCB_Wait();    
    Cam_Init();   
}
// 获取需要的图像数据
void Get_Use_Image(void)
{
  int i = 0,j = 0,row = 0,line = 0;
  
  for(i = 0; i  < R240; i+=2)  //240行，每2行采集一行，
  {
      for(j = 0;j < L320; j+=2)//320，每2列采集一列，
      {        
          Image_Use[row][line] = Image_Data[i][j];         
          line++;        
      }      
      line = 0;
      row++;      
  }
  row = 0; 
}
void Get_01_Value(void)
{
  int i = 0,j = 0;
  u8 GaveValue;
  u32 tv=0;
  char txt[16];
  
  //计算每行的均值
  for(i = 0; i <R120; i++)
  {    
    for(j = 0; j <L160; j++)
    {                            
      tv+=Image_Use[i][j];   //累加  
    } 
  }
  GaveValue=tv/R120/L160;     //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100 
  sprintf(txt,"%03d",GaveValue);//  
  TFT_P8X16Str(102,1,(uint8*)txt,WHITE,BLACK);
  //按照均值的比例进行二值化
  GaveValue=GaveValue*9/10+10;        //此处阈值设置，根据环境的光线来设定 
  for(i = 0; i < R120; i++)
  {
    for(j = 0; j < L160; j++)
    {                                
      if(Image_Use[i][j] >GaveValue) //数值越大，显示的内容越多，较浅的图像也能显示出来    
        Pixle[i][j] = 1;        
      else                                        
        Pixle[i][j] = 0;
    }    
  }
}

//三面以上反数围绕清除噪点
void Pixle_Filter(void)
{  
  int nr; //行
  int nc; //列
  
  for(nr=1; nr<R120-1; nr++)
  {  	    
    for(nc=1; nc<L160-1; nc=nc+1)
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
void Draw_Road(void)
{ 	 
  u8 i = 0, j = 0;
  
  //发送帧头标志
  TFT_SetPos(0,8,160,128);
  for(i = 0; i < R120; i++)
  {
    for(j = 0; j < L160; j++)
    {                                
     TFT_write_para16(Image_Use[i][j]); //数值越大，显示的内容越多，较浅的图像也能显示出来 
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
    int temp=0;//临时数值
    //for(nr=1; nr<MAX_ROW-1; nr++)
    temp=0;
    for(nr=8; nr<24; nr++)
    {  	    
	    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	++temp;
			    }			   
	    }
	    for(nc=0; nc<MAX_COL/2; nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	--temp;
			    }			   
	    }		  
  	}
  	OFFSET0=temp;
  	temp=0;
  	for(nr=24; nr<40; nr++)
    {  	    
	    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	++temp;
			    }			   
	    }
	    for(nc=0; nc<MAX_COL/2; nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	--temp;
			    }			   
	    }		  
  	}
  	OFFSET1=temp;    	
  	temp=0;
  	for(nr=40; nr<56; nr++)
    {  	    
	    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	++temp;
			    }			   
	    }
	    for(nc=0; nc<MAX_COL/2; nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	--temp;
			    }			   
	    }		  
  	}
  	OFFSET2=temp;   	
  	return;  
}

u8 zb[48],yb[48];
void findtixing(void)
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

/*************************************************************************
*                    北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Init(void)
*  功能说明：配置SCCB所用引脚为GPIO功能，暂时不配置数据方向
*  参数说明：无
*  函数返回：无
*  修改时间：2014年1月5日
*  备    注：
*************************************************************************/
void SCCB_Init(void)
{
  GPIO_Init(PTE, 0,GPO,0);   //配置为GPIO功能
  GPIO_Init(PTE, 1,GPO,0);//配置为GPIO功能

}

/*************************************************************************
*                    北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Wait(void)
*  功能说明：SCCB等待演示
*  参数说明：无
*  函数返回：无
*  修改时间：2014年1月5日
*  备    注：
*************************************************************************/
void SCCB_Wait(void)
{
  uint8 i=0;
 for(i=0;i<200;i++)
  { 
    asm ("nop");
  }

}

/*************************************************************************
*                    北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Star(void)
*  功能说明：启动函数
*  参数说明：无
*  函数返回：无
*  修改时间：2014年1月5日
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

}
/*************************************************************************
*                    北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Stop(void)
*  功能说明：停止函数
*  参数说明：无
*  函数返回：无
*  修改时间：2014年1月5日
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
*                    北京龙邱智能科技 K60开发板           
*
*  函数名称：uint8 SCCB_SendByte(uint8 Data)
*  功能说明：SCCB发送字节函数
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2014年1月5日
*  备    注：
*************************************************************************/
uint8 SCCB_SendByte(uint8 Data)
{
   uint8 i;
   uint8 Ack;
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
  SDA_High;
  SDA_In;
  SCCB_Wait();
  
  SCL_High;
  SCCB_Wait();
  Ack = SDA_Data;
  SCL_Low;
  SCCB_Wait();
  return Ack;
}

/*************************************************************************
*                    北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCB_RegWrite(uint8 Device,uint8 Address,uint8 Data)
*  功能说明：向设备写数据
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2014年1月5日
*  备    注：
*************************************************************************/
void SCB_RegWrite(uint8 Device,uint8 Address,uint8 Data)
{
  uint8 i;
  uint8 Ack;
  for( i=0; i<20; i++)
  {
    SCCB_Star();
    Ack = SCCB_SendByte(Device);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte(Address);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte(Data);
    if( Ack == 1 )
    {
      continue;
    }
    
    SCCB_Stop();
    if( Ack == 0 ) break;
  }
}
