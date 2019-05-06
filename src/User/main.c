/****************************************************************************************************
------------------------------------------------
合肥工业大学
【dev.env.】IAR8.2及以上
【Target  】K66FX1M0VLQ18
【Crystal 】 50.000Mhz
【busclock】110.000MHz
【pllclock】220.000MHz
=============================================================
=================================
功能外设  K66单片机挂角     备注
-------------------------------------------------------------
LED接口定义：
LED1--PTA17
LED2--PTC0
LED3--PTD15 
LED4--PTE26
-------------------------------
//正交解码接口定义：
FTM1_QDPHA  PTA12       
FTM1_QDPHB  PTA13       
-------------------------------
PWM口的接口定义在FTM.H中：
FTM3_CH7  PTC11，舵机接口
FTM0_CH0  PTC1，电机驱动接口
FTM0_CH1  PTC2，电机驱动接口
-------------------------------------------------------------

******************************************************************************************************/
#include "include.h" 
#include "math.h"

//定义舵机参数
#define     Step_Right       3325//3525 //总线频率改变，值需要重新标定 约30度
#define     Step_Middle      2825//2578//2350 //总线频率改变，中值需要重新标定
#define     Step_Left        2325//1500 //总线频率改变，值需要重新标定 约30度
//全局设置类参数定义-何冠男
#define     encode_limit     400//跑飞步数设置
#define     stop_count       50//停车阈值
#define     str_kp      38      //转向kp
#define     str_ki      0       //转向ki
#define     str_kd      1       //转向kd
#define     mtr_gate    20     //电机控制阈值
#define     uart_gate   10      //串口传输间隔

int SPEED=0;          //速度全局变量
int ECPULSE=0;        //编码器全局变量
u32 time_count=0;//差不多20ms，计算每帧脉冲
PIDP pid_speed={0.011,0,0,10000,0,0,0};//
//{P,I,D,积分限制，积分，上次err，当前err，输出}

extern u8 Field_Over_Flag;//摄像头场中断
//编码器中断服务PIT0
void PIT0_Interrupt()
{
  PIT_Flag_Clear(PIT0);    //清中断标志位
  //编码器采集速度脉冲
  ECPULSE=FTM_AB_Get(FTM1);//读取速度数值
  LED_Ctrl(LED0, RVS);     //LED指示PIT中断状态  
}
//主函数
 void main(void)
{ 
  ///////////////////////////////////////////*准备阶段，只添不减原则，没用的注释掉，别忘自己名字-何冠男*/
  DisableInterrupts;           //关闭中断
  PLL_Init(PLL220);            //初始化PLL为220M,总线为110M
  UART_Init(UART_4,9600);    //UART初始化
  LED_Init();                  //LED初始化
  LCD_Init();                  //LCD初始化
  KEY_Init();                  //独立按键初始化
  FTM_AB_Init(FTM1);           //正交编码初始化
  OV7725_Init();               //摄像头初始化
  LCD_CLS();                   //清屏幕	  
  LCD_Show_HFUTLogo();         //显示图片
  LCD_P14x16Str(0,0,"合肥工业大学"); //字符串显示  
  time_delay_ms(1000); 
  LCD_CLS();                   //清屏幕	
  LCD_Show_Frame_custome();    //画图像80*48外框 
  GPIO_Init(PTA, 25, GPO,0);
  EnableInterrupts;            //开启中断
  Motor_Init(Step_Middle);                 //电机，舵机初始化
  //UART_Irq_En(UART_4);
  /*************************************************
      开始干活


编码器及电机pwm控制专用
alpha0.0
何冠男
占空比调节计算公式
（1100-duty）/1100=占空比
也就是说duty=1100，电机不转
duty=0，电机全速
duty=880，电机20%


  *********************全局变量段****************************/

  int frame_ps=0;
  char count[8];//帧率显示字段
  char spd_disp[8];//速度显示字段
  char duty_disp[4];
  char uart_disp[64];//串口显示字段
  int road_data[6];//道路信息，解释在主函数
  int k1_state=1;//k1开始
  int k2_state=1;//k2急停
  int Angle;     //舵机角度
  int Error;     //中心线偏差
  int Error_pre=0; //前中心线偏差
  u32 Inte_err=0;     //偏差累积
  int stop_sign=0;//停止条件
  int target_speed = 900;//目前期望车速
  int current_duty=1100;//目前转空比
  int uart_counter=0;
  

  
  LCD_P6x8Str(0,0,"press k1");
  while(1)/////////////////////////////////////////////////////k1开始
  {
    k1_state=KEY_Read(KEY1);
    break;
    if(k1_state==0)
    {
      time_delay_ms(10);
      k1_state=KEY_Read(KEY1);
      if(k1_state==0)
      {
        break;
      }
    }
  }
  LCD_CLS();
  LCD_P6x8Str(0,0,"wait 2s");
  time_delay_ms(2000);
  LCD_CLS();
  LCD_P6x8Str(0,0,"K2:STOP");
  
  while(1)/////////////////////////等待开始信号
  {
    camera(road_data);
    if(road_data[3]==1)
    {
      break;
    }
  }
  Motor_Ctrl(1100);
  time_delay_ms(1000);

  while(1) ///////////////////////////////////////////////////////////////////// //主循环，在这里面把你们的模块填进去就行-何冠男
  { 
    pit_time_start(PIT3);///////////计时开始帧
    FTM_AB_clear(FTM1);/////////////清空FTM
    camera(road_data); //////摄像头返回道路数据
      /*road_data[0][1][2],远中近距中线偏移
        左负右正，范围±40
        road_data[3]==0,最前方视野没有启停线
        road_data[3]==1,最前方视野有启停线
        road_data[4][5]辅助远近*/   
      /////////////////////////////////////////////////////////////////////////////从这里开始，急停+停车
      k2_state=KEY_Read(KEY2); 
      if(k2_state==0)////////////////////k2急停
      {
        time_delay_ms(10);
        k2_state=KEY_Read(KEY2);
        if(k2_state==0)
        {
          break;
        }
      }
      if(stop_sign<0)//////////////////////停车条件
      {
        stop_sign=0;
      }
      if(road_data[3]==1)
      {
        stop_sign+=1;
        //break;
      }
      if(road_data[3]==0)
      {
        stop_sign-=1;
      }
      if(stop_sign>=stop_count)
      {
        break;
      }/////////////////////////////////////////////////////////////////////////////到这里结束，急停+停车
    ////////////////////////////////////////////////////////////////////////////////*在这后面加代码*/
      //弯道减速———丁丁 
      //if(abs(road_data[0]-road_data[1])>3)
        //Motor_Ctrl(1050);
      //else
       // Motor_Ctrl(1000);
      
      //舵机控制———丁丁
      Error=road_data[1];
      Inte_err+=Error;
      Angle=str_ang_gen(Error,Error_pre,Inte_err,Step_Middle,str_kp,str_ki,str_kd,SPEED);
      Step_Angle_Set(Angle,Step_Left,Step_Right);
      Error_pre=Error;
    /*  //电机控制
    if (abs(target_speed-SPEED)>=mtr_gate)//判断条件可修改
    {
      pid_speed.errdat = target_speed - SPEED;
      updat_pid(&pid_speed);
      current_duty-=pid_speed.pidout;
      if (current_duty>2000)
      {
       current_duty=2000;
       break;
      }
      else if(current_duty<800)
      {
        current_duty=800;
        break;
      }
      Motor_Ctrl(current_duty);
    }
*/
   
     /////////////////////////////////////////////////////////////////////////////*在这前面加代码*/         
    ECPULSE=FTM_AB_Get(FTM1);//编码器脉冲，速度计算
    SPEED=(int)(ECPULSE*45*28*50/256/105*6.28);
    if(abs(ECPULSE)>encode_limit)////////////////////////20ms>step_limit步停止
    {
       break;
    } 
    time_count=pit_time_get(PIT3);///////////读取计时，帧率
    pit_time_close(PIT3);
    frame_ps=110000000/time_count;
    sprintf(count,"%02dFPS",frame_ps);
    LCD_P6x8Str(80,1,(u8*)count);
    sprintf(spd_disp,"%4dMM",SPEED);////////////////打印速度
    LCD_P6x8Str(0,1,(u8*)spd_disp);
    sprintf(duty_disp,"%4d",current_duty);////////////////打印电机占空比
    LCD_P6x8Str(90,7,(u8*)duty_disp);
    
    uart_counter++;
    if (uart_counter>uart_gate)
    {
      uart_counter=0;
      sprintf(uart_disp,"SPD:%4d\tDTY:%4d\tANG:%4d\tERR:%3d\n",SPEED,current_duty,Angle,Error);
      UART_Put_Str(UART_4,(u8*)uart_disp);
    }

  }
////////////////////////善后工作/////////////////////////
  Motor_Ctrl_stop();
  Step_Angle_Set(Step_Middle,Step_Left,Step_Right);
  LED_Ctrl(LED1, ON);
  LCD_CLS();
  LCD_P6x8Str(0,0,"STOP");
}






