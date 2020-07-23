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
#include "PIT.h"

//-------------------------------------------------------------------------*
//函数名: pit_init
//功  能: 初始化PIT
//参  数: pitn:模块名PIT0或PIT1或PIT2或PIT3
//        cnt 中断时间，单位1ms
//返  回: 无
//简  例: pit_init(PIT0,1000); PIT0中断，1000ms，即1s进入PIT0_interrupt()一次
//-------------------------------------------------------------------------*
void PIT_Init(PITn pitn, u32 cnt)
{
    //PIT 用的是 Bus Clock 总线频率

    /* 开启时钟*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //使能PIT时钟

    /* PIT模块控制 PIT Module Control Register (PIT_MCR) */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );      //使能PIT定时器时钟 ，调试模式下继续运行

    /* 定时器加载值设置 Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt*bus_clk_M*1000;                            //设置溢出中断时间

    //定时时间到了后，TIF 置 1 。写1的时候就会清0
    PIT_Flag_Clear(pitn);                                             //清中断标志位

    /* 定时器控制寄存器 Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );   //使能 PITn定时器,并开PITn中断

    NVIC_EnableIRQ((IRQn_Type)(pitn + 48));			                                //开接收引脚的IRQ中断
}



//-------------------------------------------------------------------------*
//函数名: PIT0_interrupt
//功  能: PIT中断函数
//参  数: 无
//返  回: 无
//简  例: 由初始化决定，多长时间进入一次
//-------------------------------------------------------------------------*
short speed=0;
/*
void PIT0_Interrupt()
{
  PIT_Flag_Clear(PIT0);       //清中断标志位
 //用户添加所需代码
  LED_Ctrl(LED2, RVS);        //中断发生后LED闪烁
  //speed=FTM_AB_Get(FTM2);     //开启正交解码后，可以获取速度，正负表示方向

}
*/
void PIT1_Interrupt()
{
  PIT_Flag_Clear(PIT1);       //清中断标志位
 /*用户添加所需代码*/
}

void PIT2_Interrupt()
{
  PIT_Flag_Clear(PIT1);       //清中断标志位
  /*用户添加所需代码*/
}

void PIT3_Interrupt()
{
  PIT_Flag_Clear(PIT3);       //清中断标志位
  /*用户添加所需代码*/
}
u32 pit_time_get(PITn pitn)
{
    u32 val;

    val = (~0) - PIT_CVAL(pitn);

    if(PIT_TFLG(pitn)& PIT_TFLG_TIF_MASK)                           //判断是否时间超时
    {
        PIT_Flag_Clear(pitn);                                       //清中断标志位
        PIT_TCTRL(pitn) &= ~ PIT_TCTRL_TEN_MASK;                    //禁止PITn定时器（用于清空计数值）
        return ~0;
    }

    if(val == (~0))
    {
        val--;              //确保 不等于 ~0
    }
    return val;
}

void pit_time_start(PITn pitn)
{
    //PIT 用的是 Bus Clock 总线频率
    //溢出计数 = 总线频率 * 时间

    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                          //使能PIT时钟

    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );    //使能PIT定时器时钟 ，调试模式下继续运行

    PIT_TCTRL(pitn) &= ~( PIT_TCTRL_TEN_MASK );                     //禁用PIT ，以便设置加载值生效

    PIT_LDVAL(pitn)  = ~0;                                          //设置溢出中断时间

    PIT_Flag_Clear(pitn);                                           //清中断标志位

    PIT_TCTRL(pitn) &= ~ PIT_TCTRL_TEN_MASK;                        //禁止PITn定时器（用于清空计数值）
    PIT_TCTRL(pitn)  = ( 0
                         | PIT_TCTRL_TEN_MASK                        //使能 PITn定时器
                         //| PIT_TCTRL_TIE_MASK                      //开PITn中断
                       );
}
void pit_time_close(PITn pitn)
{
    PIT_Flag_Clear(pitn);                                       //清中断标志位
    PIT_TCTRL(pitn) &= ~ PIT_TCTRL_TEN_MASK;                    //禁止PITn定时器（用于清空计数值）
}

int QL(int* center_line) 
{
   int x1,x2,x3,y1,y2,y3;
   int length1,length2,length3;
   int S;   
   int kp;
   y1=22;
   y2=27;
   y3=32;
   x1=center_line[27];
   x2=center_line[32];
   x3=center_line[37];
   S=abs((y2-y1)*(x3-x1)-(y3-y1)*(x2-x1));
   length1=(int)sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
   length2=(int)sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
   length3=(int)sqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3));
   if(S<1)
     S=1;
   kp=(int)abs((length1*length2*length3)/S);//曲率的倒数，即曲率半径
   return kp;
}

int str_ang_gen(int Error,int Error_pre,int SPEED,int* center_line)
{
  int str_kp;
  int str_kd;
  float P;
  int Step_Middle=2825;
  int Diff_err;
  int Angle;
  Diff_err=Error-Error_pre;
  if(abs(center_line[20]-center_line[30])<3)
    P=1.5;
  else if(abs(center_line[20]-center_line[30])<12)
    P=3;
  else
    P=4.5;
  str_kp=(int)(QL(center_line)*P);
  str_kd=1;
  Diff_err=Error-Error_pre;
  Angle=(int)(Step_Middle+Error*2*str_kp+Diff_err*50*str_kd);
  return Angle;
}