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
#include "PLL.h"

uint32 core_clk_M;//单位MHZ
uint32 bus_clk_M; //单位MHZ
/*************************************************************************
* 龙丘智能科技有限公司
*
*  函数名称：pll_init
*  功能说明：时钟初始化，用于设定频率。
*  参数说明：PLL_?
*  函数返回：无
*  修改时间：2015-10-6
*  备    注：内核时钟（系统时钟）=外部时钟（50M晶振频率）/ (pll_prdiv+1)*(pll_vdiv+16);
             MCG=PLL, core = MCG, bus = MCG/5, FlexBus = MCG/3, Flash clock= MCG/8
*  例    子：pll_init(PLL180);
*************************************************************************/
void PLL_Init(clk_option opt)
{
     char pll_prdiv;
     char pll_vdiv;

     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                 | SIM_SCGC5_PORTB_MASK
                 | SIM_SCGC5_PORTC_MASK
                 | SIM_SCGC5_PORTD_MASK
                 | SIM_SCGC5_PORTE_MASK );
    u32 temp_reg;

    if(opt!= PLLUNULL )
    {
      //设置PLL时钟
      switch(opt)
      {
      case PLL100:
        pll_prdiv       = 4;
        pll_vdiv        = 4;
        break;
      case PLL120:
        pll_prdiv       = 4;
        pll_vdiv        = 8;
        break;
      case PLL130:
        pll_prdiv       = 4;
        pll_vdiv        = 10;
        break;
      case PLL140:
        pll_prdiv       = 4;
        pll_vdiv        = 12;
        break;
      case PLL150:
        pll_prdiv       = 4;
        pll_vdiv        = 14;
        break;
      case PLL160:
        pll_prdiv       = 4;
        pll_vdiv        = 16;
        break;
      case PLL170:
        pll_prdiv       = 4;
        pll_vdiv        = 18;
        break;
      case PLL175:
        pll_prdiv       = 3;
        pll_vdiv        = 12;
        break;
      case PLL180:
        pll_prdiv       = 4;
        pll_vdiv        = 20;
        break;
      case PLL200:
        pll_prdiv       = 4;
        pll_vdiv        = 24;
        break;
      case PLL220:
        pll_prdiv       = 4;
        pll_vdiv        = 28;
        break;
      case PLL225:
        pll_prdiv       = 4;
        pll_vdiv        = 29;
        break;
      case PLL230:
        pll_prdiv       = 4;    //稳定
        pll_vdiv        = 30;
        break;
      case PLL235:
        pll_prdiv       = 4;    //不稳定?
        pll_vdiv        = 31;
        break;
      case PLL237_5:           //很不稳定?
        pll_prdiv       = 3;
        pll_vdiv        = 22;
        break;  
      default:               break;
      }
    }
    
    core_clk_M= 50 * ( pll_vdiv+16 )/2/(pll_prdiv+1);
    bus_clk_M =core_clk_M/2;       //初始化后记得修改总线频率
    
    MCG_C1 = MCG_C1_CLKS(2) ;      //选择外部时钟
    MCG_C5 = MCG_C5_PRDIV(pll_prdiv);//晶振为50M，分频结果范围要在8M~16M 此时为 50/(prdiv+1)
    temp_reg = FMC_PFAPR;

    //通过M&PFD置位M0PFD来禁止预取功能
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;
    //设置系统分频器
    //MCG=PLL, core = MCG,  
    SIM_CLKDIV1 =  SIM_CLKDIV1_OUTDIV1(0)    //core = MCG/(0+1)
                 | SIM_CLKDIV1_OUTDIV2(1)    //bus  = MCG/(0+1)修改总线频率后记得把bus_clk_M的分频也修改了！
                 | SIM_CLKDIV1_OUTDIV3(3)    //FlexBus    = MCG/(2+1)
                 | SIM_CLKDIV1_OUTDIV4(6);   //Flash clock= MCG/(n+1)

    //从新存FMC_PFAPR的原始值
    FMC_PFAPR = temp_reg;

    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(pll_vdiv);//PLL =  50M* (pll_vdiv+16)/(prdiv+1) /2

    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set
    while (!(MCG_S & MCG_S_LOCK0_MASK)){}; // Wait for LOCK bit to set

    MCG_C1=0x00;

    //等待时钟状态位更新
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};    
 /*
        //设置跟踪时钟为内核时钟
    SIM_SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;
    //在PTA6引脚上使能TRACE_CLKOU功能
    PORTA_PCR6 = ( PORT_PCR_MUX(0x7));
    //使能FlexBus模块时钟
    SIM_SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
    //在PTA6引脚上使能FB_CLKOUT功能
    PORTC_PCR3 = ( PORT_PCR_MUX(0x5));
*/
}





