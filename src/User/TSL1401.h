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
#ifndef __LQ_TSL1401_H_
#define __LQ_TSL1401_H_

extern vuint16 ADV[128];         //声明数组，用于存放采集的线性数值
extern vuint16 LCDD[128];        //转换为LCD显示的数值
extern vint16  piancha;

extern void Init_TSL1401(void);
extern void Read_TSL1401(void);
extern void Show_TSL1401(void);


#endif