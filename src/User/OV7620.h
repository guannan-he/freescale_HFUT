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
#ifndef __LQ_OV7620_H_
#define __LQ_OV7620_H_

#define R240  240  //行
#define L320  320  //列

#define R120      120  //OLED显示的行数
#define L160      160  //OLED显示的列数

#define SCL_Out         DDRE1=1      //配置输出作为SCL_Out
#define SDA_Out         DDRE0=1      //配置作为输出作为SDA_Out
#define SDA_In          DDRE0=0      //配置作为输入作为SDA_In

#define SCL_High        PTE1_OUT=1      //配置输出高电平
#define SCL_Low         PTE1_OUT=0      //配置输出低电平
#define SDA_High        PTE0_OUT=1      //配置输出高电平
#define SDA_Low         PTE0_OUT=0      //配置输出低电平
#define SDA_Data        PTE0_IN         //读取引脚上的引脚状态

#define     SLH       24 
#define     FLINE     25  
#define     SLINE     40
#define     TLINE     55
#define     MAX_ROW   60
#define     MAX_COL   100 

void OV7620_Init(void);
void SendPicture(void);

void SCCB_Init(void);
void SCCB_Wait(void);
void SCCB_Stop(void);
void SCCB_Star(void);
uint8 SCCB_SendByte(uint8 Data);
void SCB_RegWrite(uint8 Device,uint8 Address,uint8 Data);

    
void MT9V034_Init(void); 
void Cam_Init(void);
void Get_Pixel(void);
void Get_Back(void);
void Draw_Road(void);
void Get_Use_Image(void);
void Get_01_Value(void);
void Pixle_Filter(void);
void Seek_Road(void);
void findtixing(void);
void TEST_LQV034(void);
#endif