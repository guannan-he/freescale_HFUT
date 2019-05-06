/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX1M0VLQ18���İ�
����    д��CHIUSIR
����    ע��
������汾��V1.0
�������¡�2016��08��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_OV7620_H_
#define __LQ_OV7620_H_

#define R240  240  //��
#define L320  320  //��

#define R120      120  //OLED��ʾ������
#define L160      160  //OLED��ʾ������

#define SCL_Out         DDRE1=1      //���������ΪSCL_Out
#define SDA_Out         DDRE0=1      //������Ϊ�����ΪSDA_Out
#define SDA_In          DDRE0=0      //������Ϊ������ΪSDA_In

#define SCL_High        PTE1_OUT=1      //��������ߵ�ƽ
#define SCL_Low         PTE1_OUT=0      //��������͵�ƽ
#define SDA_High        PTE0_OUT=1      //��������ߵ�ƽ
#define SDA_Low         PTE0_OUT=0      //��������͵�ƽ
#define SDA_Data        PTE0_IN         //��ȡ�����ϵ�����״̬

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