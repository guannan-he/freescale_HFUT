/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX1M0VLQ18���İ�
����    д��CHIUSIR
����    ע��
�������汾��V1.0
�������¡�2016��08��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef _I2C_H_
#define _I2C_H_

/**********************************  IIC(���Ÿ���) ***************************************/

#define I2C0_SCL    PTD8        // PTB0��PTB2��PTD8
#define I2C0_SDA    PTD9        // PTB1��PTB3��PTD9

#define I2C1_SCL    PTE1       // PTE1��PTC10
#define I2C1_SDA    PTE0       // PTE0��PTC11

/**********************************  IIC(���Ÿ���) ***************************************/

//����ģ���
typedef enum I2Cn
{
    I2C_0  = 0,
    I2C_1  = 1
} I2Cn;

//�����дѡ��
typedef enum MSmode
{
    write =   0x00,  /* Master write  */
    read =   0x01   /* Master read */
} MSmode;


#define I2C_DisableAck(I2Cn)        I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_TXAK_MASK

//
#define I2C_RepeatedStart(I2Cn)     I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_RSTA_MASK

//�����ź�
#define I2C_Start(I2Cn)             I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_TX_MASK+I2C_C1_MST_MASK;\
                                    //I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_MST_MASK

//��ͣ�ź�
#define I2C_Stop(I2Cn)              I2C_C1_REG(I2Cx[I2Cn]) &= ~(I2C_C1_MST_MASK+I2C_C1_TX_MASK);\
                                    //I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TX_MASK

//�������ģʽ(Ӧ��)
#define I2C_EnterRxMode(I2Cn)       I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TX_MASK;\
                                    I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TXAK_MASK
//�������ģʽ(��Ӧ��)
#define I2C_PutinRxMode(I2Cn)       I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TX_MASK

//�ȴ� I2C0_S
#define I2C_Wait(I2Cn)              while(( I2C_S_REG(I2Cx[I2Cn]) & I2C_S_IICIF_MASK)==0) {} \
                                    I2C_S_REG(I2Cx[I2Cn]) |= I2C_S_IICIF_MASK;

//дһ���ֽ�
#define I2C_write_byte(I2Cn,data)   I2C_D_REG(I2Cx[I2Cn]) = data


void I2C_Init(I2Cn A);                                        //��ʼ��I2C
void I2C_WriteAddr(I2Cn, u8 SlaveID, u8 Addr, u8 Data);       //����ַ��д������
void I2C_WriteAddr16(I2Cn i2cn, u8 SlaveID, u8 Addr, u16 Data);
void I2C_StartTransmission (I2Cn, u8 SlaveID, MSmode);        //��������
u8    I2C_ReadAddr(I2Cn, u8 SlaveID, u8 Addr);                 //��ȡ��ַ�������



#endif