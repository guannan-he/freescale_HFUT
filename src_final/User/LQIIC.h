#ifndef __LQIIC_H__
#define __LQIIC_H__


extern void LQ_IIC_Delay(void);
extern void LQ_IIC_Init(void);
extern void LQ_IIC_Start(void);
extern void LQ_IIC_Stop(void);
extern uint8_t LQ_IIC_Wait_Ack(void);
extern void LQ_IIC_Ack(void);
extern void LQ_IIC_NAck(void);
extern void LQ_IIC_Send_Byte(uint8_t txd);
extern uint8_t LQ_IIC_Read_Byte(unsigned char ack);

#endif 