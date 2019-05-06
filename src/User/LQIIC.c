
#include "include.h"

#define LQ_SDA_IN()   GPIO_Init(PTE,0, 0,1);
#define LQ_SDA_OUT()  GPIO_Init(PTE,0, 1,1);


#define LQ_IIC_SCL        PTE1_OUT 		//SCL
#define LQ_IIC_SDA_OUT    PTE0_OUT		//SDA	 
#define LQ_IIC_SDA_IN     PTE0_IN		//SDA


void LQ_IIC_Delay(void)
{
  delay_us(2);
}


void LQ_IIC_Init(void)
{
  GPIO_Init(PTE,0, 1,1);
  GPIO_Init(PTE,1, 1,1);
}

//����IIC��ʼ�ź�
void LQ_IIC_Start(void)
{
  LQ_SDA_OUT();     //sda�����
  LQ_IIC_SDA_OUT=1;	  	  
  LQ_IIC_SCL=1;
  LQ_IIC_Delay();
  LQ_IIC_SDA_OUT=0;//START:when CLK is high,DATA change form high to low 
  LQ_IIC_Delay();
  LQ_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void LQ_IIC_Stop(void)
{
  LQ_SDA_OUT();//sda�����
  LQ_IIC_SCL=0;
  LQ_IIC_SDA_OUT=0;//STOP:when CLK is high DATA change form low to high
  LQ_IIC_Delay();
  LQ_IIC_SCL=1; 
  LQ_IIC_SDA_OUT=1;//����I2C���߽����ź�
  LQ_IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t LQ_IIC_Wait_Ack(void)
{
  uint8_t ucErrTime=0;
  LQ_SDA_IN();      //SDA����Ϊ����  
  LQ_IIC_Delay();	   
  LQ_IIC_SCL=1;
  LQ_IIC_Delay();	 
  while(LQ_IIC_SDA_IN)
  {
    ucErrTime++;
    if(ucErrTime>250)
    {
      LQ_IIC_Stop();
      return 1;//û��Ӧ���ź�
    }
  }
  LQ_IIC_SCL=0;//ʱ�����0 	   
  return 0;  
} 
//����ACKӦ��
void LQ_IIC_Ack(void)
{
  LQ_IIC_SCL=0;
  LQ_SDA_OUT();
  LQ_IIC_SDA_OUT=0;
  LQ_IIC_Delay();
  LQ_IIC_SCL=1;
  LQ_IIC_Delay();
  LQ_IIC_SCL=0;
}
//������ACKӦ��		    
void LQ_IIC_NAck(void)
{
  LQ_IIC_SCL=0;
  LQ_SDA_OUT();
  LQ_IIC_SDA_OUT=1;
  LQ_IIC_Delay();
  LQ_IIC_SCL=1;
  LQ_IIC_Delay();
  LQ_IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void LQ_IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
  LQ_SDA_OUT(); 	    
  LQ_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
  for(t=0;t<8;t++)
  {              
    LQ_IIC_SDA_OUT=(txd&0x80)>>7;
    txd<<=1; 	  
    LQ_IIC_SCL=1;
    LQ_IIC_Delay(); 
    LQ_IIC_SCL=0;	
    LQ_IIC_Delay();
  }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t LQ_IIC_Read_Byte(unsigned char ack)
{
  unsigned char i,receive=0;
  LQ_SDA_IN();//SDA����Ϊ����
  for(i=0;i<8;i++ )
  {
    LQ_IIC_SCL=0; 
    LQ_IIC_Delay();
    LQ_IIC_SCL=1;
    receive<<=1;
    if(LQ_IIC_SDA_IN)receive++;   
    LQ_IIC_Delay(); 
  }					 
  if (!ack)
    LQ_IIC_NAck();//����nACK
  else
    LQ_IIC_Ack(); //����ACK   
  return receive;
}