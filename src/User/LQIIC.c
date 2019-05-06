
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

//产生IIC起始信号
void LQ_IIC_Start(void)
{
  LQ_SDA_OUT();     //sda线输出
  LQ_IIC_SDA_OUT=1;	  	  
  LQ_IIC_SCL=1;
  LQ_IIC_Delay();
  LQ_IIC_SDA_OUT=0;//START:when CLK is high,DATA change form high to low 
  LQ_IIC_Delay();
  LQ_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void LQ_IIC_Stop(void)
{
  LQ_SDA_OUT();//sda线输出
  LQ_IIC_SCL=0;
  LQ_IIC_SDA_OUT=0;//STOP:when CLK is high DATA change form low to high
  LQ_IIC_Delay();
  LQ_IIC_SCL=1; 
  LQ_IIC_SDA_OUT=1;//发送I2C总线结束信号
  LQ_IIC_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t LQ_IIC_Wait_Ack(void)
{
  uint8_t ucErrTime=0;
  LQ_SDA_IN();      //SDA设置为输入  
  LQ_IIC_Delay();	   
  LQ_IIC_SCL=1;
  LQ_IIC_Delay();	 
  while(LQ_IIC_SDA_IN)
  {
    ucErrTime++;
    if(ucErrTime>250)
    {
      LQ_IIC_Stop();
      return 1;//没有应答信号
    }
  }
  LQ_IIC_SCL=0;//时钟输出0 	   
  return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void LQ_IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
  LQ_SDA_OUT(); 	    
  LQ_IIC_SCL=0;//拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t LQ_IIC_Read_Byte(unsigned char ack)
{
  unsigned char i,receive=0;
  LQ_SDA_IN();//SDA设置为输入
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
    LQ_IIC_NAck();//发送nACK
  else
    LQ_IIC_Ack(); //发送ACK   
  return receive;
}