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
#include "include.h"

volatile u8   Image_Data[R240][L320];
volatile u8   Image_Use [R120][L160];            //�޳�֮�������
          u8   Pixle[R120][L160]; 
 
u8 control_f=0;
u8 control_f_cross=0;
u8 control_hang;
u8 motor_f=0;
u8 da_S=0;
u8 straight_f=0;  
u8 S_f=0;
u8 hang;
u8 line_route = 0;
u8 Rights[60];
u8 Lefts[60];
u8 center_line[R120];u8 hang; 
s16  Dir_Ctr_Out = 0;
s16  Dir_Error = 0;

u16  ROW_Space = 0;
u16  Line_Cont = 0;

u8   VSY_Flag=0;
u8   Cam_Flag= 0;

int  OFFSET0=0;      //��Զ������������ֵ�ۺ�ƫ����
int  OFFSET1=0;      //�ڶ���
int  OFFSET2=0;      //�����������
int  TXV=0;//���ε���߶ȣ��Ҹ߶�

//OV7620����ͷͼ��ɼ��жϴ���������ͼ����ƫ�ƣ��������
void PORTD_Interrupt(void)
{
  int n;  
  n=14;   //���ж�
  if((PORTD_ISFR & (1<<n)))
  {
      PORTD_ISFR |= (1<<n); 
      /* �û���������ж��ڳ��� */          
      if(VSY_Flag==0)
        {
            return;
        }
        //DMA�ɼ����������޸Ĳɼ��ӿ�PTD_BYTE0_IN��D0--D7��   PLCK�ӵ���PTD13��
        DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN, (void *)Image_Data[Line_Cont], PTD13, DMA_BYTE1, 320, DMA_rising);
        DMA_EN(DMA_CH4);        
        Line_Cont++;
        
        if(Line_Cont==R240)             //�ɼ�����
        {
           Line_Cont = 0;
           VSY_Flag=0;
           Cam_Flag = 1;
           DMA_DIS(DMA_CH4);
           DisableInterrupts;           
        }       
    }
  n=15;  //���ж�
  if((PORTD_ISFR & (1<<n)))
  {
      PORTD_ISFR |= (1<<n); 
      /* �û���������ж��ڳ��� */
      Line_Cont = 0;
      VSY_Flag=1;
  } 
}

//����������
void TEST_LQV034(void)
{  
  while(1)
  { 
    LED_Ctrl(LED1, RVS);       //LEDָʾ��������״̬
    if(Cam_Flag)
    {
      Get_Use_Image();  
      Draw_Road(); 
      Cam_Flag  = 0;
      EnableInterrupts
    }    
  }
}

// MT9V034 Port Init
void Cam_Init(void)
{
    u16 i=0,j=0;
       
      EXTI_Init(PTD,14,rising_down);   //���ж�
      EXTI_Init(PTD,15,falling_up);    //���ж� 

       for(i=0; i<R240; i++)
       {
          for(j=0;j< L320;j++)
          {
             Image_Data[i][j] = 0;
          }
        } 
}
void MT9V034_Init(void)
{   
    SCCB_Init();
    SCB_RegWrite(0x42,0x11,0x03);   
    SCB_RegWrite(0x42,0x14,0x24);    
    SCB_RegWrite(0x42,0x28,0x20);   
    SCCB_Wait();    
    Cam_Init();   
}
// ��ȡ��Ҫ��ͼ������
void Get_Use_Image(void)
{
  int i = 0,j = 0,row = 0,line = 0;
  
  for(i = 0; i  < R240; i+=2)  //240�У�ÿ2�вɼ�һ�У�
  {
      for(j = 0;j < L320; j+=2)//320��ÿ2�вɼ�һ�У�
      {        
          Image_Use[row][line] = Image_Data[i][j];         
          line++;        
      }      
      line = 0;
      row++;      
  }
  row = 0; 
}
void Get_01_Value(void)
{
  int i = 0,j = 0;
  u8 GaveValue;
  u32 tv=0;
  char txt[16];
  
  //����ÿ�еľ�ֵ
  for(i = 0; i <R120; i++)
  {    
    for(j = 0; j <L160; j++)
    {                            
      tv+=Image_Use[i][j];   //�ۼ�  
    } 
  }
  GaveValue=tv/R120/L160;     //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100 
  sprintf(txt,"%03d",GaveValue);//  
  TFT_P8X16Str(102,1,(uint8*)txt,WHITE,BLACK);
  //���վ�ֵ�ı������ж�ֵ��
  GaveValue=GaveValue*9/10+10;        //�˴���ֵ���ã����ݻ����Ĺ������趨 
  for(i = 0; i < R120; i++)
  {
    for(j = 0; j < L160; j++)
    {                                
      if(Image_Use[i][j] >GaveValue) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����    
        Pixle[i][j] = 1;        
      else                                        
        Pixle[i][j] = 0;
    }    
  }
}

//�������Ϸ���Χ��������
void Pixle_Filter(void)
{  
  int nr; //��
  int nc; //��
  
  for(nr=1; nr<R120-1; nr++)
  {  	    
    for(nc=1; nc<L160-1; nc=nc+1)
    {
      if((Pixle[nr][nc]==0)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]>2))         
      {
        Pixle[nr][nc]=1;
      }	
      else if((Pixle[nr][nc]==1)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]<2))         
      {
        Pixle[nr][nc]=0;
      }	
    }	  
  }  
}
void Draw_Road(void)
{ 	 
  u8 i = 0, j = 0;
  
  //����֡ͷ��־
  TFT_SetPos(0,8,160,128);
  for(i = 0; i < R120; i++)
  {
    for(j = 0; j < L160; j++)
    {                                
     TFT_write_para16(Image_Use[i][j]); //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ���� 
    }    
  }
}

/***************************************************************************
*                                                                          *
*  �������ƣ�int Seek_Road(void)                                           *
*  ����˵����Ѱ�Ұ�ɫ����ƫ��ֵ                                            *
*  ����˵������                                                            *
*  �������أ�ֵ�Ĵ�С                                                      *
*  �޸�ʱ�䣺2017-07-16                                                    *
*  ��    ע�����м�Ϊ0������һ���Ҳ��һ����ֵ����1�����                *
*            ��������ӵ�һ�п�ʼ�������ڶ��н�����                        *
*            ������Ϊ��������ֵԽ��˵��Խƫ��ߣ�                        *
*            ������Ϊ��������ֵԽ��˵��Խƫ�ұߡ�                        *
***************************************************************************/ 
void Seek_Road(void)
{  
    int nr; //��
    int nc; //��
    int temp=0;//��ʱ��ֵ
    //for(nr=1; nr<MAX_ROW-1; nr++)
    temp=0;
    for(nr=8; nr<24; nr++)
    {  	    
	    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	++temp;
			    }			   
	    }
	    for(nc=0; nc<MAX_COL/2; nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	--temp;
			    }			   
	    }		  
  	}
  	OFFSET0=temp;
  	temp=0;
  	for(nr=24; nr<40; nr++)
    {  	    
	    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	++temp;
			    }			   
	    }
	    for(nc=0; nc<MAX_COL/2; nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	--temp;
			    }			   
	    }		  
  	}
  	OFFSET1=temp;    	
  	temp=0;
  	for(nr=40; nr<56; nr++)
    {  	    
	    for(nc=MAX_COL/2;nc<MAX_COL;nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	++temp;
			    }			   
	    }
	    for(nc=0; nc<MAX_COL/2; nc=nc+1)
	    {
			    if(Pixle[nr][nc])
			    {
			      	--temp;
			    }			   
	    }		  
  	}
  	OFFSET2=temp;   	
  	return;  
}

u8 zb[48],yb[48];
void findtixing(void)
{
    int nr; //��
    int nc; //��     
     
    for(nr=0; nr<48; nr++)
    {  	    
	zb[nr]=0;
        yb[nr]=100;   
    }  	
    for(nr=0; nr<48; nr++)
    {  	    
	    for(nc=2;nc<MAX_COL-2;nc++)
	    {
		   if((Pixle[nr+8][nc-1]==0)&&(Pixle[nr+8][nc]==0)&&(Pixle[nr+8][nc+1]==1)&&(Pixle[nr+8][nc+2]==1))
		    {
		      	zb[nr]=nc;//����أ�Խ��Խ��
		    }
                   if((Pixle[nr+8][nc-1]==1)&&(Pixle[nr+8][nc]==1)&&(Pixle[nr+8][nc+1]==0)&&(Pixle[nr+8][nc+2]==0))
		    {
		      	yb[nr]=nc;//�ұ��أ�Խ��ԽС
		    }                   
	    }	    
    }
    TXV=0;
    for(nr=0; nr<47; nr++)
    {  	    
	if((zb[nr]>=zb[nr+1])&&(zb[nr]>0))   TXV++;          
        if((yb[nr]<=yb[nr+1])&&(yb[nr]<100)) TXV--;          
    }  	   
    return;  
}

/*************************************************************************
*                    �����������ܿƼ� K60������           
*
*  �������ƣ�void SCCB_Init(void)
*  ����˵��������SCCB��������ΪGPIO���ܣ���ʱ���������ݷ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014��1��5��
*  ��    ע��
*************************************************************************/
void SCCB_Init(void)
{
  GPIO_Init(PTE, 0,GPO,0);   //����ΪGPIO����
  GPIO_Init(PTE, 1,GPO,0);//����ΪGPIO����

}

/*************************************************************************
*                    �����������ܿƼ� K60������           
*
*  �������ƣ�void SCCB_Wait(void)
*  ����˵����SCCB�ȴ���ʾ
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014��1��5��
*  ��    ע��
*************************************************************************/
void SCCB_Wait(void)
{
  uint8 i=0;
 for(i=0;i<200;i++)
  { 
    asm ("nop");
  }

}

/*************************************************************************
*                    �����������ܿƼ� K60������           
*
*  �������ƣ�void SCCB_Star(void)
*  ����˵������������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014��1��5��
*  ��    ע��
*************************************************************************/
void SCCB_Star(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_High;
  SCL_High;  
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_Low;

}
/*************************************************************************
*                    �����������ܿƼ� K60������           
*
*  �������ƣ�void SCCB_Stop(void)
*  ����˵����ֹͣ����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014��1��5��
*  ��    ע��
*************************************************************************/
void SCCB_Stop(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_High;
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();

}
/*************************************************************************
*                    �����������ܿƼ� K60������           
*
*  �������ƣ�uint8 SCCB_SendByte(uint8 Data)
*  ����˵����SCCB�����ֽں���
*  ����˵����Ҫ���͵��ֽ�
*  �������أ�Ӧ���ź�
*  �޸�ʱ�䣺2014��1��5��
*  ��    ע��
*************************************************************************/
uint8 SCCB_SendByte(uint8 Data)
{
   uint8 i;
   uint8 Ack;
  SDA_Out;
  for( i=0; i<8; i++)
   {
    if(Data & 0x80) SDA_High;
    else            SDA_Low;
    
    Data <<= 1;
    SCCB_Wait();
    SCL_High;
    
    SCCB_Wait();
    SCL_Low;
    SCCB_Wait();
  }
  SDA_High;
  SDA_In;
  SCCB_Wait();
  
  SCL_High;
  SCCB_Wait();
  Ack = SDA_Data;
  SCL_Low;
  SCCB_Wait();
  return Ack;
}

/*************************************************************************
*                    �����������ܿƼ� K60������           
*
*  �������ƣ�void SCB_RegWrite(uint8 Device,uint8 Address,uint8 Data)
*  ����˵�������豸д����
*  ����˵����Ҫ���͵��ֽ�
*  �������أ�Ӧ���ź�
*  �޸�ʱ�䣺2014��1��5��
*  ��    ע��
*************************************************************************/
void SCB_RegWrite(uint8 Device,uint8 Address,uint8 Data)
{
  uint8 i;
  uint8 Ack;
  for( i=0; i<20; i++)
  {
    SCCB_Star();
    Ack = SCCB_SendByte(Device);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte(Address);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte(Data);
    if( Ack == 1 )
    {
      continue;
    }
    
    SCCB_Stop();
    if( Ack == 0 ) break;
  }
}
