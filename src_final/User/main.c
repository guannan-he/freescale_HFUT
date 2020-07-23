/****************************************************************************************************
------------------------------------------------
�Ϸʹ�ҵ��ѧ
��dev.env.��IAR8.2������
��Target  ��K66FX1M0VLQ18
��Crystal �� 50.000Mhz
��busclock��110.000MHz
��pllclock��220.000MHz
=============================================================
=================================
��������  K66��Ƭ���ҽ�     ��ע
-------------------------------------------------------------
LED�ӿڶ��壺
LED1--PTA17
LED2--PTC0
LED3--PTD15 
LED4--PTE26
-------------------------------
//��������ӿڶ��壺
FTM1_QDPHA  PTA12       
FTM1_QDPHB  PTA13       
-------------------------------
PWM�ڵĽӿڶ�����FTM.H�У�
FTM3_CH7  PTC11������ӿ�
FTM0_CH0  PTC1����������ӿ�
FTM0_CH1  PTC2����������ӿ�
-------------------------------------------------------------

******************************************************************************************************/
#include "include.h" 
#include "math.h"

//����������
#define     Step_Right       3625//3525 //����Ƶ�ʸı䣬ֵ��Ҫ���±궨 Լ30��
#define     Step_Middle      2825//2578//2350 //����Ƶ�ʸı䣬��ֵ��Ҫ���±궨
#define     Step_Left        2025//1500 //����Ƶ�ʸı䣬ֵ��Ҫ���±궨 Լ30��
//ȫ���������������-�ι���
#define     encode_limit     400//�ܷɲ�������
#define     stop_count       2//ͣ����ֵ
#define     mtr_gate    20     //���������ֵ
#define     uart_gate   25      //���ڴ�����
#define         mz_gate         500

int SPEED=0;          //�ٶ�ȫ�ֱ���
int ECPULSE=0;        //������ȫ�ֱ���
u32 time_count=0;//���20ms������ÿ֡����
PIDP pid_speed={0.009,0.0075,0.1,10000,0,0,0};//
//{P,I,D,�������ƣ����֣��ϴ�err����ǰerr�����}

extern u8 Field_Over_Flag;//����ͷ���ж�
//�������жϷ���PIT0
void PIT0_Interrupt()
{
  PIT_Flag_Clear(PIT0);    //���жϱ�־λ
  //�������ɼ��ٶ�����
  ECPULSE=FTM_AB_Get(FTM1);//��ȡ�ٶ���ֵ
  LED_Ctrl(LED0, RVS);     //LEDָʾPIT�ж�״̬  
}
//������
 void main(void)
{ 
  ///////////////////////////////////////////*׼���׶Σ�ֻ����ԭ��û�õ�ע�͵��������Լ�����-�ι���*/
  DisableInterrupts;           //�ر��ж�
  PLL_Init(PLL220);            //��ʼ��PLLΪ220M,����Ϊ110M
  UART_Init(UART_4,9600);    //UART��ʼ��
  LED_Init();                  //LED��ʼ��
  LCD_Init();                  //LCD��ʼ��
  KEY_Init();                  //����������ʼ��
  Init_LQ_9AX();                       //�����ǳ�ʼ��
  FTM_AB_Init(FTM1);           //���������ʼ��
  OV7725_Init();               //����ͷ��ʼ��
  LCD_CLS();                   //����Ļ	  
  LCD_Show_HFUTLogo();         //��ʾͼƬ
  LCD_P14x16Str(0,0,"�Ϸʹ�ҵ��ѧ"); //�ַ�����ʾ  
  time_delay_ms(1000); 
  LCD_CLS();                   //����Ļ	
  LCD_Show_Frame_custome();    //��ͼ��80*48��� 
  GPIO_Init(PTA, 25, GPO,0);
  GPIO_Init(PTE, 07, GPO,0);
  EnableInterrupts;            //�����ж�
  Motor_Init(Step_Middle);                 //����������ʼ��
  //UART_Irq_En(UART_4);
  /*************************************************
      ��ʼ�ɻ�


�����������pwm����ר��
alpha0.0
�ι���
ռ�ձȵ��ڼ��㹫ʽ
��1100-duty��/1100=ռ�ձ�
Ҳ����˵duty=1100�������ת
duty=0�����ȫ��
duty=880�����20%


  *********************ȫ�ֱ�����****************************/

  int frame_ps=0;
  char count[8];//֡����ʾ�ֶ�
  char spd_disp[8];//�ٶ���ʾ�ֶ�
  char duty_disp[4];
  char uart_disp[64];//������ʾ�ֶ�
  int road_data[6];//��·��Ϣ��������������
  int center_line[60];
  int k1_state=1;//k1��ʼ
  int k2_state=1;//k2��ͣ
  int k0_state=1;//����
  int Angle;     //����Ƕ�
  int Error;     //������ƫ��
  int Error_pre=0; //ǰ������ƫ��
  int i=0;         //����
  int stop_sign=0;//ֹͣ����
  int target_speed = 300;//Ŀǰ��������
  int current_duty=1050;//Ŀǰת�ձ�
  int uart_counter=0;
  int bit16_data16[9];//���������
  int Ay,Gz,Mz,Mz_init;

  
  LCD_P6x8Str(0,0,"press k1");
  Update9AX(bit16_data16);
  Mz_init=bit16_data16[5];
  Mz=bit16_data16[5];
  
  while(1)/////////////////////////////////////////////////////k1��ʼ
  {
    k1_state=KEY_Read(KEY1);
    k0_state=KEY_Read(KEY0);
    //break;
    if(k0_state==0)
    {time_delay_ms(10);
      
      k0_state=KEY_Read(KEY0);
      if(k0_state==0)
      {
        GPIO_Ctrl(PTE,07,1);
        break;
      }
    }
    if(k1_state==0)
    {time_delay_ms(10);
      
      k1_state=KEY_Read(KEY1);
      if(k1_state==0)
      {
        //GPIO_Ctrl(PTE,07,1);
        break;
      }
    }
  }
  LCD_CLS();
  LCD_P6x8Str(0,0,"wait 2s");
  time_delay_ms(2000);
  LCD_CLS();
  LCD_P6x8Str(0,0,"K2:STOP");
  
  while(1)/////////////////////////�ȴ���ʼ�ź�
  {
    camera(road_data,center_line);
    if(road_data[3]==1)
    {
      break;
    }
  }
  Motor_Ctrl(1000);
  time_delay_ms(1000);

  while(1) ///////////////////////////////////////////////////////////////////// //��ѭ����������������ǵ�ģ�����ȥ����-�ι���
  { 
    pit_time_start(PIT3);///////////��ʱ��ʼ֡
    FTM_AB_clear(FTM1);/////////////���FTM
    camera(road_data,center_line); //////����ͷ���ص�·����
      /*road_data[0][1][2],Զ�н�������ƫ��
        ����������Χ��40
        road_data[3]==0,��ǰ����Ұû����ͣ��
        road_data[3]==1,��ǰ����Ұ����ͣ��
        road_data[4][5]����Զ��*/   
      /////////////////////////////////////////////////////////////////////////////�����￪ʼ����ͣ+ͣ��
      k2_state=KEY_Read(KEY2); 
      if(k2_state==0)////////////////////k2��ͣ
      {
        time_delay_ms(10);
        k2_state=KEY_Read(KEY2);
        if(k2_state==0)
        {
          break;
        }
      }
      if(stop_sign<0)//////////////////////ͣ������
      {
        stop_sign=0;
      }
      if(road_data[3]==1)
      {
        stop_sign+=1;
        //break;
      }
      if(road_data[3]==0)
      {
        stop_sign-=1;
      }
      if(stop_sign>=stop_count)
      {
        break;
      }/////////////////////////////////////////////////////////////////////////////�������������ͣ+ͣ��
    ////////////////////////////////////////////////////////////////////////////////*�������Ӵ���*/
      //������١��������� 
      //if(abs(road_data[0]-road_data[1])>3)
        //Motor_Ctrl(1050);
      //else
       // Motor_Ctrl(1000);
      
      //������ơ���������
      Error=road_data[1]/2;
      if(abs(center_line[20]-center_line[30])<1)
      {
        Angle=Step_Middle;      
        target_speed=600;
       }
       else
       {
         Angle=str_ang_gen(Error,Error_pre,SPEED,center_line);
         target_speed=300;
       }
      if(abs(Mz-Mz_init)>mz_gate)
      {
          i++;
        if(i==1)
        {
          Angle=Step_Right-300;
          Step_Angle_Set(Angle,Step_Left,Step_Right);
          time_delay_ms(1000);
          FTM_AB_clear(FTM1);
        }
        else if(i==2)
        {
          Angle=Step_Right;
          Step_Angle_Set(Angle,Step_Left,Step_Right);
          time_delay_ms(1300);
          FTM_AB_clear(FTM1);
        }
        else if(i==3)
        {
          Angle=Step_Middle;
          Step_Angle_Set(Angle,Step_Left,Step_Right);
          time_delay_ms(1000);
          FTM_AB_clear(FTM1);
          i=0;
        }
      }
      Step_Angle_Set(Angle,Step_Left,Step_Right);
      Error_pre=Error;
      
      
     //�������
    if (abs(target_speed-SPEED)>=mtr_gate)//�ж��������޸�
    {
      pid_speed.errdat = target_speed - SPEED;
      updat_pid(&pid_speed);
      current_duty-=pid_speed.pidout;
      if (current_duty>2000)
      {
       current_duty=2000;
       break;
      }
      else if(current_duty<800)
      {
        current_duty=800;
        break;
      }
      Motor_Ctrl(current_duty);
    }

   
     /////////////////////////////////////////////////////////////////////////////*����ǰ��Ӵ���*/         
    ECPULSE=FTM_AB_Get(FTM1);//���������壬�ٶȼ���
    SPEED=(int)(ECPULSE*45*28*50/256/105*6.28);
    /////////////////�����Ƕ�////////////////
    Update9AX(bit16_data16);
    Ay=bit16_data16[1];
    Gz=bit16_data16[7];
    Mz=bit16_data16[5];
    /////////////////�����Ƕ�////////////////
    if(abs(ECPULSE)>encode_limit)////////////////////////20ms>step_limit��ֹͣ
    {
       break;
    } 
    
    time_count=pit_time_get(PIT3);///////////��ȡ��ʱ��֡��
    pit_time_close(PIT3);
    frame_ps=110000000/time_count;
    sprintf(count,"%02dFPS",frame_ps);
    LCD_P6x8Str(80,1,(u8*)count);
    sprintf(spd_disp,"%4dMM",SPEED);////////////////��ӡ�ٶ�
    LCD_P6x8Str(0,1,(u8*)spd_disp);
    sprintf(duty_disp,"%4d",current_duty);////////////////��ӡ���ռ�ձ�
    LCD_P6x8Str(90,7,(u8*)duty_disp);
    
    uart_counter++;
    if (uart_counter>uart_gate)
    {
      uart_counter=0;
      //sprintf(uart_disp,"%4d\t;%4d\t",SPEED,current_duty);
      //sprintf(uart_disp,"%4d\n",Mz);
      //sprintf(uart_disp,"SPD:%4d--DTY:%4d--ANG:%4d--ERR:%3d--Ay:%6d--Gz:%6d--Mz:%6d\n",SPEED,current_duty,Angle,Error,Ay,Gz,Mz);
      //UART_Put_Str(UART_4,(u8*)uart_disp);
      Mz_init=bit16_data16[5];
    }

  }
////////////////////////�ƺ���/////////////////////////
  Motor_Ctrl_stop();
  Step_Angle_Set(Step_Middle,Step_Left,Step_Right);
  LED_Ctrl(LED1, ON);
  LCD_CLS();
  LCD_P6x8Str(0,0,"STOP");
}






