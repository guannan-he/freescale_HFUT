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

#include "include.h"

#define COUNTSADDR   0x4004000C  //(&counttempaddr)
#define COUNTDADDR   0x4004000C  //(&counttempaddr)

u32 count_init[16];         //��������16��ͨ���ĳ�ʼ������ֵ

extern GPIO_MemMapPtr GPIOX[5];
extern PORT_MemMapPtr PORTX[5];
/*************************************************************************
*
*  �������ƣ�DMA_PORTx2BUFF_Init
*  ����˵����DMA��ʼ������ȡ�˿����ݵ��ڴ�
*  ����˵����DMA_CHn              ͨ���ţ�DMA_CH0 ~ DMA_CH15��
*            SADDR                Դ��ַ( (void * )&PTx_BYTEn_IN �� (void * )&PTx_WORDn_IN   )
*            DADDR                Ŀ�ĵ�ַ
*            PTxn                 �����˿�
*            DMA_BYTEn            ÿ��DMA�����ֽ���
*            count                һ����ѭ�������ֽ���
*            DMA_PORTx2BUFF_cfg   DMA��������
*  �������أ���
*  �޸�ʱ�䣺2012-2-20
*  ��    ע��
*************************************************************************/
void DMA_PORTx2BUFF_Init(DMA_CHn CHn, void *SADDR, void *DADDR, PTXn_e ptxn, DMA_BYTEn byten, u32 count, DMA_PORTx2BUFF_cfg cfg)
{
    u8 n, i, tmp;

    u8 BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //���㴫���ֽ���

    /* ����ʱ�� */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;                     //��DMA��·������ʱ��

    /* ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (u32)SADDR;                         // ����  Դ��ַ
    DMA_DADDR(CHn) =    (u32)DADDR;                         // ����Ŀ�ĵ�ַ
    DMA_SOFF(CHn)  =    0x00u;                              // ����Դ��ַƫ�� = 0x0, ������
    DMA_DOFF(CHn)  =    BYTEs;                              // ÿ�δ����Ŀ�ĵ�ַ�� BYTEs

    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)                // Դ��ַģ����ֹ  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(byten)             // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)                // Ŀ���ַģ����ֹ
                         | DMA_ATTR_DSIZE(byten)             // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
                        );

    DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //��ǰ��ѭ������ 
    DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKYES_BITER(count);//��ʼ��ѭ������


    DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0

    //��CR[EMLM] = 0 ʱ:
    DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // ͨ��ÿ�δ����ֽ�������������ΪBYTEs���ֽڡ�ע��ֵΪ0��ʾ����4GB */


    /* ���� DMA ���������Ĳ��� */
    DMA_SLAST(CHn)      =   0;                              //����  Դ��ַ�ĸ���ֵ,��ѭ��������ָ�  Դ��ַ
    DMA_DLAST_SGA(CHn)  =   (u32)( (cfg & 0x20) == 0 ? (-count)  : 0 ); //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
    DMA_CSR(CHn)        =   (0
                             | DMA_CSR_DREQ_MASK            //��ѭ��������ֹͣӲ������
                             | DMA_CSR_INTMAJOR_MASK        //��ѭ������������ж�
                            );

    /* ���� DMA ����Դ */
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            //| DMAMUX_CHCFG_TRIG_MASK                        /* Trigger Mode: Periodic   PIT���ڴ�������ģʽ   ͨ��1��ӦPIT1������ʹ��PIT1����������Ӧ��PIT��ʱ���� */
            | DMAMUX_CHCFG_SOURCE((ptxn >> 5) + DMA_Port_A) /* ͨ����������Դ:     */
                                             );

    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << (ptxn>>5));                                                               //����PORTx�˿�
    GPIO_PDDR_REG(GPIOX[(ptxn>>5)]) &= ~(1 << (ptxn & 0x1f));                                                       //���ö˿ڷ���Ϊ����
    PORT_PCR_REG(PORTX[(ptxn>>5)], (ptxn & 0x1F)) = ( 0
            | PORT_PCR_MUX(1)               // ����GPIO
            | PORT_PCR_IRQC(cfg & 0x03 )    // ȷ������ģʽ
            | ((cfg & 0xc0 ) >> 6)          // �����������������裬����û��
                                                    );
    GPIO_PDDR_REG(GPIOX[(ptxn>>5)]) &= ~(1 << (ptxn && 0x1F));                                                      //����ģʽ

    /*  ��������Դ   */
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << ((((u32)SADDR) & 0x1ff)>>6));             //����PORTx�˿�
    switch(byten)
    {
    case DMA_BYTE1:
        *((u8 *)((u32)SADDR + 4)) = 0;   //����Ϊ���뷽��Ϊʲô��4��PDIR��ַ��4�󣬾ͱ�ɶ�Ӧ��PDDR��ַ
        break;
    case DMA_BYTE2:
        *((u16 *)((u32)SADDR + 4)) = 0;
        break;
    case DMA_BYTE4:
        *((u32 *)((u32)SADDR + 4)) = 0;
        break;
    default:
        ;//assert_failed(__FILE__, __LINE__);
        break;
    }

    /*  ����Դ�ܽ�ѡ���ܽ�  */
    n = (u8)(((u32)SADDR - ((u32)(&PTA_BYTE0_IN))) & 0x3f);         //��С�����ź�
    tmp = n + (BYTEs << 3);                                         //�������ź�
    for(i = n; i < tmp; i++)
    {
        PORT_PCR_REG(PORTX[   ((((u32)SADDR)&0x1ff)>>6)    ], i) = (0
                | PORT_PCR_MUX(1)
                | 0X02             //����ԴӦ��������Ĭ�϶�ȡ������0
                                                                   );
    }

    /* �����ж� */
    DMA_EN(CHn);                                    //ʹ��ͨ��CHn Ӳ������
    //DMA_IRQ_EN(CHn);                                //����DMAͨ������
}


/*************************************************************************
*
*  �������ƣ�DMA_count_Init
*  ����˵����DMA�ۼӼ�����ʼ��
*  ����˵����DMA_CHn              ͨ���ţ�DMA_CH0 ~ DMA_CH15��
*            PTxn                 �����˿�
*            count                �ۼӼ����ж�ֵ
*            DMA_Count_cfg        DMA��������
*  �������أ���
*  �޸�ʱ�䣺2012-1-20
*  ��    ע��
*************************************************************************/
void DMA_Count_Init(DMA_CHn CHn, PTXn_e ptxn, u32 count, DMA_Count_cfg cfg)
{
    u8 byten = DMA_BYTE1;
    u8 BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //���㴫���ֽ���
    if(count > 0x7FFF )count = 0x7FFF;
    count_init[CHn] = count;

    /* ����ʱ�� */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;                     //��DMA��·������ʱ��

    /* ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (u32)COUNTSADDR;                    // ����  Դ��ַ
    DMA_DADDR(CHn) =    (u32)COUNTDADDR;                    // ����Ŀ�ĵ�ַ
    DMA_SOFF(CHn)  =    0;                                  // ����Դ��ַ����
    DMA_DOFF(CHn)  =    0;                                  // ÿ�δ����Ŀ�ĵ�ַ����

    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)                // Դ��ַģ����ֹ  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(byten)             // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)                // Ŀ���ַģ����ֹ
                         | DMA_ATTR_DSIZE(byten)             // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
                        );

    DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //��ǰ��ѭ������
    DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKYES_BITER(count);//��ʼ��ѭ������

    DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0

    DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // ͨ��ÿ�δ����ֽ�������������ΪBYTEs���ֽڡ�ע��ֵΪ0��ʾ����4GB */

    /* ���� DMA ���������Ĳ��� */
    DMA_SLAST(CHn)      =   -count;                              //����  Դ��ַ�ĸ���ֵ,��ѭ��������ָ�  Դ��ַ
    DMA_DLAST_SGA(CHn)  =   0;                                  //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
    DMA_CSR(CHn)        =   (0
                             | DMA_CSR_DREQ_MASK            //��ѭ��������ֹͣӲ������
                             | DMA_CSR_INTMAJOR_MASK        //��ѭ������������ж�
                            );

    /* ���� DMA ����Դ */
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            | DMAMUX_CHCFG_SOURCE((ptxn >> 5) + DMA_Port_A) /* ͨ����������Դ:     */
                                             );

    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << (ptxn>>5));                                                               //����PORTx�˿�
    GPIO_PDDR_REG(GPIOX[(ptxn>>5)]) &= ~(1 << (ptxn & 0x1f));                                                       //���ö˿ڷ���Ϊ����
    PORT_PCR_REG(PORTX[(ptxn>>5)], (ptxn & 0x1F)) = ( 0
            | PORT_PCR_MUX(1)               // ����GPIO
            | PORT_PCR_IRQC(cfg & 0x03 )    // ȷ������ģʽ
            | ((cfg & 0xc0 ) >> 6)          // �����������������裬����û��
                                                    );
    GPIO_PDDR_REG(GPIOX[(ptxn>>5)]) &= ~(1 << (ptxn && 0x1F));                                                      //����ģʽ

    /* �����ж� */
    DMA_EN(CHn);                                    //ʹ��ͨ��CHn Ӳ������
    //DMA_IRQ_EN(CHn);                                //����DMAͨ������
}
/*************************************************************** 

* 
* �������ƣ�void DMATransDataStart(uint8_t CHn,uint32_t address) 
* ����˵��������DMA�����Ŀ�ĵ�ַ
* ����˵���� 
* �������أ���
* �޸�ʱ�䣺2018��3��27�� 
* �� ע�� 
***************************************************************/ 
__ramfunc void DMATransDataStart(uint8_t CHn,uint32_t address) 
{ 
  DMA_DADDR(CHn) = address;                              //Ŀ�ĵ�ַ    
  DMA_EN(CHn);                                           //ʹ��ͨ��CHn Ӳ������
  //DMA_IRQ_EN(CHn);                                     //����DMAͨ������
}
__ramfunc void DMATransDataStart2(uint8_t CHn,uint32_t address,uint32_t Val) 
{ 
  DMA_DADDR(CHn) = address;                              //Ŀ�ĵ�ַ 
  DMA_CITER_ELINKNO(CHn) = DMA_CITER_ELINKNO_CITER(Val); //��ǰ��ѭ������ 
  DMA_BITER_ELINKNO(CHn) = DMA_BITER_ELINKYES_BITER(Val);//��ʼ��ѭ������ 
  //�����ж� 
  DMA_EN(CHn);                                           //ʹ��ͨ��CHn Ӳ������
  //DMA_IRQ_EN(CHn);                                     //����DMAͨ������
} 
/**************************************************************************                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�DMA_count_get
*  ����˵���������ۼӼ���ֵ
*  ����˵����DMA_CHn              ͨ���ţ�DMA_CH0 ~ DMA_CH15��
*  �������أ��ۼӼ���ֵ
*  �޸�ʱ�䣺2012-3-320
*  ��    ע��
*************************************************************************/
u32 DMA_Count_Get(DMA_CHn CHn)
{
    u32 temp =  count_init[CHn] - DMA_CITER_ELINKNO(CHn)  ;
    return temp;
}

void DMA_Count_Reset(DMA_CHn CHn)
{
    DMA_CITER_ELINKNO(CHn) = count_init[CHn] ;
}


void DMA_CH4_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH4);                                 //���ͨ�������жϱ�־λ    (���������ٴν����ж�)
    DMA_DIS(DMA_CH4);                                       //�ɼ���H�����ݺ�������DMA�жϣ�ֹͣDMA���䡣���ж��д�DMA����
    /********************/
    //���ڵ����õ�
   // if(V_Cnt >= 319)
   // {
    //  V_Cnt = 0;
     // Is_SendPhoto = 1;
  //  }
      
    /*******************/
}
