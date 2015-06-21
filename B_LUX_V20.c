//***************************************
// B_LUX_V20�ɼ�����
//****************************************
#include  <math.h>    //Keil library  
#include  <stdio.h>   //Keil library
#include "B_LUX_V20.h"
#include  <LCD1602.H>

sbit      LCM_RS=P2^0;   //LCD1602����˿�		
sbit      LCM_RW=P2^1;   //LCD1602����˿�		
sbit      LCM_EN=P2^2;   //LCD1602����˿� 
#define uchar unsigned char
#define uint  unsigned int
uint8    BUF_0[8];                       //�������ݻ�����      	
uint16   dis_data_0;                     //����


 
sbit  BEEP = P3^6;       //������
sbit ZZ=P1^0;//���ƶ�
sbit FZ=P1^1;//���ƶ�
sbit Key_UP=P3^2;   //��ת���� ��Ӧʵ����ϵ�K1
sbit Key_DOWN=P1^4;//��ת����  ��Ӧʵ����ϵ�K5
sbit Key_STOP=P1^5;//ֹͣ��    ��Ӧʵ����ϵ�K6
unsigned char KeyV,TempKeyV;      

/*---------------------------------------------------------------------
 ��������: ��ʱ���� ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_delay_nms(uint16 k)	
{						
  uint16 i,j;				
  for(i=0;i<k;i++)
  {			
    for(j=0;j<6000;j++)			
    {
      ;
    }
  }						
}					

/*---------------------------------------------------------------------
 ��������: ��ʱ5΢��  ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Delay5us()
{
  uint8 n = 1;
  
  while (n--);
}

/*---------------------------------------------------------------------
 ��������: ��ʱ5����  ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Delay5ms()
{
  uint16 n = 60000;
  
  while (n--);
}

/*---------------------------------------------------------------------
 ��������: ��ʼ�ź�
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Start()
{
  B_LUX_SDA0_H;                    //����������
  B_LUX_SCL0_H;                    //����ʱ����
  B_LUX_Delay5us();                 //��ʱ
  B_LUX_SDA0_L;                    //�����½���
  B_LUX_Delay5us();                 //��ʱ
  B_LUX_SCL0_L;                    //����ʱ����
}

/*---------------------------------------------------------------------
 ��������: ֹͣ�ź�
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Stop()
{
  B_LUX_SDA0_L;                    //����������
  B_LUX_SCL0_H;                    //����ʱ����
  B_LUX_Delay5us();                 //��ʱ
  B_LUX_SDA0_H;                    //����������
  B_LUX_Delay5us();                 //��ʱ
  B_LUX_SCL0_L;
  B_LUX_Delay5us();
}

/*---------------------------------------------------------------------
 ��������: ����Ӧ���ź�
 ����˵��: ack - Ӧ���ź�(0:ACK 1:NAK)
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_SendACK(uint8 ack)
{
  if (ack&0x01)	B_LUX_SDA0_H;		 //дӦ���ź�
  else	B_LUX_SDA0_L;
  
  B_LUX_SCL0_H;                    //����ʱ����
  B_LUX_Delay5us();                 //��ʱ
  B_LUX_SCL0_L;                    //����ʱ����
  B_LUX_SDA0_H;
  B_LUX_Delay5us();                 //��ʱ
}

/*---------------------------------------------------------------------
 ��������: ����Ӧ���ź�
 ����˵��: ��
 ��������: ����Ӧ���ź�
 ---------------------------------------------------------------------*/
uint8 B_LUX_RecvACK()
{
  uint8 CY = 0x00;
  B_LUX_SDA0_H;
  
  B_LUX_SDA0_I;
  
  B_LUX_SCL0_H;              //����ʱ����
  B_LUX_Delay5us();                 //��ʱ
  
  
  CY |= B_LUX_SDA0_DAT;    //��Ӧ���ź�
  
  B_LUX_Delay5us();                 //��ʱ
  
  B_LUX_SCL0_L;              //����ʱ����
  
  B_LUX_SDA0_O;
  
  return CY;
}

/*---------------------------------------------------------------------
 ��������: ��IIC���߷���һ���ֽ�����
 ����˵��: dat - д�ֽ�
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_SendByte(uint8 dat)
{
  uint8 i;
  
  for (i=0; i<8; i++)         			//8λ������
  {
    if (dat&0x80)	B_LUX_SDA0_H;
    else	B_LUX_SDA0_L;              //�����ݿ�
    
    B_LUX_Delay5us();             			//��ʱ
    B_LUX_SCL0_H;                		//����ʱ����
    B_LUX_Delay5us();             			//��ʱ
    B_LUX_SCL0_L;                		//����ʱ����
    B_LUX_Delay5us();             			//��ʱ
    dat <<= 1;              			//�Ƴ����ݵ����λ
  }
  
  B_LUX_RecvACK();
}

/*---------------------------------------------------------------------
 ��������: ��IIC���߽���һ���ֽ�����
 ����˵��: ��
 ��������: �����ֽ�
 ---------------------------------------------------------------------*/
uint8 B_LUX_RecvByte()
{
  uint8 i;
  uint8 dat = 0;
  B_LUX_SDA0_I;
  
  B_LUX_SDA0_H;                  //ʹ���ڲ�����,׼����ȡ����,
  for (i=0; i<8; i++)         	//8λ������
  {
    B_LUX_SCL0_H;              //����ʱ����
    B_LUX_Delay5us();             	//��ʱ
    dat |= B_LUX_SDA0_DAT;     //������               
    B_LUX_SCL0_L;              //����ʱ����
    B_LUX_Delay5us();             	//��ʱ
    
    dat <<= 1;	
  }
  B_LUX_SDA0_O;
  
  return dat;
}

/*---------------------------------------------------------------------
 ��������: дBH1750
 ����˵��: REG_Address - �Ĵ�����ַ
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Single_Write(uint8 REG_Address)
{
  B_LUX_Start();                  //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress);   //�����豸��ַ+д�ź�
  B_LUX_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
  //  BH1750_SendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf22ҳ 
  B_LUX_Stop();                   //����ֹͣ�ź�
}

/*---------------------------------------------------------------------
 ��������: ��������BH1750�ڲ�����
 ����˵��: ��
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Multiple_read(void)
{   
  uint8 i;	
  B_LUX_Start();                          //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress+1);         //�����豸��ַ+���ź�
  
  for (i=0; i<3; i++)                        //������ȡ6����ַ���ݣ��洢��BUF
  {
    BUF_0[i] = B_LUX_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
    if (i == 3)
    {
      
      B_LUX_SendACK(1);                   //���һ��������Ҫ��NOACK
    }
    else
    {		
      B_LUX_SendACK(0);                   //��ӦACK
    }
  }
  
  B_LUX_Stop();                           //ֹͣ�ź�
  B_LUX_Delay5ms();
}

/*---------------------------------------------------------------------
 ��������: ��ʼ�����մ�����
 ����˵��: ��
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Init()
{
  
  //P1SEL &= ~(0x48);
  
  B_LUX_SCL0_O;
  B_LUX_SDA0_O;
  
  B_LUX_delay_nms(100);	    //��ʱ100ms
  
  B_LUX_Single_Write(0x01); 
  
}

/*---------------------------------------------------------------------
 ��������: ���ն�ȡ����
 ����˵��: ��
 ��������: ���ع���ֵ
 ---------------------------------------------------------------------*/
uint32 B_LUX_GetLux()
{  
  fint32 temp;
  B_LUX_Single_Write(0x01);   // power on
  B_LUX_Single_Write(0x10);   // H- resolution mode 
  
  B_LUX_delay_nms(180);       //��ʱ180ms
  
  B_LUX_Multiple_read();      //�����������ݣ��洢��BUF��
  
  B_LUX_Single_Write(0x00);   // power off
  
  dis_data_0=BUF_0[0];
  dis_data_0=(dis_data_0<<8)+BUF_0[1];//�ϳ����ݣ�����������
  
  temp=(float)dis_data_0/1.2;
  return (uint32)(temp*1.4);
}


void delay1ms()
{
   unsigned char i,j;	
	 for(i=0;i<10;i++)
	  for(j=0;j<33;j++)
	   ;		 
}


  void delay(unsigned char n)
 {
   unsigned char i;
	for(i=0;i<n;i++)
	   delay1ms();
 }

void  motor_ffw()
 { 
   uchar i;
   uint  j;
   for (j=0; j<8; j++)         //ת1*nȦ 
    { 
	    
      for (i=0; i<8; i++)       //һ������ת45��
        {
          P1 = FFW[i];          //ȡ����
          delay(2);            //����ת��
        }
    }
 }

 void  motor_rev()
{
     uchar i;
	 uint  j;
	 for (j=0; j<8; j++)       //ת1��nȦ
      {
	    
        for (i=0; i<8; i++)     //һ������ת45��
        {
          P1 = REV[i];          //ȡ����
          delay(2);            //����ת��
        }
      }
 }

void delayB(uchar x)    //x*0.14MS
 {
   uchar i;
   while(x--)
   {
     for (i=0; i<13; i++)
     { }
   }
 }

void beep()
 {
   uchar i;
   for (i=0;i<100;i++)
    { 
     delayB(4);
     BEEP=!BEEP;                 //BEEPȡ��
    } 
     BEEP=1;                    //�رշ�����
 }





main()

{
	unsigned char code digit[ ]={"0123456789"};
	unsigned char D1,D2,D3,D4; 
	uint32 x;
	uchar r,N=640;
	
	B_LUX_Start();
	B_LUX_Init();
	B_LUX_GetLux();
	
	x= B_LUX_GetLux();
	D1=x%10;              //�����λ����
	D2=(x%100)/10;        //����ʮλ����
	D3= (x%1000)/100;        //�����λ����
	D4= x/1000	;
				

	LCD_Initial();
	GotoXY(0,0);
	LCD_Write(1,digit[D4]);
	LCD_Write(1,digit[D3]);
	LCD_Write(1,digit[D2]);
	LCD_Write(1,digit[D1]);
	Print("   LUX");
   	 
	 
	 FZ=1;
     ZZ=1;
	 if(x>200)	Key_UP=0;
	 
	 else       Key_UP=1;
	 
	 if(x<50)	Key_DOWN=0;
	 else       Key_DOWN=1;
	 
	 

    // while(1){
 	if (!Key_UP)
    KeyV = 1;
  if (!Key_DOWN)
    KeyV = 2;
	    if (!Key_STOP)
    KeyV = 3;
   if (KeyV!= 0)     
    {
      delay(10);   
      if (!Key_UP)
        TempKeyV = 1;	
      if (!Key_DOWN)
        TempKeyV = 2;	
      if (!Key_STOP)
        TempKeyV = 3;
		 if (KeyV == TempKeyV)  
	   {
		 
	   	   if (KeyV == 1){
		    beep();
		ZZ=1;
		FZ=0;
}
if(KeyV==2){
 beep();
		ZZ=0;
		FZ=1;
	}
	if(KeyV==3){
	 beep();
		ZZ=0;
		FZ=0;
	}					
	}  
} 
KeyV=0;
TempKeyV=0; 
 }


	

//}

