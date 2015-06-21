//***************************************
// B_LUX_V20采集程序
//****************************************
#include  <math.h>    //Keil library  
#include  <stdio.h>   //Keil library
#include "B_LUX_V20.h"
#include  <LCD1602.H>

sbit      LCM_RS=P2^0;   //LCD1602命令端口		
sbit      LCM_RW=P2^1;   //LCD1602命令端口		
sbit      LCM_EN=P2^2;   //LCD1602命令端口 
#define uchar unsigned char
#define uint  unsigned int
uint8    BUF_0[8];                       //接收数据缓存区      	
uint16   dis_data_0;                     //变量


 
sbit  BEEP = P3^6;       //蜂鸣器
sbit ZZ=P1^0;//控制端
sbit FZ=P1^1;//控制端
sbit Key_UP=P3^2;   //正转按键 对应实验板上的K1
sbit Key_DOWN=P1^4;//反转按键  对应实验板上的K5
sbit Key_STOP=P1^5;//停止键    对应实验板上的K6
unsigned char KeyV,TempKeyV;      

/*---------------------------------------------------------------------
 功能描述: 延时纳秒 不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
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
 功能描述: 延时5微秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Delay5us()
{
  uint8 n = 1;
  
  while (n--);
}

/*---------------------------------------------------------------------
 功能描述: 延时5毫秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Delay5ms()
{
  uint16 n = 60000;
  
  while (n--);
}

/*---------------------------------------------------------------------
 功能描述: 起始信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Start()
{
  B_LUX_SDA0_H;                    //拉高数据线
  B_LUX_SCL0_H;                    //拉高时钟线
  B_LUX_Delay5us();                 //延时
  B_LUX_SDA0_L;                    //产生下降沿
  B_LUX_Delay5us();                 //延时
  B_LUX_SCL0_L;                    //拉低时钟线
}

/*---------------------------------------------------------------------
 功能描述: 停止信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Stop()
{
  B_LUX_SDA0_L;                    //拉低数据线
  B_LUX_SCL0_H;                    //拉高时钟线
  B_LUX_Delay5us();                 //延时
  B_LUX_SDA0_H;                    //产生上升沿
  B_LUX_Delay5us();                 //延时
  B_LUX_SCL0_L;
  B_LUX_Delay5us();
}

/*---------------------------------------------------------------------
 功能描述: 发送应答信号
 参数说明: ack - 应答信号(0:ACK 1:NAK)
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_SendACK(uint8 ack)
{
  if (ack&0x01)	B_LUX_SDA0_H;		 //写应答信号
  else	B_LUX_SDA0_L;
  
  B_LUX_SCL0_H;                    //拉高时钟线
  B_LUX_Delay5us();                 //延时
  B_LUX_SCL0_L;                    //拉低时钟线
  B_LUX_SDA0_H;
  B_LUX_Delay5us();                 //延时
}

/*---------------------------------------------------------------------
 功能描述: 接收应答信号
 参数说明: 无
 函数返回: 返回应答信号
 ---------------------------------------------------------------------*/
uint8 B_LUX_RecvACK()
{
  uint8 CY = 0x00;
  B_LUX_SDA0_H;
  
  B_LUX_SDA0_I;
  
  B_LUX_SCL0_H;              //拉高时钟线
  B_LUX_Delay5us();                 //延时
  
  
  CY |= B_LUX_SDA0_DAT;    //读应答信号
  
  B_LUX_Delay5us();                 //延时
  
  B_LUX_SCL0_L;              //拉低时钟线
  
  B_LUX_SDA0_O;
  
  return CY;
}

/*---------------------------------------------------------------------
 功能描述: 向IIC总线发送一个字节数据
 参数说明: dat - 写字节
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_SendByte(uint8 dat)
{
  uint8 i;
  
  for (i=0; i<8; i++)         			//8位计数器
  {
    if (dat&0x80)	B_LUX_SDA0_H;
    else	B_LUX_SDA0_L;              //送数据口
    
    B_LUX_Delay5us();             			//延时
    B_LUX_SCL0_H;                		//拉高时钟线
    B_LUX_Delay5us();             			//延时
    B_LUX_SCL0_L;                		//拉低时钟线
    B_LUX_Delay5us();             			//延时
    dat <<= 1;              			//移出数据的最高位
  }
  
  B_LUX_RecvACK();
}

/*---------------------------------------------------------------------
 功能描述: 从IIC总线接收一个字节数据
 参数说明: 无
 函数返回: 接收字节
 ---------------------------------------------------------------------*/
uint8 B_LUX_RecvByte()
{
  uint8 i;
  uint8 dat = 0;
  B_LUX_SDA0_I;
  
  B_LUX_SDA0_H;                  //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         	//8位计数器
  {
    B_LUX_SCL0_H;              //拉高时钟线
    B_LUX_Delay5us();             	//延时
    dat |= B_LUX_SDA0_DAT;     //读数据               
    B_LUX_SCL0_L;              //拉低时钟线
    B_LUX_Delay5us();             	//延时
    
    dat <<= 1;	
  }
  B_LUX_SDA0_O;
  
  return dat;
}

/*---------------------------------------------------------------------
 功能描述: 写BH1750
 参数说明: REG_Address - 寄存器地址
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Single_Write(uint8 REG_Address)
{
  B_LUX_Start();                  //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress);   //发送设备地址+写信号
  B_LUX_SendByte(REG_Address);    //内部寄存器地址，请参考中文pdf22页 
  //  BH1750_SendByte(REG_data);       //内部寄存器数据，请参考中文pdf22页 
  B_LUX_Stop();                   //发送停止信号
}

/*---------------------------------------------------------------------
 功能描述: 连续读出BH1750内部数据
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Multiple_read(void)
{   
  uint8 i;	
  B_LUX_Start();                          //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+1);         //发送设备地址+读信号
  
  for (i=0; i<3; i++)                        //连续读取6个地址数据，存储中BUF
  {
    BUF_0[i] = B_LUX_RecvByte();          //BUF[0]存储0x32地址中的数据
    if (i == 3)
    {
      
      B_LUX_SendACK(1);                   //最后一个数据需要回NOACK
    }
    else
    {		
      B_LUX_SendACK(0);                   //回应ACK
    }
  }
  
  B_LUX_Stop();                           //停止信号
  B_LUX_Delay5ms();
}

/*---------------------------------------------------------------------
 功能描述: 初始化光照传感器
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Init()
{
  
  //P1SEL &= ~(0x48);
  
  B_LUX_SCL0_O;
  B_LUX_SDA0_O;
  
  B_LUX_delay_nms(100);	    //延时100ms
  
  B_LUX_Single_Write(0x01); 
  
}

/*---------------------------------------------------------------------
 功能描述: 光照读取函数
 参数说明: 无
 函数返回: 返回光照值
 ---------------------------------------------------------------------*/
uint32 B_LUX_GetLux()
{  
  fint32 temp;
  B_LUX_Single_Write(0x01);   // power on
  B_LUX_Single_Write(0x10);   // H- resolution mode 
  
  B_LUX_delay_nms(180);       //延时180ms
  
  B_LUX_Multiple_read();      //连续读出数据，存储在BUF中
  
  B_LUX_Single_Write(0x00);   // power off
  
  dis_data_0=BUF_0[0];
  dis_data_0=(dis_data_0<<8)+BUF_0[1];//合成数据，即光照数据
  
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
   for (j=0; j<8; j++)         //转1*n圈 
    { 
	    
      for (i=0; i<8; i++)       //一个周期转45度
        {
          P1 = FFW[i];          //取数据
          delay(2);            //调节转速
        }
    }
 }

 void  motor_rev()
{
     uchar i;
	 uint  j;
	 for (j=0; j<8; j++)       //转1×n圈
      {
	    
        for (i=0; i<8; i++)     //一个周期转45度
        {
          P1 = REV[i];          //取数据
          delay(2);            //调节转速
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
     BEEP=!BEEP;                 //BEEP取反
    } 
     BEEP=1;                    //关闭蜂鸣器
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
	D1=x%10;              //计算个位数字
	D2=(x%100)/10;        //计算十位数字
	D3= (x%1000)/100;        //计算百位数字
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

