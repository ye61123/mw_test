#include "i2c.h"

void I2C_Init()
{
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
}

//I2C起始信号
void I2C_Start(void)
{
    I2C_SCL_H;
    I2C_SDA_H;
    Delay_us(5);
    I2C_SDA_L;
    Delay_us(5);
    I2C_SCL_L;
}

//产生I2C停止信号
void I2C_Stop(void)
{
   I2C_SDA_L;
   I2C_SCL_L;
   Delay_us(5);
	
   I2C_SCL_H;
   Delay_us(4);
   I2C_SDA_H;
   Delay_us(5);
}

//I2C等待应答信号
unsigned char I2C_Wait_Ack(void)
{
    unsigned char flag_ack=0;
    I2C_SCL_L;
    I2C_SDA_H;
    Delay_us(5);
	
    I2C_SCL_H;
    Delay_us(5);
    while(READ_SDA && flag_ack<4)
    {
       flag_ack++;
       Delay_us(1);
    }
    if(flag_ack>=4)
    {
     I2C_Stop();
     return 1;
 
    }
    I2C_SCL_L;
    return 0;
}

//产生I2C应答信号
void I2C_Ack(void)
{
   I2C_SCL_L;
   I2C_SDA_L;
   Delay_us(5);
	 
   I2C_SCL_H;
   Delay_us(5);
 
}

//I2C无应答时处理
void I2C_NAck(void)
{
    I2C_SCL_L;
    I2C_SDA_H;
    Delay_us(4);
	
    I2C_SCL_H;
    Delay_us(4); 
}

//I2C发送一位
void I2C_Send_Byte(unsigned char txd)
{
    unsigned char t=0; 
    /*时钟信号拉低，数据准备好再拉高进行传输*/    
    for(t = 0; t < 8; t++)
    {     
     I2C_SCL_L;
     if(txd&0x80)
	I2C_SDA_H;
     else
	I2C_SDA_L;      
     Delay_us(5);   
        
    /*SCL拉高传输数据*/
    I2C_SCL_H;
    Delay_us(5);
    txd <<= 1;   
    }	 
}
 
//I2C读取一位
unsigned char I2C_Read_Byte(void)
{
   unsigned char receive = 0;
   unsigned char i=8; 
   for(i = 0; i < 8; i++ )
   {
        /*SCL拉低*/
        I2C_SCL_L;   
        Delay_us(4);  
        /*拉高SCL产生一个有效的时钟信号*/
        I2C_SCL_H;
        /*读取总线上的数据*/
        receive<<=1;
        
       if(READ_SDA)
	  receive|=1; 
        Delay_us(4); 
    }
        return receive;
}


//void Delay_us(uint32_t udelay)
//{
//  uint32_t startval,tickn,delays,wait;
// 
//  startval = SysTick->VAL;
//  tickn = HAL_GetTick();
//  //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
//  delays =udelay * 72; //sysc / 1000 * udelay;
//  if(delays > startval)
//    {
//      while(HAL_GetTick() == tickn)
//        {
// 
//        }
//      wait = 72000 + startval - delays;
//      while(wait < SysTick->VAL)
//        {
// 
//        }
//    }
//  else
//    {
//      wait = startval - delays;
//      while(wait < SysTick->VAL && HAL_GetTick() == tickn)
//        {
// 
//        }
//    }
//}


void Delay_us(uint32_t nus)
{
		unsigned char  fac_us;
    uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD;                   //LOAD
    ticks=nus*fac_us;
    told=SysTick->VAL;
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;       // SYSTICK.
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break;
        }
    }
}
