#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,(BitAction)BitValue);
	Delay_us(10);
}
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_11,(BitAction)BitValue);
	Delay_us(10);
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
	Delay_us(10);
	return BitValue;
}


void MyI2C_Init(void)
{
	//SCL和SDA初始化为开漏模式
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);      //开始时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;     //初始化结构体变量
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;    //开漏输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 |GPIO_Pin_11;   //设置引脚10或11
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//把SCL和SDA置高电平
	GPIO_SetBits(GPIOB,GPIO_Pin_10 | GPIO_Pin_11);
	

	
}
 
void MyI2C_Start(void)   //可兼容起始条件和重复起始条件
{
	//产生起始条件(SCL和SDA都释放输出1，然后先拉低SDA，再拉点SDL)
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);   //先释放SDA 
    MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
	
}
void MyI2C_Stop(void)
{
	//先拉低SDA，再释放SCL,SDA
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		MyI2C_W_SDA(Byte & (0x80>>i));
        MyI2C_W_SCL(1);
	    MyI2C_W_SCL(0);  //释放SCL后拉低SCL，驱动时钟走一个脉冲
	
	}
	
}
uint8_t MyI2C_ReceiveByte(void)  //接收
{
	//SCL低电平时SDA变换数据，高电平SDA读取数据。
	//SCL高电平期间若SDA变化，则是起始/终止条件
	//数据传输：SCL高电平不许动SDA   起始终止：SCL必须动SDA
	uint8_t i,Byte=0x00;
	MyI2C_W_SDA(1);
	for(i=0;i<8;i++)
	{
		MyI2C_W_SCL(1);
        if(MyI2C_R_SDA()==1)  // 若if不成立就默认0x00
        {Byte |= (0x80>>i);} //置1
		
        MyI2C_W_SCL(0);
	}
	return Byte;
	
}
void MyI2C_SendAck(uint8_t AckBit)
{
	//进来时SCL低电平
    MyI2C_W_SDA(AckBit);  //主机把AckBit放到SDA上
    MyI2C_W_SCL(1);  //从机读取应答
	MyI2C_W_SCL(0);  //低电平进入下一个时序单元

	
}
uint8_t MyI2C_ReceiveAck(void)  //应答
{
	//进来时SCL低电平
	uint8_t AckBit;
	MyI2C_W_SDA(1);//主机释放SDA，防止干扰从机
    MyI2C_W_SCL(1);
    AckBit=MyI2C_R_SDA();
    MyI2C_W_SCL(0);
	return AckBit;
	
}





