/************************************************************/
//文件名：nrf24l01.c
//功能:linux下的nrf24l01驱动程序
//使用说明: (1)友善之臂tiny4412
//          (2)主控芯片cortex-A9
//          (3)linux3.5内核

//作者:juring-江

//日期：2015-03-09
/************************************************************/

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <plat/gpio-cfg.h>

#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>


typedef unsigned char uchar;


//NRF24L01引脚相关的宏定义

#define CE      EXYNOS4_GPA1(1)		//con15 3号管脚
#define CSN   	EXYNOS4_GPA1(0)		//con15 4号管脚
#define SCK     EXYNOS4_GPD1(1)		//con15 5号管脚
#define MOSI    EXYNOS4_GPD1(0)		//con15 6号管脚
#define MISO    EXYNOS4_GPB(0)		//con15 9号管脚
#define IRQ     EXYNOS4_GPB(1)  	//con15 10号管脚


#define DEVICE_NAME     "NRF24L01" //设备名称，在可以 /proc/devices 查看


//NRF24L01
#define TX_ADR_WIDTH    5        // 5 uint8s TX address width
#define RX_ADR_WIDTH    5        // 5 uint8s RX address width
#define TX_PLOAD_WIDTH  32    // 20 uint8s TX payload
#define RX_PLOAD_WIDTH  32       // 20 uint8s TX payload
uchar const TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};    //本地地址
uchar const RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};    //接收地址


//NRF24L01寄存器指令
#define READ_REG        0x00    // 读寄存器指令
#define WRITE_REG       0x20    // 写寄存器指令
#define RD_RX_PLOAD     0x61    // 读取接收数据指令
#define WR_TX_PLOAD     0xA0    // 写待发数据指令
#define FLUSH_TX        0xE1    // 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2    // 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3    // 定义重复装载数据指令
#define NOP             0xFF    // 保留


//SPI(nRF24L01)寄存器地址
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

uchar opencount = 0;
uchar 	/*bdata*/ sta;

typedef struct uchar_bits
{
	uchar bit0:1;		//位域表示方法，表示bit0只占一个位
	uchar bit1:1;
	uchar bit2:1;
	uchar bit3:1;
	uchar bit4:1;
	uchar bit5:1;
	uchar bit6:1;
	uchar bit7:1;
}uchar_bits, uint8_bits;

#define RX_DR  (((uchar_bits *)&sta)->bit6) //sbit	RX_DR=sta^6;
#define TX_DS  (((uchar_bits *)&sta)->bit5) //sbit	TX_DS=sta^5;
#define MAX_RT (((uchar_bits *)&sta)->bit4) //sbit	MAX_RT=sta^4;

extern unsigned int s3c2410_gpio_getpin(unsigned int pin);
extern void s3c2410_gpio_setpin(unsigned int pin, unsigned int to);
extern void s3c2410_gpio_pullup(unsigned int pin, unsigned int to);
//=========================================//

/**********************************************/

//函数：uchar SPI_RW(uchar tmp)
//功能：SPI时序

/*********************************************/
uchar SPI_RW(uchar tmp)
{
	 uchar bit_ctr;

    for(bit_ctr=0 ;bit_ctr<8 ;bit_ctr++) // output 8-bit
    {
 		if(tmp & 0x80)         // output 'tmp', MSB to MOSI
    s3c2410_gpio_setpin(MOSI, 1);	//MOSI=1
 		else
  	s3c2410_gpio_setpin(MOSI, 0);	//MOSI=0
	ndelay(60);
	 tmp <<= 1;           // shift next bit into MSB..
       s3c2410_gpio_setpin(SCK, 1);       // Set SCK high..
 	ndelay(60);
      tmp |=  s3c2410_gpio_getpin(MISO) ;      // byte |= MISO;  
       s3c2410_gpio_setpin(SCK, 0);        // ..then set SCK low again
 	ndelay(60);
    }
    return(tmp);                    // return read tmp 
}

/**************************************************
//函数：uchar NRFWriteReg(uchar RegAddr,uchar date)
//功能：NRF24L01写寄存器函数
 **************************************************/
uchar NRF24L01_Write_Reg(uchar RegAddr,uchar date)
{
	uchar status;

	s3c2410_gpio_setpin(CSN, 0); //CSN=0;                   // CSN low, init SPI transaction
	  ndelay(60);
	  
	status = SPI_RW(RegAddr);      // select register
	SPI_RW(date);             // ..and write value to it..
	s3c2410_gpio_setpin(CSN, 1); //CSN=1                   // CSN high again
 	 ndelay(60);
	 
	return(status);            // return nRF24L01 status byte
}

/**************************************************
//函数：uchar NRFWriteReg(uchar RegAddr,uchar date)
//功能：NRF24L01读寄存器函数
 **************************************************/
uchar NRF24L01_Read_Reg(uchar RegAddr)
{
   uchar status;

	s3c2410_gpio_setpin(CSN, 0); //CSN=0;                   // CSN low, init SPI transaction
  	ndelay(60);

	SPI_RW(RegAddr);      // select register
	status = SPI_RW(0);             // ..and write value to it..
	s3c2410_gpio_setpin(CSN, 1); //CSN=1                   // CSN high again
  	ndelay(60);
  
	return(status);            // return nRF24L01 status byte
}

/***********************************************************************************************
//函数：uchar NRF24L01_Read_RxDate(uchar RegAddr,uchar *RxDate,uchar DateLen)
//功能: 用于读数据，RegAddr：为寄存器地址，RxDate：为待读出数据地址，DateLen：读出数据的长度
 ***********************************************************************************************/

uchar NRF24L01_Read_RxDate(uchar RegAddr,uchar *RxDate,uchar DateLen)
{
	uchar status,i;
    
    s3c2410_gpio_setpin(CSN, 0);                            // Set CSN low, init SPI tranaction
    ndelay(60);
    status = SPI_RW(RegAddr);               // Select register to write to and read status uint8
    
    for(i = 0;i < DateLen;i++)
    {
        RxDate[i] = SPI_RW(0);    // 
        ndelay(20);
    }
    
    s3c2410_gpio_setpin(CSN, 1);    //csn=1
    ndelay(60);
    
    return(status);                    // return nRF24L01 status uint8
}

/***********************************************************************************************
//函数：uchar NRF24L01_Write_TxDate(uchar RegAddr,uchar *TxDate,uchar DateLen)
//功能: 用于读数据，RegAddr：为寄存器地址，TxDate：为待写入数据地址，DateLen：读出数据的长度
 ***********************************************************************************************/
uchar NRF24L01_Write_TxDate(uchar RegAddr,uchar *TxDate,uchar DateLen)
{ 
   uchar BackDate,i;
   s3c2410_gpio_setpin(CSN, 0);  
    ndelay(60);
   BackDate=SPI_RW(RegAddr);
   for(i=0;i<DateLen;i++)
     {
	    SPI_RW(*TxDate++);
		ndelay(20);
	 }   
  	s3c2410_gpio_setpin(CSN, 1);    //csn=1
    ndelay(60);
   return(BackDate);
}



/*****************设置接收模式并接收数据******************************/
void SetRX_Mode(void)
{
	s3c2410_gpio_setpin(CE, 0);  //CE=0;
	NRF24L01_Write_TxDate(WRITE_REG + RX_ADDR_P0, (uchar *)RX_ADDRESS, RX_ADR_WIDTH); // Use the same address on the RX device as the TX device
		/******下面有关寄存器配置**************/
	NRF24L01_Write_Reg(WRITE_REG + EN_AA, 0x01);     
	NRF24L01_Write_Reg(WRITE_REG + EN_RXADDR, 0x01); 
	NRF24L01_Write_Reg(WRITE_REG + RF_CH, 0);        
	NRF24L01_Write_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); 
	NRF24L01_Write_Reg(WRITE_REG + RF_SETUP, 0x07);   
	NRF24L01_Write_Reg(WRITE_REG + CONFIG, 0x0f);     

	s3c2410_gpio_setpin(CE, 1);  //CE=1; 
	udelay(130); //inerDelay_us(130);
}

uchar nRF24L01_RxPacket(uchar* rx_buf)
{
	uchar revale=0;

	//SetRX_Mode();

	sta=NRF24L01_Read_Reg(STATUS);	// read register STATUS's value
	if(RX_DR)				// if receive data ready (RX_DR) interrupt
	{
		s3c2410_gpio_setpin(CE, 0); //CE=0; 
		NRF24L01_Read_RxDate(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		revale =1;//we have receive data
	}
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag

	return revale;
}

//函数：void nRF24L01_TxPacket(uchar * tx_buf)
//功能：发送 tx_buf中数据
void nRF24L01_TxPacket(uchar * tx_buf)
{
	printk("tx in\n");
	s3c2410_gpio_setpin(CE, 0); //CE=0;
	printk("tx wite buf start \n");

	NRF24L01_Write_TxDate(WRITE_REG + TX_ADDR, (uchar *)TX_ADDRESS, TX_ADR_WIDTH);    // 
	printk("tx wite buf over \n");

	NRF24L01_Write_TxDate(WRITE_REG + RX_ADDR_P0, (uchar *)TX_ADDRESS, TX_ADR_WIDTH); 
	NRF24L01_Write_TxDate(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 

printk("tx wite reg start \n");
	NRF24L01_Write_Reg(WRITE_REG + EN_AA, 0x01); 
printk("tx wite reg over \n");     // 
	NRF24L01_Write_Reg(WRITE_REG + EN_RXADDR, 0x01);  // 
	NRF24L01_Write_Reg(WRITE_REG + SETUP_RETR, 0x1a); //
	NRF24L01_Write_Reg(WRITE_REG + RF_CH, 0);        // 
	NRF24L01_Write_Reg(WRITE_REG + RF_SETUP, 0x07);   // 
	NRF24L01_Write_Reg(WRITE_REG + CONFIG, 0x0e);     // 
	s3c2410_gpio_setpin(CE, 1);  //CE=1;
	udelay(10); //inerDelay_us(10);
	//sta=SPI_Read(STATUS);	// read register STATUS's value
	//SPI_RW_Reg(WRITE_REG+STATUS,SPI_Read(READ_REG+STATUS));	// clear interrupt flag(TX_DS)

}

uchar init_NRF24L01(void)
{

  s3c2410_gpio_pullup(MOSI,0);

  s3c_gpio_cfgpin(CE, S3C_GPIO_OUTPUT); 	//CE_OUT;
  s3c_gpio_cfgpin(CSN, S3C_GPIO_OUTPUT);	//CSN_OUT;
  s3c_gpio_cfgpin(SCK, S3C_GPIO_OUTPUT); 	//SCK_OUT;
  s3c_gpio_cfgpin(MOSI, S3C_GPIO_OUTPUT);	//MOSI_OUT;
  s3c_gpio_cfgpin(MISO, S3C_GPIO_INPUT);	//MISO_IN;
  s3c_gpio_cfgpin(IRQ, S3C_GPIO_INPUT);		//IRQ_IN;

    udelay(500);

    s3c2410_gpio_setpin(CE, 0);    // ce=0
    ndelay(60);
   	s3c2410_gpio_setpin(CSN, 1);   // csn=1
    ndelay(60);
    s3c2410_gpio_setpin(SCK, 0);   // Spi clock line init high
    ndelay(60);
	s3c2410_gpio_setpin(IRQ, 1);
	ndelay(60);
    printk("test 1 \n");
    mdelay(1);

    return (1);
}


//=========================================//
static int NRF24L01_open(struct inode *inode, struct file *file)
{
	
	uchar flag = 0;

  if(opencount == 1)
    return -EBUSY;
  
  flag = init_NRF24L01();

  mdelay(100);
  if(flag == 0)
    {
      printk("uable to open device!\n");
      return -1;
    }
  else
   {
      opencount++;
      printk("device opened !\n");
      return 0;
   }
			
}

static ssize_t NRF24L01_read(struct file *filp, char *buffer,
              size_t count, loff_t *ppos)
{
	uchar Rxbuf[20]={0};
	SetRX_Mode();
	if(nRF24L01_RxPacket(Rxbuf))
	{
		printk("RX start......\n");
 		printk(" RxBuf[1] value=%d\n",Rxbuf[1]);
		printk(" RxBuf[2] value=%d\n",Rxbuf[2]);
	}
	//copy_to_user(void __user * to,const void * from,unsigned long n)
return 0;

}

static ssize_t NRF24L01_write(struct file *filp, const char *buffer,
             size_t count, loff_t *ppos)
{
	printk("RX start......\n");
	//copy_from_user(void * to,const void __user * from,unsigned long n)
return 0;
}

static int NRF24L01_close(struct inode *inode, struct file *file)
{
  opencount--;
  printk(DEVICE_NAME " released !\n");
  return 0;
}
static struct file_operations NRF24L01_dev_fops = {
	.owner			= THIS_MODULE,
	//.unlocked_ioctl	= tiny4412_leds_ioctl,
	.open		= NRF24L01_open,
	.release	= NRF24L01_close,
	.write		= NRF24L01_write,
  	.read 		= NRF24L01_read,
	
};

static struct miscdevice NRF24L01_dev = {
	.minor			= MISC_DYNAMIC_MINOR,//杂项设备次设备号255会自动分配
	.name			= DEVICE_NAME,
	.fops			= &NRF24L01_dev_fops,
};

static int __init NRF24L01_dev_init(void) {
	int ret;

	ret = misc_register(&NRF24L01_dev);

	printk(DEVICE_NAME"\tinitialized\n");

	return ret;
}

static void __exit NRF24L01_dev_exit(void) 
{
	misc_deregister(&NRF24L01_dev);
	printk("NRF24L01 drv exit\n");
}

module_init(NRF24L01_dev_init);
module_exit(NRF24L01_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juring-江");
