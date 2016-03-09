/************************************************************/
//�ļ�����nrf24l01.c
//����:linux�µ�nrf24l01��������
//ʹ��˵��: (1)����֮��tiny4412
//          (2)����оƬcortex-A9
//          (3)linux3.5�ں�

//����:juring-��

//���ڣ�2015-03-09
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


//NRF24L01������صĺ궨��

#define CE      EXYNOS4_GPA1(1)		//con15 3�Źܽ�
#define CSN   	EXYNOS4_GPA1(0)		//con15 4�Źܽ�
#define SCK     EXYNOS4_GPD1(1)		//con15 5�Źܽ�
#define MOSI    EXYNOS4_GPD1(0)		//con15 6�Źܽ�
#define MISO    EXYNOS4_GPB(0)		//con15 9�Źܽ�
#define IRQ     EXYNOS4_GPB(1)  	//con15 10�Źܽ�


#define DEVICE_NAME     "NRF24L01" //�豸���ƣ��ڿ��� /proc/devices �鿴


//NRF24L01
#define TX_ADR_WIDTH    5        // 5 uint8s TX address width
#define RX_ADR_WIDTH    5        // 5 uint8s RX address width
#define TX_PLOAD_WIDTH  32    // 20 uint8s TX payload
#define RX_PLOAD_WIDTH  32       // 20 uint8s TX payload
uchar const TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};    //���ص�ַ
uchar const RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};    //���յ�ַ


//NRF24L01�Ĵ���ָ��
#define READ_REG        0x00    // ���Ĵ���ָ��
#define WRITE_REG       0x20    // д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61    // ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0    // д��������ָ��
#define FLUSH_TX        0xE1    // ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2    // ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3    // �����ظ�װ������ָ��
#define NOP             0xFF    // ����


//SPI(nRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define STATUS          0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������

uchar opencount = 0;
uchar 	/*bdata*/ sta;

typedef struct uchar_bits
{
	uchar bit0:1;		//λ���ʾ��������ʾbit0ֻռһ��λ
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

//������uchar SPI_RW(uchar tmp)
//���ܣ�SPIʱ��

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
//������uchar NRFWriteReg(uchar RegAddr,uchar date)
//���ܣ�NRF24L01д�Ĵ�������
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
//������uchar NRFWriteReg(uchar RegAddr,uchar date)
//���ܣ�NRF24L01���Ĵ�������
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
//������uchar NRF24L01_Read_RxDate(uchar RegAddr,uchar *RxDate,uchar DateLen)
//����: ���ڶ����ݣ�RegAddr��Ϊ�Ĵ�����ַ��RxDate��Ϊ���������ݵ�ַ��DateLen���������ݵĳ���
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
//������uchar NRF24L01_Write_TxDate(uchar RegAddr,uchar *TxDate,uchar DateLen)
//����: ���ڶ����ݣ�RegAddr��Ϊ�Ĵ�����ַ��TxDate��Ϊ��д�����ݵ�ַ��DateLen���������ݵĳ���
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



/*****************���ý���ģʽ����������******************************/
void SetRX_Mode(void)
{
	s3c2410_gpio_setpin(CE, 0);  //CE=0;
	NRF24L01_Write_TxDate(WRITE_REG + RX_ADDR_P0, (uchar *)RX_ADDRESS, RX_ADR_WIDTH); // Use the same address on the RX device as the TX device
		/******�����йؼĴ�������**************/
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

//������void nRF24L01_TxPacket(uchar * tx_buf)
//���ܣ����� tx_buf������
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
	.minor			= MISC_DYNAMIC_MINOR,//�����豸���豸��255���Զ�����
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
MODULE_AUTHOR("Juring-��");
