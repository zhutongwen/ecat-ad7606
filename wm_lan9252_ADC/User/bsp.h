/****************************************Copyright (c)****************************************************
**                                      
**                               
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               TouchPanel.h
** Descriptions:            The TouchPanel application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-7
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef _BSP_H_
#define _BSP_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "string.h"



//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_Addr(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_Addr(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAO(Pin)   BIT_ADDR(GPIOA_ODR_Addr, Pin)  //输出 
#define PAI(Pin)    BIT_ADDR(GPIOA_IDR_Addr, Pin)  //输入 

#define PBO(Pin)   BIT_ADDR(GPIOB_ODR_Addr, Pin)  //输出 
#define PBI(Pin)    BIT_ADDR(GPIOB_IDR_Addr, Pin)  //输入 

#define PCO(Pin)   BIT_ADDR(GPIOC_ODR_Addr, Pin)  //输出 
#define PCI(Pin)    BIT_ADDR(GPIOC_IDR_Addr, Pin)  //输入 

#define PDO(Pin)   BIT_ADDR(GPIOD_ODR_Addr, Pin)  //输出 
#define PDI(Pin)    BIT_ADDR(GPIOD_IDR_Addr, Pin)  //输入 

#define PEO(Pin)   BIT_ADDR(GPIOE_ODR_Addr, Pin)  //输出 
#define PEI(Pin)    BIT_ADDR(GPIOE_IDR_Addr, Pin)  //输入

#define PFO(Pin)   BIT_ADDR(GPIOF_ODR_Addr, Pin)  //输出 
#define PFI(Pin)    BIT_ADDR(GPIOF_IDR_Addr, Pin)  //输入

#define PGO(Pin)   BIT_ADDR(GPIOG_ODR_Addr, Pin)  //输出 
#define PGI(Pin)    BIT_ADDR(GPIOG_IDR_Addr, Pin)  //输入

#define PHO(Pin)   BIT_ADDR(GPIOH_ODR_Addr, Pin)  //输出 
#define PHI(Pin)    BIT_ADDR(GPIOH_IDR_Addr, Pin)  //输入

#define PIO(Pin)   BIT_ADDR(GPIOI_ODR_Addr, Pin)  //输出 
#define PII(Pin)    BIT_ADDR(GPIOI_IDR_Addr, Pin)  //输入


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************
#define CMD_SERIAL_READ 0x03
#define CMD_FAST_READ 0x0B
#define CMD_DUAL_OP_READ 0x3B
#define CMD_DUAL_IO_READ 0xBB
#define CMD_QUAD_OP_READ 0x6B
#define CMD_QUAD_IO_READ 0xEB
#define CMD_SERIAL_WRITE 0x02
#define CMD_DUAL_DATA_WRITE 0x32
#define CMD_DUAL_ADDR_DATA_WRITE 0xB2
#define CMD_QUAD_DATA_WRITE 0x62
#define CMD_QUAD_ADDR_DARA_WRITE 0xE2

#define CMD_SERIAL_READ_DUMMY 0
#define CMD_FAST_READ_DUMMY 1
#define CMD_DUAL_OP_READ_DUMMY 1
#define CMD_DUAL_IO_READ_DUMMY 2
#define CMD_QUAD_OP_READ_DUMMY 1
#define CMD_QUAD_IO_READ_DUMMY 4
#define CMD_SERIAL_WRITE_DUMMY 0
#define CMD_DUAL_DATA_WRITE_DUMMY 0
#define CMD_DUAL_ADDR_DATA_WRITE_DUMMY 0
#define CMD_QUAD_DATA_WRITE_DUMMY 0
#define CMD_QUAD_ADDR_DARA_WRITE_DUMMY 0

#define ESC_CSR_CMD_REG		0x304
#define ESC_CSR_DATA_REG	0x300
#define ESC_WRITE_BYTE 		0x80
#define ESC_READ_BYTE 		0xC0
#define ESC_CSR_BUSY		0x80	

/*---------------------------------------------
-    Microcontroller definitions
-----------------------------------------------*/



extern	uint16_t uhADCxConvertedValue;

///////////////////////////////////////////////////////////////////////////////
//9252 HW DEFINES
#define ECAT_REG_BASE_ADDR              0x0300

#define CSR_DATA_REG_OFFSET             0x00
#define CSR_CMD_REG_OFFSET              0x04
#define PRAM_READ_ADDR_LEN_OFFSET       0x08
#define PRAM_READ_CMD_OFFSET            0x0c
#define PRAM_WRITE_ADDR_LEN_OFFSET      0x10
#define PRAM_WRITE_CMD_OFFSET           0x14

#define PRAM_SPACE_AVBL_COUNT_MASK      0x1f
#define IS_PRAM_SPACE_AVBL_MASK         0x01

#define CSR_DATA_REG                    ECAT_REG_BASE_ADDR+CSR_DATA_REG_OFFSET
#define CSR_CMD_REG                     ECAT_REG_BASE_ADDR+CSR_CMD_REG_OFFSET
#define PRAM_READ_ADDR_LEN_REG          ECAT_REG_BASE_ADDR+PRAM_READ_ADDR_LEN_OFFSET
#define PRAM_READ_CMD_REG               ECAT_REG_BASE_ADDR+PRAM_READ_CMD_OFFSET
#define PRAM_WRITE_ADDR_LEN_REG         ECAT_REG_BASE_ADDR+PRAM_WRITE_ADDR_LEN_OFFSET
#define PRAM_WRITE_CMD_REG              ECAT_REG_BASE_ADDR+PRAM_WRITE_CMD_OFFSET

#define PRAM_READ_FIFO_REG              0x04
#define PRAM_WRITE_FIFO_REG             0x20

#define HBI_INDEXED_DATA0_REG           0x04
#define HBI_INDEXED_DATA1_REG           0x0c
#define HBI_INDEXED_DATA2_REG           0x14

#define HBI_INDEXED_INDEX0_REG          0x00
#define HBI_INDEXED_INDEX1_REG          0x08
#define HBI_INDEXED_INDEX2_REG          0x10

#define HBI_INDEXED_PRAM_READ_WRITE_FIFO    0x18

#define PRAM_RW_ABORT_MASK      (0x40000000)
#define PRAM_RW_BUSY_32B        (0x80000000)
#define PRAM_RW_BUSY_8B         (0x80)
#define PRAM_SET_READ           (0x40)
#define PRAM_SET_WRITE          0

///////////////////////////////////////////////////////////////////////////////////////
//ºê¶¨Òå¼Ä´æÆ÷µÄµØÖ·£¬ÒÔ±ãÖ±½Ó¶ÔÏµÍ³Ê±ÖÓµÄ¼Ä´æÆ÷²Ù×÷£¬²Î¿¼  "STM32F3_STM32F4_Cortex-M4 programming manual.pdf"
#define SYSTICK_CTRLR      	(*((volatile uint32_t *)0xE000E010))	
#define SYSTICK_LOAD   		(*((volatile uint32_t *)0xE000E014))
#define SYSTICK_CURRENT  	(*((volatile uint32_t *)0xE000E018))
#define SYSTICK_TENMS    	(*((volatile uint32_t *)0xE000E01C))
	

#define LED_0 	PBO(0)
#define LED_1 	PBO(1)
#define LED_2 	PBO(10)
#define LED_3 	PBO(11)
#define LED_4 	PBO(12)
#define LED_5 	PBO(13)
#define LED_6 	PBO(14)
#define LED_7 	PBO(15)

#define KEY_0  	PCI(0)
#define KEY_1 	PCI(1)
#define KEY_2  	PCI(2)
#define KEY_3 	PCI(3)
#define KEY_4  	PAI(6)
#define KEY_5 	PAI(7)
#define KEY_6  	PCI(4)
#define KEY_7 	PCI(5)

//adc
#define SPI1_NSS 		PBO(6)
#define ADC_OS0 		PEO(2)
#define ADC_OS1 		PEO(1)
#define ADC_OS2 		PBO(9)
#define ADC_CONVERT 	PBO(8)
#define ADC_REST 		PBO(7)
#define ADC_BUSY 		PBI(5)


typedef union
{
	u32 dword;
	u16 word[2];
	u8 	byte[4];	
}U32;


/* Private function prototypes -----------------------------------------------*/	
void LedInit(void);
void KeyInit(void);
void Uart2Init(void);
void OutPut_Data(u16 *OutData);
void AdcInit(void);
void FsmcInit(void);
void Tim2Init(void);
void ExitPe0Init(void); //IRQ 中断
void ExitPd6Init(void); //SYNC0 中断
void ExitPa11Init(void);//SYNC1 中断
void SystickInit(void);

void Ad7606Init(void);

void PMPReadDRegister(u8 *ReadBuffer, u16 Address, u16 Count);
void PMPWriteRegister( u8 *WriteBuffer, u16 Address, u16 Count);
u32 PMPReadDWord (u16 Address);
void PMPWriteDWord (u16 Address, u32 Val);

#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


