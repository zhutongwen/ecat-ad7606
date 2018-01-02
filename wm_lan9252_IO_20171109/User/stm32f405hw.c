/**
\addtogroup EL9800_HW EL9800 Platform (Serial ESC Access)
@{
*/

/**
\file    stm32f405hw.c
\author EthercatSSC@beckhoff.com
\brief Implementation
Hardware access implementation for EL9800 onboard PIC18/PIC24 connected via SPI to ESC

\version 5.11

<br>Changes to version V5.10:<br>
V5.11 ECAT10: change PROTO handling to prevent compiler errors<br>
V5.11 EL9800 2: change PDI access test to 32Bit ESC access and reset AL Event mask after test even if AL Event is not enabled<br>
<br>Changes to version V5.01:<br>
V5.10 ESC5: Add missing swapping<br>
V5.10 HW3: Sync1 Isr added<br>
V5.10 HW4: Add volatile directive for direct ESC DWORD/WORD/BYTE access<br>
           Add missing swapping in mcihw.c<br>
           Add "volatile" directive vor dummy variables in enable and disable SyncManger functions<br>
           Add missing swapping in EL9800hw files<br>
<br>Changes to version V5.0:<br>
V5.01 HW1: Invalid ESC access function was used<br>
<br>Changes to version V4.40:<br>
V5.0 ESC4: Save SM disable/Enable. Operation may be pending due to frame handling.<br>
<br>Changes to version V4.30:<br>
V4.40 : File renamed from spihw.c to el9800hw.c<br>
<br>Changes to version V4.20:<br>
V4.30 ESM: if mailbox Syncmanger is disabled and bMbxRunning is true the SyncManger settings need to be revalidate<br>
V4.30 EL9800: EL9800_x hardware initialization is moved to el9800.c<br>
V4.30 SYNC: change synchronisation control function. Add usage of 0x1C32:12 [SM missed counter].<br>
Calculate bus cycle time (0x1C32:02 ; 0x1C33:02) CalcSMCycleTime()<br>
V4.30 PDO: rename PDO specific functions (COE_xxMapping -> PDO_xxMapping and COE_Application -> ECAT_Application)<br>
V4.30 ESC: change requested address in GetInterruptRegister() to prevent acknowledge events.<br>
(e.g. reading an SM config register acknowledge SM change event)<br>
GENERIC: renamed several variables to identify used SPI if multiple interfaces are available<br>
V4.20 MBX 1: Add Mailbox queue support<br>
V4.20 SPI 1: include SPI RxBuffer dummy read<br>
V4.20 DC 1: Add Sync0 Handling<br>
V4.20 PIC24: Add EL9800_4 (PIC24) required source code<br>
V4.08 ECAT 3: The AlStatusCode is changed as parameter of the function AL_ControlInd<br>
<br>Changes to version V4.02:<br>
V4.03 SPI 1: In ISR_GetInterruptRegister the NOP-command should be used.<br>
<br>Changes to version V4.01:<br>
V4.02 SPI 1: In HW_OutputMapping the variable u16OldTimer shall not be set,<br>
             otherwise the watchdog might exceed too early.<br>
<br>Changes to version V4.00:<br>
V4.01 SPI 1: DI and DO were changed (DI is now an input for the uC, DO is now an output for the uC)<br>
V4.01 SPI 2: The SPI has to operate with Late-Sample = FALSE on the Eva-Board<br>
<br>Changes to version V3.20:<br>
V4.00 ECAT 1: The handling of the Sync Manager Parameter was included according to<br>
              the EtherCAT Guidelines and Protocol Enhancements Specification<br>
V4.00 APPL 1: The watchdog checking should be done by a microcontroller<br>
                 timer because the watchdog trigger of the ESC will be reset too<br>
                 if only a part of the sync manager data is written<br>
V4.00 APPL 4: The EEPROM access through the ESC is added

*/


/*--------------------------------------------------------------------------------------
------
------    Includes
------
--------------------------------------------------------------------------------------*/
#include "ecat_def.h"
#if STM32F405_HW
#include "ecatslv.h"

#define    _STM32F405_HW_ 1
#include "stm32f405hw.h"
#undef    _STM32F405_HW_

#include "ecatappl.h"
#include "bsp.h"

/*--------------------------------------------------------------------------------------
------
------    internal Types and Defines
------
--------------------------------------------------------------------------------------*/

typedef union
{
	unsigned short	Word;
	unsigned char	Byte[2];
} UBYTETOWORD;

typedef union 
{
    UINT8           Byte[2];
    UINT16          Word;
}
UALEVENT;

#if INTERRUPTS_SUPPORTED
	/*-----------------------------------------------------------------------------------------
	------
	------    Global Interrupt setting
	------
	-----------------------------------------------------------------------------------------*/
	#define	DISABLE_GLOBAL_INT      __disable_irq()					
	#define	ENABLE_GLOBAL_INT       __enable_irq()				
	#define	DISABLE_AL_EVENT_INT    DISABLE_GLOBAL_INT
										
	#define	ENABLE_AL_EVENT_INT		ENABLE_GLOBAL_INT

	/*-----------------------------------------------------------------------------------------
	------
	------    ESC Interrupt
	------
	-----------------------------------------------------------------------------------------*/
	#if AL_EVENT_ENABLED
		#define	INIT_ESC_INT           	ExitPe0Init();					
		#define	EcatIsr                	EXTI0_IRQHandler
		#define	ACK_ESC_INT         	EXTI_ClearITPendingBit(EXTI_Line0);  
		#define	IS_ESC_INT_ACTIVE		((GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)) == 0) 
	#endif //#if AL_EVENT_ENABLED

	/*-----------------------------------------------------------------------------------------
	------
	------    SYNC0 Interrupt
	------
	-----------------------------------------------------------------------------------------*/
	#if DC_SUPPORTED
		#define	INIT_SYNC0_INT        	ExitPd6Init();		
		#define	Sync0Isr                EXTI9_5_IRQHandler 
		#define	DISABLE_SYNC0_INT       NVIC_DisableIRQ(EXTI9_5_IRQn);	 
		#define	ENABLE_SYNC0_INT        NVIC_EnableIRQ(EXTI9_5_IRQn);	
		#define	ACK_SYNC0_INT           EXTI_ClearITPendingBit(EXTI_Line6);
		#define	IS_SYNC0_INT_ACTIVE     ((GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)) == 0) 
																						
		/*ECATCHANGE_START(V5.10) HW3*/
		#define	INIT_SYNC1_INT          ExitPa11Init();
		#define	Sync1Isr                EXTI15_10_IRQHandler
		#define	DISABLE_SYNC1_INT       NVIC_DisableIRQ(EXTI15_10_IRQn);
		#define	ENABLE_SYNC1_INT        NVIC_EnableIRQ(EXTI15_10_IRQn); 
		#define	ACK_SYNC1_INT           EXTI_ClearITPendingBit(EXTI_Line11);
		#define	IS_SYNC1_INT_ACTIVE     ((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)) == 0) 
		/*ECATCHANGE_END(V5.10) HW3*/
	#endif //#if DC_SUPPORTED
#endif	//#if INTERRUPTS_SUPPORTED

/*-----------------------------------------------------------------------------------------
------
------    Hardware timer
------
-----------------------------------------------------------------------------------------*/

#if ECAT_TIMER_INT
	#define ECAT_TIMER_ACK_INT      			
	#define	TimerIsr                     	SysTick_Handler	
	#define INIT_ECAT_TIMER            		SystickInit(); 
#else    //#if ECAT_TIMER_INT
	#define INIT_ECAT_TIMER      			Tim2Init();
#endif //#else #if ECAT_TIMER_INT


/*--------------------------------------------------------------------------------------
------
------    internal Variables
------
--------------------------------------------------------------------------------------*/
UALEVENT         EscALEvent;            //contains the content of the ALEvent register (0x220), this variable is updated on each Access to the Esc

/*--------------------------------------------------------------------------------------
------
------    internal functions
------
--------------------------------------------------------------------------------------*/
/*******************************************************************************
  Description:
    The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
    It will be saved in the global "EscALEvent"
  *****************************************************************************/
static void GetInterruptRegister(void)
{
	DISABLE_AL_EVENT_INT;
	HW_EscReadIsr((MEM_ADDR *)&EscALEvent.Word, 0x220, 2);
	ENABLE_AL_EVENT_INT;
}

/*******************************************************************************
  Description:
    The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
        It will be saved in the global "EscALEvent"
  *****************************************************************************/
static void ISR_GetInterruptRegister(void)
{
    HW_EscReadIsr((MEM_ADDR *)&EscALEvent.Word, 0x220, 2);
}

/*--------------------------------------------------------------------------------------
------
------    exported hardware access functions
------
--------------------------------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0 if initialization was successful
 \brief    This function intialize the Process Data Interface (PDI) and the host controller.
*////////////////////////////////////////////////////////////////////////////////////////
u8 HW_Init(void)
{
    u16 intMask;
	DISABLE_GLOBAL_INT;

	LedInit();
	KeyInit();
//	Uart2Init();
//	AdcInit();
	Ad7606Init();
	FsmcInit();

    do
    {
        intMask = 0x0093;
        HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
        intMask = 0;
			 HW_EscReadDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    } while (intMask!= 0x0093);
	
	//IRQ enable,IRQ polarity, IRQ buffer type in Interrupt Configuration register.
    //Wrte 0x54 - 0x00000101 
    PMPWriteDWord (0x54,0x00000101);
    

    //Write in Interrupt Enable register -->
    //Write 0x5c - 0x00000001
    PMPWriteDWord (0x5C, 0x00000001);    
    PMPReadDWord(0x58);

	intMask = 0x00;
	  
    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
				

#if AL_EVENT_ENABLED
    INIT_ESC_INT;
    ENABLE_ESC_INT();
#endif

#if DC_SUPPORTED
    INIT_SYNC0_INT
    INIT_SYNC1_INT

    ENABLE_SYNC0_INT;
    ENABLE_SYNC1_INT;
#endif

    INIT_ECAT_TIMER;
	  
	
#if INTERRUPTS_SUPPORTED
    /* enable all interrupts */
    ENABLE_GLOBAL_INT;
#endif
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This function shall be implemented if hardware resources need to be release
        when the sample application stops
*////////////////////////////////////////////////////////////////////////////////////////
void HW_Release(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  This function gets the current content of ALEvent register
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 HW_GetALEventRegister(void)
{
	GetInterruptRegister();
	return EscALEvent.Word;
}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
        The behaviour is equal to "HW_GetALEventRegister()"
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 HW_GetALEventRegister_Isr(void)
{
	ISR_GetInterruptRegister();
	return EscALEvent.Word;
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  This function operates the SPI read access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscRead( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    UINT16 i;
    UINT8 *pTmpData = (UINT8 *)pData;

    /* loop for all bytes to be read */
    while ( Len > 0 )
    {
        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;
        }
		
		DISABLE_AL_EVENT_INT;
		PMPReadDRegister(pTmpData,Address,i);
		ENABLE_AL_EVENT_INT;

		Len -= i;
		pTmpData += i;
		Address += i;
    }
}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

\brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
        The behaviour is equal to "HW_EscRead()"
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscReadIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
   UINT16 i;
   UINT8 *pTmpData = (UINT8 *)pData;
    /* loop for all bytes to be read */
   while ( Len > 0 )
   {
        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;
        }

        PMPReadDRegister(pTmpData, Address,i);

        Len -= i;
        pTmpData += i;
        Address += i;
    }
}
#endif //#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

  \brief  This function operates the SPI write access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscWrite( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    UINT16 i;
    UINT8 *pTmpData = (UINT8 *)pData;

    /* loop for all bytes to be written */
    while ( Len )
    {
        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;

            if(Address & 01)
            {
               i=1;
            }
            else if (Address & 02)
            {
               i= (i&1) ? 1:2;
            }
            else if (i == 03)
            {
                i=1;
            }
        }
        DISABLE_AL_EVENT_INT;
        /* start transmission */
        PMPWriteRegister(pTmpData, Address, i);
        ENABLE_AL_EVENT_INT;
        /* next address */
        Len -= i;
        pTmpData += i;
        Address += i;
    }
}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  The SPI PDI requires an extra ESC write access functions from interrupts service routines.
        The behaviour is equal to "HW_EscWrite()"
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscWriteIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    UINT16 i ;
    UINT8 *pTmpData = (UINT8 *)pData;
    /* loop for all bytes to be written */
    while ( Len )
    {
        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;

            if(Address & 01)
            {
               i=1;
            }
            else if (Address & 02)
            {
               i= (i&1) ? 1:2;
            }
            else if (i == 03)
            {
                i=1;
            }
        }        
       /* start transmission */
       PMPWriteRegister(pTmpData, Address, i);
       /* next address */
        Len -= i;
        pTmpData += i;
        Address += i;
    }
}
/*******************************************************************************
  Function:
    void __attribute__ ((__interrupt__, no_auto_psv)) EscIsr(void)

  Summary:
    Interrupt service routine for the PDI interrupt from the EtherCAT Slave Controller

  *****************************************************************************/

void  EcatIsr(void)
{		 
   PDI_Isr();
   ACK_ESC_INT;
}
#endif     // AL_EVENT_ENABLED

#if DC_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    Interrupt service routine for the interrupts from SYNC0
*////////////////////////////////////////////////////////////////////////////////////////

void Sync0Isr(void)
{
 // DISABLE_ESC_INT();    
   Sync0_Isr();   
   /* reset the interrupt flag */
   ACK_SYNC0_INT;
  // ENABLE_ESC_INT();
}
/*ECATCHANGE_START(V5.10) HW3*/
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    Interrupt service routine for the interrupts from SYNC1
*////////////////////////////////////////////////////////////////////////////////////////

void Sync1Isr(void)
{	
	DISABLE_ESC_INT();
	Sync1_Isr();
	ACK_SYNC1_INT;
	ENABLE_ESC_INT();
}
/*ECATCHANGE_END(V5.10) HW3*/
#endif

#if ECAT_TIMER_INT
// Timer 2 ISR (0.1ms)
void TimerIsr(void)
{		
//	LED_0 = ~LED_0;
	ECAT_CheckTimer();
	ECAT_TIMER_ACK_INT;
}

#endif

#endif //#if STM32F405_HW
/** @} */
