/**
\addtogroup EL9800Appl EL9800 application
@{
*/

/**
\file el9800appl.c
\author EthercatSSC@beckhoff.com
\brief Implementation

\version 5.11

<br>Changes to version V5.10:<br>
V5.11 ECAT11: create application interface function pointer, add eeprom emulation interface functions<br>
V5.11 EL9800 1: reset outputs on fallback from OP state<br>
<br>Changes to version V5.01:<br>
V5.10 ECAT6: Add "USE_DEFAULT_MAIN" to enable or disable the main function<br>
<br>Changes to version V5.0:<br>
V5.01 EL9800 2: Add TxPdo Parameter object 0x1802<br>
<br>Changes to version V4.30:<br>
V4.50 ECAT2: Create generic application interface functions. Documentation in Application Note ET9300.<br>
V4.50 COE2: Handle invalid PDO assign values.<br>
V4.30 : create file
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"

#if STM32F405_APPLICATION

/* ECATCHANGE_START(V5.11) ECAT11*/
#include "applInterface.h"
/* ECATCHANGE_END(V5.11) ECAT11*/

//#include "lan9252.h"

#define _EVALBOARD_
#include "stm32f405appl.h"
#undef _EVALBOARD_

#if MCI_HW
#include "mcihw.h"
#endif
#if EL9800_HW
#include "lan9252.h"
#endif

/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

u16 ad7606_value[8];

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void    APPL_AckErrorInd(UINT16 stateTrans)
{ 
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
           all general settings were checked to start the mailbox handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
             all general settings were checked to start the input handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case the application need to be complete 
            the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopOutputHandler(void)
{
/*ECATCHANGE_START(V5.11) EL9800 1*/
    sDOOutputs.leds = 0;
    
    LED_0 = 0;
    LED_1 = 0;
    LED_2 = 0;
    LED_3 = 0;
    LED_4 = 0;
    LED_5 = 0;
    LED_6 = 0;
    LED_7 = 0;
/*ECATCHANGE_END(V5.11) EL9800 1*/
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16* pInputSize,UINT16* pOutputSize)
{
#if COE_SUPPORTED
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM * pPDO = NULL;
    UINT16 PDOSubindex0 = 0;
    UINT32 *pPDOEntry = NULL;
    UINT16 PDOEntryCnt = 0;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;

    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);
        if(pPDO != NULL)
        {
            PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
			//				result = *(UINT8 *)pPDO->pVarPtr;
			//				result=(OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3);
                pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                OutputSize += (UINT16) ((*pPDOEntry) & 0xFF);
            }
        }
        else
        {
            /*assigned PDO was not found in object dictionary. return invalid mapping*/
            OutputSize = 0;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    OutputSize = (OutputSize + 7) >> 3;

    if(result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);
            if(pPDO != NULL)
            {
                PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
                for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
                    pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                    // we increment the expected output size depending on the mapped Entry
                    InputSize += (UINT16) ((*pPDOEntry) & 0xFF);
                }
            }
            else
            {
                /*assigned PDO was not found in object dictionary. return invalid mapping*/
                InputSize = 0;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }
    InputSize = (InputSize + 7) >> 3;

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
#else
    *pInputSize = 6;
    *pOutputSize = 2;
    return ALSTATUSCODE_NOERROR;
#endif

}


/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data
\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_InputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;

    /* we go through all entries of the TxPDO Assign object to get the assigned TxPDOs */
   for (j = 0; j < sTxPDOassign.u16SubIndex0; j++)
   {
      switch (sTxPDOassign.aEntries[j])
      {
      /* TxPDO 1 */
      case 0x1A00:
         *pTmpData++ = SWAPWORD((UINT16)sDIInputs.keys);
         break;
      /* TxPDO 3 */
      case 0x1A02:
		*pTmpData++ = SWAPWORD((UINT16)sAIInputs.i16Analoginput_0);
		*pTmpData++ = SWAPWORD((UINT16)sAIInputs.i16Analoginput_1);
		*pTmpData++ = SWAPWORD((UINT16)sAIInputs.i16Analoginput_2);
		*pTmpData++ = SWAPWORD((UINT16)sAIInputs.i16Analoginput_3);
         break;
      }
   }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;

    /* we go through all entries of the RxPDO Assign object to get the assigned RxPDOs */
    for (j = 0; j < sRxPDOassign.u16SubIndex0; j++)
    {
        switch (sRxPDOassign.aEntries[j])
        {
        /* RxPDO 2 */
        case 0x1601:
            ((UINT16 *) &sDOOutputs)[1] = SWAPWORD(*pTmpData++);
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR 
            or from the mainloop if no synchronisation is supported
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{ 
    LED_0 = sDOOutputs.leds >> 0;
    LED_1 = sDOOutputs.leds >> 1;
    LED_2 = sDOOutputs.leds >> 2;
    LED_3 = sDOOutputs.leds >> 3;
    LED_4 = sDOOutputs.leds >> 4;
    LED_5 = sDOOutputs.leds >> 5;
    LED_6 = sDOOutputs.leds >> 6;
    LED_7 = sDOOutputs.leds >> 7;
	
	if(KEY_0) 	sDIInputs.keys |= 0x01;
	else		sDIInputs.keys &= ~0x01;
	
	if(KEY_1)   sDIInputs.keys |= 0x02;
	else		sDIInputs.keys &= ~0x02;
	
	if(KEY_2)   sDIInputs.keys |= 0x04;
	else		sDIInputs.keys &= ~0x04;
	
	if(KEY_3)   sDIInputs.keys |= 0x08;
	else		sDIInputs.keys &= ~0x08;
	
	if(KEY_4)   sDIInputs.keys |= 0x10;
	else		sDIInputs.keys &= ~0x10;
	
	if(KEY_5)   sDIInputs.keys |= 0x20;
	else		sDIInputs.keys &= ~0x20;
	
	if(KEY_6)   sDIInputs.keys |= 0x40;
	else		sDIInputs.keys &= ~0x40;
	
	if(KEY_7)   sDIInputs.keys |= 0x80;
	else		sDIInputs.keys &= ~0x80;
					
	sAIInputs.i16Analoginput_0  = ad7606_value[0];
	sAIInputs.i16Analoginput_1  = ad7606_value[1];
	sAIInputs.i16Analoginput_2  = ad7606_value[2];
	sAIInputs.i16Analoginput_3  = ad7606_value[3];
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     index               index of the requested object.
 \param     subindex            subindex of the requested object.
 \param     objSize             size of the requested object data, calculated with OBJ_GetObjectLength
 \param     pData               Pointer to the buffer where the data can be copied to
 \param     bCompleteAccess     Indicates if a complete read of all subindices of the
                                object shall be done or not

 \return    ABORTIDX_XXX

 \brief     Handles SDO read requests to TxPDO Parameter
*////////////////////////////////////////////////////////////////////////////////////////
UINT8 ReadObject0x1802( UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess )
{

    if(bCompleteAccess)
        return ABORTIDX_UNSUPPORTED_ACCESS;

    if(subindex == 0)
    {
        *pData = TxPDO1802Subindex0;
    }
    else if(subindex == 6)
    {
        /*clear destination buffer (no excluded TxPDO set)*/
        if(dataSize > 0)
            MBXMEMSET(pData,0x00,dataSize);
    }
    else if(subindex == 7)
    {
        /*min size is one Byte*/
        UINT8 *pu8Data = (UINT8*)pData;
        
        //Reset Buffer
        *pu8Data = 0; 

        //*pu8Data = sAIInputs.bTxPDOState;
    }
    else if(subindex == 9)
    {
        /*min size is one Byte*/
        UINT8 *pu8Data = (UINT8*)pData;
        
        //Reset Buffer
        *pu8Data = 0; 

        //*pu8Data = sAIInputs.bTxPDOToggle;
    }
    else
        return ABORTIDX_SUBINDEX_NOT_EXISTING;

    return 0;
}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GetDeviceID()
{
    UINT16 Value = (UINT16)(SWITCH_8<<7)|(SWITCH_7<<6)|(SWITCH_6<<5)|(SWITCH_5<<4)|(SWITCH_4<<3)|(SWITCH_3<<2)|(SWITCH_2<<1)|(SWITCH_1);
    return Value;
}
#endif

#if USE_DEFAULT_MAIN
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This is the main function
*////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
    /* initialize the Hardware and the EtherCAT Slave Controller */	
    HW_Init();
    MainInit();
    bRunApplication = TRUE;
    do
    {		
		ADC_CONVERT = 0;
		MainLoop();
		ADC_CONVERT = 1;
		SPI1_NSS = 0;
		MainLoop();
		SPI1->DR = 0;
		while(ADC_BUSY)
		{
			while(ADC_BUSY);
		}
		while((SPI1->SR & 0x001) == 0);
		ad7606_value[3] = SPI_I2S_ReceiveData(SPI1);
		SPI1->DR = 0;
		while((SPI1->SR & 0x001) == 0);
		ad7606_value[2] = SPI_I2S_ReceiveData(SPI1);
		SPI1->DR = 0;
		while((SPI1->SR & 0x001) == 0);
		ad7606_value[1] = SPI_I2S_ReceiveData(SPI1);
		SPI1->DR = 0;
		while((SPI1->SR & 0x001) == 0);
		ad7606_value[0] = SPI_I2S_ReceiveData(SPI1);
		SPI1_NSS = 1;
		
		
    } while (bRunApplication == TRUE);

    HW_Release();
    return 0;
}
#endif //#if USE_DEFAULT_MAIN
#endif //#if STM32F405_APPLICATION

/** @} */
