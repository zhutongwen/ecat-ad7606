/**
 * \addtogroup EL9800Appl EL9800 application
 * @{
 * This application includes three functional modules:<br>
 * Digital Input<br>
 * Digital Outputs<br>
 * Analog Input<br>
 * The objects of the modules are defined according the ETG.5001.<br>
 */

/**
\file el9800appl.h
\author EthercatSSC@beckhoff.com
\brief EL9800 Application defines, typedefs and global variables

\version 5.01

<br>Changes to version - :<br>
V5.01 : Start file change log
 */
#ifndef _EL9800APPL_H_
#define _EL9800APPL_H_

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecatappl.h"
#include "objdef.h"
#include "bsp.h"


/*-----------------------------------------------------------------------------------------
------
------    Defines and Typedef
------
-----------------------------------------------------------------------------------------*/

/**
 * \addtogroup PdoMappingObjects PDO Mapping Objects
 *
 * Digital Input PDO mapping : 0x1A00<br>
 * Digital Output PDO mapping : 0x1601<br>
 * Analog Input PDO mapping : 0x1A02
 * @{
 */
/** \brief 0x1601 (Digital output RxPDO) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32   aEntries[9]; /**< \brief Entry buffer*/
} OBJ_STRUCT_PACKED_END
TOBJ1601;


/** \brief 0x1A00 (Digital input TxPDO) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32   aEntries[9]; /**< \brief Entry buffer*/
} OBJ_STRUCT_PACKED_END
TOBJ1A00;


/** \brief 0x1A02 (Analog input TxPDO) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32   aEntries[8]; /**< \brief Entry buffer*/
} OBJ_STRUCT_PACKED_END
TOBJ1A02;
/** @}*/


/**
 * \addtogroup SmAssignObjects SyncManager Assignment Objects
 * SyncManager 2 : 0x1C12 
 * <br>SyncManager 3 : 0x1C13
 * @{
 */
/** \brief 0x1C12 (SyncManager 2 assignment) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT16   aEntries[1]; /**< \brief Entry buffer*/
} OBJ_STRUCT_PACKED_END
TOBJ1C12;


/** \brief 0x1C13 (SyncManager 3 assignment) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT16   aEntries[2]; /**< \brief Entry buffer*/
} OBJ_STRUCT_PACKED_END
TOBJ1C13;
/** @}*/


/**
 * \addtogroup PDO Process Data Objects
 * Digital Inputs : 0x6000<br>
 * Analog Inputs : 0x6010<br> 
 * Digital Outputs : 0x7020
 * @{
 */
/** \brief 0x6000 (Digital input object) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT16	keys;		 /**< \brief Switchs*/
} OBJ_STRUCT_PACKED_END
TOBJ6000;


/** \brief 0x6020 (Analog input object) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
	UINT16  u16SubIndex0; 		/**< \brief SubIndex 0*/
	INT16   i16Analoginput_0; 	/**< \brief (SI17) Analog input value*/
	INT16   i16Analoginput_1; 	/**< \brief (SI17) Analog input value*/
	INT16   i16Analoginput_2; 	/**< \brief (SI17) Analog input value*/
	INT16   i16Analoginput_3; 	/**< \brief (SI17) Analog input value*/
} OBJ_STRUCT_PACKED_END
TOBJ6020;


/** \brief 0x7000 (Digital output object) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; 	/**< \brief SubIndex 0*/
   UINT16    leds; 			/**< \brief LEDs*/
} OBJ_STRUCT_PACKED_END
TOBJ7000;
/** @}*/

#endif //_EL9800APPL_H_

#ifdef _EVALBOARD_
    #define PROTO
#else
    #define PROTO extern
#endif


#ifdef _EVALBOARD_
/**
 * \addtogroup SmAssignObjects SyncManager Assignment Objects
 * @{
 */
/**
 * \brief Entry descriptions of SyncManager assign objects
 *
 * SubIndex0<br>
 * SubIndex1 (for all other entries the same description will be used (because the object code is ARRAY))
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asPDOAssignEntryDesc[] = {
   {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE}};
/** @}*/

   
/**
 * \addtogroup EnumObjects Enum Objects
 * @{
 * Presentation (Signed/Unsigned) : 0x800
 */

/*---------------------------------------------
-    0x0800: ENUM (Signed/Unsigned Presentation)
-----------------------------------------------*/
CHAR sEnum0800_Value00[] = "\000\000\000\000Signed presentation"; /**< \brief Value = 0x00, Text = Signed presentation */
CHAR sEnum0800_Value01[] = "\001\000\000\000Unsigned presentation"; /**< \brief Value = 0x01, Text = Unsigned presentation */
CHAR *apEnum0800[2] = { sEnum0800_Value00, sEnum0800_Value01};/**< \brief List of Enum (0x800) values*/

/**
 * \brief Enum entry description
 *
 * SubIndex0<br>
 * Enum (Signed Presentation)
 * enum (Unsigned Presentation)
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x0800[] =
   {{DEFTYPE_UNSIGNED8, 8, ACCESS_READ | OBJACCESS_NOPDOMAPPING},
    {DEFTYPE_OCTETSTRING, 8*SIZEOF(sEnum0800_Value00), ACCESS_READ | OBJACCESS_NOPDOMAPPING},
   {DEFTYPE_OCTETSTRING, 8*SIZEOF(sEnum0800_Value01), ACCESS_READ | OBJACCESS_NOPDOMAPPING}};
/** @}*/


/**
* \addtogroup PdoMappingObjects PDO Mapping Objects
* @{
*/
/**
 * \brief Object 0x1601 (Digital output RxPDO) entry descriptions
 * 
 * SubIndex 0 : read only<br>
 * SubIndex x : read only<br>
 *  (x > 0)
 */
OBJCONST TSDOINFOENTRYDESC	OBJMEM asEntryDesc0x1601[] = {
   {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ },
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}};

/**
 * \brief Object 0x1601 (Digital output RxPDO) name
 *
 * In this example no specific entry name is defined ("SubIndex xxx" is used)
 */
OBJCONST UCHAR OBJMEM aName0x1601[] = "DO RxPDO-Map\000\377";
#endif //#ifdef _EVALBOARD_


/**
 * \brief Object 0x1601 (Digital output RxPDO) variable to handle object data
 * 
 * SubIndex 0 : 1<br>
 * SubIndex 1 : 0x7000.1 1bit (Reference to LED1)<br>
 * SubIndex 2 : 0x7000.1 1bit (Reference to LED2)<br>
 * SubIndex 3 : 0x7000.1 1bit (Reference to LED3)<br>
 * SubIndex 4 : 0x7000.1 1bit (Reference to LED4)<br>
 * SubIndex 5 : 0x7000.1 1bit (Reference to LED5), only for PIC24<br>
 * SubIndex 6 : 0x7000.1 1bit (Reference to LED6), only for PIC24<br>
 * SubIndex 7 : 0x7000.1 1bit (Reference to LED7), only for PIC24<br>
 * SubIndex 8 : 0x7000.1 1bit (Reference to LED8), only for PIC24
 */
PROTO TOBJ1601 sDORxPDOMap
#ifdef _EVALBOARD_
 = {1, {0x70000110}}
#endif
;
/** @}*/


/**
 * \addtogroup PdoParameter PDO Parameter
 * @{
 *
 * Parameter for PDO mapping object 0x1A02 : 0x1802
 */
#ifdef _EVALBOARD_
/**
 * \brief Entry descriptions of TxPDO Parameter object (0x1802)
 * 
 * Subindex 0<br>
 * SubIndex 1 - 5 : not defined<br>
 * SubIndex 6 : Exclude TxPDOs<br>
 * SubIndex 7 : TxPDO State<br>
 * SubIndex 8 : not defined<br>
 * SubIndex 9 : TxPDO Toggle
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1802[] = {
   {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ },
   {0, 0x0, 0},
   {0, 0x0, 0},
   {0, 0x0, 0},
   {0, 0x0, 0},
   {0, 0x0, 0},
   {DEFTYPE_OCTETSTRING, 0x00, ACCESS_READ | OBJACCESS_TXPDOMAPPING},
   {DEFTYPE_BOOLEAN, 0x01, ACCESS_READ | OBJACCESS_TXPDOMAPPING},
   {0, 0x0, 0},
   {DEFTYPE_BOOLEAN, 0x01, ACCESS_READ | OBJACCESS_TXPDOMAPPING}};

/**
 * \brief Object 0x1802 (TxPDO Parameter) object and entry names
 */
OBJCONST UCHAR OBJMEM aName0x1802[] = "TxPDO Parameter\000\000\000\000\000\000Exclude TxPDOs\000TxPDOState\000\000TxPDO Toggle\000\377";
#endif //#ifdef _EVALBOARD_



/**
 * \brief Object 0x1802 (TxPDO Parameter) variable to handle object data
 * 
 * Only Subindex0 for this Object is stored here (all values are stored in other structures, see "ReadObject0x1802" for more details)
 */
PROTO UINT16 TxPDO1802Subindex0
#ifdef _EVALBOARD_
    = 9
#endif
    ;
/** @}*/


/**
 * \addtogroup PdoMapping PDO Mapping Objects
 * @{
 */
/**
 * \brief Object 0x1A00 (Digital input TxPDO) entry descriptions
 *
 * SubIndex 0 : read only<br>
 * SubIndex x : read only<br>
 *  (x > 0)
*/
#ifdef _EVALBOARD_
OBJCONST TSDOINFOENTRYDESC	OBJMEM asEntryDesc0x1A00[] = {
   {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ },
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}};


/**
 * \brief Object 0x1A00 (Digital input TxPDO) object and entry names
 *
 * In this example no specific entry name is defined ("SubIndex xxx" is used)
 */
OBJCONST UCHAR OBJMEM aName0x1A00[] = "DI TxPDO-Map\000\377";
#endif //#ifdef _EVALBOARD_


/**
 * \brief Object 0x1A00 (Digital Input TxPDO) variable to handle object data
 * 
 * SubIndex 0 : 1<br>
 * SubIndex 1 : 0x6000.1 1bit (Reference to SWITCH1)<br>
 * SubIndex 2 : 0x6000.1 1bit (Reference to SWITCH2)<br>
 * SubIndex 3 : 0x6000.1 1bit (Reference to SWITCH3)<br>
 * SubIndex 4 : 0x6000.1 1bit (Reference to SWITCH4)<br>
 * SubIndex 5 : 0x6000.1 1bit (Reference to SWITCH5), only for PIC24<br>
 * SubIndex 6 : 0x6000.1 1bit (Reference to SWITCH6), only for PIC24<br>
 * SubIndex 7 : 0x6000.1 1bit (Reference to SWITCH7), only for PIC24<br>
 * SubIndex 8 : 0x6000.1 1bit (Reference to SWITCH8), only for PIC24
 */
PROTO TOBJ1A00 sDITxPDOMap
#ifdef _EVALBOARD_
 = {1, 0x60000110}
#endif
;


/**
 * \brief Object 0x1A02 (Analog input TxPDO) entry descriptions
 *
 * SubIndex 0 : read only<br>
 * SubIndex x : read only<br>
 *  (x > 0)
*/
#ifdef _EVALBOARD_
OBJCONST TSDOINFOENTRYDESC	OBJMEM asEntryDesc0x1A02[] = {
   {DEFTYPE_UNSIGNED8,  0x8,  ACCESS_READ|ACCESS_WRITE_PREOP }, /* Subindex 000 */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ|ACCESS_WRITE_PREOP },
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ|ACCESS_WRITE_PREOP },
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ|ACCESS_WRITE_PREOP },
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ|ACCESS_WRITE_PREOP }
}; /* SubIndex 001: SubIndex 001 */


/**
 * \brief Object 0x1A02 (Analog input TxPDO) object and entry names
 *
 * In this example no specific entry name is defined ("SubIndex xxx" is used)
 */
OBJCONST UCHAR OBJMEM aName0x1A02[] = "AI TxPDO-Map\000\377";
#endif //#ifdef _EVALBOARD_



/**
 * \brief Object 0x1A02 (Analog Input TxPDO) variable to handle object data
 * 
 * SubIndex 0 : 1<br>
 * SubIndex 1 : 0x6020.1 16bit (Reference to Analog input value)
 */
PROTO TOBJ1A02 sAITxPDOMap
#ifdef _EVALBOARD_
= {4, {0x60200110, 0x60200210, 0x60200310, 0x60200410}}
#endif
;
/** @}*/


/**
 * \addtogroup SmAssignObjects SyncManager Assignment Objects
 * @{
 */
#ifdef _EVALBOARD_
/**
 * \brief 0x1C12 (SyncManager 2 assignment) object name 
 * 
 * No entry names defined because the object code is ARRAY and all entry names are "SubIndex 000"
 */
OBJCONST UCHAR OBJMEM aName0x1C12[] = "RxPDO assign";
#endif //#ifdef _EVALBOARD_


/**
 * \brief 0x1C12 (SyncManager 2 assignment) variable to handle object data
 * 
 * SubIndex 0 : 1<br>
 * SubIndex 1 : 0x1601
 */
PROTO TOBJ1C12 sRxPDOassign
#ifdef _EVALBOARD_
= {0x01, {0x1601}}
#endif
;


#ifdef _EVALBOARD_
/**
 * \brief 0x1C13 (SyncManager 3 assignment) object name 
 * 
 * No entry names defined because the object code is ARRAY and all entry names are "SubIndex 000"
 */
OBJCONST UCHAR OBJMEM aName0x1C13[] = "TxPDO assign";
#endif //#ifdef _EVALBOARD_


/**
 * \brief 0x1C13 (SyncManager 3 assignment) variable to handle object data
 * 
 * SubIndex 0 : 1<br>
 * SubIndex 1 : 0x1A00<br>
 * SubIndex 2 : 0x1A02
 */
PROTO TOBJ1C13 sTxPDOassign
#ifdef _EVALBOARD_
= {0x02, {0x1A02, 0x1A00}}
#endif
;
/** @}*/


/**
 * \addtogroup PDO Process Data Objects
 * @{
 */
#ifdef _EVALBOARD_
/**
 * \brief Object 0x6000 (Digital input object) entry descriptions
 *
 * SubIndex 0 : read only<br>
 * SubIndex x : (One description for each switch) read only and TxPdo mappable<br>
 *  (x > 0)
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6000[] = {
   {DEFTYPE_UNSIGNED8,  0x8,  ACCESS_READ }, /* Subindex 000 */
   {DEFTYPE_UNSIGNED16, 0x16, ACCESS_READ | OBJACCESS_TXPDOMAPPING}, /* SubIndex 001: Switchs */
   {DEFTYPE_UNSIGNED16, 0x16, ACCESS_READ | OBJACCESS_TXPDOMAPPING}}; /* Subindex 009 for align */

/**
 * \brief 0x6000 (Digital input object) object and entry names
 */
OBJCONST UCHAR OBJMEM aName0x6000[] = "DI Inputs\000Switchs\000\000\377";
#endif //#ifdef _EVALBOARD_


/**
 * \brief 0x6000 (Digital input object) variable to handle object data
 * 
 * SubIndex 0 : 1<br>
 * SubIndex x : every switch value is 0 by default
 */
PROTO TOBJ6000 sDIInputs
#ifdef _EVALBOARD_
= {1, 0x00}
#endif
;


#ifdef _EVALBOARD_
/**
 * \brief Object 0x6020 (Analog input object) entry descriptions
 *
 * SubIndex 0 : read only<br>
 * SubIndex 001: Analog input
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6020[] = {
   {DEFTYPE_UNSIGNED8, 0x8,  ACCESS_READ },
   {DEFTYPE_INTEGER16, 0x10, ACCESS_READ | OBJACCESS_TXPDOMAPPING},
   {DEFTYPE_INTEGER16, 0x10, ACCESS_READ | OBJACCESS_TXPDOMAPPING},
   {DEFTYPE_INTEGER16, 0x10, ACCESS_READ | OBJACCESS_TXPDOMAPPING},
   {DEFTYPE_INTEGER16, 0x10, ACCESS_READ | OBJACCESS_TXPDOMAPPING}
};


/**
 * \brief 0x6020 (Analog input object) object and entry names
 */
OBJCONST UCHAR OBJMEM aName0x6020[] = "AI Inputs\000Analog input\000\377";
#endif //#ifdef _EVALBOARD_

/**
 * \brief 0x6020 (Analog input object) variable to handle object data
 * 
 */
PROTO TOBJ6020 sAIInputs
#ifdef _EVALBOARD_
= {4, 0, 0, 0, 0}
#endif
;



/**
 * \brief Object 0x7000 (Digital output object) entry descriptions
 *
 * SubIndex 0 : read only<br>
 * SubIndex x : (One description for each led) read only and RxPdo mappable<br>
 *  (x > 0)
*/
#ifdef _EVALBOARD_
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x7000[] = {
   {DEFTYPE_UNSIGNED8,  0x8,  ACCESS_READ }, /* Subindex 000 */
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ | OBJACCESS_RXPDOMAPPING}, /* SubIndex 001: LEDs */
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ | OBJACCESS_RXPDOMAPPING}}; /* Subindex 008 for align */


/**
 * \brief 0x7000 (Digital output object) object and entry names
 */
OBJCONST UCHAR OBJMEM aName0x7000[] = "DO Outputs\000LEDs\000\000\377";
#endif //#ifdef _EVALBOARD_


/**
 * \brief 0x7000 (Analog input object) variable to handle object data
 * 
 */
PROTO TOBJ7000 sDOOutputs
#ifdef _EVALBOARD_
= {1, 0x00}
#endif
;
/** @}*/


#ifdef _EVALBOARD_
PROTO UINT8 ReadObject0x1802( UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess );
#endif    //#ifdef _EVALBOARD_
/*if _PIC18 is enabled the object dictionary is fixed defined in coeappl.c*/

/**
 *\brief EL9800 Application specific object dictionary
 * 
 */
PROTO TOBJECT    OBJMEM ApplicationObjDic[]
#ifdef _EVALBOARD_
= {
   /* Enum 0x0800 */
   {NULL,NULL, 0x0800, {DEFTYPE_ENUM, 0x02 | (OBJCODE_REC << 8)}, asEntryDesc0x0800, 0, apEnum0800 },
   /* Object 0x1601 */
   {NULL,NULL,  0x1601, {DEFTYPE_PDOMAPPING, 9 | (OBJCODE_REC << 8)}, asEntryDesc0x1601, aName0x1601, &sDORxPDOMap, NULL, NULL, 0x0000 },
   /* Object 0x1802 */
   {NULL,NULL,  0x1802, {DEFTYPE_RECORD, 9 | (OBJCODE_REC << 8)}, asEntryDesc0x1802, aName0x1802,&TxPDO1802Subindex0, ReadObject0x1802, NULL, 0x0000 },
   /* Object 0x1A00 */
   {NULL,NULL,   0x1A00, {DEFTYPE_PDOMAPPING, 9 | (OBJCODE_REC << 8)}, asEntryDesc0x1A00, aName0x1A00, &sDITxPDOMap, NULL, NULL, 0x0000 },
   /* Object 0x1A02 */
   {NULL,NULL,   0x1A02, {DEFTYPE_PDOMAPPING, 8 | (OBJCODE_REC << 8)}, asEntryDesc0x1A02, aName0x1A02, &sAITxPDOMap, NULL, NULL, 0x0000 },
    /* Object 0x1C12 */
   {NULL,NULL,   0x1C12, {DEFTYPE_UNSIGNED16, 1 | (OBJCODE_ARR << 8)}, asPDOAssignEntryDesc, aName0x1C12, &sRxPDOassign, NULL, NULL, 0x0000 },
   /* Object 0x1C13 */
   {NULL,NULL,   0x1C13, {DEFTYPE_UNSIGNED16, 2 | (OBJCODE_ARR << 8)}, asPDOAssignEntryDesc, aName0x1C13, &sTxPDOassign, NULL, NULL, 0x0000 },
   /* Object 0x6000 */
   {NULL,NULL,   0x6000, {DEFTYPE_RECORD, 8 | (OBJCODE_REC << 8)}, asEntryDesc0x6000, aName0x6000, &sDIInputs, NULL, NULL, 0x0000 },
   /* Object 0x6020 */
   {NULL,NULL,   0x6020, {DEFTYPE_RECORD, 17 | (OBJCODE_REC << 8)}, asEntryDesc0x6020, aName0x6020, &sAIInputs, NULL, NULL, 0x0000 },
   /* Object 0x7000 */
   {NULL,NULL,   0x7000, {DEFTYPE_RECORD, 8 | (OBJCODE_REC << 8)}, asEntryDesc0x7000, aName0x7000, &sDOOutputs, NULL, NULL, 0x0000 },
   {NULL,NULL, 0xFFFF, {0, 0}, NULL, NULL, NULL, NULL}}
#endif    //#ifdef _EVALBOARD_
;

PROTO void APPL_Application(void);

PROTO void   APPL_AckErrorInd(UINT16 stateTrans);
PROTO UINT16 APPL_StartMailboxHandler(void);
PROTO UINT16 APPL_StopMailboxHandler(void);
PROTO UINT16 APPL_StartInputHandler(UINT16 *pIntMask);
PROTO UINT16 APPL_StopInputHandler(void);
PROTO UINT16 APPL_StartOutputHandler(void);
PROTO UINT16 APPL_StopOutputHandler(void);

PROTO UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize);
PROTO void APPL_InputMapping(UINT16* pData);
PROTO void APPL_OutputMapping(UINT16* pData);


#undef PROTO
/** @}*/
