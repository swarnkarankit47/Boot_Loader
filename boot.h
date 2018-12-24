
/* function declaration is done here, global variables are also present here */

#ifndef BOOH_H
#define BOOT_H

#include "types.h"
#include "boot_cfg.h"

t_eStates GeCurrentState;

/* Better is to Place version number in ROM using pragmas */
 uint_8 GaImageBuffer[IMAGE_SIZE] = {
 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,
 0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12,0xAB,0xCD,0xEF,0x12, 
};

/***************************Bootloader_Functions************************/
void Boot_InitVar(void);
uint_32 Boot_CalculateChecksum(uint_8 * StartAddress, uint_32 length);
t_eStatus Boot_RestartTargetDevice(void);
t_eStatus Boot_SendHelloCommand(void);
void Boot_ConfigPortPin(void);
uint_32 Boot_ReadCurrentTick(void);
uint_32 Boot_GetImageVersion(uint_8 partition, t_eStatus * status);
t_eStatus Boot_IsVersionEqual(uint_32 ver1, uint_32 ver2);
t_eStatus Boot_SendResumeCommadToTarget(void);
t_eStatus Boot_ErasePrimaryImageInTarget(uint_32 addr, uint_32 size);
void Boot_Delay(uint_32 delay);
t_eStatus Boot_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * pcommand);
t_eStatus Boot_SendCommand(t_sCmdStruct command);
t_eStatus Boot_CheckFlashEraseStatus(void);
t_eStatus Boot_WritePrimaryImage( uint_8 Buff[], uint_32 size);
t_eStatus Boot_VerifyCheckSumTargetFlash(uint_32 checksum);
t_eStatus Boot_StoreNewImageVerAndChecksum(uint_32 ver, uint_32 checksum, uint_32 addr);
uint_32 Boot_TxPageFromImageBuff(uint_32 startindex, uint_32 size, t_eStatus * status);
t_eStatus Boot_ValidateChecksumInTargetBuff(uint_32 checksum);
t_eStatus Boot_WriteToBufferInTarget(uint_8 data, uint_32 index);
t_eStatus Boot_CheckFlashWriteStatus(void);
void Boot_CommProtocolConfig(void);
void Boot_CommProtocolDeconfig(void);
t_eStatus Boot_WriteBuffToFlashInTarget(uint_32 addr);
t_eStatus Boot_InitialiseBuffInTarget(void);
void Boot_GiveSignalOnResetPin(void);
void Boot_IndicateFinishStatus(void);
/***********************************************************************/


#endif /* BOOT_H */
