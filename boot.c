

/* Boot related functionality is present in this file */


#include "boot.h"
#include "boot_cfg.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"

int main(void) {
 
  uint_32 LulPartitionOneVer;
  uint_32 LulPartitionTwoVer;
  t_eStatus LePartitionOne;
  t_eStatus LePartitionTwo;
  uint_32 GulChecksum;
  uint_32 LulRetryCount;
  t_eStatus LeResult;
  t_eStatus LeStatus[2];
    
  /* starting from initial state in state machine */
  GeCurrentState = INITIAL_STATE;
 
 while(1)
 {

/*
Assumptions : 1. I used protocol as mentioned in Command_Description file.
              2. I am assuming that image size is of 1KB as defined in boot cfg
                 file.
              2. I am asking version of each partition using partition number.
              3. If new version is not present in target so what I am doing is 
              that I am always changing partition 1(assuming it is a primary image).
*/

/*
Description of states :
    
INITIAL_STATE :Variables and communication peripheral initialisation 
and tagrget device reset is done.
                          
QUERY_COMPARE_STATE_INIT : Sending Hello command(in defined interval) to see if
device is ready to accept commands or not?
                           
QUERY_COMPARE_STATE_SEND_QUERY : Here I am checking if new versio is not already 
present in target device, if it is not present we will move to next stage else 
we will give resume command to target.

ERASE_STATE : Erasing primary image present in target and after erasing move to 
next stage. If target device is taking longer time for erasing then we will wait
for defined time and if still not responding we will make target device reset.

DOWNLOADING_STATE : If we have erased successfully in previous state then we 
we will transfer our image to target device and at same time we will keep checking 
that image is transmitted successfully by checking checksum(after each 256 Bytes).
If this process is completed successfully then we will move to next stage.

VERIFY_CHECKSUM_STATE : In this stage we will verify checksum new full image present
in flash of target deivce with image we had in host device. If it is fine then we 
will move to next stage else we will start whole process again.

FINAL_STATE : After successful transfer of image we will store new version number and 
checksum of transfered image in target device and make target device reset so that 
it can start it's boot process again.

IDLE_STATE : Here we will deconfig comuunication peripheral which are not needed for now.
and will indicate status through LED that firmware transffer process is not finished.

*/
  
     switch(GeCurrentState)
     {

         case INITIAL_STATE :
         /* initialising local variables */
         LulPartitionOneVer = 0;
         LulPartitionTwoVer = 0;
         LePartitionOne = NOT_OK;
         LePartitionTwo = NOT_OK;
         
         /* Initialise global variables if any */
         Boot_InitVar();
         /* Configure communication protocol */
         Boot_CommProtocolConfig();
         /* Restart target device in boot mode */
         if(OK == Boot_RestartTargetDevice())
         {
           /* Target reset command is sent successfully at target */
         }
         else
         {
           /* Target device did not respond even after defined number of retry,
              start whole process again */
           GeCurrentState = INITIAL_STATE;
         }
         if(GeCurrentState != INITIAL_STATE)
         {
           /* giving delay for target device setup */
           Boot_Delay(SETUP_DELAY_FOR_TARGET);
           /* Now we can move to next state */
           GeCurrentState = QUERY_COMPARE_STATE_INIT;           
         }
         else
         {
           /* for MISRA */
         }
        
         break;
         
         case QUERY_COMPARE_STATE_INIT :
         /* we are sending hello command HELLO_QUERY_NUM number of times
            at each interval defined by HELLO_QUERY_INTERVAL */
         LulRetryCount = HELLO_QUERY_NUM;
         do{
           if( OK == Boot_SendHelloCommand() )
           {
             /* It means response received from target device */
             GeCurrentState = QUERY_COMPARE_STATE_SEND_QUERY;
             break;
           }
           /* Giving few ms delay(defined in boot cfg file) between each query
              so that target port is not continuously busy because of hello 
              command */
           Boot_Delay(RESOLUTION_CONV_MS_TO_CLK_COUNT(HELLO_QUERY_INTERVAL));
           LulRetryCount--;
         }while( LulRetryCount > 0);
          
         if( GeCurrentState != QUERY_COMPARE_STATE_SEND_QUERY)
         {
           /* timeout occured so we will start process again */
           GeCurrentState = INITIAL_STATE;
         }
         else
         {
           /* do nothing proceed to next stage */
         }
         
         break;
         
         case QUERY_COMPARE_STATE_SEND_QUERY :
           /* Getting versions of both images present in both partitions, 
              assume version is of 4 byte */
           LulPartitionOneVer = Boot_GetImageVersion( PARTITION_ONE,  &LeStatus[0]);
           LulPartitionTwoVer = Boot_GetImageVersion( PARTITION_TWO , &LeStatus[1]);
           if((LeStatus[0] == OK) && (LeStatus[1] == OK))
           {
             /* we got the response of version get command for both partition */
             /* comparing both versions(NEW_VERSION) with new version which we want to 
              transfer */
             LePartitionOne = Boot_IsVersionEqual(NEW_VERSION, LulPartitionOneVer);
             LePartitionTwo = Boot_IsVersionEqual(NEW_VERSION, LulPartitionTwoVer);
             if( (LePartitionOne != OK) && (LePartitionTwo != OK) )
             {
               /* Version is not present in any partition so we can 
                transfer new version now, first move to erase state */              
               GeCurrentState = ERASE_STATE;
             }
             else
             {
               /* Same verion is present in one of the 2 partitions so 
                we will not update now and send resume command to target so that it 
                can jump to application present in right partition */
               Boot_SendResumeCommadToTarget();
               /* In IDLE state we will deconfig all peripherals which are not needed */
               GeCurrentState = IDLE_STATE;
             }             
           }
           else
           {
             /* target device did not respond so start process again */
             GeCurrentState = INITIAL_STATE;
           }
                     
         break;
         
         case ERASE_STATE :
           /* Erasing primary image in target so that it can be re-written
              with new image. Erasing will take some time depends on 
              micro controller */
           if(OK == Boot_ErasePrimaryImageInTarget(PRIMARY_IMAGE_ADDR, IMAGE_SIZE))
           {
             /* we will give delay here to wait flash erase process to finish at
              target device, this delay depends on target device and can be 
              found in it's data sheet */
             Boot_Delay(FLASH_ERASE_WAIT);
             /* here we are checking if flash erase process is finished after 
              defined time, if not, we are asking again from target device if it 
              is finished or not for fixed number of times */
             LulRetryCount = RETRY_COUNT_ERASE_FLASH;
             do{

             if( Boot_CheckFlashEraseStatus() == OK)
             {
             /* erase is finished at target device so we can move to download 
                state */
               GeCurrentState = DOWNLOADING_STATE;
               break;
             }
             else
             {
               Boot_Delay(WAIT_TO_SEND_QUERY);
             }
           
             }while(LulRetryCount--);

             if(GeCurrentState == DOWNLOADING_STATE)
             {
               /* do nothing as erase process is finished successfully */
             }
             else
             {
             /* Slave is busy in erasing for more than the required time so 
                we will reset target device by giving signal on reset pin as 
                it is not responding to commands */
               Boot_GiveSignalOnResetPin();
               /* making state as initial means we will re start whole process again
               */
               GeCurrentState = INITIAL_STATE;
             }            
           }
           else
           {
             /* target device did not respond to command so start whole process again */
             GeCurrentState = INITIAL_STATE;
           }

         break;
         
         case DOWNLOADING_STATE :
         /* Now primary image area is erased so we can move our new image data
            to buffer present at target device and program the flash area of 
            target device page by page(256 Byte) using that buffer */
         if( OK == Boot_WritePrimaryImage(GaImageBuffer, IMAGE_SIZE) )
         {
           GeCurrentState = VERIFY_CHECKSUM_STATE;
         }
         else
         {
           GeCurrentState = INITIAL_STATE;
         }

         break;
         
         case VERIFY_CHECKSUM_STATE :

         /* Calculating checksum for new image which we have transferred */
         GulChecksum = Boot_CalculateChecksum((uint_8*)GaImageBuffer , IMAGE_SIZE);
         /* verify if same checksum is present in target device for transferred image */
         LeResult = Boot_VerifyCheckSumTargetFlash(GulChecksum);
         
         if(LeResult == OK)
         {
           GeCurrentState = FINAL_STATE;
         }
         else
         {
           /* Updation of fw is not done correctly so repeat process again */
           GeCurrentState = INITIAL_STATE;          
         }
         break;
         
         case FINAL_STATE :  
         /* storing new version and checksum of new image(which is transferred)
            in target device */         
         if( OK == Boot_StoreNewImageVerAndChecksum(NEW_VERSION, GulChecksum, PRIMARY_IMAGE_ADDR))
         {
           /* after transferring image successfully, restart target device */
           Boot_RestartTargetDevice();
           /* now go in to the idle state and deconfig all the peripherals not
              needed now */
           GeCurrentState = IDLE_STATE;           
         }
         else
         {
           /* problem occured in communicating with target device so start whole process again */
           GeCurrentState = INITIAL_STATE;
         }
         break;
         
         case IDLE_STATE :
         Boot_CommProtocolDeconfig();
         Boot_IndicateFinishStatus();
         while(1)
         {
           /* Do nothing */
         }
         
         default :
         /* for MISRA warning removal */
         break;
         
     }
     
 }
 

}

/* 
This function is used to reset target device 
*/
t_eStatus Boot_RestartTargetDevice(void)
{
  t_sCmdStruct LsCommand;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  
  LsCommand.Command = TARGET_RESET;
  LsCommand.Type = 0;
  LsCommand.Value1 = 0;
  LsCommand.Value2 = 0;

  if(OK == Boot_SendCommand(LsCommand))
  {
  
  }
  else
  {
    LeStatus = NOT_OK;
  }
  
  return LeStatus;
}
/* sends hello command */
t_eStatus Boot_SendHelloCommand(void)
{
  t_eStatus LeStatus;
  t_sCmdStruct LsCommand;
  
  LeStatus = NOT_OK;
  
  LsCommand.Command = HELLO;
  LsCommand.Type = 0;
  LsCommand.Value1 = 0;
  LsCommand.Value2 = 0;
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }
  
  return LeStatus;
}

/* Get version of fw present in partition */
uint_32 Boot_GetImageVersion(uint_8 partition, t_eStatus * status)
{
  t_sCmdStruct LsCommand;
  t_sCmdStruct LsRetCmd;
  uint_32 LulRetVer;

  LsCommand.Command = GET_VERSION;
  LsCommand.Type = 0;
  LsCommand.Value1 = partition;
  LsCommand.Value2 = 0;
  
  *status = OK;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    *status = NOT_OK;
  }

  if(OK == Boot_ReceiveCommand(GET_CHECKSUM_FLASH, &LsRetCmd)) 
  {
    LulRetVer = LsRetCmd.Value1;
  }
  else
  {
    *status = NOT_OK;
  }

  return LulRetVer;
}
/* check if two versions are equal or not */
t_eStatus Boot_IsVersionEqual(uint_32 version1, uint_32 version2)
{
  t_eStatus LeStatus;
  
  LeStatus = NOT_OK;
  
  if(version1 == version2)
  {
    LeStatus = OK;
  }
  
  return LeStatus;
  
}
/* sends command to erase primary image present in target device */
t_eStatus Boot_ErasePrimaryImageInTarget(uint_32 imageadrr, uint_32 length)
{
  t_sCmdStruct LsCommand;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  LsCommand.Command = ERASE_ALL;
  LsCommand.Type = 0;
  LsCommand.Value1 = imageadrr;
  LsCommand.Value2 = length;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }

  return LeStatus;  
}

/* write primary image at target with new image present at host,
   It transmit bytes page by page(256 Byte) from new image buffer and check 
   for checksum also and once it is finished then it gives command to write 
   that page in flash memrory at target*/
t_eStatus Boot_WritePrimaryImage( uint_8 * bufferaddr, uint_32 imagesize)
{
  uint_32 LulPageToProgram;
  uint_32 LulPageIdx;
  uint_32 LulImageBuffStartIndextoTx;
  t_eStatus LeStatus;
  uint_32 LulTxPageCheckSum;
  uint_32 LulSizeInBytes;
  uint_32 LulRetryCount;
  t_eResult LeBuffTxRetry;
  t_eResult LeWriteFlashStatus;
  uint_32 LulRemainigBytesToTx;
  
  LeStatus = OK;
  
  /* calculating number of pages(minimum number of bytes we can write to flash 
     at one time), we need, to transfer our full image,here page size
     is 256 Bytes. Suppose we have imagesize of 520 Bytes then 
     total number of pages we need to transfer will be 3, two pages of 
     256 Bytes and one of 8 Bytes     */
  LulPageToProgram = (imagesize >> 8) + ((imagesize & 0x000000FF)? 1:0);
  
  for(LulPageIdx = 0; LulPageIdx < LulPageToProgram; LulPageIdx++)
  {
    /* calculating index from where we will start transfer 256 bytes page from
       image buffer located in ROM */
    LulImageBuffStartIndextoTx = (LulPageIdx*FLASH_WRITE_BLOCK_SIZE);
    /* total bytes remained for transfer */
    LulRemainigBytesToTx =  imagesize - LulImageBuffStartIndextoTx;
    if(LulRemainigBytesToTx >= FLASH_WRITE_BLOCK_SIZE)
    {
      /* we will tx full page(256B) this time */
      LulSizeInBytes = FLASH_WRITE_BLOCK_SIZE;
    }
    else
    {
      /* bytes to tx are less than a page */
      LulSizeInBytes = LulRemainigBytesToTx;
    }     
    /* Send Command each time to initialise target buffer of 256B */
    if( OK == Boot_InitialiseBuffInTarget())
    {
      /* transmit next page from new image buffer and at the same time we are 
       calculating checksum of bytes we have transferred in target buffer */
      LulTxPageCheckSum = Boot_TxPageFromImageBuff(LulImageBuffStartIndextoTx, LulSizeInBytes, &LeStatus);
      if(LeStatus == OK)
      {
        /* check if transmitted bytes checksum is same as checksum of target buffer */
        if( OK == Boot_ValidateChecksumInTargetBuff(LulTxPageCheckSum) )
        {
   
        }
        else
        {
          /* buffer in target is not updated successfully so will re transfer same 
           page again */
           LulRetryCount = RETRY_COUNT_FOR_BUFF_TX;
           LeBuffTxRetry = FAIL;
           do{
             /* Send Command to initialise target buffer of 256 B */
             (void)Boot_InitialiseBuffInTarget();
             LulTxPageCheckSum = Boot_TxPageFromImageBuff(LulImageBuffStartIndextoTx, LulSizeInBytes, &LeStatus);
             if( (OK == LeStatus) && (OK == Boot_ValidateChecksumInTargetBuff(LulTxPageCheckSum)) )
             {
               LeBuffTxRetry = PASS;
               break;
             }
             else
             {
             
             }
           }while(LulRetryCount--);
         
           if(LeBuffTxRetry == PASS)
           {
          
           }
           else
           {
             /* If buffer in target is not being updated successfully even after
                retries then we will restart boot process again */
             GeCurrentState = INITIAL_STATE;
           }
        }        
      }
    
    }
    {
      LeStatus = NOT_OK;
    }
    
    if((GeCurrentState != INITIAL_STATE)&& (LeStatus == OK))
    {
      /* buffer in target is updated successfully so we can give command to 
         target to update flash area with data present in target buffer */
         Boot_WriteBuffToFlashInTarget(PRIMARY_IMAGE_ADDR + LulImageBuffStartIndextoTx);

      /* As flash write will take defined time so we will wait here and ask if 
         flashing is finished? If not, we will wait for sometime and again ask
         if flashing is finished? If not again, we will repeat this process for 
         defined number of times. At the end if it still not finished we will 
         start whole process again from begining */
         LulRetryCount = RETRY_COUNT_WRITE_FLASH;
         LeWriteFlashStatus = FAIL;
         do{
           if( Boot_CheckFlashWriteStatus() == OK)
           {
             LeWriteFlashStatus = PASS;
             break;
           }
           else
           {
             Boot_Delay(WAIT_TO_SEND_QUERY);
           }
         
         }while(LulRetryCount--);

         if(LeWriteFlashStatus == PASS)
         {
           /* flash is written successfully at target */
         }
         else
         {
           /* Timout occured for flash write process in target device */
           /* Slave is busy in writing for more than the required time so 
              we will reset target device by giving signal on reset pin as 
              it is not responding to commands */
           Boot_GiveSignalOnResetPin();
           GeCurrentState = INITIAL_STATE;
         }            
    }
  }
  
  if(GeCurrentState == INITIAL_STATE)
  {
    LeStatus = NOT_OK;
  }
  return LeStatus;
}

/*
startindex : from where we will start transferring data from new image buffer.
size       : size in bytes we will transfer.             
*/
uint_32 Boot_TxPageFromImageBuff(uint_32 startindex, uint_32 size, t_eStatus * status)
{
  uint_32 LulBuffIdx;
  uint_32 LulPageCheckSum;
  uint_32 LulByteIdx;
  
  LulPageCheckSum = 0;
  LulByteIdx = 0;
  *status = OK;
  
  for(LulBuffIdx = startindex; LulBuffIdx < (startindex + size); LulBuffIdx++)
  {
    if(OK == Boot_WriteToBufferInTarget(GaImageBuffer[LulBuffIdx], LulByteIdx ))
    {
      /* checksum is calculated by adding all bytes and keep lower 32 bits
       from result */
      LulPageCheckSum = LulPageCheckSum + (uint_32)GaImageBuffer[LulBuffIdx];
      LulByteIdx++; 
    }
    else
    {
      *status = NOT_OK;
      break;
    }
   
  }
  
  return LulPageCheckSum;
}

/* This sends command to write target buffer to flash area.
   A counter can be maintained at target side to keep track which flash block 
   is need to be written */
t_eStatus Boot_WriteBuffToFlashInTarget(uint_32 flashaddr)
{
  t_sCmdStruct LsCommand;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  LsCommand.Command = WRITE_FLASH_BLOCK;
  LsCommand.Type = 0;
  LsCommand.Value1 = flashaddr;
  LsCommand.Value2 = FLASH_WRITE_BLOCK_SIZE;

  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }
  
  return LeStatus;  
}

/* validate if target device buffer contains correct values after transferring
   by comparing checksum at both side */
t_eStatus Boot_ValidateChecksumInTargetBuff(uint_32 checksum)
{
  t_sCmdStruct LsCommand;
  t_sCmdStruct LsRetCmd;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  LsCommand.Command = GET_CHECKSUM_BUFF;
  LsCommand.Type = 0;
  LsCommand.Value1 = 0;
  LsCommand.Value2 = 0;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }
  if(OK == Boot_ReceiveCommand(GET_CHECKSUM_BUFF, &LsRetCmd))
  {
    if( LsRetCmd.Value1 == checksum )
    {

    }
    else
    {
      LeStatus = NOT_OK;
    }   
  }
  else
  {
    LeStatus = NOT_OK;
  }   
 
  return LeStatus;
    
}
/* Initialise target buffer each time before writing data to it */
t_eStatus Boot_InitialiseBuffInTarget(void)
{
  t_sCmdStruct LsCommand;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  
  LsCommand.Command = INIT_BUFF_IN_TARGET;
  LsCommand.Type = 0;
  LsCommand.Value1 = 0;
  LsCommand.Value2 = 0;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }
  return LeStatus;  
}

/* verify checksum of data present in target flash with new image buffer checksum
   at host side */
t_eStatus Boot_VerifyCheckSumTargetFlash(uint_32 checksum)
{
  t_sCmdStruct LsCommand;
  t_sCmdStruct LsRetCmd;
  t_eStatus LeStatus;
  
  LeStatus = OK;    
  LsCommand.Command = GET_CHECKSUM_FLASH;
  LsCommand.Type = 0;
  LsCommand.Value1 = PRIMARY_IMAGE_ADDR;
  LsCommand.Value2 = IMAGE_SIZE;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }
  if(OK == Boot_ReceiveCommand(GET_CHECKSUM_FLASH, &LsRetCmd))
  {
    if( LsRetCmd.Value1 == checksum )
    {

    }
    else
    {
      LeStatus = NOT_OK;
    }     
  }
  else
  {
    LeStatus = NOT_OK;
  }

  return LeStatus;    
}

/* store new version and checksum info in target device */
t_eStatus Boot_StoreNewImageVerAndChecksum(uint_32 version, uint_32 checksum, uint_32 address)
{
  t_sCmdStruct LsCommand;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  LsCommand.Command = STORE_VERSION;
  LsCommand.Type = 0;
  LsCommand.Value1 = version;
  LsCommand.Value2 = address;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }

  LsCommand.Command = STORE_CHECKSUM_FLASH;
  LsCommand.Type = 0;
  LsCommand.Value1 = checksum;
  LsCommand.Value2 = address;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
  
  }
  else
  {
    LeStatus = NOT_OK;
  }

  return LeStatus;
}

/* send command to target that no image transfer is needed and it can jump to 
   appl */
t_eStatus Boot_SendResumeCommadToTarget(void)
{
  t_sCmdStruct LsCommand;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  LsCommand.Command = RESUME_TARGET;
  LsCommand.Type = 0;
  LsCommand.Value1 = 0;
  LsCommand.Value2 = 0;
  
if(OK == Boot_SendCommand(LsCommand))
{
 
}
else
{
  LeStatus = NOT_OK;
}

return LeStatus; 
}

/* This function sends command using selected comm protocol in boot cfg file */
t_eStatus Boot_SendCommand(t_sCmdStruct command)
{
  t_eStatus LeStatus;
  t_eStatus LeCommandSent;
  uint_32 LulCommandRetry;
 
  LeStatus = OK;
  
#if (PROTOCOL_USED  == SPI)  
  if( OK == SPI_SendCommand(command) )
#elif(PROTOCOL_USED  == I2C)
  if( OK == I2C_SendCommand(command) )
#elif(PROTOCOL_USED  == UART)
  if(OK == UART_SendCommand(command))
#else
#error Not a valid protocol selected.
#endif 
  {
    /* Command is acknowledged by target */
  }
  else
  {
    LulCommandRetry = RETRY_COUNT_COMMAND;
    LeCommandSent = NOT_OK;
    do{
#if (PROTOCOL_USED  == SPI)  
  if( OK == SPI_SendCommand(command) )
#elif(PROTOCOL_USED  == I2C)
  if( OK == I2C_SendCommand(command) )
#elif(PROTOCOL_USED  == UART)
  if(OK == UART_SendCommand(command))
#else
#error Not a valid protocol selected.
#endif 
      {
        LeCommandSent = OK;
        break;
      }
      else
      {
        /* giving some delay in between command retries */
        Boot_Delay(RESOLUTION_CONV_US_TO_CLK_COUNT(COMMAND_RETRY_INTERVAL));
      }
      
    }while(LulCommandRetry--);
    if(LeCommandSent == NOT_OK)
    {
      LeStatus = NOT_OK;
    }
  }
  
  return LeStatus;
}

/* this functions receives particular command from target device */
t_eStatus Boot_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * pcommand)
{
  t_eStatus LeStatus;
  
  LeStatus = OK;
  
#if (PROTOCOL_USED  == SPI)  
  if( OK == SPI_ReceiveCommand(commandcode, pcommand))
#elif(PROTOCOL_USED  == I2C)
  if( OK == I2C_ReceiveCommand(commandcode, pcommand))
#elif(PROTOCOL_USED  == UART)
  if( OK == UART_ReceiveCommand(commandcode, pcommand))
#else
#error Not a valid protocol selected.
#endif
  {
    
  }
  else
  {
    LeStatus = NOT_OK;
  }
  return LeStatus;
}

/* function for giving delay */
void Boot_Delay(uint_32 delay)
{
  uint_32 LulTick1;
  uint_32 LulTick2;
  uint_32 LulTimeDiff;
  
  LulTick1 = Boot_ReadCurrentTick();
  do{
    LulTick2 = Boot_ReadCurrentTick();
    if(LulTick2 >= LulTick1)
    {
      LulTimeDiff = LulTick2 - LulTick1;
    }
    else
    {
      /* if timer overflow occured once */
      LulTimeDiff = (MAX_TIMER_COUNT - LulTick1) + LulTick2;
    }
  }while(LulTimeDiff < RESOLUTION_CONV_MS_TO_CLK_COUNT(delay));
}

/* Initialise global variables if any */
void Boot_InitVar(void)
{
  
}

/* Brief --> This function calculate checksum.
   StartAddress --> address from where checksum calculation will start.
   length --> Total length in bytes for which checksum will be calculated
   
   Checksum is calculated such that we add-up all byte values and use
   the lower 32bit of the result as checksum.
*/
uint_32 Boot_CalculateChecksum(uint_8 * StartAddress, uint_32 length)
{
 uint_32 LulRetChecksum;
 uint_8 * LulAddr;
 
 LulRetChecksum = 0;
 
 for(LulAddr = StartAddress; LulAddr < (StartAddress + length); LulAddr++)
 {
   LulRetChecksum = LulRetChecksum +  (uint_32)(*LulAddr); 
 }
 
 return LulRetChecksum;
}


void Boot_ConfigPortPin(void)
{
  
}

/* It will return value in hardware counter register, just for removing 
   warning I am returning 0 */
uint_32 Boot_ReadCurrentTick(void)
{
  
  return 0;
}

/* send flash write status get command and return the write flash status at 
   target */
t_eStatus Boot_CheckFlashWriteStatus(void)
{
  t_sCmdStruct LsCommand;
  t_sCmdStruct LsRetCmd;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  
  LsCommand.Command = WRITE_STATUS_CHECK;
  LsCommand.Type = 0;
  LsCommand.Value1 = 0;
  LsCommand.Value2 = 0;
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }   

  if(OK == Boot_ReceiveCommand(WRITE_STATUS_CHECK, &LsRetCmd))
  {
    if(LsRetCmd.Value1 == WRITE_FINISH)
    {

    }
    else
    {
      LeStatus = NOT_OK;
    }     
  }
  else
  {
    LeStatus = NOT_OK;;
  }
  
  return LeStatus;
}

/* sends command to write buffer in target with given data at targetbuffindex
   index in target buffer */
t_eStatus Boot_WriteToBufferInTarget(uint_8 data, uint_32 targetbuffindex)
{
  t_sCmdStruct LsCommand;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  LsCommand.Command = WRITE_BUFFER;
  LsCommand.Type = 0;
  LsCommand.Value1 = targetbuffindex;
  LsCommand.Value2 = data;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }    
    
  return LeStatus;
}

/* for config of protocol */
void Boot_CommProtocolConfig(void)
{
#if (PROTOCOL_USED  == SPI)  
  SPI_Config();
#elif(PROTOCOL_USED  == I2C)
  I2C_Config();
#elif(PROTOCOL_USED  == UART)
  UART_Config();
#else
#error Not a valid protocol selected.
#endif 
}

/* sends command to check flash erase status */
t_eStatus Boot_CheckFlashEraseStatus(void)
{
  t_sCmdStruct LsCommand;
  t_sCmdStruct LsRetCmd;
  t_eStatus LeStatus;
  
  LeStatus = OK;
  
  LsCommand.Command = ERASE_STATUS_CHECK;
  LsCommand.Type = 0;
  LsCommand.Value1 = 0;
  LsCommand.Value2 = 0;
  
  if(OK == Boot_SendCommand(LsCommand))
  {
 
  }
  else
  {
    LeStatus = NOT_OK;
  }  
  
  if(OK == Boot_ReceiveCommand(ERASE_STATUS_CHECK, &LsRetCmd))
  {
    if(LsRetCmd.Value1 == ERASE_FINISH)
    {

    }
    else
    {
      LeStatus = NOT_OK;
    }     
  }
  else
  {
    LeStatus = NOT_OK;
  } 
  return LeStatus;
}

void Boot_IndicateFinishStatus(void)
{
  /* we can write code to turn on led on some pin here to indicate if process is
     finished */
}

void Boot_GiveSignalOnResetPin(void)
{
 /* write code to give signal on reset pin in target device here */
}

/* Deconfig peripheral */
void Boot_CommProtocolDeconfig(void)
{
#if (PROTOCOL_USED  == SPI)  
  SPI_Deconfig();
#elif(PROTOCOL_USED  == I2C)
  I2C_Deconfig();
#elif(PROTOCOL_USED  == UART)
  UART_Deconfig();
#else
#error Not a valid protocol selected.
#endif 
}

