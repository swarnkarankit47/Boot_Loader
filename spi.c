
/* function definition is present here  */

#include "spi.h"
#include "types.h"

void SPI_Config(void)
{
  /* Configuration for spi is done */
}

void SPI_Deconfig(void)
{
  /* deconfig spi peripheral */
}

/* This function sends command data over SPI and waits for acknowledge frame 
   also, if all goes fine then returns status as OK */
t_eStatus SPI_SendCommand(t_sCmdStruct commandstruct)
{
  /* Command will be send over SPI */
  return OK;
}

t_eStatus SPI_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * commandframe)
{
  /* Command with commandcode will be received here */    
  return OK;
}


