
/* function definition is present here */

#include "Uart.h"


void UART_Config(void)
{
  /* Configuration for UART is done */
}

void UART_Deconfig(void)
{
  /* deconfig UART peripheral */
}

/* This function sends command data over UART and waits for acknowledge frame 
   also, if all goes fine then returns status as OK */
t_eStatus UART_SendCommand(t_sCmdStruct commandstruct)
{
  /* Command will be send over UART */
  return OK;
}

t_eStatus UART_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * commandframe)
{
  /* Command with commandcode will be received here */
  /* I am receiving command in blocking mode */
  return OK;
}


