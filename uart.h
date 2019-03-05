
/* function declaration is done here */

#ifndef UART_H
#define UART_H

#include "types.h"

void NewFunc(void);
void UART_Config(void);
void UART_Deconfig(void);
t_eStatus UART_SendCommand(t_sCmdStruct commandstruct);
t_eStatus UART_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * commandframe);

#endif /* UART_H */
