
/* function declaration is done here */

#ifndef SPI_H
#define SPI_H

#include "types.h"


void SPI_Config(void);
void SPI_Deconfig(void);
t_eStatus SPI_SendCommand(t_sCmdStruct commandstruct);
t_eStatus SPI_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * commandframe);

#endif /* SPI_H */