
/* function declaration is done here */

#ifndef I2C_H
#define I2C_H

#include "types.h"


void I2C_Config(void);
void I2C_Deconfig(void);
t_eStatus I2C_SendCommand(t_sCmdStruct commandstruct);
t_eStatus I2C_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * commandframe);

#endif /* I2C_H */