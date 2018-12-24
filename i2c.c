
/* function definition is present for I2C protocol */

#include "i2c.h"
#include "types.h"

void I2C_Config(void)
{
  /* Configuration for I2C is done */
}

void I2C_Deconfig(void)
{
  /* deconfig I2C peripheral */
}

/* This function sends command data over I2C and waits for acknowledge frame 
   also, if all goes fine then returns status as OK */
t_eStatus I2C_SendCommand(t_sCmdStruct commandstruct)
{
  /* Command will be send over I2C */
  return OK;
}

t_eStatus I2C_ReceiveCommand(uint_8 commandcode, t_sCmdStruct * commandframe)
{
  /* Command with commandcode will be received here */  
  return OK;
}


