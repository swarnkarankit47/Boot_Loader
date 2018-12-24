
/* types used through project is defined here */

#ifndef TYPES_H
#define TYPES_H

typedef unsigned char uint_8;
typedef unsigned short uint_16;
typedef unsigned long uint_32;

typedef enum
{
  OK,
  NOT_OK,
} t_eStatus;

typedef enum
{
    FAIL,
    PASS
} t_eResult;

typedef enum
{
    IDLE_STATE,
    INITIAL_STATE,
    QUERY_COMPARE_STATE_INIT,
    QUERY_COMPARE_STATE_SEND_QUERY,
    ERASE_STATE,
    DOWNLOADING_STATE,
    VERIFY_CHECKSUM_STATE,
    FINAL_STATE
} t_eStates;



/* struct for download protocol frame format */
typedef struct Cmd{
 uint_8 Command;
 uint_8 Type;
 uint_32 Value1;
 uint_32 Value2;
} t_sCmdStruct;


#endif /* TYPES_H */