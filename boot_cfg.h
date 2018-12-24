
/* Configuration related defines are present here which are used in boot file */

#ifndef BOOT_CFG_H
#define BOOT_CFG_H

#define PARTITION_ONE     1U
#define PARTITION_TWO     2U

#define SPI               1U
#define UART              2U
#define I2C               3U 

#define PROTOCOL_USED     I2C

#define NEW_VERSION       0x10000001 
#define IMAGE_SIZE        1024U /* In bytes */

/* Protocol Commands opcode */
#define HELLO                242U
#define TARGET_RESET         135U
#define GET_VERSION          136U
#define STORE_VERSION        137U
#define RESUME_TARGET        138U
#define INIT_BUFF_IN_TARGET  139U 
#define ERASE_STATUS_CHECK   140U
#define WRITE_STATUS_CHECK   141U
#define ERASE_ALL            200U
#define WRITE_BUFFER         201U
#define WRITE_FLASH_BLOCK    202U
#define GET_CHECKSUM_BUFF    203U
#define GET_CHECKSUM_FLASH   204U
#define STORE_CHECKSUM_FLASH 205U

/* Minumum number of bytes we can write to flash in one time */
#define FLASH_WRITE_BLOCK_SIZE   256U
#define TARGET_BUFF_SIZE         FLASH_WRITE_BLOCK_SIZE   

/* converting seconds to clock count, for example we have 1MHz clock source 
   input to timer module so it's resolution will be 1us/lsb */
#define RESOLUTION_CONV_S_TO_CLK_COUNT(x)  (x*1000000)/* seconds to clock count */
#define RESOLUTION_CONV_MS_TO_CLK_COUNT(x) (x*1000) /* milli seconds to clock count */
#define RESOLUTION_CONV_US_TO_CLK_COUNT(x) (x) /* micro seconds to clock count */

/* wait times */
#define WAIT_TIME            10U   /* In seconds */
#define FLASH_ERASE_WAIT     500U  /* In ms */
#define WAIT_TO_SEND_QUERY   10U   /* In ms */
#define SETUP_DELAY_FOR_TARGET  10U /* In ms */


/* number of retries in case no reponse is received from target device */
#define RETRY_COUNT_FOR_QUERY    5U
#define RETRY_COUNT_FOR_BUFF_TX  5U
#define RETRY_COUNT_WRITE_FLASH  5U
#define RETRY_COUNT_ERASE_FLASH  5U
#define RETRY_COUNT_COMMAND      5U

#define HELLO_QUERY_INTERVAL     10U /* in milli second */
#define HELLO_QUERY_NUM         500U /* 500 queries in 10 milli second interval */
#define COMMAND_RETRY_INTERVAL  200U /* in micro second */

/* Image address */
#define PRIMARY_IMAGE_ADDR       0x12345678

/* Max timer count */
#define MAX_TIMER_COUNT          0xFFFFFFFF

/* flash status */
#define WRITE_FINISH    1U
#define ERASE_FINISH    1U 

#endif /* BOOT_CFG_H */