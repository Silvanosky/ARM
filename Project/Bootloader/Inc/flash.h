#ifndef FLASH_H_
#define FLASH_H_

#include "stm32f4xx_hal.h"
#include <stddef.h>

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */

//The signature for the program is 1KByte
#define FLASH_SIGN_SIZE ((uint32_t)0x00000400u)
// A program has an allowed size of 32KByte
#define FLASH_PRG_SIZE ((uint32_t)0x00008000u - FLASH_SIGN_SIZE)
#define FLASH_SECTOR_SIZE ((uint32_t)0x00008000u)
/*
--------------
|   |- Bootloader 16KByte
|16 |
|   |
|
|   |- Signature 1KByte
|32 |- App
|   |
|
|   |- Signature 1KByte
|32 |- App2 (cache)
|   |
|
|- Data ALL
|
|
|
|
--------------
*/



/* Status report for the functions. */
typedef enum {
  FLASH_OK              = 0x00u, /**< The action was successful. */
  FLASH_ERROR_SIZE      = 0x01u, /**< The binary is too big. */
  FLASH_ERROR_WRITE     = 0x02u, /**< Writing failed. */
  FLASH_ERROR_READBACK  = 0x04u, /**< Writing was successful, but the content of the memory is wrong. */
  FLASH_ERROR           = 0xFFu  /**< Generic error. */
} flash_status;

typedef struct {
	size_t sign_size;
	size_t app_size;
	__IO uint8_t signature[1024];
	__IO uint8_t app[1];
} app_t;

/* Start and end addresses of the user application. */
#define FLASH_BTL_START_ADDRESS (ADDR_FLASH_SECTOR_0)

#define FLASH_APP (ADDR_FLASH_SECTOR_1)
#define FLASH_APP_SIGN (FLASH_APP + sizeof(app_t))
#define FLASH_APP_START_ADDRESS (FLASH_APP_SIGN + FLASH_SIGN_SIZE)

#define FLASH_APP2 (ADDR_FLASH_SECTOR_4)
#define FLASH_APP2_SIGN (FLASH_APP2 + sizeof(app_t))
//#define FLASH_APP2_START_ADDRESS (FLASH_APP2)

#define FLASH_BTL_END_ADDRESS (ADDR_FLASH_SECTOR_1 - 0x1u)
#define FLASH_APP_END_ADDRESS (ADDR_FLASH_SECTOR_3 - 0x1u)
#define FLASH_APP2_END_ADDRESS (ADDR_FLASH_SECTOR_5 - 0x1u)

flash_status flash_erase(uint32_t address, uint32_t NbOfSectors);
flash_status flash_write(uint32_t address, uint32_t *data, uint32_t length);

void flash_jump_to_app(__IO app_t* app);
#endif /* FLASH_H_ */
