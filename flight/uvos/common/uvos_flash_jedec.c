#include "uvos.h"

#ifdef UVOS_INCLUDE_FLASH

#include "uvos_flash_jedec_priv.h"
#include "uvos_flash_jedec_catalog.h"

#define JEDEC_WRITE_ENABLE        0x06
#define JEDEC_WRITE_DISABLE       0x04
#define JEDEC_READ_STATUS         0x05
#define JEDEC_WRITE_STATUS        0x01
#define JEDEC_READ_DATA           0x03
#define JEDEC_FAST_READ           0x0b
#define JEDEC_DEVICE_ID           0x9F
#define JEDEC_PAGE_WRITE          0x02

#define JEDEC_STATUS_BUSY         0x01
#define JEDEC_STATUS_WRITEPROTECT 0x02
#define JEDEC_STATUS_BP0          0x04
#define JEDEC_STATUS_BP1          0x08
#define JEDEC_STATUS_BP2          0x10
#define JEDEC_STATUS_TP           0x20
#define JEDEC_STATUS_SEC          0x40
#define JEDEC_STATUS_SRP0         0x80

enum uvos_jedec_dev_magic {
  UVOS_JEDEC_DEV_MAGIC = 0xcb55aa55,
};

// ! Device handle structure
struct jedec_flash_dev {
  uint32_t spi_id;
  uint32_t slave_num;
  bool     claimed;

  uint8_t  manufacturer;
  uint8_t  memorytype;
  uint8_t  capacity;

  const struct uvos_flash_jedec_cfg * cfg;
#if defined(FLASH_FREERTOS)
  xSemaphoreHandle transaction_lock;
#endif
  enum uvos_jedec_dev_magic magic;
};

#define FLASH_FAST_PRESCALER UVOS_SPI_PRESCALER_64
#define FLASH_PRESCALER      UVOS_SPI_PRESCALER_64

// ! Private functions
static int32_t UVOS_Flash_Jedec_Validate( struct jedec_flash_dev * flash_dev );
static struct jedec_flash_dev * UVOS_Flash_Jedec_alloc( void );

static int32_t UVOS_Flash_Jedec_ReadID( struct jedec_flash_dev * flash_dev );
static int32_t UVOS_Flash_Jedec_ReadStatus( struct jedec_flash_dev * flash_dev );
static int32_t UVOS_Flash_Jedec_ClaimBus( struct jedec_flash_dev * flash_dev, bool fast );
static int32_t UVOS_Flash_Jedec_ReleaseBus( struct jedec_flash_dev * flash_dev );
static int32_t UVOS_Flash_Jedec_WriteEnable( struct jedec_flash_dev * flash_dev );
static int32_t UVOS_Flash_Jedec_Busy( struct jedec_flash_dev * flash_dev );

/**
 * @brief Allocate a new device
 */
static struct jedec_flash_dev * UVOS_Flash_Jedec_alloc( void )
{
  struct jedec_flash_dev * flash_dev;

  flash_dev = ( struct jedec_flash_dev * ) UVOS_malloc( sizeof( *flash_dev ) );
  if ( !flash_dev ) {
    return NULL;
  }

  flash_dev->claimed = false;
  flash_dev->magic   = UVOS_JEDEC_DEV_MAGIC;
#if defined(FLASH_FREERTOS)
  flash_dev->transaction_lock = xSemaphoreCreateMutex();
#endif
  return flash_dev;
}

/**
 * @brief Validate the handle to the spi device
 */
static int32_t UVOS_Flash_Jedec_Validate( struct jedec_flash_dev * flash_dev )
{
  if ( flash_dev == NULL ) {
    return -1;
  }
  if ( flash_dev->magic != UVOS_JEDEC_DEV_MAGIC ) {
    return -2;
  }
  if ( flash_dev->spi_id == 0 ) {
    return -3;
  }
  return 0;
}

/**
 * @brief Initialize the flash device and enable write access
 */
int32_t UVOS_Flash_Jedec_Init( uintptr_t * flash_id, uint32_t spi_id, uint32_t slave_num )
{
  struct jedec_flash_dev * flash_dev = UVOS_Flash_Jedec_alloc();

  if ( !flash_dev ) {
    return -1;
  }

  flash_dev->spi_id    = spi_id;
  flash_dev->slave_num = slave_num;
  flash_dev->cfg = NULL;

  ( void )UVOS_Flash_Jedec_ReadID( flash_dev );

  for ( uint32_t i = 0; i < uvos_flash_jedec_catalog_size; ++i ) {
    const struct uvos_flash_jedec_cfg flash_jedec_entry = uvos_flash_jedec_catalog[i];

    if ( ( flash_dev->manufacturer == flash_jedec_entry.expect_manufacturer )
         && ( flash_dev->memorytype == flash_jedec_entry.expect_memorytype )
         && ( flash_dev->capacity == flash_jedec_entry.expect_capacity ) ) {
      flash_dev->cfg = &uvos_flash_jedec_catalog[i];
      break;
    }
  }

  if ( !flash_dev->cfg ) {
    return -1;
  }

  /* Give back a handle to this flash device */
  *flash_id = ( uintptr_t )flash_dev;

  return 0;
}


/**
 * @brief Claim the SPI bus for flash use and assert CS pin
 * @return 0 for sucess, -1 for failure to get semaphore
 */
static int32_t UVOS_Flash_Jedec_ClaimBus( struct jedec_flash_dev * flash_dev, bool fast )
{
  if ( UVOS_SPI_ClaimBus( flash_dev->spi_id ) < 0 ) {
    return -1;
  }
  UVOS_SPI_SetClockSpeed( flash_dev->spi_id, fast ? FLASH_FAST_PRESCALER : FLASH_PRESCALER );
  UVOS_SPI_RC_PinSet( flash_dev->spi_id, flash_dev->slave_num, 0 );
  flash_dev->claimed = true;

  return 0;
}

/**
 * @brief Release the SPI bus sempahore and ensure flash chip not using bus
 */
static int32_t UVOS_Flash_Jedec_ReleaseBus( struct jedec_flash_dev * flash_dev )
{
  UVOS_SPI_RC_PinSet( flash_dev->spi_id, flash_dev->slave_num, 1 );
  UVOS_SPI_ReleaseBus( flash_dev->spi_id );
  flash_dev->claimed = false;
  return 0;
}

/**
 * @brief Returns if the flash chip is busy
 * @returns -1 for failure, 0 for not busy, 1 for busy
 */
static int32_t UVOS_Flash_Jedec_Busy( struct jedec_flash_dev * flash_dev )
{
  int32_t status = UVOS_Flash_Jedec_ReadStatus( flash_dev );

  if ( status < 0 ) {
    return -1;
  }
  return status & JEDEC_STATUS_BUSY;
}

/**
 * @brief Execute the write enable instruction and returns the status
 * @returns 0 if successful, -1 if unable to claim bus
 */
static int32_t UVOS_Flash_Jedec_WriteEnable( struct jedec_flash_dev * flash_dev )
{
  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) != 0 ) {
    return -1;
  }

  uint8_t out[] = { JEDEC_WRITE_ENABLE };
  UVOS_SPI_TransferBlock( flash_dev->spi_id, out, NULL, sizeof( out ), NULL );
  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  return 0;
}


/**
 * @brief Read the status register from flash chip and return it
 */
static int32_t UVOS_Flash_Jedec_ReadStatus( struct jedec_flash_dev * flash_dev )
{
  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) < 0 ) {
    return -1;
  }

  uint8_t out[2] = { JEDEC_READ_STATUS, 0 };
  uint8_t in[2]  = { 0, 0 };
  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, in, sizeof( out ), NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -2;
  }

  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  return in[1];
}

/**
 * @brief Read the status register from flash chip and return it
 */
static int32_t UVOS_Flash_Jedec_ReadID( struct jedec_flash_dev * flash_dev )
{
  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) < 0 ) {
    return -2;
  }

  uint8_t out[] = { JEDEC_DEVICE_ID, 0, 0, 0 };
  uint8_t in[4];
  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, in, sizeof( out ), NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -3;
  }

  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  flash_dev->manufacturer = in[1];
  flash_dev->memorytype   = in[2];
  flash_dev->capacity     = in[3];

  return flash_dev->manufacturer;
}

/**********************************
 *
 * Provide a UVOS flash driver API
 *
 *********************************/
#include "uvos_flash.h"

#if FLASH_USE_FREERTOS_LOCKS

/**
 * @brief Grab the semaphore to perform a transaction
 * @return 0 for success, -1 for timeout
 */
int32_t UVOS_Flash_Jedec_StartTransaction( uintptr_t flash_id )
{
  struct jedec_flash_dev * flash_dev = ( struct jedec_flash_dev * )flash_id;

  if ( UVOS_Flash_Jedec_Validate( flash_dev ) != 0 ) {
    return -1;
  }

#if defined(UVOS_INCLUDE_FREERTOS)
  if ( xSemaphoreTake( flash_dev->transaction_lock, portMAX_DELAY ) != pdTRUE ) {
    return -2;
  }
#endif

  return 0;
}

/**
 * @brief Release the semaphore to perform a transaction
 * @return 0 for success, -1 for timeout
 */
int32_t UVOS_Flash_Jedec_EndTransaction( uintptr_t flash_id )
{
  struct jedec_flash_dev * flash_dev = ( struct jedec_flash_dev * )flash_id;

  if ( UVOS_Flash_Jedec_Validate( flash_dev ) != 0 ) {
    return -1;
  }

#if defined(UVOS_INCLUDE_FREERTOS)
  if ( xSemaphoreGive( flash_dev->transaction_lock ) != pdTRUE ) {
    return -2;
  }
#endif

  return 0;
}

#else /* FLASH_USE_FREERTOS_LOCKS */

int32_t UVOS_Flash_Jedec_StartTransaction( __attribute__( ( unused ) ) uintptr_t flash_id )
{
  return 0;
}

int32_t UVOS_Flash_Jedec_EndTransaction( __attribute__( ( unused ) ) uintptr_t flash_id )
{
  return 0;
}

#endif /* FLASH_USE_FREERTOS_LOCKS */

/**
 * @brief Erase a sector on the flash chip
 * @param[in] add Address of flash to erase
 * @returns 0 if successful
 * @retval -1 if unable to claim bus
 * @retval
 */
int32_t UVOS_Flash_Jedec_EraseSector( uintptr_t flash_id, uint32_t addr )
{
  struct jedec_flash_dev * flash_dev = ( struct jedec_flash_dev * )flash_id;

  if ( UVOS_Flash_Jedec_Validate( flash_dev ) != 0 ) {
    return -1;
  }

  uint8_t ret;
  uint8_t out[] = { flash_dev->cfg->sector_erase, ( addr >> 16 ) & 0xff, ( addr >> 8 ) & 0xff, addr & 0xff };

  if ( ( ret = UVOS_Flash_Jedec_WriteEnable( flash_dev ) ) != 0 ) {
    return ret;
  }

  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) != 0 ) {
    return -1;
  }

  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, NULL, sizeof( out ), NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -2;
  }

  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  // Keep polling when bus is busy too
  while ( UVOS_Flash_Jedec_Busy( flash_dev ) != 0 ) {
#if defined(FLASH_FREERTOS)
    vTaskDelay( 2 );
#endif
#ifdef UVOS_INCLUDE_WDG
    UVOS_WDG_Clear();
#endif
  }

  return 0;
}

/**
 * @brief Execute the whole chip
 * @returns 0 if successful, -1 if unable to claim bus
 */
int32_t UVOS_Flash_Jedec_EraseChip( uintptr_t flash_id )
{
  struct jedec_flash_dev * flash_dev = ( struct jedec_flash_dev * )flash_id;

  if ( UVOS_Flash_Jedec_Validate( flash_dev ) != 0 ) {
    return -1;
  }

  uint8_t ret;
  uint8_t out[] = { flash_dev->cfg->chip_erase };

  if ( ( ret = UVOS_Flash_Jedec_WriteEnable( flash_dev ) ) != 0 ) {
    return ret;
  }

  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) != 0 ) {
    return -1;
  }

  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, NULL, sizeof( out ), NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -2;
  }

  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  // Keep polling when bus is busy too
  int i = 0;
  while ( UVOS_Flash_Jedec_Busy( flash_dev ) != 0 ) {
#if defined(FLASH_FREERTOS)
    vTaskDelay( 1 );
    if ( ( i++ ) % 100 == 0 ) {
#else
    if ( ( i++ ) % 10000 == 0 ) {
#endif

#ifdef UVOS_LED_HEARTBEAT
      UVOS_LED_Toggle( UVOS_LED_HEARTBEAT );
#endif
    }
  }

  return 0;
}

/**
 * @brief Write one page of data (up to 256 bytes) aligned to a page start
 * @param[in] addr Address in flash to write to
 * @param[in] data Pointer to data to write to flash
 * @param[in] len Length of data to write (max 256 bytes)
 * @return Zero if success or error code
 * @retval -1 Unable to claim SPI bus
 * @retval -2 Size exceeds 256 bytes
 * @retval -3 Length to write would wrap around page boundary
 */
int32_t UVOS_Flash_Jedec_WriteData( uintptr_t flash_id, uint32_t addr, uint8_t * data, uint16_t len )
{
  struct jedec_flash_dev * flash_dev = ( struct jedec_flash_dev * )flash_id;

  if ( UVOS_Flash_Jedec_Validate( flash_dev ) != 0 ) {
    return -1;
  }

  uint8_t ret;
  uint8_t out[4] = { JEDEC_PAGE_WRITE, ( addr >> 16 ) & 0xff, ( addr >> 8 ) & 0xff, addr & 0xff };

  /* Can only write one page at a time */
  if ( len > 0x100 ) {
    return -2;
  }

  /* Ensure number of bytes fits after starting address before end of page */
  if ( ( ( addr & 0xff ) + len ) > 0x100 ) {
    return -3;
  }
  if ( ( ret = UVOS_Flash_Jedec_WriteEnable( flash_dev ) ) != 0 ) {
    return ret;
  }

  /* Execute write page command and clock in address.  Keep CS asserted */
  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) != 0 ) {
    return -1;
  }
  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, NULL, sizeof( out ), NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -1;
  }

  /* Clock out data to flash */
  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, data, NULL, len, NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -1;
  }

  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  // Keep polling when bus is busy too
#if defined(FLASH_FREERTOS)
  while ( UVOS_Flash_Jedec_Busy( flash_dev ) != 0 ) {
    vTaskDelay( 1 );
  }
#else

  // Query status this way to prevent accel chip locking us out
  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) < 0 ) {
    return -1;
  }

  UVOS_SPI_TransferByte( flash_dev->spi_id, JEDEC_READ_STATUS );
  while ( UVOS_SPI_TransferByte( flash_dev->spi_id, JEDEC_READ_STATUS ) & JEDEC_STATUS_BUSY ) {
    ;
  }

  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

#endif
  return 0;
}

/**
 * @brief Write multiple chunks of data in one transaction
 * @param[in] addr Address in flash to write to
 * @param[in] data Pointer to data to write to flash
 * @param[in] len Length of data to write (max 256 bytes)
 * @return Zero if success or error code
 * @retval -1 Unable to claim SPI bus
 * @retval -2 Size exceeds 256 bytes
 * @retval -3 Length to write would wrap around page boundary
 */
int32_t UVOS_Flash_Jedec_WriteChunks( uintptr_t flash_id, uint32_t addr, struct uvos_flash_chunk chunks[], uint32_t num )
{
  struct jedec_flash_dev * flash_dev = ( struct jedec_flash_dev * )flash_id;

  if ( UVOS_Flash_Jedec_Validate( flash_dev ) != 0 ) {
    return -1;
  }

  uint8_t ret;
  uint8_t out[4] = { JEDEC_PAGE_WRITE, ( addr >> 16 ) & 0xff, ( addr >> 8 ) & 0xff, addr & 0xff };

  /* Can only write one page at a time */
  uint32_t len   = 0;
  for ( uint32_t i = 0; i < num; i++ ) {
    len += chunks[i].len;
  }

  if ( len > 0x100 ) {
    return -2;
  }

  /* Ensure number of bytes fits after starting address before end of page */
  if ( ( ( addr & 0xff ) + len ) > 0x100 ) {
    return -3;
  }
  if ( ( ret = UVOS_Flash_Jedec_WriteEnable( flash_dev ) ) != 0 ) {
    return ret;
  }

  /* Execute write page command and clock in address.  Keep CS asserted */
  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, true ) != 0 ) {
    return -1;
  }

  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, NULL, sizeof( out ), NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -1;
  }

  for ( uint32_t i = 0; i < num; i++ ) {
    struct uvos_flash_chunk * chunk = &chunks[i];

    /* Clock out data to flash */
    if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, chunk->addr, NULL, chunk->len, NULL ) < 0 ) {
      UVOS_Flash_Jedec_ReleaseBus( flash_dev );
      return -1;
    }
  }
  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  // Skip checking for busy with this to get OS running again fast

  return 0;
}

/**
 * @brief Read data from a location in flash memory
 * @param[in] addr Address in flash to write to
 * @param[in] data Pointer to data to write from flash
 * @param[in] len Length of data to write (max 256 bytes)
 * @return Zero if success or error code
 * @retval -1 Unable to claim SPI bus
 */
int32_t UVOS_Flash_Jedec_ReadData( uintptr_t flash_id, uint32_t addr, uint8_t * data, uint16_t len )
{
  struct jedec_flash_dev * flash_dev = ( struct jedec_flash_dev * )flash_id;

  if ( UVOS_Flash_Jedec_Validate( flash_dev ) != 0 ) {
    return -1;
  }
  bool fast_read = flash_dev->cfg->fast_read != 0;
  if ( UVOS_Flash_Jedec_ClaimBus( flash_dev, fast_read ) == -1 ) {
    return -1;
  }
  /* Execute read command and clock in address.  Keep CS asserted */
  if ( !fast_read ) {
    uint8_t out[] = { JEDEC_READ_DATA, ( addr >> 16 ) & 0xff, ( addr >> 8 ) & 0xff, addr & 0xff };
    if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, NULL, sizeof( out ), NULL ) < 0 ) {
      UVOS_Flash_Jedec_ReleaseBus( flash_dev );
      return -2;
    }
  } else {
    uint8_t cmdlen = flash_dev->cfg->fast_read_dummy_bytes + 4;
    uint8_t out[cmdlen];
    memset( out, 0x0, cmdlen );
    out[0] = flash_dev->cfg->fast_read;
    out[1] = ( addr >> 16 ) & 0xff;
    out[2] = ( addr >> 8 ) & 0xff;
    out[3] = addr & 0xff;
    if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, out, NULL, cmdlen, NULL ) < 0 ) {
      UVOS_Flash_Jedec_ReleaseBus( flash_dev );
      return -2;
    }
  }

  /* Copy the transfer data to the buffer */
  if ( UVOS_SPI_TransferBlock( flash_dev->spi_id, NULL, data, len, NULL ) < 0 ) {
    UVOS_Flash_Jedec_ReleaseBus( flash_dev );
    return -3;
  }

  UVOS_Flash_Jedec_ReleaseBus( flash_dev );

  return 0;
}

/* Provide a flash driver to external drivers */
const struct uvos_flash_driver uvos_jedec_flash_driver = {
  .start_transaction = UVOS_Flash_Jedec_StartTransaction,
  .end_transaction   = UVOS_Flash_Jedec_EndTransaction,
  .erase_chip   = UVOS_Flash_Jedec_EraseChip,
  .erase_sector = UVOS_Flash_Jedec_EraseSector,
  .write_chunks = UVOS_Flash_Jedec_WriteChunks,
  .write_data   = UVOS_Flash_Jedec_WriteData,
  .read_data    = UVOS_Flash_Jedec_ReadData,
};

#endif /* UVOS_INCLUDE_FLASH */
