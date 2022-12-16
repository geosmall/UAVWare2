#include <UAVWare.h>
#include <uvos.h>
#include <lfs.h>
#include "uw_fs.h"
#include <uvos_flash_jedec_priv.h>

#define FS_FILE_NAME_SIZE  32 /* Length of file name, used in FS buffers */
#define LFS_CACHE_SIZE 256
#define LFS_LOOKAHEAD_SIZE 512

static int block_device_read( const struct lfs_config * c, lfs_block_t block,  lfs_off_t off, void * buffer, lfs_size_t size );
static int block_device_prog( const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size );
static int block_device_erase( const struct lfs_config * c, lfs_block_t block );
static int block_device_sync( const struct lfs_config * c );

// Optional statically allocated read buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t read_buf[ LFS_CACHE_SIZE ];

// Optional statically allocated program buffer. Must be cache_size.
// By default lfs_malloc is used to allocate this buffer.
uint8_t prog_buf[ LFS_CACHE_SIZE ];

// Optional statically allocated lookahead buffer. Must be lookahead_size
// and aligned to a 32-bit boundary. By default lfs_malloc is used to
// allocate this buffer.
uint8_t lookahead_buf[ LFS_LOOKAHEAD_SIZE ] __attribute__ ( ( aligned ( 4 ) ) );

// configuration of the file system is provided by this struct
const struct lfs_config FS_cfg = {
  // block device operations
  .read  = block_device_read,
  .prog  = block_device_prog,
  .erase = block_device_erase,
  .sync  = block_device_sync,
  // chip info
  .read_size = LFS_CACHE_SIZE,
  .prog_size = LFS_CACHE_SIZE,
  .block_size = 4096,
  .block_count = 4096,
  .block_cycles = 500,
  .cache_size = LFS_CACHE_SIZE,
  .lookahead_size = LFS_LOOKAHEAD_SIZE,
  .read_buffer = read_buf,
  .prog_buffer = prog_buf,
  .lookahead_buffer = lookahead_buf,
};

/* variables used by the file system */
static bool _FS_isMounted = false;
static lfs_t _FS_lfs;
static uintptr_t _flash_id;

static int block_device_read( const struct lfs_config * c, lfs_block_t block,  lfs_off_t off, void * buffer, lfs_size_t size )
{
  uint32_t res;

  // res = W25_Read(block * c->block_size + off, buffer, size);
  res = UVOS_Flash_Jedec_ReadData( _flash_id, block * c->block_size + off, buffer, size );
  if ( res ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_prog( const struct lfs_config * c, lfs_block_t block, lfs_off_t off, const void * buffer, lfs_size_t size )
{
  uint32_t res;

  // res = W25_ProgramPage( block * c->block_size + off, buffer, size );
  res = UVOS_Flash_Jedec_WriteData( _flash_id, block * c->block_size + off, ( uint8_t * )buffer, size );
  if ( res ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_erase( const struct lfs_config * c, lfs_block_t block )
{
  uint32_t res;

  // res = W25_EraseSector4K( block * c->block_size );
  res = UVOS_Flash_Jedec_EraseSector( _flash_id, block * c->block_size );
  if ( res ) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

static int block_device_sync( const struct lfs_config * c )
{
  return LFS_ERR_OK;
}

uint32_t UW_fs_init( uintptr_t flash_id )
{
  /* Save local pointer to SPI flash */
  _flash_id = flash_id;

  UVOS_COM_SendString( UVOS_COM_DEBUG, "Attempting to mount existing media\r\n" );
  if ( UW_fs_mount() != FS_ERR_OK ) {
    /* FS mount failed, try formatting underlying FS */
    UVOS_COM_SendString( UVOS_COM_DEBUG, "Could not mount media, attemping to format\r\n" );
    if ( UW_fs_format() != FS_ERR_OK ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "Format failed\r\n" );
      return FS_ERR_FAILED;
    }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "Attempting to mount freshly formatted media\r\n" );
    if ( UW_fs_mount() != FS_ERR_OK ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "Mount after format failed\r\n" );
      return FS_ERR_FAILED;
    }
  }

  return FS_ERR_OK;
}

uint32_t UW_fs_format( void )
{
  int res;

  if ( _FS_isMounted ) {
    // CLS1_SendStr("File system is already mounted, unmount it first.\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is already mounted, unmount it first.\r\n" );
    return FS_ERR_FAILED;
  }
  // if (io!=NULL) {
  //   CLS1_SendStr("Formatting ...", io->stdOut);
  // }
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Formatting ..." );
  res = lfs_format( &_FS_lfs, &FS_cfg );
  if ( res == LFS_ERR_OK ) {
    // if (io!=NULL) {
    //   CLS1_SendStr(" done.\r\n", io->stdOut);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, " done.\r\n" );
    return FS_ERR_OK;
  } else {
    // if (io!=NULL) {
    //   CLS1_SendStr(" FAILED!\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, " FAILED!\r\n" );
    return FS_ERR_FAILED;
  }
}

uint32_t UW_fs_mount( void )
{
  int res;

  if ( _FS_isMounted ) {
    // if (io!=NULL) {
    //   // CLS1_SendStr("File system is already mounted.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is already mounted, unmount it first.\r\n" );
    return FS_ERR_FAILED;
  }
  // if (io!=NULL) {
  //   CLS1_SendStr("Mounting ...", io->stdOut);
  // }
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Mounting ..." );
  res = lfs_mount( &_FS_lfs, &FS_cfg );
  if ( res == LFS_ERR_OK ) {
    // if (io!=NULL) {
    //   CLS1_SendStr(" done.\r\n", io->stdOut);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, " done.\r\n" );
    _FS_isMounted = true;
    return FS_ERR_OK;
  } else {
    // if (io!=NULL) {
    //   CLS1_SendStr(" FAILED!\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, " FAILED!\r\n" );
    return FS_ERR_FAILED;
  }
}

uint32_t UW_fs_unmount( void )
{
  int res;

  if ( !_FS_isMounted ) {
    // if (io!=NULL) {
    //   CLS1_SendStr("File system is already unmounted.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is already unmounted.\r\n" );
    return FS_ERR_FAILED;
  }
  // if (io!=NULL) {
  //   CLS1_SendStr("Unmounting ...", io->stdOut);
  // }
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Unmounting ..." );
  res = lfs_unmount( &_FS_lfs );
  if ( res == LFS_ERR_OK ) {
    // if (io!=NULL) {
    //   CLS1_SendStr(" done.\r\n", io->stdOut);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, " done.\r\n" );
    _FS_isMounted = false;
    return FS_ERR_OK;
  } else {
    // if (io!=NULL) {
    //   CLS1_SendStr(" FAILED!\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, " FAILED!\r\n" );
    return FS_ERR_FAILED;
  }
}

#define PRN_BUFFER_SIZE  128

// uint8_t FS_Dir(const char *path, CLS1_ConstStdIOType *io) {
uint32_t UW_fs_dir( const char * path )
{
  int res;
  lfs_dir_t dir;
  struct lfs_info info;

  char prn_buffer[ PRN_BUFFER_SIZE ] = {0};

  if ( !_FS_isMounted ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "File system is not mounted, mount it first.\r\n" );
    return FS_ERR_FAILED;
  }
  if ( path == NULL ) {
    path = "/"; /* default path */
  }
  res = lfs_dir_open( &_FS_lfs, &dir, path );
  if ( res != LFS_ERR_OK ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED lfs_dir_open()!\r\n" );
    return FS_ERR_FAILED;
  }
  for ( ;; ) {
    res = lfs_dir_read( &_FS_lfs, &dir, &info );
    if ( res < 0 ) {
      UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED lfs_dir_read()!\r\n" );
      return FS_ERR_FAILED;
    }
    if ( res == 0 ) { /* no more files */
      break;
    }
    switch ( info.type ) {
    case LFS_TYPE_REG: UVOS_COM_SendString( UVOS_COM_DEBUG, "reg " ); break;
    case LFS_TYPE_DIR: UVOS_COM_SendString( UVOS_COM_DEBUG, "dir " ); break;
    default:           UVOS_COM_SendString( UVOS_COM_DEBUG, "?   " ); break;
    }
    static const char * prefixes[] = {"", "K", "M", "G"}; /* prefixes for kilo, mega and giga */
    for ( int i = sizeof( prefixes ) / sizeof( prefixes[0] ) - 1; i >= 0; i-- ) {
      if ( info.size >= ( 1 << 10 * i ) - 1 ) {
        uint16_t prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%*u%sB ", 4 - ( i != 0 ), ( unsigned int )info.size >> 10 * i, prefixes[i] );
        UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
        break;
      }
    } /* for */
    UVOS_COM_SendString( UVOS_COM_DEBUG, info.name );
    UVOS_COM_SendString( UVOS_COM_DEBUG, "\r\n" );
  } /* for */
  res = lfs_dir_close( &_FS_lfs, &dir );
  if ( res != LFS_ERR_OK ) {
    UVOS_COM_SendString( UVOS_COM_DEBUG, "FAILED lfs_dir_close()!\r\n" );
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_CopyFile(const char *srcPath, const char *dstPath, CLS1_ConstStdIOType *io) {
uint8_t UW_fs_copy_file( const char * srcPath, const char * dstPath )
{
  lfs_file_t fsrc, fdst;
  __attribute__( ( unused ) ) volatile uint8_t res;
  int result, nofBytesRead;
  uint8_t buffer[32];   /* copy buffer */

#ifdef LFS_NO_MALLOC
  static uint8_t                file_buffer_src[LFS_CACHE_SIZE];
  static struct lfs_file_config file_cfg_src = {
    .buffer = file_buffer_src
  };
  static uint8_t                file_buffer_dst[LFS_CACHE_SIZE];
  static struct lfs_file_config file_cfg_dst = {
    .buffer = file_buffer_dst
  };
#endif

  if ( !_FS_isMounted ) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }

  /* open source file */
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &fsrc, srcPath, LFS_O_RDONLY, &file_cfg_src );
#else
  result = lfs_file_open( &_FS_lfs, &fsrc, srcPath, LFS_O_RDONLY );
#endif
  if ( result < 0 ) {
    // CLS1_SendStr((const unsigned char*)"*** Failed opening source file!\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed opening source file!\r\n" );
    return FS_ERR_FAILED;
  }
  /* create destination file */
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &fdst, dstPath, LFS_O_WRONLY | LFS_O_CREAT, &file_cfg_dst );
#else
  result = lfs_file_open( &_FS_lfs, &fdst, dstPath, LFS_O_WRONLY | LFS_O_CREAT );
#endif
  if ( result < 0 ) {
    lfs_file_close( &_FS_lfs, &fsrc );
    // CLS1_SendStr((const unsigned char*)"*** Failed opening destination file!\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed opening destination file!\r\n" );
    return FS_ERR_FAILED;
  }
  /* now copy source to destination */
  for ( ;; ) {
    nofBytesRead = lfs_file_read( &_FS_lfs, &fsrc, buffer, sizeof( buffer ) );
    if ( nofBytesRead < 0 ) {
      // CLS1_SendStr((const unsigned char*)"*** Failed reading source file!\r\n", io->stdErr);
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed reading source file!\r\n" );
      res = FS_ERR_FAILED;
      break;
    }
    if ( nofBytesRead == 0 ) { /* end of file */
      break;
    }
    result = lfs_file_write( &_FS_lfs, &fdst, buffer, nofBytesRead );
    if ( result < 0 ) {
      // CLS1_SendStr((const unsigned char*)"*** Failed writing destination file!\r\n", io->stdErr);
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed writing destination file!\r\n" );
      res = FS_ERR_FAILED;
      break;
    }
  } /* for */
  /* close all files */
  result = lfs_file_close( &_FS_lfs, &fsrc );
  if ( result < 0 ) {
    // CLS1_SendStr((const unsigned char*)"*** Failed closing source file!\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed closing source file!\r\n" );
    res = FS_ERR_FAILED;
  }
  result = lfs_file_close( &_FS_lfs, &fdst );
  if ( result < 0 ) {
    // CLS1_SendStr((const unsigned char*)"*** Failed closing destination file!\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed closing destination file!\r\n" );
    res = FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_MoveFile(const char *srcPath, const char *dstPath, CLS1_ConstStdIOType *io) {
uint8_t UW_fs_move_file( const char * srcPath, const char * dstPath )
{
  if ( !_FS_isMounted ) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
  if ( lfs_rename( &_FS_lfs, srcPath, dstPath ) < 0 ) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: failed renaming file or directory.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: failed renaming file or directory.\r\n" );
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

#if 0 // GLS

static uint8_t readFromFile( void * hndl, uint32_t addr, uint8_t * buf, size_t bufSize )
{
  lfs_file_t * fp;

  fp = ( lfs_file_t * )hndl;
  if ( lfs_file_read( &_FS_lfs, fp, buf, bufSize ) < 0 ) {
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_PrintHexFile(const char *filePath, CLS1_ConstStdIOType *io) {
uint8_t UW_fs_print_hex_file( const char * filePath )
{
  lfs_file_t file;
  uint8_t res = FS_ERR_OK;
  int32_t fileSize;
  int result;

#ifdef LFS_NO_MALLOC
  static uint8_t                file_buffer[ LFS_CACHE_SIZE ];
  static struct lfs_file_config file_cfg = {
    .buffer = file_buffer
  };
#endif

  if ( !_FS_isMounted ) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &file, filePath, LFS_O_RDONLY, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &file, filePath, LFS_O_RDONLY );
#endif
  if ( result < 0 ) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: Failed opening file.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: Failed opening file.\r\n" );
    return FS_ERR_FAILED;
  }
  fileSize = lfs_file_size( &_FS_lfs, &file );
  if ( fileSize < 0 ) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: getting file size\r\n", io->stdErr);
    //   (void)lfs_file_close(&_FS_lfs, &file);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: getting file size\r\n" );
    lfs_file_close( &_FS_lfs, &file );
    return FS_ERR_FAILED;
  }
  res = CLS1_PrintMemory( &file, 0, fileSize - 1, 4, 16, readFromFile );
  if ( res != FS_ERR_OK ) {
    // CLS1_SendStr("ERROR while calling PrintMemory()\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR while calling PrintMemory()\r\n" );
  }
  lfs_file_close( &_FS_lfs, &file );
  return res;
}
#endif // GLS

// uint8_t FS_RemoveFile(const char *filePath, CLS1_ConstStdIOType *io) {
uint8_t UW_fs_remove_file( const char * filePath )
{
  int result;

  if ( !_FS_isMounted ) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
  result = lfs_remove( &_FS_lfs, filePath );
  if ( result < 0 ) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: Failed removing file.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: Failed removing file.\r\n" );
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_RunBenchmark(CLS1_ConstStdIOType *io) {
uint32_t UW_fs_run_benchmark( void )
{
  lfs_file_t file;
  int result;
  uint32_t i;
  uint8_t read_buf[10];
  // TIMEREC time, startTime;
  unsigned int start_microseconds;
  unsigned int microseconds;
  uint16_t prn_buflen;
  char prn_buffer[ PRN_BUFFER_SIZE ] = {0};

#ifdef LFS_NO_MALLOC
  static uint8_t                file_buffer[LFS_CACHE_SIZE];
  static struct lfs_file_config file_cfg = {
    .buffer = file_buffer
  };
#endif

  if ( !_FS_isMounted ) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    UVOS_COM_SendString( UVOS_COM_DEBUG, "ERROR: File system is not mounted.\r\n" );
    return FS_ERR_FAILED;
  }
  /* write benchmark */
  // CLS1_SendStr((const unsigned char*)"Benchmark: write/copy/read a 100kB file:\r\n", io->stdOut);
  // CLS1_SendStr((const unsigned char*)"Delete existing benchmark files...\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Benchmark: write/copy/read a 100kB file:\r\n" );
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Delete existing benchmark files...\r\n" );
  UW_fs_remove_file( "./bench.txt" );
  UW_fs_remove_file( "./copy.txt" );

  // CLS1_SendStr((const unsigned char*)"Create benchmark file...\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Create benchmark file...\r\n" );
  // (void)TmDt1_GetTime(&startTime);
  start_microseconds = micros();
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &file, "./bench.txt", LFS_O_WRONLY | LFS_O_CREAT, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &file, "./bench.txt", LFS_O_WRONLY | LFS_O_CREAT );
#endif
  if ( result < 0 ) {
    // CLS1_SendStr((const unsigned char*)"*** Failed creating benchmark file!\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed creating benchmark file!\r\n" );
    return FS_ERR_FAILED;
  }
  for ( i = 0; i < 10240; i++ ) {
    if ( lfs_file_write( &_FS_lfs, &file, "benchmark ", sizeof( "benchmark " ) - 1 ) < 0 ) {
      // CLS1_SendStr((const unsigned char*)"*** Failed writing file!\r\n", io->stdErr);
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed writing file!\r\n" );
      lfs_file_close( &_FS_lfs, &file );
      return FS_ERR_FAILED;
    }
  }
  lfs_file_close( &_FS_lfs, &file );
//   (void)TmDt1_GetTime(&time);
//   start_mseconds = startTime.Hour*60*60*1000 + startTime.Min*60*1000 + startTime.Sec*1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//   + startTime.Sec100*10
// #endif
//   ;
//   mseconds = time.Hour*60*60*1000 + time.Min*60*1000 + time.Sec*1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//   + time.Sec100*10
// #endif
//   - start_mseconds;
  microseconds = micros() - start_microseconds;
  // CLS1_SendNum32s(mseconds, io->stdOut);
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u", microseconds / 1000 );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  // CLS1_SendStr((const unsigned char*)" ms for writing (", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, " ms for writing (" );
  // CLS1_SendNum32s((100 * 1000) / mseconds, io->stdOut);
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u",  (unsigned int)( 100.0f  / ( microseconds / 1000000.0f ) ) );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  // CLS1_SendStr((const unsigned char*)" kB/s)\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, " kB/s)\r\n" );

  /* read benchmark */
  // CLS1_SendStr((const unsigned char*)"Read 100kB benchmark file...\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Read 100kB benchmark file...\r\n" );
  // (void)TmDt1_GetTime(&startTime);
  start_microseconds = micros();
#ifdef LFS_NO_MALLOC
  result = lfs_file_opencfg( &_FS_lfs, &file, "./bench.txt", LFS_O_RDONLY, &file_cfg );
#else
  result = lfs_file_open( &_FS_lfs, &file, "./bench.txt", LFS_O_RDONLY );
#endif
  if ( result < 0 ) {
    // CLS1_SendStr((const unsigned char*)"*** Failed opening benchmark file!\r\n", io->stdErr);
    UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed opening benchmark file!\r\n" );
    return FS_ERR_FAILED;
  }
  for ( i = 0; i < 10240; i++ ) {
    if ( lfs_file_read( &_FS_lfs, &file, &read_buf[0], sizeof( read_buf ) ) < 0 ) {
      // CLS1_SendStr((const unsigned char*)"*** Failed reading file!\r\n", io->stdErr);
      UVOS_COM_SendString( UVOS_COM_DEBUG, "*** Failed reading file!\r\n" );
      lfs_file_close( &_FS_lfs, &file );
      return FS_ERR_FAILED;
    }
  }
  lfs_file_close( &_FS_lfs, &file );
//   (void)TmDt1_GetTime(&time);
//   start_mseconds = startTime.Hour * 60 * 60 * 1000 + startTime.Min * 60 * 1000 + startTime.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//                    + startTime.Sec100 * 10
// #endif
//                    ;
//   mseconds = time.Hour * 60 * 60 * 1000 + time.Min * 60 * 1000 + time.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//              + time.Sec100 * 10
// #endif
//              - start_mseconds;
  microseconds = micros() - start_microseconds;
  // CLS1_SendNum32s(mseconds, io->stdOut);
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u", microseconds / 1000 );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  // CLS1_SendStr((const unsigned char*)" ms for reading (", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, " ms for reading (" );
  // CLS1_SendNum32s((100 * 1000) / mseconds, io->stdOut);
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u",  (unsigned int)( 100.0f  / ( microseconds / 1000000.0f ) ) );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  // CLS1_SendStr((const unsigned char*)" kB/s)\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, " kB/s)\r\n" );

  /* copy benchmark */
  // CLS1_SendStr((const unsigned char*)"Copy 100kB file...\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, "Copy 100kB file...\r\n" );
  // (void)TmDt1_GetTime(&startTime);
  start_microseconds = micros();
  UW_fs_copy_file( ( const char * )"./bench.txt", ( const char * )"./copy.txt" );
//   (void)TmDt1_GetTime(&time);
//   start_mseconds = startTime.Hour * 60 * 60 * 1000 + startTime.Min * 60 * 1000 + startTime.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//                    + startTime.Sec100 * 10
// #endif
//                    ;
//   mseconds = time.Hour * 60 * 60 * 1000 + time.Min * 60 * 1000 + time.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//              + time.Sec100 * 10
// #endif
//              - start_mseconds;
  // CLS1_SendNum32s(mseconds, io->stdOut);
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u", microseconds / 1000 );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  // CLS1_SendStr((const unsigned char*)" ms for copy (", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, " ms for copy (" );
  // CLS1_SendNum32s((100 * 1000) / mseconds, io->stdOut);
  prn_buflen = snprintf( prn_buffer, sizeof( prn_buffer ), "%u",  (unsigned int)( 100.0f  / ( microseconds / 1000000.0f ) ) );
  UVOS_COM_SendBuffer( UVOS_COM_DEBUG, ( uint8_t * ) prn_buffer, prn_buflen );
  // CLS1_SendStr((const unsigned char*)" kB/s)\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, " kB/s)\r\n" );
  // CLS1_SendStr((const unsigned char*)"done!\r\n", io->stdOut);
  UVOS_COM_SendString( UVOS_COM_DEBUG, "done!\r\n" );
  return FS_ERR_OK;
}

// static uint8_t FS_PrintStatus(CLS1_ConstStdIOType *io) {
__attribute__( ( unused ) ) static uint32_t UW_fs_print_status( void )
{
  __attribute__( ( unused ) ) uint8_t buf[24];

  // CLS1_SendStatusStr((const unsigned char*)"FS", (const unsigned char*)"\r\n", io->stdOut);
  // CLS1_SendStatusStr((const unsigned char*)"  mounted", _FS_isMounted ? "yes\r\n" : "no\r\n", io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count * FS_cfg.block_size);
  // UTIL1_strcat(buf, sizeof(buf), " bytes\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  space", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.read_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  read_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.prog_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  prog_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  block_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  block_count", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.lookahead);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  lookahead", buf, io->stdOut);
  return FS_ERR_OK;
}
