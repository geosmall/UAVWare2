/*
 * fs.h
 *
 *  Created on: 06.01.2019
 *      Author: Erich Styger
 */

#ifndef SOURCES_FS_H_
#define SOURCES_FS_H_

#include <stdint.h>
#include <stdbool.h>

#define FS_ERR_OK                          0x00U /*!< OK */
#define FS_ERR_FAILED                      0x01U /*!< Requested functionality or process failed. */

#ifdef __cplusplus
extern "C" {
#endif

// #include "CLS1.h"

// uint8_t FS_ParseCommand(const unsigned char* cmd, bool *handled, const CLS1_StdIOType *io);

uint32_t UW_fs_init( uintptr_t flash_id );
uint32_t UW_fs_format( void );
uint32_t UW_fs_mount( void );
uint32_t UW_fs_chip_erase( void );
// uint8_t UW_fs_copy_file( const char * srcPath, const char * dstPath );
uint32_t UW_fs_run_benchmark( void );

#ifdef __cplusplus
}
#endif

#endif /* SOURCES_FS_H_ */
