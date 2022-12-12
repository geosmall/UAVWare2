#ifndef UAVWARE_H
#define UAVWARE_H

/* UVOS Includes */
#include <uvos.h>
#include "mpu.h"
#include "uw_time.h"
#include "uw_led.h"
#include "uw_receiver.h"


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

extern void setup( void );
extern void loop( void );

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // UAVWARE_H