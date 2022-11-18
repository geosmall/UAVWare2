#include <uavware.h>
// #include <uavobjectsinit.h>
// #include <systemmod.h>
#include "printf.h"

  /* Global Variables */

  /* Local Variables */

/**
 * UAVWare Main function:
 *
 * Initialize UVOS<BR>
 * Create the "System" task (SystemModInitializein Modules/System/systemmod.c) <BR>
 * Start FreeRTOS Scheduler (vTaskStartScheduler)<BR>
 * If something goes wrong, blink LED1 and LED2 every 100ms
 *
 */
int main( void )
{
  /* NOTE: Do NOT modify the following start-up sequence */
  /* Any new initialization functions should be added in UAVWareInit() */
  // vPortInitialiseBlocks();

  /* Brings up System using CMSIS functions, initializes periph clock, gpio pins. */
  UVOS_SYS_Init();

  /* board driver init */
  UVOS_Board_Init();

  /* Start up System module, create main System thread */
  // SystemModStart();

  /* Start the FreeRTOS scheduler */
  // vTaskStartScheduler();

  printf_( "Debug console initialized\r\n" );

  /* If all is well we will never reach here as the scheduler will now be running. */
  /* Do some UVOS_LED_HEARTBEAT to user that something bad just happened */
  UVOS_LED_On( UVOS_LED_HEARTBEAT );
  UVOS_LED_Off( UVOS_LED_ALARM );
  for ( ;; ) {
    UVOS_LED_Toggle( UVOS_LED_HEARTBEAT );
    UVOS_LED_Toggle( UVOS_LED_ALARM );
    UVOS_DELAY_WaitmS( 500 );
  }

  return 0;

}