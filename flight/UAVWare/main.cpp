#include "UAVWare.h"

/* Global Variables */

/* Local Variables */

/**
 * UAVWare Main function:
 *
 * Initialize UVOS<BR>
 * If something goes wrong, blink LED1 and LED2 every 100ms
 *
 */
int main( void )
{

  // Call application setup()
  setup();

  // Call application main loop
  for ( ;; ) {
    loop();
  }

  /* If all is well we will never reach here as the scheduler will now be running. */
  /* Do some UVOS_LED_HEARTBEAT to user that something bad just happened */

  UVOS_LED_Off( UVOS_LED_HEARTBEAT );
  for ( ;; ) {
    UVOS_LED_Toggle( UVOS_LED_HEARTBEAT );
    UVOS_DELAY_WaitmS( 100 );
  }

  return 0;

}

// uint32_t UAVWare_init( void )
// {
//   /* Brings up System using CMSIS functions, initializes periph clock, gpio pins. */
//   UVOS_SYS_Init();

//   /* board driver init */
//   return UVOS_Board_Init();
// }