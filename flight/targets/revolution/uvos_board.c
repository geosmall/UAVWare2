#include "uavware.h"
// #include <uvos_board_info.h>

// #include <uavobjectsinit.h>
// #include <hwsettings.h>
// #include <manualcontrolsettings.h>
// #include <oplinksettings.h>
// #include <oplinkstatus.h>
// #include <oplinkreceiver.h>
// #include <uvos_oplinkrcvr_priv.h>
// #include <uvos_openlrs.h>
// #include <uvos_openlrs_rcvr_priv.h>
// #include <taskinfo.h>
// #include <uvos_ws2811.h>
// #include <auxmagsettings.h>

#ifdef UVOS_INCLUDE_INSTRUMENTATION
#include <uvos_instrumentation.h>
#endif

/*
 * Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */
#include "board_hw_defs.c.inc"

/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, SPEKTRUM1, SPEKTRUM2, SBUS
 * NOTE: No slot in this map for NONE.
 */
uint32_t uvos_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE];

#define UVOS_COM_TELEM_RF_RX_BUF_LEN     512
#define UVOS_COM_TELEM_RF_TX_BUF_LEN     512

#define UVOS_COM_TELEM_USB_RX_BUF_LEN    65
#define UVOS_COM_TELEM_USB_TX_BUF_LEN    65

#define UVOS_COM_MAVLINK_RX_BUF_LEN      64
#define UVOS_COM_MAVLINK_TX_BUF_LEN      128

#if defined(UVOS_INCLUDE_DEBUG_CONSOLE)
#define UVOS_COM_DEBUGCONSOLE_RX_BUF_LEN 64
#define UVOS_COM_DEBUGCONSOLE_TX_BUF_LEN 64
uint32_t uvos_com_debug_id;
#endif /* UVOS_INCLUDE_DEBUG_CONSOLE */

// uint32_t uvos_com_gps_id       = 0;
// uint32_t uvos_com_telem_usb_id = 0;
uint32_t uvos_com_telem_rf_id  = 0;
uint32_t uvos_com_rf_id        = 0;
// uint32_t uvos_com_bridge_id    = 0;
// uint32_t uvos_com_overo_id     = 0;
// uint32_t uvos_com_hkosd_id     = 0;
// uint32_t uvos_com_vcp_id       = 0;
// uint32_t uvos_com_msp_id       = 0;
uint32_t uvos_com_mavlink_id   = 0;

/*
 * Setup a com port based on the passed cfg, driver and buffer sizes.
 * tx size = 0 make the port rx only
 * rx size = 0 make the port tx only
 * having both tx and rx size = 0 is not valid and will fail further down in UVOS_COM_Init()
 */
static void UVOS_Board_configure_com( const struct uvos_usart_cfg * usart_port_cfg, uint16_t rx_buf_len, uint16_t tx_buf_len,
                                      const struct uvos_com_driver * com_driver, uint32_t * uvos_com_id )
{
  uint32_t uvos_usart_id;

  if ( UVOS_USART_Init( &uvos_usart_id, usart_port_cfg ) ) {
    UVOS_Assert( 0 );
  }

  uint8_t * rx_buffer = 0, * tx_buffer = 0;

  if ( rx_buf_len > 0 ) {
    rx_buffer = ( uint8_t * )UVOS_malloc( rx_buf_len );
    UVOS_Assert( rx_buffer );
  }

  if ( tx_buf_len > 0 ) {
    tx_buffer = ( uint8_t * )UVOS_malloc( tx_buf_len );
    UVOS_Assert( tx_buffer );
  }

  if ( UVOS_COM_Init( uvos_com_id, com_driver, uvos_usart_id,
                      rx_buffer, rx_buf_len,
                      tx_buffer, tx_buf_len ) ) {
    UVOS_Assert( 0 );
  }
}

static void UVOS_Board_configure_ibus( const struct uvos_usart_cfg * usart_cfg )
{
  uint32_t uvos_usart_ibus_id;

  if ( UVOS_USART_Init( &uvos_usart_ibus_id, usart_cfg ) ) {
    UVOS_Assert( 0 );
  }

  uint32_t uvos_ibus_id;
  if ( UVOS_IBUS_Init( &uvos_ibus_id, &uvos_usart_com_driver, uvos_usart_ibus_id ) ) {
    UVOS_Assert( 0 );
  }

  uint32_t uvos_ibus_rcvr_id;
  if ( UVOS_RCVR_Init( &uvos_ibus_rcvr_id, &uvos_ibus_rcvr_driver, uvos_ibus_id ) ) {
    UVOS_Assert( 0 );
  }

  // uvos_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_IBUS] = uvos_ibus_rcvr_id;
}

/**
 * UVOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from uavware.c
 */

void putchar_( char c )
{
#ifdef UVOS_COM_DEBUG
  UVOS_COM_SendChar( UVOS_COM_DEBUG, c );
#endif
}

// uint32_t millis( void )
// {
//   return 0;
// }

void DMA_Transaction_Complete( bool crc_ok, uint8_t crc_val )
{

}

void UVOS_Board_Init( void )
{

#if defined(UVOS_INCLUDE_LED)
  const struct uvos_gpio_cfg * led_cfg  = &uvos_led_cfg;
  UVOS_Assert( led_cfg );
  UVOS_LED_Init( led_cfg );
#endif /* UVOS_INCLUDE_LED */

#if defined(UVOS_INCLUDE_RTC)
  UVOS_RTC_Init( &uvos_rtc_main_cfg );
#endif

  /* Set up pulse timers */
  UVOS_TIM_InitClock( &tim_3_cfg );
  UVOS_TIM_InitClock( &tim_5_cfg );

#if defined( UVOS_INCLUDE_DEBUG_CONSOLE )
  UVOS_Board_configure_com( &uvos_usart_flexi_cfg, UVOS_COM_DEBUGCONSOLE_RX_BUF_LEN, UVOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &uvos_usart_com_driver, &uvos_com_debug_id );
#endif

// #if defined(UVOS_INCLUDE_IBUS)
//   UVOS_Board_configure_ibus( &uvos_usart_ibus_flexi_cfg );
// #endif /* UVOS_INCLUDE_IBUS */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config( void )
{
  LL_FLASH_SetLatency( LL_FLASH_LATENCY_5 );
  while ( LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5 ) {
  }
  LL_PWR_SetRegulVoltageScaling( LL_PWR_REGU_VOLTAGE_SCALE1 );
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while ( LL_RCC_HSE_IsReady() != 1 ) {

  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_PLL_ConfigDomain_SYS( LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2 );
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while ( LL_RCC_PLL_IsReady() != 1 ) {

  }
  LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
  LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_4 );
  LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_2 );
  LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );

  /* Wait till System clock is ready */
  while ( LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL ) {

  }
  LL_Init1msTick( 168000000 );
  LL_SetSystemCoreClock( 168000000 );
}