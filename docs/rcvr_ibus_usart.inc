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

  uvos_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_IBUS] = uvos_ibus_rcvr_id;
}

//----------------------------------------------------------------------------------------

// RCVR driver AND device

// 1a.) RCVR Driver struct declaration (uvos_rcvr.h)

struct uvos_rcvr_driver {    //  <<<---<<<
  void    ( *init )( uint32_t id );
  int32_t ( *read )( uint32_t id, uint8_t channel );
  // xSemaphoreHandle (*get_semaphore)(uint32_t id, uint8_t channel);
  uint32_t  ( *get_flag )( uint32_t id );
  uint8_t ( *get_quality )( uint32_t id );
};

// 1b.) RCVR Driver struct definition (uvos_ibus.c)

const struct uvos_rcvr_driver uvos_ibus_rcvr_driver = {    // <<<---<<<
  .read = UVOS_IBUS_Read,
};

// 1c.) RCVR device declaration (uvos_rcvr.c)

struct uvos_rcvr_dev {
  enum uvos_rcvr_dev_magic magic;
  uint32_t lower_id;
  const struct uvos_rcvr_driver * driver;  //  <<<---<<<
};

// 1d.) RCVR device definition (uvos_rcvr.c)

static struct uvos_rcvr_dev uvos_rcvr_devs[UVOS_RCVR_MAX_DEVS];  //  <<<---<<<
static uint8_t uvos_rcvr_num_devs;
static struct uvos_rcvr_dev * UVOS_RCVR_alloc( void )

// 1e.) RCVR driver is passed in as part of UVOS_RCVR_Init() call

int32_t UVOS_RCVR_Init( uint32_t * rcvr_id, const struct uvos_rcvr_driver * driver, uint32_t lower_id )

// 1f.) IBUS-RCVR driver export (uvos_ibus_priv.h)

extern const struct uvos_rcvr_driver uvos_ibus_rcvr_driver;  //  <<<---<<<

//----------------------------------------------------------------------------------------

// IBUS device (IBUS uses COM driver)

// 2a.) IBus device declaration (uvos_ibus.c)

struct uvos_ibus_dev {    //  <<<---<<<
  uint32_t magic;
  int      buf_pos;
  int      rx_timer;
  int      failsafe_timer;
  uint16_t checksum;
  uint16_t channel_data[ UVOS_IBUS_NUM_INPUTS ];
  uint8_t  rx_buf[ UVOS_IBUS_BUFLEN ];
};

// 2b.) IBus device definition (uvos_ibus.c)

int32_t UVOS_IBUS_Init( uint32_t * ibus_id, const struct uvos_com_driver * driver, uint32_t lower_id )
{
  struct uvos_ibus_dev * ibus_dev = UVOS_IBUS_Alloc();  //  <<<---<<<

//----------------------------------------------------------------------------------------

// COM driver

// 3a.) COM driver struct declaration (uvos_com.h)

struct uvos_com_driver {
  // void     ( *init )( uint32_t id );
  void     ( *set_baud )( uint32_t id, uint32_t baud );
  // void     ( *set_ctrl_line )( uint32_t id, uint32_t mask, uint32_t state );
  void     ( *tx_start )( uint32_t id, uint16_t tx_bytes_avail );
  void     ( *rx_start )( uint32_t id, uint16_t rx_bytes_avail );
  void     ( *bind_rx_cb )( uint32_t id, uvos_com_callback rx_in_cb, uint32_t context );
  void     ( *bind_tx_cb )( uint32_t id, uvos_com_callback tx_out_cb, uint32_t context );
  // void     ( *bind_ctrl_line_cb )( uint32_t id, uvos_com_callback_ctrl_line ctrl_line_cb, uint32_t context );
  void     ( *bind_baud_rate_cb )( uint32_t id, uvos_com_callback_baud_rate baud_rate_cb, uint32_t context );
  uint32_t ( *available )( uint32_t id );
};

// 3b.) COM driver const struct of function pointers defined below in USART (uvos_usart.c)

// 3c.) COM device declaration (uvos_com.c)

struct uvos_com_dev {    //  <<<---<<<
  enum uvos_com_dev_magic magic;
  uint32_t lower_id;
  const struct uvos_com_driver * driver;    //  <<<---<<<

#if defined(UVOS_INCLUDE_FREERTOS)
  xSemaphoreHandle tx_sem;
  xSemaphoreHandle rx_sem;
  xSemaphoreHandle sendbuffer_sem;
#endif

// 3d.) COM device definition as part of UVOS_COM_Init() (uvos_com.c)

static struct uvos_com_dev uvos_com_devs[UVOS_COM_MAX_DEVS];
static uint8_t uvos_com_num_devs;
static struct uvos_com_dev * UVOS_COM_alloc( void )
{
  struct uvos_com_dev * com_dev;  //  <<<---<<<

  if ( uvos_com_num_devs >= UVOS_COM_MAX_DEVS ) {
    return NULL;
  }

  com_dev = &uvos_com_devs[uvos_com_num_devs++];  //  <<<---<<<

  memset( com_dev, 0, sizeof( struct uvos_com_dev ) );
  com_dev->magic = UVOS_COM_DEV_MAGIC;

  return com_dev;
}

// 3e.) COM driver is passed in as part of UVOS_COM_Init() call (uvos_com.c)

int32_t UVOS_COM_Init( uint32_t * com_id, const struct uvos_com_driver * driver, uint32_t lower_id, uint8_t * rx_buffer, uint16_t rx_buffer_len, uint8_t * tx_buffer, uint16_t tx_buffer_len )

// 3f.) USART-COM driver export (uvos_usart_priv.h)

extern const struct uvos_com_driver uvos_usart_com_driver;  //  <<<---<<<

//----------------------------------------------------------------------------------------

// USART driver AND device

// 4a.) COM driver struct definition (uvos_usart.c)

const struct uvos_com_driver uvos_usart_com_driver = {
  .set_baud      = UVOS_USART_ChangeBaud,
  // .set_ctrl_line = UVOS_USART_SetCtrlLine,
  .tx_start      = UVOS_USART_TxStart,
  .rx_start      = UVOS_USART_RxStart,
  .bind_tx_cb    = UVOS_USART_RegisterTxCallback,
  .bind_rx_cb    = UVOS_USART_RegisterRxCallback,
};