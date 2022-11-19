#if defined(UVOS_INCLUDE_LED)

// GPIO_InitStruct.Pin = testPin2_Pin|SPI_MPU_NSS_Pin;
// GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
// GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
// GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
// GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

#include <uvos_led_priv.h>
static const struct uvos_gpio uvos_leds[] = {
  [UVOS_LED_HEARTBEAT] = {
    .pin               =             {
      .gpio = GPIOA,
      .init =             {
        .Pin   = LL_GPIO_PIN_5,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .Mode  = LL_GPIO_MODE_OUTPUT,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull  = LL_GPIO_PULL_NO
      },
    },
    .active_low         = true
  },
};

static const struct uvos_gpio_cfg uvos_led_cfg = {
  .gpios     = uvos_leds,
  .num_gpios = NELEMENTS( uvos_leds ),
};

#endif /* UVOS_INCLUDE_LED */

#include <uvos_usart_priv.h>

#ifdef UVOS_INCLUDE_COM_TELEM

/*
 * MAIN USART
 */
static const struct uvos_usart_cfg uvos_usart_main_cfg = {
  .regs  = USART1,
  .init  = {
    .BaudRate            = 57600,
    .DataWidth           = LL_USART_DATAWIDTH_8B,
    .StopBits            = LL_USART_STOPBITS_1,
    .Parity              = LL_USART_PARITY_NONE,
    .TransferDirection   = LL_USART_DIRECTION_TX_RX,
    .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
  },
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .rx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_9,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
  .tx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_9,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
};
#endif /* UVOS_INCLUDE_COM_TELEM */

#ifdef UVOS_INCLUDE_COM_FLEXI
/*
 * FLEXI PORT
 */
static const struct uvos_usart_cfg uvos_usart_flexi_cfg = {
  .regs  = USART2,
  .init  = {
    .BaudRate            = 57600,
    .DataWidth           = LL_USART_DATAWIDTH_8B,
    .StopBits            = LL_USART_STOPBITS_1,
    .Parity              = LL_USART_PARITY_NONE,
    .TransferDirection   = LL_USART_DIRECTION_TX_RX,
    .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
  },
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = USART2_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .rx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_3,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
  .tx                                        = {
    .gpio = GPIOA,
    .init = {
      .Pin   = LL_GPIO_PIN_2,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
};

#endif /* UVOS_INCLUDE_COM_FLEXI */

#if defined(UVOS_INCLUDE_IBUS)
/*
 * IBUS on Flexi Port USART
 */
#include <uvos_ibus_priv.h>

static const struct uvos_usart_cfg uvos_usart_ibus_flexi_cfg = {
  .regs  = USART1,
  .init  = {
    .BaudRate            = 115200,
    .DataWidth           = LL_USART_DATAWIDTH_8B,
    .StopBits            = LL_USART_STOPBITS_1,
    .Parity              = LL_USART_PARITY_NONE,
    .TransferDirection   = LL_USART_DIRECTION_RX,
    .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
  },
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_HIGH,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
  .rx                                        = {
    .gpio = GPIOB,
    .init = {
      .Pin   = LL_GPIO_PIN_11,
      .Mode = LL_GPIO_MODE_ALTERNATE,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_UP,
      .Alternate = LL_GPIO_AF_7,
    },
  },
  .tx                                        = {
    .gpio = GPIOB,
    .init = {
      .Pin   = LL_GPIO_PIN_10,
      .Mode = LL_GPIO_MODE_OUTPUT,
      .Speed = LL_GPIO_SPEED_FREQ_LOW,
      .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
      .Pull = LL_GPIO_PULL_NO,
    },
  },
};

#endif /* UVOS_INCLUDE_IBUS */

#if defined(UVOS_INCLUDE_RTC)
/*
 * Realtime Clock (RTC)
 */
#include <uvos_rtc_priv.h>

void UVOS_RTC_IRQ_Handler( void );
void RTC_WKUP_IRQHandler() __attribute__( ( alias( "UVOS_RTC_IRQ_Handler" ) ) );
static const struct uvos_rtc_cfg uvos_rtc_main_cfg = {
  .hse_clkdiv  = LL_RCC_RTC_HSE_DIV_8, // Divide 8 Mhz crystal down to 1
  .WUT_clkdiv = LL_RTC_WAKEUPCLOCK_DIV_16, // Divide 1 MHz by 16 gives 62.5 khz clock
  .WUT_reload = 100, // Every 100 cycles gives 625 Hz
  .irq                                       = {
    .init                                  = {
      .NVIC_IRQChannel    = RTC_WKUP_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

void UVOS_RTC_IRQ_Handler( void )
{
  UVOS_RTC_irq_handler();
}

#endif /* if defined(PIOS_INCLUDE_RTC) */

#include "uvos_tim_priv.h"

static const LL_TIM_InitTypeDef tim_3_5_time_base = {
  .Prescaler         = ( UVOS_PERIPHERAL_APB1_CLOCK / 1000000 ) - 1,
  .CounterMode       = LL_TIM_COUNTERMODE_UP,
  .Autoreload        = ( ( 1000000 / UVOS_SERVO_UPDATE_HZ ) - 1 ),
  .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
  .RepetitionCounter = 0x0000,
};
static const LL_TIM_InitTypeDef tim_9_10_time_base = {
  .Prescaler         = ( UVOS_PERIPHERAL_APB2_CLOCK / 1000000 ) - 1,
  .CounterMode       = LL_TIM_COUNTERMODE_UP,
  .Autoreload        = ( ( 1000000 / UVOS_SERVO_UPDATE_HZ ) - 1 ),
  .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
  .RepetitionCounter = 0x0000,
};
static const LL_TIM_InitTypeDef tim_11_time_base = {
  .Prescaler         = ( UVOS_PERIPHERAL_APB2_CLOCK / 1000000 ) - 1,
  .CounterMode       = LL_TIM_COUNTERMODE_UP,
  .Autoreload        = ( ( 1000000 / UVOS_SCHED_UPDATE_HZ ) - 1 ),
  .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1,
  .RepetitionCounter = 0x0000,
};

static const struct uvos_tim_clock_cfg tim_3_cfg = {
  .timer = TIM3,
  .time_base_init                            = &tim_3_5_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM3_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_5_cfg = {
  .timer = TIM5,
  .time_base_init                            = &tim_3_5_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM5_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_9_cfg = {
  .timer = TIM9,
  .time_base_init                            = &tim_9_10_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_BRK_TIM9_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_10_cfg = {
  .timer = TIM10,
  .time_base_init                            = &tim_9_10_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_UP_TIM10_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

static const struct uvos_tim_clock_cfg tim_11_cfg = {
  .timer = TIM11,
  .time_base_init                            = &tim_11_time_base,
  .irq   = {
    .init                                  = {
      .NVIC_IRQChannel    = TIM1_TRG_COM_TIM11_IRQn,
      .NVIC_IRQChannelPreemptionPriority = UVOS_IRQ_PRIO_MID,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    },
  },
};

#if defined(UVOS_INCLUDE_RCVR)

#include "uvos_rcvr_priv.h"

#endif /* UVOS_INCLUDE_RCVR */