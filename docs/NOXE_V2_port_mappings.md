/* See STM32F4yyxx Reference Manual, Section 2.3 Memory map
STM32F411xE   : HCLK= 96 MHz, PCLK1(APB1)=48 MHz, PCLK2(APB2)=96 MHz
								APB1 (x2 multiplier) Timer Clocks = 96 MHz
								APB2 (x1 multiplier) Timer Clocks = 96 MHz

STM32F405xG   : HCLK=168 MHz, PCLK1(APB1)=84 MHz, PCLK2(APB2)=168 MHz

// Reference STM32 datasheet processor block diagram
For SPI_1, SPI_4, SPI_5 and SPI_6
  - CLK source is PCKL2
For SPI_2 and SPI_3
  - CLK source is PCKL1

F411
For TIM_1, TIM_9, TIM_10, TIM_11
  - CLK source is PCKL2 x1 = 96 MHz
For TIM_2, TIM_3, TIM_4, TIM_5
  - CLK source is PCKL1 x2 = 96 MHz

F405
For TIM_1, TIM_8, TIM_9, TIM_10, TIM_11
  - CLK source is PCKL2
For TIM_2, TIM_3, TIM_4, TIM_5, TIM_12, TIM_13, TIM_14
  - CLK source is PCKL1

*/

Servo out header

Servo out 1: PA8  - TIM1_CH1 (29)
Servo out 2: PA9  - TIM1_CH2 (30)
Servo out 3: PA10 - TIM1_CH3 (31)
Servo out 4: PB0  - TIM3_CH3 (18)
Servo out 5: PB4  - TIM3_CH1 (40)
Servo out 6: PB5  - TIM3_CH2 (41)

const act_cfg_t act_cfg_param[ ACT_NUM ] = {
	[ 0 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.Pin = LL_GPIO_PIN_8,
				// .Mode = LL_GPIO_MODE_OUTPUT,
				// .Speed = LL_GPIO_SPEED_FREQ_LOW,
				// .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
				// .Pull = LL_GPIO_PULL_NO,
				// .Alternate = LL_GPIO_AF_0,
			},
		},
		.TIM_Inst = TIM1,
		.TIM_CCRx = &TIM1->CCR1,
		.TIM_CH = LL_TIM_CHANNEL_CH1,
		.TIM_CLK = 96000000,
	},
	[ 1 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.Pin = LL_GPIO_PIN_9,
			},
		},
		.TIM_Inst = TIM1,
		.TIM_CCRx = &TIM1->CCR2,
		.TIM_CH = LL_TIM_CHANNEL_CH2,
		.TIM_CLK = 96000000,
	},
	[ 2 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.Pin = LL_GPIO_PIN_10,
			},
		},
		.TIM_Inst = TIM1,
		.TIM_CCRx = &TIM1->CCR3,
		.TIM_CH = LL_TIM_CHANNEL_CH3,
		.TIM_CLK = 96000000,
	},
	[ 3 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.Pin = LL_GPIO_PIN_0,
			},
		},
		.TIM_Inst = TIM3,
		.TIM_CCRx = &TIM3->CCR3,
		.TIM_CH = LL_TIM_CHANNEL_CH3,
		.TIM_CLK = 96000000,
	},
	[ 4 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.Pin = LL_GPIO_PIN_4,
			},
		},
		.TIM_Inst = TIM3,
		.TIM_CCRx = &TIM3->CCR1,
		.TIM_CH = LL_TIM_CHANNEL_CH1,
		.TIM_CLK = 96000000,
	},
	[ 5 ] = {
		.type = ePWM,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.Pin = LL_GPIO_PIN_5,
			},
		},
		.TIM_Inst = TIM3,
		.TIM_CCRx = &TIM3->CCR2,
		.TIM_CH = LL_TIM_CHANNEL_CH2,
		.TIM_CLK = 96000000,
	},
};