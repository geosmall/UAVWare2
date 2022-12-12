/* See STM32F4yyxx Reference Manual, Section 2.3 Memory map
STM32F411xE   : HCLK= 96MHz, PCLK1(APB1)=48MHz, PCLK2(APB2)=96MHz
STM32F405xG   : HCLK=168MHz, PCLK1(APB1)=84MHz, PCLK2(APB2)=168MHz

// Reference STM32 datasheet processor block diagram
For SPI_1, SPI_4, SPI_5 and SPI_6
  - CLK source is PCKL2
For SPI_2 and SPI_3
  - CLK source is PCKL1

F411
For TIM_1, TIM_9, TIM_10, TIM_11
  - CLK source is PCKL2
For TIM_2, TIM_3, TIM_4, TIM_5
  - CLK source is PCKL1

F405
For TIM_1, TIM_8, TIM_9, TIM_10, TIM_11
  - CLK source is PCKL2
For TIM_2, TIM_3, TIM_4, TIM_5, TIM_12, TIM_13, TIM_14
  - CLK source is PCKL1

*/

Flex-IO Port Functions (10 Pin)

1:  GND
2:  VCC_UNREG
3:  PB12 - CAN2 RX (33)
4:  PB13 - CAN2 TX (34)
5:  PB14 - TIM12_CH1 (35)
6:  PB15 - TIM12_CH2 (36)
7:  PC6  - TIM8_CH1 (37)
8:  PC7  - TIM8_CH2 (38)
9:  PC8  - TIM8_CH3 (39)
10: PC9  - TIM8_CH4 (40)

Servo out header (6x3 Pin)

Servo out 1: PB0 - TIM3_CH3 (26)
Servo out 2: PB1 - TIM3_CH4 (27)
Servo out 3: PA3 - TIM9_CH2 (17)
Servo out 4: PA2 - TIM2_CH3 (16)
Servo out 5: PA1 - TIM5_CH2 (15)
Servo out 6: PA0 - TIM5_CH1 (14)
