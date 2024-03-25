/*
 * Copyright (C) 2022 Jesús Bautista Villar
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef CONFIG_CRAZYFLIE_H
#define CONFIG_CRAZYFLIE_H

#define BOARD_APOGEE

/* 8MHz external clock and 168MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

/* RED_L, on PC00 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO0
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* GREEN_L, on PC01 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO1
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* GREEN_R, on PC02 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO2
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/* RED_R, on PC03 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOC
#define LED_4_GPIO_PIN GPIO3
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)

/* BLUE_L, on PD02 */
#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO GPIOD
#define LED_5_GPIO_PIN GPIO2
#define LED_5_GPIO_ON gpio_clear
#define LED_5_GPIO_OFF gpio_set
#define LED_5_AFIO_REMAP ((void)0)

/*** UART **********************************************************************/

/* UART2 (E_2) */
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

/* UART3 (E_1) */
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOC
#define UART3_GPIO_RX GPIO11
#define UART3_GPIO_PORT_TX GPIOC
#define UART3_GPIO_TX GPIO10

/* UART6 NRF */ //TODO: Fix it
#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6
#define UART2_GPIO_PORT_CTS GPIOA //
#define UART2_GPIO_CTS GPIO4 //
// #define UART2_GPIO_PORT_RTS GPIOD
// #define UART2_GPIO_RTS GPIO4

/*** I2C *************************************************************/

/* External */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7

/* (IMU, baro) */
#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SDA GPIO9

/**)
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 0
#endif

/*** SPI (External) ************************************************/
#define SPI1_GPIO_AF GPIO_AF5 //
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

// SLAVE0
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO12
// SLAVE1
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO4
// SLAVE2
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO5
// SLAVE3
#define SPI_SELECT_SLAVE3_PORT GPIOB
#define SPI_SELECT_SLAVE3_PIN GPIO8

/*** ADC ***********************************************************************/

// TODO for AUX
// No VBAT monitoring ?

/*** PWM ***********************************************************************/

/*
 * Actuators for fixedwing
 */
/* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

// SERVO DEFINITIONS
#define PWM_USE_TIM2  1
#define PWM_USE_TIM4  1

#define USE_PWM1  1 
#define USE_PWM2  1 
#define USE_PWM3  1 
#define USE_PWM4  1 

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM2
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO1
#define PWM_SERVO_1_AF GPIO_AF1
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM2
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO11
#define PWM_SERVO_2_AF GPIO_AF1
#define PWM_SERVO_2_OC TIM_OC4
#define PWM_SERVO_2_OC_BIT (1<<3)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM2
#define PWM_SERVO_3_GPIO GPIOA
#define PWM_SERVO_3_PIN GPIO15
#define PWM_SERVO_3_AF GPIO_AF1
#define PWM_SERVO_3_OC TIM_OC1
#define PWM_SERVO_3_OC_BIT (1<<0)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM4
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

// servos 1-2-3 on TIM2
#define PWM_TIM2_CHAN_MASK (PWM_SERVO_1_OC_BIT | PWM_SERVO_2_OC_BIT | PWM_SERVO_3_OC_BIT)

// servos 4 on TIM4
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_4_OC_BIT)

// /*
//  * PPM
//  */
// #define USE_PPM_TIM1 1

// #define PPM_CHANNEL         TIM_IC1
// #define PPM_TIMER_INPUT     TIM_IC_IN_TI1
// #define PPM_IRQ             NVIC_TIM1_CC_IRQ
// #define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// // Capture/Compare InteruptEnable and InterruptFlag
// #define PPM_CC_IE           TIM_DIER_CC1IE
// #define PPM_CC_IF           TIM_SR_CC1IF
// #define PPM_GPIO_PORT       GPIOA
// #define PPM_GPIO_PIN        GPIO8
// #define PPM_GPIO_AF         GPIO_AF1

#endif /* CONFIG_CRAZYFLIE_H */
