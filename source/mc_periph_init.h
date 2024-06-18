/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MC_PERIPH_INIT_H_
#define _MC_PERIPH_INIT_H_

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "app_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Independent register definitions for baremetal port init */

#define ADC0_CLK_MAIN 		    0U
#define ADC0_CLK_PLL0 		    1U
#define ADC0_CLK_FRO 		    2U
#define ADC0_CLK_XO 		    4U

#define CTIMER_CLK_MAIN 	    0U
#define CTIMER_CLK_PLL0 	    1U
#define CTIMER_CLK_PLL1 	    2U
#define CTIMER_CLK_FRO96M 	    3U
#define CTIMER_CLK_FRO1M 	    4U
#define CTIMER_CLK_MCLK 	    5U
#define CTIMER_CLK_OSC32k 	    6U
#define CTIMER_CLK_NoClk 	    7U

#define ADC1_CLK_MAIN 		    0U
#define ADC1_CLK_PLL0 		    1U
#define ADC1_CLK_FRO 		    2U
#define ADC1_CLK_XO 		    4U

/* Define port and pins for serial communication */
#define PORT_USART0             0U
#define PIN_USART_TX            30U             /* USART0_TX */
#define PIN_USART_RX            29U             /* USART0_RX */



#define OVERCURRENT(x)      (uint8_t)((x / I_MAX_SCALE) * (float)255U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void InitPinsRegisters(void);
void InitADC0(void);
void InitHSCMP0(void);
void InitPWM0(void);
void InitSlowLoop(void);
void InitClock(void);


/******************************************************************************
 * Timing
 ******************************************************************************/
/* MCU core clock in Hz */
#define MCU_CLOCK_FREQ          (150000000U)
/* PWM frequency in Hz*/
#define M1_PWM_FREQ             (10000U)
/* PWM modulo = FTM_input_clock / M1_PWM_FREQ */
#define M1_PWM_MODULO           (MCU_CLOCK_FREQ / M1_PWM_FREQ)
/* Output PWM deadtime value in nanoseconds */
#define M1_PWM_DEADTIME (500)
/* PWM vs. Fast control loop ratio */
#define M1_FOC_FREQ_VS_PWM_FREQ (1U)
/* Slow control loop frequency */
#define M1_SLOW_LOOP_FREQ       (1000U)

#endif /* _MC_PERIPH_INIT_H_  */
