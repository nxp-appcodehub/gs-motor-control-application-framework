/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "mlib_FP.h"
#include "motor_control.h"

#define PINTOOL_PINMUX			1
#define PINTOOL_EXP_REGISTERS	2

#define PIN_INIT_STYLE			PINTOOL_PINMUX


typedef void (*tPointerFcn)(void); /* pointer to function */

extern volatile uint8_t ui8MotorState;

/* Enable PWM outputs */
#define PWM_EN(x)        (x->OUTEN |= (PWM_OUTEN_PWMA_EN(0xF) | PWM_OUTEN_PWMB_EN(0xF)))
/* Disable PWM outputs */
#define PWM_DIS(x)       (x->OUTEN &= ~(PWM_OUTEN_PWMA_EN(0xF) | PWM_OUTEN_PWMB_EN(0xF)))

typedef struct
{
	frac16_t f16Ia;
	frac16_t f16Ib;
	frac16_t f16Ic;
	frac16_t f16Idcb;
	frac16_t f16Vdcb;
} sAdcResult;

typedef struct
{
	uint32_t ui32Ia;
	uint32_t ui32Ib;
	uint32_t ui32Ic;
	uint32_t ui32Idcb;
	uint32_t ui32Vdcb;
} sAdcRawResult;

typedef struct
{
	frac16_t f16DutyA;
	frac16_t f16DutyB;
	frac16_t f16DutyC;
} sMotorDuty;

/* Define state number of application state machine */
#define APP_ST_INIT            0
#define APP_ST_STOP            1
#define APP_ST_START           2
#define APP_ST_RUN             3
#define APP_ST_ERROR           4


/* Three instruction added after interrupt flag clearing as required */
#define M1_END_OF_ISR \
    {                 \
        __DSB();      \
        __ISB();      \
    }

#endif /* MAIN_H_ */
