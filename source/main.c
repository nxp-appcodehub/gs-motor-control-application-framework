/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "mc_periph_init.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_usart.h"
#include "motor_control.h"

#define ENABLE_FREEMASTER	0

#if ENABLE_FREEMASTER
#include "freemaster_serial_usart.h"
#include "freemaster.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t ui32Ctimer0IsrCnt = 0;
uint32_t ui32ADC0IsrCnt = 0;

#if ENABLE_FREEMASTER
static FMSTR_U8 FreeMASTER_RecBuffer0[2048];

FMSTR_REC_BUFF FreeMASTER_Recorder_0 =
{
  .name = "Description of recorder 0",
  .addr = (FMSTR_ADDR)FreeMASTER_RecBuffer0,
  .size = sizeof(FreeMASTER_RecBuffer0),
  .basePeriod_ns = 100000UL
};
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void InitUSART0(void);
static void InitFreeMASTER(void);
static void UpdatePWM(frac16_t f16PhA, frac16_t f16PhB, frac16_t f16PhC, PWM_Type *pPWMBase);
static void SyncAdcPwm(void);
static tPointerFcn AppStateMachineFast[] = {AppInitFast,AppStopFast,AppStartFast,AppRunFast,AppErrorFast};
static tPointerFcn AppStateMachineSlow[] = {AppInitSlow,AppStopSlow,AppStartSlow,AppRunSlow,AppErrorSlow};
/*******************************************************************************
 * Code
 ******************************************************************************/

/*****************************************************************************
*
* Function: int main(void)
*
* Description: Application main function processing peripheral function
* 			   calling and infinite loop with FreeMASTER
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
int main(void)
{
    uint32_t ui32PrimaskReg;
    ui32PrimaskReg = DisableGlobalIRQ(); 	/* Disable interrupts */
    BOARD_BootClockPLL150M();				/* Set core clock */
#if PIN_INIT_STYLE == PINTOOL_PINMUX
    BOARD_Init_Pins();	    				/* Init pins and inputmux from pintool code generation */
#elif PIN_INIT_STYLE == PINTOOL_EXP_REGISTERS
	InitPinsRegisters();	                /* Init pins and inputmux from pintool exported registers */
#endif
	RTCESL_PQ_Init();
	InitSlowLoop();     	                /* Init slow loop timer */
	InitADC0();         	                /* Init ADC0 */
	InitHSCMP0();       	                /* Init High Speed Comparator 0 */
	InitPWM0();         	                /* 6-channel PWM0 peripheral init */
	InitUSART0();       	                /* FreeMASTER communication layer initialization */
	InitFreeMASTER();   	                /* FreeMASTER middleware initialization */
    ui8MotorState = APP_ST_INIT;			/* Initial state */
    SyncAdcPwm();							/* Re-sync ADC-PWM */
    EnableGlobalIRQ(ui32PrimaskReg);		/* Enable interrupts  */
    /* Infinite loop */
    while (1)
    {
#if ENABLE_FREEMASTER
        FMSTR_Poll();						/* FreeMASTER Polling function */
#endif
    }
}


/*****************************************************************************
*
* Function: void CTIMER0_IRQHandler(void)
*
* Description: Ctimer Slow loop interrupt handler (1ms period)
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void CTIMER0_IRQHandler(void)
{
	ui32Ctimer0IsrCnt++;    				/* Isr check counter */
    CTIMER0->IR |= CTIMER_IR_MR0INT(1U);    /* Clear the match interrupt flag. */
	AppStateMachineSlow[ui8MotorState]();	/* Call state machine */
    M1_END_OF_ISR;							/* Add empty instructions for correct interrupt flag clearing */
}

/*****************************************************************************
*
* Function: void ADC0_IRQHandler(void)
*
* Description: ADC Fast loop interrupt handler
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void ADC0_IRQHandler(void)
{
    ui32ADC0IsrCnt++;						/* Isr check counter */
    ADC0->STAT |= ADC_STAT_FOF0_MASK;		/* Clear the FIFO INT flag */
    AdcGetResults();    					/* Get ADC0 results */
    ScaleValues();							/* Scale, convert to float */
	AppStateMachineFast[ui8MotorState]();	/* Call state machine */
#if ENABLE_FREEMASTER
    FMSTR_Recorder(0);						/* Call FreeMASTER recorder */
#endif
    M1_END_OF_ISR;							/* Add empty instructions for correct interrupt flag clearing */
}

/*****************************************************************************
*
* Function: static void InitUSART0(void)
*
* Description: USART Module initialization for FreeMASTER
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
static void InitUSART0(void)
{
    usart_config_t config;

    /* Attach main clock divide to FLEXCOMM0 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

    USART_GetDefaultConfig(&config);
    /* Override the Default configuration to satisfy FreeMASTER needs */
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;
    /* Clock for USART0 peripheral is 12 MHz */
    USART_Init((USART_Type*)USART0, &config, 12000000U);
    /* Register communication module used by FreeMASTER driver. */
#if ENABLE_FREEMASTER
    FMSTR_SerialSetBaseAddress((USART_Type*)USART0);
#endif

}

/*****************************************************************************
*
* Function: static void InitFreeMASTER(void)
*
* Description: FreeMASTER middleware initialization
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
static void InitFreeMASTER(void)
{
#if ENABLE_FREEMASTER
	FMSTR_Init();									/* FreeMASTER middleware initialization */
	FMSTR_RecorderCreate(0, &FreeMASTER_Recorder_0);/* FreeMASTER recorder 0 configuration initialization  */
#endif
}

/*****************************************************************************
*
* Function: static void SyncAdcPwm(void)
*
* Description: PWM to ADC sync function
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
static void SyncAdcPwm(void)
{
	ADC0->CTRL |= ADC_CTRL_RSTFIFO0_MASK;	/* Reset FIFO 0 */
	ADC0->CTRL |= ADC_CTRL_RSTFIFO1_MASK;	/* Reset FIFO 1 */
    PWM0->MCTRL|= PWM_MCTRL_RUN(0xF);		/* Run PWM */
}

