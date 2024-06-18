/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mc_periph_init.h"
#include "fsl_common.h"
#include "fsl_power.h"
#include "pin_mux.h"
#include "fsl_inputmux_connections.h"
#include "lpc55s36_lowlevel_adc.h"
#include "exported_registers.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedef
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Local functions
 ******************************************************************************/


/*****************************************************************************
*
* Function: void InitPinsRegisters(void)
*
* Description: Manual init of pins, ports by using exported registers from pintool
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void InitPinsRegisters(void)
{
	SYSCON->AHBCLKCTRLSET[0] = INIT_PINS_SYSCON_AHBCLKCTRL0_VALUE;
	IOCON->PIO[0][10] = INIT_PINS_IOCON_PIO0_10_VALUE;
	IOCON->PIO[0][13] = INIT_PINS_IOCON_PIO0_13_VALUE;
	IOCON->PIO[0][14] = INIT_PINS_IOCON_PIO0_14_VALUE;
	IOCON->PIO[0][15] = INIT_PINS_IOCON_PIO0_15_VALUE;
	IOCON->PIO[0][16] = INIT_PINS_IOCON_PIO0_16_VALUE;
	IOCON->PIO[0][17] = INIT_PINS_IOCON_PIO0_17_VALUE;
	IOCON->PIO[0][22] = INIT_PINS_IOCON_PIO0_22_VALUE;
	IOCON->PIO[0][23] = INIT_PINS_IOCON_PIO0_23_VALUE;
	IOCON->PIO[0][29] = INIT_PINS_IOCON_PIO0_29_VALUE;
	IOCON->PIO[0][30] = INIT_PINS_IOCON_PIO0_30_VALUE;
	IOCON->PIO[0][31] = INIT_PINS_IOCON_PIO0_31_VALUE;
	IOCON->PIO[1][17] = INIT_PINS_IOCON_PIO1_17_VALUE;
	IOCON->PIO[1][20] = INIT_PINS_IOCON_PIO1_20_VALUE;
	IOCON->PIO[1][22] = INIT_PINS_IOCON_PIO1_22_VALUE;
	IOCON->PIO[1][28] = INIT_PINS_IOCON_PIO1_28_VALUE;
	IOCON->PIO[1][4] = INIT_PINS_IOCON_PIO1_4_VALUE;
	IOCON->PIO[1][5] = INIT_PINS_IOCON_PIO1_5_VALUE;
	IOCON->PIO[1][6] = INIT_PINS_IOCON_PIO1_6_VALUE;
	IOCON->PIO[1][8] = INIT_PINS_IOCON_PIO1_8_VALUE;

	GPIO->DIR[0] = INIT_PINS_GPIO_DIR0_VALUE;
	GPIO->DIR[1] = INIT_PINS_GPIO_DIR1_VALUE;
	GPIO->PIN[0] = INIT_PINS_GPIO_PIN0_VALUE;
	GPIO->PIN[1] = INIT_PINS_GPIO_PIN1_VALUE;

    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL0_MUX_MASK;
    INPUTMUX->PWM0_FAULT[0]  = INIT_PINS_INPUTMUX_PWM0_FAULT0_VALUE;
    INPUTMUX->ADC0_TRIG[0]   = INIT_PINS_INPUTMUX_ADC0_TRIG0_VALUE;
    INPUTMUX->ENC0_PHASEA    = INIT_PINS_INPUTMUX_ENC0_PHASEA_VALUE;
    INPUTMUX->ENC0_PHASEB    = INIT_PINS_INPUTMUX_ENC0_PHASEB_VALUE;
}

/*****************************************************************************
*
* Function: void InitPWM0(void)
*
* Description: Register based init of PWM for 3-phase center alligned operation
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void InitPWM0(void)
{
	PWM_Type *PWMBase = (PWM_Type *)PWM0;

	/* Enable clock to PWM */
	SYSCON->AHBCLKCTRLSET[3] = SYSCON_AHBCLKCTRL3_PWM0_MASK;
	/* Enable clock to submodules */
	SYSCON->PWM0SUBCTL = (SYSCON_PWM0SUBCTL_CLK0_EN_MASK | SYSCON_PWM0SUBCTL_CLK1_EN_MASK | SYSCON_PWM0SUBCTL_CLK2_EN_MASK | SYSCON_PWM0SUBCTL_CLK3_EN_MASK);
	/* Value register initial values, duty cycle 50% */
	PWMBase->SM[0].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
	PWMBase->SM[1].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
	PWMBase->SM[2].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
	PWMBase->SM[3].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));

	PWMBase->SM[0].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
	PWMBase->SM[1].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
	PWMBase->SM[2].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
	PWMBase->SM[3].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));

	PWMBase->SM[0].VAL2 = 0;
	PWMBase->SM[1].VAL2 = 0;
	PWMBase->SM[2].VAL2 = 0;
	PWMBase->SM[3].VAL2 = 0;

	PWMBase->SM[0].VAL3 = 0;
	PWMBase->SM[1].VAL3 = 0;
	PWMBase->SM[2].VAL3 = 0;
	PWMBase->SM[3].VAL3 = 0;

	/* PWM0 module 0 trigger on VAL4 enabled for ADC synchronization */
	PWMBase->SM[0].VAL4 = PWM_VAL4_VAL4((uint16_t)((-(M1_PWM_MODULO / 2))));
	PWMBase->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(0b010000);

	/* Set deadtime (number of Fast Peripheral Clocks) DTCNT0,1 = T_dead * f_fpc = 0.5us * 150MHz = 75 */
	PWMBase->SM[0].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
	PWMBase->SM[1].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
	PWMBase->SM[2].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
	PWMBase->SM[3].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
	PWMBase->SM[0].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
	PWMBase->SM[1].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
	PWMBase->SM[2].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
	PWMBase->SM[3].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);

	/* Full cycle reload */
	PWMBase->SM[0].CTRL = PWM_CTRL_DBLEN(0)  	/*! DBLEN - Double Switching Enable
											    *  0b0..Double switching disabled.
											    *  0b1..Double switching enabled.
											    */
					    | PWM_CTRL_DBLX(0)  	/*! DBLX - PWMX Double Switching Enable
											    *  0b0..PWMX double pulse disabled.
											    *  0b1..PWMX double pulse enabled.
											    */
						| PWM_CTRL_LDMOD(0)  	/*! LDMOD - Load Mode Select
											    *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
											    *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
											    *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
											    */
						| PWM_CTRL_SPLIT(0)  	/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
											    *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
											    *  0b1..DBLPWM is split to PWMA and PWMB.
											    */
						| PWM_CTRL_PRSC(0) 		/*  PRSC - Prescaler
											    *  0b000..Prescaler 1
											    *  0b001..Prescaler 2
											    *  0b010..Prescaler 4
											    *  0b011..Prescaler 8
											    *  0b100..Prescaler 16
											    *  0b101..Prescaler 32
											    *  0b110..Prescaler 64
											    *  0b111..Prescaler 128
											    */
						| PWM_CTRL_COMPMODE(0)	/*! COMPMODE - Compare Mode
												 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
												 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
												 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
												 *       output in the following period.
												 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
												 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
												 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
												 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
												 */
						| PWM_CTRL_FULL(1)		/*! FULL - Full Cycle Reload
												 *  0b0..Full-cycle reloads disabled.
												 *  0b1..Full-cycle reloads enabled.
												 */
						| PWM_CTRL_HALF(0)		/*! HALF - Half Cycle Reload
												 *  0b0..Half-cycle reloads disabled.
												 *  0b1..Half-cycle reloads enabled.
												 */
						| PWM_CTRL_LDFQ(0);		/*! LDFQ - Load Frequency
												 *  0b0000..Every PWM opportunity
												 *  0b0001..Every 2 PWM opportunities
												 *  0b0010..Every 3 PWM opportunities
												 *  0b0011..Every 4 PWM opportunities
												 *  0b0100..Every 5 PWM opportunities
												 *  0b0101..Every 6 PWM opportunities
												 *  0b0110..Every 7 PWM opportunities
												 *  0b0111..Every 8 PWM opportunities
												 *  0b1000..Every 9 PWM opportunities
												 *  0b1001..Every 10 PWM opportunities
												 *  0b1010..Every 11 PWM opportunities
												 *  0b1011..Every 12 PWM opportunities
												 *  0b1100..Every 13 PWM opportunities
												 *  0b1101..Every 14 PWM opportunities
												 *  0b1110..Every 15 PWM opportunities
												 *  0b1111..Every 16 PWM opportunities
												 */

	/* Full cycle reload */
	PWMBase->SM[1].CTRL = PWM_CTRL_DBLEN(0)  	/*! DBLEN - Double Switching Enable
											    *  0b0..Double switching disabled.
											    *  0b1..Double switching enabled.
											    */
					    | PWM_CTRL_DBLX(0)  	/*! DBLX - PWMX Double Switching Enable
											    *  0b0..PWMX double pulse disabled.
											    *  0b1..PWMX double pulse enabled.
											    */
						| PWM_CTRL_LDMOD(0)  	/*! LDMOD - Load Mode Select
											    *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
											    *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
											    *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
											    */
						| PWM_CTRL_SPLIT(0)  	/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
											    *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
											    *  0b1..DBLPWM is split to PWMA and PWMB.
											    */
						| PWM_CTRL_PRSC(0) 		/*  PRSC - Prescaler
											    *  0b000..Prescaler 1
											    *  0b001..Prescaler 2
											    *  0b010..Prescaler 4
											    *  0b011..Prescaler 8
											    *  0b100..Prescaler 16
											    *  0b101..Prescaler 32
											    *  0b110..Prescaler 64
											    *  0b111..Prescaler 128
											    */
						| PWM_CTRL_COMPMODE(0)	/*! COMPMODE - Compare Mode
												 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
												 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
												 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
												 *       output in the following period.
												 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
												 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
												 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
												 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
												 */
						| PWM_CTRL_FULL(1)		/*! FULL - Full Cycle Reload
												 *  0b0..Full-cycle reloads disabled.
												 *  0b1..Full-cycle reloads enabled.
												 */
						| PWM_CTRL_HALF(0)		/*! HALF - Half Cycle Reload
												 *  0b0..Half-cycle reloads disabled.
												 *  0b1..Half-cycle reloads enabled.
												 */
						| PWM_CTRL_LDFQ(0);		/*! LDFQ - Load Frequency
												 *  0b0000..Every PWM opportunity
												 *  0b0001..Every 2 PWM opportunities
												 *  0b0010..Every 3 PWM opportunities
												 *  0b0011..Every 4 PWM opportunities
												 *  0b0100..Every 5 PWM opportunities
												 *  0b0101..Every 6 PWM opportunities
												 *  0b0110..Every 7 PWM opportunities
												 *  0b0111..Every 8 PWM opportunities
												 *  0b1000..Every 9 PWM opportunities
												 *  0b1001..Every 10 PWM opportunities
												 *  0b1010..Every 11 PWM opportunities
												 *  0b1011..Every 12 PWM opportunities
												 *  0b1100..Every 13 PWM opportunities
												 *  0b1101..Every 14 PWM opportunities
												 *  0b1110..Every 15 PWM opportunities
												 *  0b1111..Every 16 PWM opportunities
												 */

	/* Full cycle reload */
	PWMBase->SM[2].CTRL = PWM_CTRL_DBLEN(0)  	/*! DBLEN - Double Switching Enable
											    *  0b0..Double switching disabled.
											    *  0b1..Double switching enabled.
											    */
					    | PWM_CTRL_DBLX(0)  	/*! DBLX - PWMX Double Switching Enable
											    *  0b0..PWMX double pulse disabled.
											    *  0b1..PWMX double pulse enabled.
											    */
						| PWM_CTRL_LDMOD(0)  	/*! LDMOD - Load Mode Select
											    *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
											    *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
											    *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
											    */
						| PWM_CTRL_SPLIT(0)  	/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
											    *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
											    *  0b1..DBLPWM is split to PWMA and PWMB.
											    */
						| PWM_CTRL_PRSC(0) 		/*  PRSC - Prescaler
											    *  0b000..Prescaler 1
											    *  0b001..Prescaler 2
											    *  0b010..Prescaler 4
											    *  0b011..Prescaler 8
											    *  0b100..Prescaler 16
											    *  0b101..Prescaler 32
											    *  0b110..Prescaler 64
											    *  0b111..Prescaler 128
											    */
						| PWM_CTRL_COMPMODE(0)	/*! COMPMODE - Compare Mode
												 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
												 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
												 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
												 *       output in the following period.
												 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
												 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
												 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
												 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
												 */
						| PWM_CTRL_FULL(1)		/*! FULL - Full Cycle Reload
												 *  0b0..Full-cycle reloads disabled.
												 *  0b1..Full-cycle reloads enabled.
												 */
						| PWM_CTRL_HALF(0)		/*! HALF - Half Cycle Reload
												 *  0b0..Half-cycle reloads disabled.
												 *  0b1..Half-cycle reloads enabled.
												 */
						| PWM_CTRL_LDFQ(0);		/*! LDFQ - Load Frequency
												 *  0b0000..Every PWM opportunity
												 *  0b0001..Every 2 PWM opportunities
												 *  0b0010..Every 3 PWM opportunities
												 *  0b0011..Every 4 PWM opportunities
												 *  0b0100..Every 5 PWM opportunities
												 *  0b0101..Every 6 PWM opportunities
												 *  0b0110..Every 7 PWM opportunities
												 *  0b0111..Every 8 PWM opportunities
												 *  0b1000..Every 9 PWM opportunities
												 *  0b1001..Every 10 PWM opportunities
												 *  0b1010..Every 11 PWM opportunities
												 *  0b1011..Every 12 PWM opportunities
												 *  0b1100..Every 13 PWM opportunities
												 *  0b1101..Every 14 PWM opportunities
												 *  0b1110..Every 15 PWM opportunities
												 *  0b1111..Every 16 PWM opportunities
												 */

	/* Full cycle reload */
	PWMBase->SM[3].CTRL = PWM_CTRL_DBLEN(0)  	/*! DBLEN - Double Switching Enable
											    *  0b0..Double switching disabled.
											    *  0b1..Double switching enabled.
											    */
					    | PWM_CTRL_DBLX(0)  	/*! DBLX - PWMX Double Switching Enable
											    *  0b0..PWMX double pulse disabled.
											    *  0b1..PWMX double pulse enabled.
											    */
						| PWM_CTRL_LDMOD(0)  	/*! LDMOD - Load Mode Select
											    *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
											    *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
											    *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
											    */
						| PWM_CTRL_SPLIT(0)  	/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
											    *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
											    *  0b1..DBLPWM is split to PWMA and PWMB.
											    */
						| PWM_CTRL_PRSC(0) 		/*  PRSC - Prescaler
											    *  0b000..Prescaler 1
											    *  0b001..Prescaler 2
											    *  0b010..Prescaler 4
											    *  0b011..Prescaler 8
											    *  0b100..Prescaler 16
											    *  0b101..Prescaler 32
											    *  0b110..Prescaler 64
											    *  0b111..Prescaler 128
											    */
						| PWM_CTRL_COMPMODE(0)	/*! COMPMODE - Compare Mode
												 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
												 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
												 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
												 *       output in the following period.
												 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
												 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
												 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
												 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
												 */
						| PWM_CTRL_FULL(1)		/*! FULL - Full Cycle Reload
												 *  0b0..Full-cycle reloads disabled.
												 *  0b1..Full-cycle reloads enabled.
												 */
						| PWM_CTRL_HALF(0)		/*! HALF - Half Cycle Reload
												 *  0b0..Half-cycle reloads disabled.
												 *  0b1..Half-cycle reloads enabled.
												 */
						| PWM_CTRL_LDFQ(0);		/*! LDFQ - Load Frequency
												 *  0b0000..Every PWM opportunity
												 *  0b0001..Every 2 PWM opportunities
												 *  0b0010..Every 3 PWM opportunities
												 *  0b0011..Every 4 PWM opportunities
												 *  0b0100..Every 5 PWM opportunities
												 *  0b0101..Every 6 PWM opportunities
												 *  0b0110..Every 7 PWM opportunities
												 *  0b0111..Every 8 PWM opportunities
												 *  0b1000..Every 9 PWM opportunities
												 *  0b1001..Every 10 PWM opportunities
												 *  0b1010..Every 11 PWM opportunities
												 *  0b1011..Every 12 PWM opportunities
												 *  0b1100..Every 13 PWM opportunities
												 *  0b1101..Every 14 PWM opportunities
												 *  0b1110..Every 15 PWM opportunities
												 *  0b1111..Every 16 PWM opportunities
												 */

	/* Fault0 mapping */
	/* Each of the four bits of this field is one-to-one associated with the four FAULTx inputs of fault channel 0 */
	PWMBase->SM[0].DISMAP[0] = PWM_DISMAP_DIS0A(1)
			                 | PWM_DISMAP_DIS0B(1)
							 | PWM_DISMAP_DIS0X(1);
	PWMBase->SM[1].DISMAP[0] = PWM_DISMAP_DIS0A(1)
			                 | PWM_DISMAP_DIS0B(1)
							 | PWM_DISMAP_DIS0X(1);
	PWMBase->SM[2].DISMAP[0] = PWM_DISMAP_DIS0A(1)
			                 | PWM_DISMAP_DIS0B(1)
							 | PWM_DISMAP_DIS0X(1);
	PWMBase->SM[3].DISMAP[0] = PWM_DISMAP_DIS0A(1)
			                 | PWM_DISMAP_DIS0B(1)
							 | PWM_DISMAP_DIS0X(1);

	/* PWMs are re-enabled at PWM full cycle */
	PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x1);

	/* PWM fault filter - 3 Fast periph. clocks sample rate, 5 agreeing samples to activate */
	PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(2);

	/* All interrupts disabled, safe manual fault clearing, inverse logic (trigger level = high) */
	PWMBase->FCTRL &= ~(PWM_FCTRL_FLVL_MASK | PWM_FCTRL_FAUTO_MASK | PWM_FCTRL_FSAFE_MASK | PWM_FCTRL_FIE_MASK);	/* Clear FCTRL register prior further settings */
	PWMBase->FCTRL |= PWM_FCTRL_FIE(0U); /* FAULT 0 & FAULT 1 - Interrupt disable */

	/* Inverse the fault logic (DCB current over-current signal are active high. */
	PWMBase->FCTRL |= PWM_FCTRL_FLVL(0xFU);

	PWMBase->FCTRL |= PWM_FCTRL_FAUTO(0U);
	PWMBase->FCTRL |= PWM_FCTRL_FSAFE(0xFU);

	/* Clear all fault flags */
	PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);

	/* Start PWMs (set load OK flags and run - we need to trigger the ADC) */
	PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF);
	PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF);

}

/*****************************************************************************
*
* Function: void InitADC0(void)
*
* Description: Register based init of ADC0
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void InitADC0(void)
{
	uint32_t GCCa;
	uint32_t GCCb;
	uint32_t GCRa;
	uint32_t GCRb;

	ADC_Type *ADCBase = (ADC_Type *)ADC0;

	/* Clock divider setup */
	/* Reset counter */
	SYSCON->ADC0CLKDIV = SYSCON_ADC0CLKDIV_RESET_MASK;
	/* Set division to 2 */
	SYSCON->ADC0CLKDIV = (2U - 1U);
	/* Attach FRO_HF to ADC0 */
	SYSCON->ADC0CLKSEL =  ADC0_CLK_FRO & SYSCON_ADC0CLKSEL_SEL_MASK;
	/* Disable VREF power down */
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_VREF_MASK;        
        
	/* Init ADC */
	/* Enable the clock. */
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL0_ADC0_MASK;

	/* Reset the module. */
	ADCBase->CTRL |= ADC_CTRL_RST_MASK;          /* ADC logic reset */
	ADCBase->CTRL &= ~ADC_CTRL_RST_MASK;
	ADCBase->CTRL |= ADC_CTRL_RSTFIFO0_MASK;     /* Reset FIFO 0 */
	ADCBase->CTRL |= ADC_CTRL_RSTFIFO1_MASK;     /* Reset FIFO 1 */

	/* Disable the module before setting configuration. */
	ADCBase->CTRL &= ~ADC_CTRL_ADCEN_MASK;

	/* Configure the module generally. */
	ADCBase->CTRL &= ~ADC_CTRL_DOZEN_MASK;

	/* Set calibration average mode. */
	ADCBase->CTRL = ADC_CTRL_CAL_AVGS(7U);

	/* ADCBase configuration. */
	ADCBase->CFG |= ADC_CFG_PWREN_MASK  |  /* ADC analog circuits pre-enable */
			        ADC_CFG_PUDLY(0x80) |  /* Power up delay */
			        ADC_CFG_REFSEL(2U)  |  /* Reference voltage */
			        ADC_CFG_PWRSEL(3U);    /* Power configuration */

	/* Enable the module after setting configuration. */
	ADCBase->CTRL |= ADC_CTRL_ADCEN_MASK;

	/* Offset calibration. */
	ADCBase->CTRL |= ADC_CTRL_CALOFS_MASK;
	while (ADC_STAT_CAL_RDY_MASK != (ADCBase->STAT & ADC_STAT_CAL_RDY_MASK))
	{
	}

	/* AutoCalibration */
	assert((0u == ((ADC_FCTRL_FCOUNT_MASK & ADCBase->FCTRL[0]) >> ADC_FCTRL_FCOUNT_SHIFT))
		&& (0u == ((ADC_FCTRL_FCOUNT_MASK & ADCBase->FCTRL[1]) >> ADC_FCTRL_FCOUNT_SHIFT)));

	/* Request gain calibration. */
	ADCBase->CTRL |= ADC_CTRL_CAL_REQ_MASK;
	while ((ADC_GCC_RDY_MASK != (ADCBase->GCC[0] & ADC_GCC_RDY_MASK)) ||
		   (ADC_GCC_RDY_MASK != (ADCBase->GCC[1] & ADC_GCC_RDY_MASK)))
	{
	}

	/* Calculate gain offset. */
	GCCa = (ADCBase->GCC[0] & ADC_GCC_GAIN_CAL_MASK);
	GCCb = (ADCBase->GCC[1] & ADC_GCC_GAIN_CAL_MASK);

	GCRa = (uint16_t)((GCCa << 16U) / (0x1FFFFU - GCCa));
	/* Gain_CalA = (131072 / (131072-(ADC_GCC_GAIN_CAL(ADCBase->GCC[0])) - 1. */
	GCRb = (uint16_t)((GCCb << 16U) / (0x1FFFFU - GCCb));
	/* Gain_CalB = (131072 / (131072-(ADC_GCC_GAIN_CAL(ADCBase->GCC[1])) - 1. */

	ADCBase->GCR[0] = ADC_GCR_GCALR(GCRa);
	ADCBase->GCR[1] = ADC_GCR_GCALR(GCRb);

	/* Indicate the values are valid. */
	ADCBase->GCR[0] |= ADC_GCR_RDY_MASK;
	ADCBase->GCR[1] |= ADC_GCR_RDY_MASK;
	while (ADC_STAT_CAL_RDY_MASK != (ADCBase->STAT & ADC_STAT_CAL_RDY_MASK))
	{
	}

	/* Init commands */
	/* Configure the CMD 1 */
	ADCBase->CMD[ACT_CMD1].CMDL = ADC_CMDL_ADCH(3U)     /* IA ADC0_3A  IC ADC0_3B */
	                            | ADC_CMDL_CTYPE(CMDL_CTYPE_DUAL_SINGLE_AB)
							    | ADC_CMDL_MODE(CMDL_MODE_STANDARD)
							    | ADC_CMDL_ALTB_ADCH(0U)
							    | ADC_CMDL_ALTBEN(0);
	ADCBase->CMD[ACT_CMD1].CMDH = ADC_CMDH_CMPEN(0)
			        		    | ADC_CMDH_WAIT_TRIG(0)
							    | ADC_CMDH_LWI(0)
							    | ADC_CMDH_STS(CMDH_SAMPLETIME_3_5)
							    | ADC_CMDH_AVGS(CMDH_AVERAGE_1)
							    | ADC_CMDH_LOOP(0)
							    | ADC_CMDH_NEXT(NEXT_CMD2);

	ADCBase->CMD[ACT_CMD2].CMDL = ADC_CMDL_ADCH(8U)     /* IB ADC0_8A ADC0_8B */
	                            | ADC_CMDL_CTYPE(CMDL_CTYPE_SINGLE_ENDED_A)
							    | ADC_CMDL_MODE(CMDL_MODE_STANDARD)
							    | ADC_CMDL_ALTB_ADCH(0U)
							    | ADC_CMDL_ALTBEN(0);
	ADCBase->CMD[ACT_CMD2].CMDH = ADC_CMDH_CMPEN(0)
			        		    | ADC_CMDH_WAIT_TRIG(0)
							    | ADC_CMDH_LWI(0)
							    | ADC_CMDH_STS(CMDH_SAMPLETIME_3_5)
							    | ADC_CMDH_AVGS(CMDH_AVERAGE_1)
							    | ADC_CMDH_LOOP(0)
							    | ADC_CMDH_NEXT(NEXT_CMD3);

	ADCBase->CMD[ACT_CMD3].CMDL = ADC_CMDL_ADCH(1U)     /* VDCB ADC0_1A */
	                            | ADC_CMDL_CTYPE(CMDL_CTYPE_SINGLE_ENDED_A)
							    | ADC_CMDL_MODE(CMDL_MODE_STANDARD)
							    | ADC_CMDL_ALTB_ADCH(0U)
							    | ADC_CMDL_ALTBEN(0);
	ADCBase->CMD[ACT_CMD3].CMDH = ADC_CMDH_CMPEN(0)
			        		    | ADC_CMDH_WAIT_TRIG(0)
							    | ADC_CMDH_LWI(0)
							    | ADC_CMDH_STS(CMDH_SAMPLETIME_3_5)
							    | ADC_CMDH_AVGS(CMDH_AVERAGE_1)
							    | ADC_CMDH_LOOP(0)
							    | ADC_CMDH_NEXT(NEXT_CMD4);

	ADCBase->CMD[ACT_CMD4].CMDL = ADC_CMDL_ADCH(4U)     /* IDCB ADC0_4B */
	                            | ADC_CMDL_CTYPE(CMDL_CTYPE_SINGLE_ENDED_B)
							    | ADC_CMDL_MODE(CMDL_MODE_STANDARD)
							    | ADC_CMDL_ALTB_ADCH(4U)
							    | ADC_CMDL_ALTBEN(0);
	ADCBase->CMD[ACT_CMD4].CMDH = ADC_CMDH_CMPEN(0)
			        		    | ADC_CMDH_WAIT_TRIG(0)
							    | ADC_CMDH_LWI(0)
							    | ADC_CMDH_STS(CMDH_SAMPLETIME_3_5)
							    | ADC_CMDH_AVGS(CMDH_AVERAGE_1)
							    | ADC_CMDH_LOOP(0)
							    | ADC_CMDH_NEXT(NEXT_CMD0);

	/* TCTRL - Trigger Control Register */
	ADCBase->TCTRL[0] = ADC_TCTRL_TCMD(1U)      |  		/*! TCMD - Trigger Command Select
			                                           	 *  0b0000..Not a valid selection from the command buffer. Trigger event is ignored.
			                                             *  0b0001..CMD1
			                                             *  0b0010-0b1110..Corresponding CMD is executed
			                                             *  0b1111..CMD15
			                                             */
						ADC_TCTRL_TDLY(0U)      |     	/*! TDLY - Trigger Delay Select */
						ADC_TCTRL_RSYNC(0U)		|		/*! RSYNC - Trigger Resync
						                                 *  0b0..Disable
						                                 *  0b1..Enable
						                                 */
						ADC_TCTRL_TPRI(0U)		|		/*! TPRI - Trigger Priority Setting
						                                 *  0b00..Highest priority, Level 1
						                                 *  0b01-0b10..Set to corresponding priority level.
						                                 *  0b11..Lowest priority, Level 4
						                                 */
						ADC_TCTRL_FIFO_SEL_B(1U)|		/*! FIFO_SEL_B - SAR Result Destination for Channel B
														 *  0b0..FIFO 0
														 *  0b1..FIFO 1
						 							     */
						ADC_TCTRL_FIFO_SEL_A(0U)|		/*! FIFO_SEL_A - SAR Result Destination for Channel A
						                                 *  0b0..FIFO 0
						                                 *  0b1..FIFO 1
						                                 */
						ADC_TCTRL_HTEN(1U);    			/*! HTEN - Trigger Enable
													     *  0b0..Disabled
													     *  0b1..Enabled
													     */

	/* Set watermark for FIFO0 */
	ADCBase->FCTRL[0] |=ADC_FCTRL_FWMARK(3);
	/* Enable FIFO0 watermark interrupt */
	ADCBase->IE |= ADC_IE_FWMIE0(1);
	NVIC_SetPriority(ADC0_IRQn, 1U);
	NVIC_EnableIRQ(ADC0_IRQn);
}

/*****************************************************************************
*
* Function: void InitHSCMP0(void)
*
* Description: Register based init of the comparator 0 module for dc-bus over current
*              detection to generate eFlexPWM0 fault
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void InitHSCMP0(void)
{
	/* Enable clock */
	SYSCON->AHBCLKCTRLSET[3] = SYSCON_AHBCLKCTRL3_HSCMP0_MASK;
	/* Disable power down */
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_VREF_MASK
					  | PMC_PDRUNCFG0_PDEN_HSCMP0_MASK;
	PMC->PDRUNCFGCLR1 = PMC_PDRUNCFG1_PDEN_CMPBIAS_MASK
					  | PMC_PDRUNCFG1_PDEN_HSCMP0_DAC_MASK;

	HSCMP0->CCR0 =	HSCMP_CCR0_LINKEN(0U)     | /*! LINKEN - CMP-to-DAC Link Enable
	                                             *  0b0..Disable the CMP-to-DAC link: enabling or disabling the DAC is independent from enabling or disabling the CMP.
	                                             *  0b1..Enable the CMP-to-DAC link: the DAC enable/disable is controlled by the CMP_EN bit instead of DCR[DAC_EN].
	                                             */
					HSCMP_CCR0_CMP_STOP_EN(0U)| /*! CMP_STOP_EN - Comparator STOP Mode Enable
					                             *  0b0..Disable the analog comparator regardless of CMP_EN.
					                             *  0b1..Allow the analog comparator to be enabled by CMP_EN.
					                             */
					HSCMP_CCR0_CMP_EN(0);       /*! CMP_EN - Comparator Enable
					                             *  0b0..Disable (The analog logic remains off and consumes no power.)
					                             *  0b1..Enable
					                             */

	HSCMP0->CCR1 =  HSCMP_CCR1_FILT_PER(0U) |	/*! FILT_PER - Filter Sample Period */
					HSCMP_CCR1_FILT_CNT(0U) |	/*! FILT_CNT - Filter Sample Count
					                             *  0b000..Filter is bypassed: COUT = COUTA
					                             *  0b001..1 consecutive sample (Comparator output is simply sampled.)
					                             *  0b010..2 consecutive samples
					                             *  0b011..3 consecutive samples
					                             *  0b100..4 consecutive samples
					                             *  0b101..5 consecutive samples
					                             *  0b110..6 consecutive samples
					                             *  0b111..7 consecutive samples
					                             */
					HSCMP_CCR1_EVT_SEL(0U)  |	/*! EVT_SEL - COUT Event Select
					                             *  0b00..Rising edge
					                             *  0b01..Falling edge
					                             *  0b1x..Both edges
					                             */
					HSCMP_CCR1_WINDOW_CLS(0U) | /*! WINDOW_CLS - COUT Event Window Close
					                             *  0b0..COUT event cannot close the window
					                             *  0b1..COUT event can close the window
					                             */
					HSCMP_CCR1_WINDOW_INV(0U) | /*! WINDOW_INV - WINDOW/SAMPLE Signal Invert
					                             *  0b0..Do not invert
					                             *  0b1..Invert
					                             */
					HSCMP_CCR1_COUTA_OW(0U) |   /*! COUTA_OW - COUTA Output Level for Closed Window
					                             *  0b0..COUTA is 0
					                             *  0b1..COUTA is 1
					                             */
					HSCMP_CCR1_COUTA_OWEN(0U) | /*! COUTA_OWEN - COUTA_OW Enable
					                             *  0b0..COUTA holds the last sampled value
					                             *  0b1..COUTA is defined by the COUTA_OW bit
					                             */
					HSCMP_CCR1_COUT_PEN(0U) |   /*! COUT_PEN - Comparator Output Pin Enable
					                             *  0b0..Not available
					                             *  0b1..Available
					                             */
					HSCMP_CCR1_COUT_SEL(0U) |   /*! COUT_SEL - Comparator Output Select
					                             *  0b0..Use COUT (filtered)
					                             *  0b1..Use COUTA (unfiltered)
					                             */
					HSCMP_CCR1_COUT_INV(0U) |   /*! COUT_INV - Comparator Invert
					                             *  0b0..Do not invert
					                             *  0b1..Invert
					                             */
					HSCMP_CCR1_DMA_EN(0U)   |   /*! DMA_EN - DMA Enable
					                             *  0b0..Disable
					                             *  0b1..Enable
					                             */
					HSCMP_CCR1_SAMPLE_EN(0U)|   /*! SAMPLE_EN - Sampling Enable
					                             *  0b0..Disable
					                             *  0b1..Enable
					                             */
					HSCMP_CCR1_WINDOW_EN(0U);   /*! WINDOW_EN - Windowing Enable
					                             *  0b0..Disable
					                             *  0b1..Enable
					                             */

	HSCMP0->CCR2  = HSCMP_CCR2_INMSEL(0)   |	/*! INMSEL - Input Minus Select
	                                             *  0b00..IN0: from the 8-bit DAC output
	                                             *  0b01..IN1: from the analog 8-1 mux
	                                             *  0b10..Reserved
	                                             *  0b11..Reserved
	                                             */
			        HSCMP_CCR2_INPSEL(1)   |	/*! INPSEL - Input Plus Select
			                                     *  0b00..IN0: from the 8-bit DAC output
			                                     *  0b01..IN1: from the analog 8-1 mux
			                                     *  0b10..Reserved
			                                     *  0b11..Reserved
			                                     */
					HSCMP_CCR2_MSEL(7U)    |	/*! MSEL - Minus Input MUX Select
					                             *  0b000..Input 0m
					                             *  0b001..Input 1m
					                             *  0b010..Input 2m
					                             *  0b011..Input 3m
					                             *  0b100..Input 4m
					                             *  0b101..Input 5m
					                             *  0b110..Reserved
					                             *  0b111..Internal DAC output
					                             */
					HSCMP_CCR2_PSEL(3U)    |	/*! PSEL - Plus Input MUX Select
					                             *  0b000..Input 0p
					                             *  0b001..Input 1p
					                             *  0b010..Input 2p
					                             *  0b011..Input 3p
					                             *  0b100..Input 4p
					                             *  0b101..Input 5p
					                             *  0b110..Reserved
					                             *  0b111..Internal DAC output
					                             */
					HSCMP_CCR2_HYSTCTR(0U) |	/*! HYSTCTR - Comparator Hysteresis Control
				                                 *  0b00..Level 0
				                                 *  0b01..Level 1
				                                 *  0b10..Level 2
				                                 *  0b11..Level 3
				                                 */
					HSCMP_CCR2_OFFSET(0U)  |	/*! OFFSET - Comparator Offset Control
					                             *  0b0..Level 0: The hysteresis selected by HYSTCTR is valid for both directions (rising and falling).
					                             *  0b1..Level 1: Hysteresis does not apply when INP (input-plus) crosses INM (input-minus) in the rising
					                             *                direction or when INM crosses INP in the falling direction. Hysteresis still applies
					                             *                for INP crossing INM in the falling direction.
					                             */
					HSCMP_CCR2_CMP_NPMD(0U)| 	/*! CMP_NPMD - CMP Nano Power Mode Select
					                             *  0b0..Disable (Mode is determined by CMP_HPMD.)
					                             *  0b1..Enable
					                             */
					HSCMP_CCR2_CMP_HPMD(1U);	/*! CMP_HPMD - CMP High Power Mode Select
					                             *  0b0..Low power(speed) comparison mode
					                             *  0b1..High power(speed) comparison mode
					                             */


	HSCMP0->DCR = HSCMP_DCR_DAC_DATA(OVERCURRENT(7.5F)) |   /* HSCMP current threshold 7.5A. */
			      HSCMP_DCR_DACOE(1U)       			|	/*! DACOE - DAC Output Enable
			                                                 *  0b0..Disable
			                                                 *  0b1..Enable
			                                                 */
				  HSCMP_DCR_VRSEL(0U)					|   /*! VRSEL - DAC Reference High Voltage Source Select
				                                             *  0b0..vrefh0
				                                             *  0b1..vrefh1
				                                             */
			      HSCMP_DCR_DAC_HPMD(1)                 |   /*! DAC_HPMD - DAC High Power Mode Select
			                                                 *  0b0..Disable
			                                                 *  0b1..Enable
			                                                 */
			      HSCMP_DCR_DAC_EN(1);                      /*! DAC_EN - DAC Enable
			                                                 *  0b0..Disable
			                                                 *  0b1..Enable
			                                                 */
	/* HSCMP enable */
	HSCMP0->CCR0 |= HSCMP_CCR0_CMP_EN(1U);
}

/*****************************************************************************
*
* Function: void InitSlowLoop(void)
*
* Description: Register based init of the CTIMER0 peripheral.
*              Performs slow control loop counter ISR.
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void InitSlowLoop(void)
{
    /* Use 96 MHz clock for some of the Ctimer0. */
    SYSCON->CTIMERCLKSEL[0] =  SYSCON_CTIMERCLKSEL_SEL(CTIMER_CLK_FRO96M);
    /* Enable Timer0 clock. */
    SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL1_TIMER0_MASK;

    /* Enable Timer0 clock reset. */
    SYSCON->PRESETCTRLSET[1] = SYSCON_PRESETCTRL1_TIMER0_RST_MASK;             /* Set bit. */

    while (0u == (SYSCON->PRESETCTRL1 & SYSCON_PRESETCTRL1_TIMER0_RST_MASK))   /* Wait until it reads 0b`1 */
    {
    }

    /* Clear Timer0 clock reset. */
    SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL1_TIMER0_RST_MASK;             /* Clear bit */
    while (SYSCON_PRESETCTRL1_TIMER0_RST_MASK ==                               /* Wait until it reads 0b0 */
          (SYSCON->PRESETCTRL1 & SYSCON_PRESETCTRL1_TIMER0_RST_MASK))
    {
    }

    /* Configure match control register. */
    CTIMER0->MCR |= CTIMER_MCR_MR0R(1U)  |   /* Enable reset of TC after it matches with MR0. */
                    CTIMER_MCR_MR0I(1U);     /* Enable interrupt generation after TC matches with MR0. */
    
    /* Configure match register. */
    CTIMER0->MR[0] = (CLOCK_GetFreq(kCLOCK_FroHf))  /* Get CTimer0 frequency for correct set Match register value. */
                     / M1_SLOW_LOOP_FREQ;           /* Set slow control loop frequency in Hz. */
    
    /* Configure interrupt register. */
    CTIMER0->IR = CTIMER_IR_MR0INT_MASK;     /* Set interrupt flag for match channel 0. */
    NVIC_SetPriority(CTIMER0_IRQn, 2U);
    NVIC_EnableIRQ(CTIMER0_IRQn);            /* Enable LEVEL1 interrupt and update the call back function. */

    /* Configure timer control register. */
    CTIMER0->TCR |= CTIMER_TCR_CEN_MASK;     /* Start the timer counter. */
}





