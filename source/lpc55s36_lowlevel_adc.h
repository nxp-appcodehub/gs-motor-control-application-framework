/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LPC55S36_LOWLEVEL_ADC_H_
#define LPC55S36_LOWLEVEL_ADC_H_

#include "fsl_device_registers.h"


#define  CMDL_CTYPE_SINGLE_ENDED_A		0b00	/* Single-Ended mode. Only A-side channel is converted.*/
#define  CMDL_CTYPE_SINGLE_ENDED_B 		0b01	/* Single-Ended mode. Only B-side channel is converted.*/
#define  CMDL_CTYPE_DIFFERENTIAL_AB  	0b10	/* Differential mode. A-B.*/
#define  CMDL_CTYPE_DUAL_SINGLE_AB 		0b11	/* Dual-Single-Ended mode. Both A-side and B-side channels are converted independently.*/

#define  CMDL_MODE_STANDARD				0b0		/* Standard resolution. Single-ended 12-bit conversion; differential 13-bit conversion with 2's complement output.*/
#define  CMDL_MODE_HIRES				0b1     /* High resolution. Single-ended 16-bit conversion; differential 16-bit conversion with 2's complement output.*/
#define  CMDL_MODE						CMDL_MODE_STANDARD


#define  CMDH_SAMPLETIME_3_5				1U
#define  CMDH_SAMPLETIME_5_5				2U
#define  CMDH_SAMPLETIME_7_5				3U
#define  CMDH_SAMPLETIME_11_5				4U
#define  CMDH_SAMPLETIME_19_5				5U
#define  CMDH_SAMPLETIME_35_5				6U
#define  CMDH_SAMPLETIME_67_5				7U
#define  CMDH_SAMPLETIME_131_5				8U

#define  CMDH_AVERAGE_1						0U
#define  CMDH_AVERAGE_2						1U
#define  CMDH_AVERAGE_4						2U
#define  CMDH_AVERAGE_8						3U
#define  CMDH_AVERAGE_16					4U
#define  CMDH_AVERAGE_32					5U
#define  CMDH_AVERAGE_64					6U
#define  CMDH_AVERAGE_128					7U

#define  ADC_FIFO_RESULTS				4U		/* Set expected number of results in FIFO */

#define  ACT_CMD1 		0U
#define  ACT_CMD2 		1U
#define  ACT_CMD3 		2U
#define  ACT_CMD4 		3U
#define  ACT_CMD5 		4U
#define  ACT_CMD6 		5U
#define  ACT_CMD7 		6U
#define  ACT_CMD8 		7U
#define  ACT_CMD9 		8U
#define  ACT_CMD10 		9U
#define  ACT_CMD11 		10U
#define  ACT_CMD12 		11U
#define  ACT_CMD13 		12U
#define  ACT_CMD14 		13U
#define  ACT_CMD15 		14U

#define  NEXT_CMD0 		0U
#define  NEXT_CMD1 		1U
#define  NEXT_CMD2 		2U
#define  NEXT_CMD3 		3U
#define  NEXT_CMD4 		4U
#define  NEXT_CMD5 		5U
#define  NEXT_CMD6 		6U
#define  NEXT_CMD7 		7U
#define  NEXT_CMD8 		8U
#define  NEXT_CMD9 		9U
#define  NEXT_CMD10 	10U
#define  NEXT_CMD11 	11U
#define  NEXT_CMD12 	12U
#define  NEXT_CMD13 	13U
#define  NEXT_CMD14 	14U
#define  NEXT_CMD15 	15U

void ADC_SetConversionCMDL(ADC_Type *base, uint8_t thisCommand, uint8_t convType, uint8_t channelA, uint8_t channelB)
{

	base->CMD[thisCommand].CMDL = ADC_CMDL_ADCH(channelA)  |  	/* Channel number (ADC0IN3A (CUR_A) and ADC0IN3B (CUR_C)) */
	                    		  ADC_CMDL_CTYPE(convType) |  	/* Conversion type dual single-ended CUR_A, CUR_C */
	                              ADC_CMDL_MODE(CMDL_MODE) |    /* Resolution of conversion */
								  ADC_CMDL_ALTB_ADCH(channelB);	/* Set B channel */

	if(convType>CMDL_CTYPE_SINGLE_ENDED_A)
	{
		base->CMD[thisCommand].CMDL |=	ADC_CMDL_ALTBEN_MASK;
	}

}

void ADC_SetConversionCMDH(ADC_Type *base, uint8_t thisCommand, uint8_t nextCommand, uint8_t sampleTime, uint8_t average)
{

	base->CMD[thisCommand].CMDH = ADC_CMDH_NEXT(nextCommand) |	    /* Next command select */
								  ADC_CMDH_STS(sampleTime)   |
								  ADC_CMDH_AVGS(average);

}

#endif /* LPC55S36_LOWLEVEL_ADC_H_ */
