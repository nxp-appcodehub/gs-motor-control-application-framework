/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "mlib_FP.h"
#include "main.h"
#include "fsl_device_registers.h"

#define CALIB_LOOP_CNT      128

void AppInitFast(void);
void AppRunFast(void);
void AppStopFast(void);
void AppStartFast(void);
void AppErrorFast(void);
void AppTransInitToStopFast(void);
void AppTransStopToStartFast(void);
void AppTransStartToRunFast(void);
void AppTransRunToStopFast(void);
void AppTransStopToErrorFast(void);
void AppTransRunToErrorFast(void);
void AppTransErrorToStopFast(void);

void AppInitSlow(void);
void AppRunSlow(void);
void AppStopSlow(void);
void AppStartSlow(void);
void AppErrorSlow(void);

bool_t Calibration(sAdcResult *f16InputRawValue, sAdcResult *f16OutputOffsetValue, uint16_t *ui16CountInput, uint16_t ui16CalibLoop);
void AdcGetResults(void);
void ScaleValues(void);
static bool_t OverCurrentFault(PWM_Type *pPWMBase);
static void UpdatePWM(frac16_t f16PhA, frac16_t f16PhB, frac16_t f16PhC, PWM_Type *pPWMBase);

#endif /* MOTOR_CONTROL_H_ */
