/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver - tuner abstraction
 *
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _AVL_TUNER_H_
#define _AVL_TUNER_H_

#include "avl_bsp.h"

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

#define AVL_TUNER_EC_OK 0 // There is no error.

  struct AVL_Tuner
  {
    uint16_t usTunerI2CAddr;
    uint8_t ucTunerLocked; //1 Lock;   0 unlock

    uint32_t uiRFFrequencyHz;
    uint32_t uiLPFHz; //only valid for satellite tuner

    uint8_t ucBlindScanMode;

    void *vpMorePara;

    uint32_t (*fpInitializeFunc)(struct AVL_Tuner *);
    uint32_t (*fpLockFunc)(struct AVL_Tuner *);
    uint32_t (*fpGetLockStatusFunc)(struct AVL_Tuner *);
    uint32_t (*fpGetRFStrength)(struct AVL_Tuner *, int32_t *);

    //Maximum tuner low pass filter bandwidth in Hz
    uint32_t (*fpGetMaxLPF)(struct AVL_Tuner *, uint32_t *);

    //Minimum tuner low pass filter bandwidth in Hz
    uint32_t (*fpGetMinLPF)(struct AVL_Tuner *, uint32_t *);

    //Low pass filter bandwidth step size in Hz
    uint32_t (*fpGetLPFStepSize)(struct AVL_Tuner *, uint32_t *);

    //Tuner AGC gain slope in dB per Volt (dB/V).
    //Tuners with non-inverted AGC sense have a positive slope.
    //Tuners with inverted AGC sense have a negative slope.
    //If the gain slope is not known, implement a function that
    //  returns 1 if the AGC sense is non-inverted,
    //  and returns -1 if the AGC sense is inverted.
    uint32_t (*fpGetAGCSlope)(struct AVL_Tuner *, int32_t *);

    //Voltage at which gain reaches minimum value.  Voltage in millivolts.
    //For a tuner with non-inverted sense (positive slope), this will be a small value.
    //For a tuner with inverted sense (negative slope), this will be a large value.
    uint32_t (*fpGetMinGainVoltage)(struct AVL_Tuner *, uint32_t *);

    //Voltage at which gain reaches its maximum value. Voltage in millivolts.
    //For a tuner with non-inverted sense (positive slope), this will be a large value.
    //For a tuner with inverted sense (negative slope), this will be a small value.
    uint32_t (*fpGetMaxGainVoltage)(struct AVL_Tuner *, uint32_t *);

    //RF Frequency step size in Hz
    uint32_t (*fpGetRFFreqStepSize)(struct AVL_Tuner *, uint32_t *);

  };

#ifdef AVL_CPLUSPLUS
}
#endif

#endif
