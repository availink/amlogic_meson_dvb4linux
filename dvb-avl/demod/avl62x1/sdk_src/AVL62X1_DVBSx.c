/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver
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

#include "AVL62X1_DVBSx.h"


uint16_t Init_AVL62X1_ChipObject(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_Uninitialized;
  r = avl_bsp_init_semaphore(&(pAVL_Chip->chip_priv->m_semRx));
  r |= avl_bsp_init_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));

  // there is internal protection to assure it will be initialized only once.
  r |= avl_bms_initialize();

  return (r);
}

uint16_t IBase_CheckChipReady_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t uiCoreReadyWord = 0;
  uint32_t uiCoreRunning = 0;

  r = avl_bms_read32((uint16_t)(pAVL_Chip->chip_pub->i2c_addr), hw_AVL62X1_cpucore_top_srst, &uiCoreRunning);
  r |= avl_bms_read32((uint16_t)(pAVL_Chip->chip_pub->i2c_addr), rs_AVL62X1_core_ready_word, &uiCoreReadyWord);
  if ((AVL_EC_OK == r))
  {
    if ((1 == uiCoreRunning) || (uiCoreReadyWord != 0x5AA57FF7))
    {
      r = AVL_EC_GENERAL_FAIL;
    }
  }

  return (r);
}

//download and boot firmware, load default configuration, read back clock frequencies
uint16_t IBase_Initialize_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= IBase_DownloadPatch_AVL62X1(pAVL_Chip);

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_mpeg_ref_clk_Hz_iaddr, pAVL_Chip->chip_pub->req_mpeg_clk_freq_hz);
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_DMD_xtal_frequency_caddr, (uint8_t)(pAVL_Chip->chip_pub->ref_clk));

  //load defaults command will load S2X defaults and program PLL based on XTAL and MPEG ref clk configurations
  r |= IBase_SendRxOP_AVL62X1(CMD_LD_DEFAULT, pAVL_Chip);

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_fec_clk_Hz_iaddr, &(pAVL_Chip->chip_priv->fec_clk_freq_hz));
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_sys_clk_Hz_iaddr, &(pAVL_Chip->chip_priv->core_clk_freq_hz));
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_mpeg_ref_clk_Hz_iaddr, &(pAVL_Chip->chip_priv->mpeg_clk_freq_hz));

  return (r);
}

uint16_t IBase_GetVersion_AVL62X1(struct avl62x1_ver_info *pVer_info, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t uiTemp = 0;
  uint8_t ucBuff[4] = {0};

  r = avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, 0x40000, &uiTemp);
  pVer_info->m_Chip = uiTemp;

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_DMD_patch_ver_iaddr, &uiTemp);
  avl_int_to_bytes(uiTemp, ucBuff);
  pVer_info->m_Patch.m_Major = ucBuff[0];
  pVer_info->m_Patch.m_Minor = ucBuff[1];
  pVer_info->m_Patch.m_Build = ucBuff[2];
  pVer_info->m_Patch.m_Build = (uint16_t)((pVer_info->m_Patch.m_Build) << 8) + ucBuff[3];

  pVer_info->m_API.m_Major = AVL62X1_API_VER_MAJOR;
  pVer_info->m_API.m_Minor = AVL62X1_API_VER_MINOR;
  pVer_info->m_API.m_Build = AVL62X1_API_VER_BUILD;

  return (r);
}

uint16_t IBase_GetFunctionMode_AVL62X1(enum avl62x1_functional_mode *pFuncMode, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t ucTemp = 0;

  r = avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_active_demod_mode_caddr, &ucTemp);
  *pFuncMode = (avl62x1_functional_mode)ucTemp;

  return (r);
}

uint16_t IBase_GetRxOPStatus_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint16_t uiCmd = 0;

  r = avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_DMD_command_saddr, &uiCmd);
  if (AVL_EC_OK == r)
  {
    if (CMD_IDLE != uiCmd)
    {
      r = AVL_EC_RUNNING;
    }
    else if (CMD_FAILED == uiCmd)
    {
      r = AVL_EC_COMMAND_FAILED;
    }
  }

  return (r);
}

uint16_t IBase_SendRxOP_AVL62X1(uint8_t ucOpCmd, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t pucBuff[4] = {0};
  uint16_t uiTemp = 0;
  const uint16_t uiTimeDelay = 10;
  const uint16_t uiMaxRetries = 50; //the time out window is 10*50 = 500ms ( change for sysy using.)
  uint32_t i = 0;

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semRx));

  while (AVL_EC_OK != IBase_GetRxOPStatus_AVL62X1(pAVL_Chip))
  {
    if (uiMaxRetries < i++)
    {
      r |= AVL_EC_RUNNING;
      break;
    }
    avl_bsp_delay(uiTimeDelay);
  }
  if (AVL_EC_OK == r)
  {
    pucBuff[0] = 0;
    pucBuff[1] = ucOpCmd;
    uiTemp = avl_bytes_to_short(pucBuff);
    r |= avl_bms_write16((uint16_t)(pAVL_Chip->chip_pub->i2c_addr), c_AVL62X1_DMD_command_saddr, uiTemp);
  }

  i = 0;
  while (AVL_EC_OK != IBase_GetRxOPStatus_AVL62X1(pAVL_Chip))
  {
    if (uiMaxRetries < i++)
    {
      r |= AVL_EC_RUNNING;
      break;
    }
    avl_bsp_delay(uiTimeDelay);
  }

  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semRx));

  return (r);
}

uint16_t IBase_GetSPOPStatus_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint16_t uiCmd = 0;

  r = avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_command_saddr, &uiCmd);
  if (AVL_EC_OK == r)
  {
    if (CMD_IDLE != uiCmd)
    {
      r = AVL_EC_RUNNING;
    }
    else if (CMD_FAILED == uiCmd)
    {
      r = AVL_EC_COMMAND_FAILED;
    }
  }

  return (r);
}

uint16_t IBase_SendSPOP_AVL62X1(uint8_t ucOpCmd, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t pucBuff[4] = {0};
  uint16_t uiTemp = 0;
  const uint16_t uiTimeDelay = 10;
  const uint16_t uiMaxRetries = 50; //the time out window is 10*50 = 500ms
  uint32_t i = 0;

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semRx));

  while (AVL_EC_OK != IBase_GetSPOPStatus_AVL62X1(pAVL_Chip))
  {
    if (uiMaxRetries < i++)
    {
      r |= AVL_EC_RUNNING;
      break;
    }
    avl_bsp_delay(uiTimeDelay);
  }
  if (AVL_EC_OK == r)
  {
    pucBuff[0] = 0;
    pucBuff[1] = ucOpCmd;
    uiTemp = avl_bytes_to_short(pucBuff);
    r |= avl_bms_write16((uint16_t)(pAVL_Chip->chip_pub->i2c_addr), c_AVL62X1_SP_sp_command_saddr, uiTemp);
  }

  i = 0;
  while (AVL_EC_OK != IBase_GetSPOPStatus_AVL62X1(pAVL_Chip))
  {
    if (uiMaxRetries < i++)
    {
      r |= AVL_EC_RUNNING;
      break;
    }
    avl_bsp_delay(uiTimeDelay);
  }

  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semRx));

  return (r);
}

uint16_t IBase_Halt_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= IBase_SendRxOP_AVL62X1(CMD_HALT, pAVL_Chip);

  return (r);
}

uint16_t IBase_Sleep_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= IBase_SendRxOP_AVL62X1(CMD_SLEEP, pAVL_Chip);

  return (r);
}

uint16_t IBase_Wakeup_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= IBase_SendRxOP_AVL62X1(CMD_WAKE, pAVL_Chip);

  return (r);
}

uint16_t IBase_Initialize_TunerI2C_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  //uint32_t uiSetMode = 2;  //bit mode
  uint32_t uiTemp = 0;
  uint32_t bit_rpt_divider = 0;

  //reset tuner i2c block
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_i2c_srst, 1);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_i2c_srst, 0);

  //tuner_i2c_cntrl: {rpt_addr[23:16],...,i2c_mode[8],...,src_sel[0]}
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_i2c_cntrl, &uiTemp);
  uiTemp = (uiTemp & 0xFFFFFFFE);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_i2c_cntrl, uiTemp);

  //hw_i2c_bit_rpt_cntrl: {doubleFFen, stop_check, rpt_sel, rpt_enable}
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl, 0x6);

  bit_rpt_divider = (0x2A) * (pAVL_Chip->chip_priv->core_clk_freq_hz / 1000) / (240 * 1000);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_clk_div, bit_rpt_divider);

  //configure GPIO
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_clk2_sel, 7);  //M3_SCL
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_data2_sel, 8); //M3_SDA
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__m3_scl_sel, 6);    //I2C_CLK2
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__m3_sda_sel, 5);    //I2C_DATA2

  return (r);
}

uint16_t IRx_Initialize_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_input_format_caddr, 0x0); // 0: 2's complement, 1: offset binary
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_input_select_caddr, 0x1); //0: Digital, 1: ADC in

  return (r);
}

uint16_t IRx_GetTunerPola_AVL62X1(enum avl62x1_spectrum_polarity *pTuner_Pol, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t uiTemp = 0;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_carrier_spectrum_invert_status_caddr, &uiTemp);
  *pTuner_Pol = (enum avl62x1_spectrum_polarity)uiTemp;

  return (r);
}

uint16_t IRx_SetTunerPola_AVL62X1(enum avl62x1_spectrum_polarity enumTuner_Pol, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_tuner_spectrum_invert_caddr, (uint8_t)enumTuner_Pol);

  return (r);
}

//the FW (versions >= 24739) supports setting the PLS seed manually.
uint16_t IRx_ConfigPLS_AVL62X1(uint32_t uiShiftValue, struct avl62x1_chip *pAVL_Chip)
{
  uint32_t uiTemp = 0;
  uint16_t r = AVL_EC_OK;

  r = avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr, &uiTemp);
  uiTemp &= (!(1 << 23)); //bit[23] eq 0 -> Manually configure scramble code according to bits [18:0]
  uiTemp &= (!(1 << 18)); //bit[18] eq 0 -> bits[17:0] represent the sequence shift of the x(i) sequence in the Gold code as defined as the "code number n" in the DVB-S2 standard
  uiTemp |= 0x3FFFF;
  uiTemp &= uiShiftValue;
  r = avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr, uiTemp);

  return (r);
}

uint16_t IRx_SetPLSAutoDetect_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr, AVL62X1_PL_SCRAM_AUTO);

  return (r);
}

uint16_t IRx_DriveAGC_AVL62X1(enum avl62x1_switch enumOn_Off, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t uiTemp = 0;
  int32_t rfagc_slope = 0;
  uint32_t min_gain = 0;
  uint32_t max_gain = 0;
  uint32_t min_gain_mV = 0;
  uint32_t max_gain_mV = 0;

  if (AVL62X1_ON == enumOn_Off)
  {
    //set RF AGC polarity according to AGC slope sign
    if (pAVL_Chip->chip_pub->pTuner->fpGetAGCSlope == nullptr)
    {
      rfagc_slope = -1;
    }
    else
    {
      pAVL_Chip->chip_pub->pTuner->fpGetAGCSlope(pAVL_Chip->chip_pub->pTuner, &rfagc_slope);
    }
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_rf_agc_pol_caddr, (uint8_t)(rfagc_slope < 0));

    //set min and max gain values according to AGC saturation points
    if (pAVL_Chip->chip_pub->pTuner->fpGetMinGainVoltage == nullptr || pAVL_Chip->chip_pub->pTuner->fpGetMaxGainVoltage == nullptr)
    {
      //set some reasonable defaults
      if (rfagc_slope > 0)
      {
        min_gain_mV = 100;
        max_gain_mV = 3200;
      }
      else
      {
        min_gain_mV = 3200;
        max_gain_mV = 100;
      }
    }
    else
    {
      pAVL_Chip->chip_pub->pTuner->fpGetMinGainVoltage(pAVL_Chip->chip_pub->pTuner, &min_gain_mV);
      pAVL_Chip->chip_pub->pTuner->fpGetMaxGainVoltage(pAVL_Chip->chip_pub->pTuner, &max_gain_mV);
      min_gain_mV = AVL_MIN(min_gain_mV, 3300);
      max_gain_mV = AVL_MIN(max_gain_mV, 3300);
    }

    if (rfagc_slope > 0)
    {
      min_gain = (min_gain_mV * (1 << 16)) / 3300;
      max_gain = (max_gain_mV * (1 << 16)) / 3300;
    }
    else
    {
      min_gain = ((3300 - min_gain_mV) * (1 << 16)) / 3300;
      max_gain = ((3300 - max_gain_mV) * (1 << 16)) / 3300;
    }

    min_gain = AVL_MIN(min_gain, 0xFFFF);
    max_gain = AVL_MIN(max_gain, 0xFFFF);

    r |= avl_bms_write16((pAVL_Chip->chip_pub->i2c_addr), c_AVL62X1_S2X_rf_agc_min_gain_saddr, min_gain);
    r |= avl_bms_write16((pAVL_Chip->chip_pub->i2c_addr), c_AVL62X1_S2X_rf_agc_max_gain_saddr, max_gain);

    //enable sigma delta output
    r |= avl_bms_read32((pAVL_Chip->chip_pub->i2c_addr), aagc__analog_agc_sd_control_reg, &uiTemp);
    uiTemp |= (0x1 << 0x1); //agc_sd_on bit
    r |= avl_bms_write32((pAVL_Chip->chip_pub->i2c_addr), aagc__analog_agc_sd_control_reg, uiTemp);

    //configure GPIO
    r |= avl_bms_write32((pAVL_Chip->chip_pub->i2c_addr), gpio_debug__agc1_sel, 6);
    r |= avl_bms_write32((pAVL_Chip->chip_pub->i2c_addr), gpio_debug__agc2_sel, 6);
  }
  else if (AVL62X1_OFF == enumOn_Off)
  {
    r |= avl_bms_read32((pAVL_Chip->chip_pub->i2c_addr), aagc__analog_agc_sd_control_reg, &uiTemp);
    uiTemp &= ~(0x1 << 0x1); //agc_sd_on bit
    r |= avl_bms_write32((pAVL_Chip->chip_pub->i2c_addr), aagc__analog_agc_sd_control_reg, uiTemp);

    //configure GPIO
    r |= avl_bms_write32((pAVL_Chip->chip_pub->i2c_addr), gpio_debug__agc1_sel, 2); //high-Z
    r |= avl_bms_write32((pAVL_Chip->chip_pub->i2c_addr), gpio_debug__agc2_sel, 2); //high-Z
  }
  else
  {
    r |= AVL_EC_GENERAL_FAIL;
  }
  return (r);
}

uint16_t IRx_GetCarrierFreqOffset_AVL62X1(int32_t *piFreqOffsetHz, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t uiTemp = 0;

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_carrier_freq_Hz_iaddr, &uiTemp);
  *piFreqOffsetHz = (int32_t)uiTemp;

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_carrier_freq_err_Hz_iaddr, &uiTemp);
  *piFreqOffsetHz += (int32_t)uiTemp;
  *piFreqOffsetHz -= pAVL_Chip->chip_priv->carrier_freq_offset_hz;

  return (r);
}

uint16_t IRx_GetSROffset_AVL62X1(int32_t *piSROffsetPPM, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  int32_t sr_error = 0;
  uint32_t sr = 0;

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_symbol_rate_Hz_iaddr, &sr);
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_symbol_rate_error_Hz_iaddr, (uint32_t *)&sr_error);

  *piSROffsetPPM = (int32_t)(((long long int)sr_error * ((long long int)1000000)) / (long long int)sr);
  return (r);
}

uint16_t IRx_ErrorStatMode_AVL62X1(struct avl62x1_error_stats_config *stErrorStatConfig, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  struct avl_uint64 time_tick_num = {0, 0};

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__auto1_manual0_mode__offset, (uint32_t)stErrorStatConfig->eErrorStatMode);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__timetick1_bytetick0__offset, (uint32_t)stErrorStatConfig->eAutoErrorStatType);

  avl_mult_32to64(&time_tick_num, pAVL_Chip->chip_priv->mpeg_clk_freq_hz / 1000, stErrorStatConfig->uiTimeThresholdMs);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__time_tick_low__offset, time_tick_num.low_word);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__time_tick_high__offset, time_tick_num.high_word);

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__byte_tick_low__offset, (uint32_t)stErrorStatConfig->uiNumberThresholdByte);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__byte_tick_high__offset, 0); //high 32-bit is not used

  if (stErrorStatConfig->eErrorStatMode == AVL62X1_ERROR_STATS_AUTO)
  {
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__tick_clear_req__offset, 0);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__tick_clear_req__offset, 1);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__tick_clear_req__offset, 0);
  }

  return (r);
}

uint16_t IRx_ResetBER_AVL62X1(struct avl62x1_ber_config *pBERConfig, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t uiTemp = 0;
  uint16_t uiLFSR_Sync = 0;
  uint32_t uiCnt = 0;
  uint32_t uiByteCnt = 0;
  uint32_t uiBER_FailCnt = 0;
  uint32_t uiBitErrors = 0;

  pAVL_Chip->chip_priv->error_stats.m_LFSR_Sync = 0;
  pAVL_Chip->chip_priv->error_stats.m_LostLock = 0;
  pAVL_Chip->chip_priv->error_stats.m_SwCntNumBits.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_SwCntNumBits.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_SwCntBitErrors.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_SwCntBitErrors.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_NumBits.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_NumBits.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_BitErrors.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_BitErrors.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_BER = 0;

  //ber software reset
  r = avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &uiTemp);
  uiTemp |= 0x00000002;
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

  //alway inverted
  pBERConfig->eBERFBInversion = AVL62X1_LFSR_FB_INVERTED;

  //set Test Pattern and Inversion
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &uiTemp);
  uiTemp &= 0xFFFFFFCF;
  //BER_Test_Pattern:bit 5 --- 0:LFSR_15; 1:LFSR_23
  //BER_FB_Inversion:bit 4 --- 0:NOT_INVERTED; 1:INVERTED
  uiTemp |= ((uint32_t)(pBERConfig->eBERTestPattern) << 5) | ((uint32_t)(pBERConfig->eBERFBInversion) << 4);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

  uiTemp &= 0xFFFFFE3F;
  uiTemp |= ((pBERConfig->uiLFSRStartPos) << 6); //For SFU or other standard, the start position of LFSR is 1, just follow the 0x47 sync word
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

  while (!uiLFSR_Sync)
  {
    uiTemp |= 0x00000006;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
    uiTemp &= 0xFFFFFFFD;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

    uiCnt = 0;
    uiByteCnt = 0;
    while ((uiByteCnt < 1000) && (uiCnt < 200))
    {
      r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__byte_num, &uiByteCnt);
      uiCnt++;
    }

    uiTemp |= 0x00000006;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
    uiTemp &= 0xFFFFFFF9;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

    uiCnt = 0;
    uiByteCnt = 0;
    while ((uiByteCnt < 10000) && (uiCnt < 200))
    {
      uiCnt++;
      r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__byte_num, &uiByteCnt);
    }

    uiTemp &= 0xFFFFFFF9;
    uiTemp |= 0x00000002;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__byte_num, &uiByteCnt);
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__ber_err_cnt, &uiBitErrors);
    if (uiCnt == 200)
    {
      break;
    }
    else if ((uiByteCnt << 3) < (10 * uiBitErrors))
    {
      uiBER_FailCnt++;
      if (uiBER_FailCnt > 10)
      {
        break;
      }
    }
    else
    {
      uiLFSR_Sync = 1;
    }
  }

  if (uiLFSR_Sync == 1)
  {
    uiTemp &= 0xFFFFFFF9;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
  }

  pBERConfig->uiLFSRSynced = uiLFSR_Sync;
  pAVL_Chip->chip_priv->error_stats.m_LFSR_Sync = uiLFSR_Sync;

  return (r);
}

uint16_t IRx_GetBER_AVL62X1(uint32_t *puiBER_x1e9, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t uiHwCntBitErrors = 0;
  uint32_t uiHwCntNumBits = 0;
  uint32_t uiTemp = 0;
  struct avl_uint64 uiTemp64;

  r = avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__ber_err_cnt, &uiHwCntBitErrors);
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__byte_num, &uiHwCntNumBits);
  uiHwCntNumBits <<= 3;

  //Keep the hw counts into sw struct to avoid hw registers overflow
  if (uiHwCntNumBits > (uint32_t)(1 << 31))
  {
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &uiTemp);
    uiTemp |= 0x00000002;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__ber_err_cnt, &uiHwCntBitErrors);
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__byte_num, &uiHwCntNumBits);
    uiTemp &= 0xFFFFFFFD;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
    uiHwCntNumBits <<= 3;
    avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_SwCntNumBits, uiHwCntNumBits);
    avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_SwCntBitErrors, uiHwCntBitErrors);
    uiHwCntNumBits = 0;
    uiHwCntBitErrors = 0;
  }

  pAVL_Chip->chip_priv->error_stats.m_NumBits.high_word = pAVL_Chip->chip_priv->error_stats.m_SwCntNumBits.high_word;
  pAVL_Chip->chip_priv->error_stats.m_NumBits.low_word = pAVL_Chip->chip_priv->error_stats.m_SwCntNumBits.low_word;
  avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_NumBits, uiHwCntNumBits);
  pAVL_Chip->chip_priv->error_stats.m_BitErrors.high_word = pAVL_Chip->chip_priv->error_stats.m_SwCntBitErrors.high_word;
  pAVL_Chip->chip_priv->error_stats.m_BitErrors.low_word = pAVL_Chip->chip_priv->error_stats.m_SwCntBitErrors.low_word;
  avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_BitErrors, uiHwCntBitErrors);

  // Compute the BER, this BER is multiplied with 1e9 because the float operation is not supported
  avl_mult_32to64(&uiTemp64, pAVL_Chip->chip_priv->error_stats.m_BitErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
  pAVL_Chip->chip_priv->error_stats.m_BER = avl_divide_64(pAVL_Chip->chip_priv->error_stats.m_NumBits, uiTemp64);

  //keep the BER user wanted
  *puiBER_x1e9 = pAVL_Chip->chip_priv->error_stats.m_BER;

  return (r);
}

uint16_t IRx_GetAcqRetries_AVL62X1(uint8_t *pucRetryNum, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_acq_retry_count_caddr, pucRetryNum);
  return (r);
}

uint16_t IRx_SetMpegMode_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_serial_caddr, (uint8_t)(pAVL_Chip->chip_pub->mpeg_mode));
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts0_tsp1_caddr, (uint8_t)(pAVL_Chip->chip_pub->mpeg_format));
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_clock_edge_caddr, (uint8_t)(pAVL_Chip->chip_pub->mpeg_clk_pol));
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_clock_phase_caddr, (uint8_t)(pAVL_Chip->chip_pub->mpeg_clk_phase)); //only valid for Parallel
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_adapt_clk_caddr, (uint8_t)(pAVL_Chip->chip_pub->mpeg_clk_adapt == AVL62X1_MPCA_Adaptive));
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, (c_AVL62X1_SP_sp_mpeg_bus_misc_2_iaddr + 2), 0x3); //enhance the MPEG driver ability.
  r |= IRx_SetMpegErrorPolarity_AVL62X1(pAVL_Chip->chip_pub->mpeg_err_pol, pAVL_Chip);
  r |= IRx_SetMpegValidPolarity_AVL62X1(pAVL_Chip->chip_pub->mpeg_valid_pol, pAVL_Chip);


  if (pAVL_Chip->chip_pub->mpeg_mode == AVL62X1_MPM_Serial)
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_serial_outpin_caddr, (uint8_t)(pAVL_Chip->chip_pub->mpeg_serial_pin)); // serial TS Pin
    r |= IRx_EnableMpegContiClock_AVL62X1(pAVL_Chip);
  }

  if (pAVL_Chip->chip_pub->mpeg_mode == AVL62X1_MPM_Parallel)
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_adapt_clk_caddr, AVL62X1_MPCA_Adaptive);
    r |= IRx_DisableMpegContiClock_AVL62X1(pAVL_Chip);
  }

  return (r);
}

uint16_t IRx_SetMpegBitOrder_AVL62X1(enum avl62x1_mpeg_bit_order e_BitOrder, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_serial_msb_caddr, (uint8_t)e_BitOrder);

  return (r);
}

uint16_t IRx_SetMpegErrorPolarity_AVL62X1(enum avl62x1_mpeg_err_polarity e_ErrorPol, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_error_polarity_caddr, (uint8_t)e_ErrorPol);

  return (r);
}

uint16_t IRx_SetMpegValidPolarity_AVL62X1(enum avl62x1_mpeg_err_polarity e_ValidPol, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_sp_ts_valid_polarity_caddr, (uint8_t)e_ValidPol);

  return (r);
}

uint16_t IRx_EnableMpegContiClock_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8((uint16_t)(pAVL_Chip->chip_pub->i2c_addr), c_AVL62X1_SP_sp_enable_ts_continuous_caddr, 1);

  return (r);
}

uint16_t IRx_DisableMpegContiClock_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8((uint16_t)(pAVL_Chip->chip_pub->i2c_addr), c_AVL62X1_SP_sp_enable_ts_continuous_caddr, 0);

  return (r);
}

uint16_t IRx_DriveMpegOutput_AVL62X1(enum avl62x1_switch enumOn_Off, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  if (AVL62X1_ON == enumOn_Off)
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, ts_output_intf__mpeg_bus_off, 0x00);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ts_output_intf__mpeg_bus_e_b, 0x00);
  }
  else if (AVL62X1_OFF == enumOn_Off)
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, ts_output_intf__mpeg_bus_off, 0xFF);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ts_output_intf__mpeg_bus_e_b, 0xFFF);
  }
  else
  {
    r |= AVL_EC_GENERAL_FAIL;
  }
  return (r);
}

uint16_t IDiseqc_Initialize_AVL62X1(struct avl62x1_diseqc_params *pDiseqcPara, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  if (AVL_EC_OK == r)
  {
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_srst, 1);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_samp_frac_n, 2000000);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_samp_frac_d, pAVL_Chip->chip_priv->core_clk_freq_hz);

    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tone_frac_n, (pDiseqcPara->uiToneFrequencyKHz) << 1);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tone_frac_d, (pAVL_Chip->chip_priv->core_clk_freq_hz / 1000));

    // Initialize the tx_control
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
    i1 &= 0x00000300;
    i1 |= 0x20; //reset tx_fifo
    i1 |= ((uint32_t)(pDiseqcPara->eTXGap) << 6);
    i1 |= ((uint32_t)(pDiseqcPara->eTxWaveForm) << 4);
    i1 |= (1 << 3); //enable tx gap.
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
    i1 &= ~(0x20); //release tx_fifo reset
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);

    // Initialize the rx_control
    i1 = ((uint32_t)(pDiseqcPara->eRxWaveForm) << 2);
    i1 |= (1 << 1); //active the receiver
    i1 |= (1 << 3); //envelop high when tone present
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_rx_cntrl, i1);
    i1 = (uint32_t)(pDiseqcPara->eRxTimeout);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__rx_msg_tim, i1);

    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_srst, 0);

    if (AVL_EC_OK == r)
    {
      pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_Initialized;
    }
  }
  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));

  return (r);
}

uint16_t IDiseqc_IsSafeToSwitchMode_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;

  switch (pAVL_Chip->chip_priv->diseqc_op_status)
  {
  case AVL62X1_DOS_InModulation:
  case AVL62X1_DOS_InTone:
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_st, &i1);
    if (1 != ((i1 & 0x00000040) >> 6)) //check if the last transmit is done
    {
      r |= AVL_EC_RUNNING;
    }
    break;
  case AVL62X1_DOS_InContinuous:
  case AVL62X1_DOS_Initialized:
    break;
  default:
    r |= AVL_EC_GENERAL_FAIL;
    break;
  }
  return (r);
}

uint16_t IBase_DownloadPatch_AVL62X1(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t *patch_data = 0;
  uint32_t patch_idx = 0;
  uint32_t total_patch_len = 0;
  uint32_t standard = 0;
  uint32_t args_addr = 0;
  uint32_t data_section_offset = 0;
  uint32_t reserved_len = 0;
  uint32_t script_len = 0;
  uint8_t unary_op = 0;
  uint8_t binary_op = 0;
  uint8_t addr_mode_op = 0;
  uint32_t script_start_idx = 0;
  uint32_t num_cmd_words = 0;
  uint32_t next_cmd_idx = 0;
  uint32_t num_cond_words = 0;
  uint32_t condition = 0;
  uint32_t operation = 0;
  uint32_t value = 0;
  uint32_t cmd = 0;
  uint32_t num_rvs = 0;
  uint32_t num_rvs2 = 0;
  uint32_t rv0_idx = 0;
  uint32_t exp_crc_val = 0;
  uint32_t start_addr = 0;
  uint32_t crc_result = 0;
  uint32_t length = 0;
  uint32_t dest_addr = 0;
  uint32_t src_data_offset = 0;
  uint32_t data = 0;
  uint16_t data1 = 0;
  uint8_t data2 = 0;
  uint32_t src_addr = 0;
  uint32_t descr_addr = 0;
  uint32_t num_descr = 0;
  uint32_t type = 0;
  uint32_t ready = 0;
  uint32_t dma_max_tries = 0;
  uint32_t dma_tries = 0;
  uint32_t rv = 0;
  int8_t temp[3];
  uint8_t *pPatchDatatemp = 0;
  uint8_t *pPatchDatatemp1 = 0;
  uint32_t cond = 0;
  uint8_t got_cmd_exit = 0;
  uint16_t num_records = 0;
  uint16_t record_length = 0;
  uint16_t addr_offset = 0;
  uint16_t record_cnt = 0;
  uint32_t match_value = 0;
  uint32_t max_polls = 0;
  uint32_t polls = 0;
  uint32_t variable_array[PATCH_VAR_ARRAY_SIZE];

  patch_data = pAVL_Chip->chip_priv->patch_data;


  patch_idx = 4; //INDEX IS A BYTE OFFSET
  total_patch_len = patch_read32_AVL62X1(patch_data, &patch_idx);
  standard = patch_read32_AVL62X1(patch_data, &patch_idx);
  args_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
  data_section_offset = patch_read32_AVL62X1(patch_data, &patch_idx);
  reserved_len = patch_read32_AVL62X1(patch_data, &patch_idx);
  patch_idx += 4 * reserved_len; //skip over reserved area for now
  script_len = patch_read32_AVL62X1(patch_data, &patch_idx);

  if ((patch_idx / 4 + script_len) != data_section_offset)
  {
    r = AVL_EC_GENERAL_FAIL;
    return (r);
  }

  script_start_idx = patch_idx / 4;

  while ((patch_idx / 4 < (script_start_idx + script_len)) && !got_cmd_exit)
  {
    num_cmd_words = patch_read32_AVL62X1(patch_data, &patch_idx);
    next_cmd_idx = patch_idx + (num_cmd_words - 1) * 4; //BYTE OFFSET
    num_cond_words = patch_read32_AVL62X1(patch_data, &patch_idx);

    if (num_cond_words == 0)
    {
      condition = 1;
    }
    else
    {
      for (cond = 0; cond < num_cond_words; cond++)
      {
        operation = patch_read32_AVL62X1(patch_data, &patch_idx);
        value = patch_read32_AVL62X1(patch_data, &patch_idx);
        unary_op = (operation >> 8) & 0xFF;
        binary_op = operation & 0xFF;
        addr_mode_op = ((operation >> 16) & 0x3);

        if ((addr_mode_op == PATCH_OP_ADDR_MODE_VAR_IDX) && (binary_op != PATCH_OP_BINARY_STORE))
        {
          value = variable_array[value]; //grab variable value
        }

        switch (unary_op)
        {
        case PATCH_OP_UNARY_LOGICAL_NEGATE:
          value = !value;
          break;
        case PATCH_OP_UNARY_BITWISE_NEGATE:
          value = ~value;
          break;
        case PATCH_OP_UNARY_BITWISE_AND:
          //value = FIXME
          break;
        case PATCH_OP_UNARY_BITWISE_OR:
          //value = FIXME
          break;
        }
        switch (binary_op)
        {
        case PATCH_OP_BINARY_LOAD:
          condition = value;
          break;
        case PATCH_OP_BINARY_STORE:
          variable_array[value] = condition;
          break;
        case PATCH_OP_BINARY_AND:
          condition = condition && value;
          break;
        case PATCH_OP_BINARY_OR:
          condition = condition || value;
          break;
        case PATCH_OP_BINARY_BITWISE_AND:
          condition = condition & value;
          break;
        case PATCH_OP_BINARY_BITWISE_OR:
          condition = condition | value;
          break;
        case PATCH_OP_BINARY_EQUALS:
          condition = condition == value;
          break;
        case PATCH_OP_BINARY_NOT_EQUALS:
          condition = condition != value;
          break;
        default:
          r = AVL_EC_GENERAL_FAIL;
          return (r);
        }
      } //for conditions
    }

    if (condition)
    {
      cmd = patch_read32_AVL62X1(patch_data, &patch_idx);

      switch (cmd)
      {
      case PATCH_CMD_PING: //1
      {
        r |= IBase_SendRxOP_AVL62X1(CMD_PING, pAVL_Chip);

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        rv0_idx = patch_read32_AVL62X1(patch_data, &patch_idx);
        variable_array[rv0_idx] = (r == AVL_EC_OK);
        patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's
        break;
      }
      case PATCH_CMD_VALIDATE_CRC: //0
      {
        exp_crc_val = patch_read32_AVL62X1(patch_data, &patch_idx);
        start_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        length = patch_read32_AVL62X1(patch_data, &patch_idx);

        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_DMD_fw_command_args_addr_iaddr, args_addr);
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, args_addr + 0, start_addr);
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, args_addr + 4, length);
        r |= IBase_SendRxOP_AVL62X1(CMD_CALC_CRC, pAVL_Chip);

        r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, args_addr + 8, &crc_result);

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        rv0_idx = patch_read32_AVL62X1(patch_data, &patch_idx);
        variable_array[rv0_idx] = (crc_result == exp_crc_val);

        patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's

        break;
      }
      case PATCH_CMD_LD_TO_DEVICE: //2
      {
        length = patch_read32_AVL62X1(patch_data, &patch_idx); //in words
        dest_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        src_data_offset = patch_read32_AVL62X1(patch_data, &patch_idx);
        src_data_offset += data_section_offset; //add in base offset
        src_data_offset *= 4;                   //convert to byte offset
#define BURST
#ifdef BURST
        length *= 4; //Convert to byte length

        pPatchDatatemp = patch_data + src_data_offset;
        pPatchDatatemp1 = pPatchDatatemp - 3;
        temp[0] = *(pPatchDatatemp - 1);
        temp[1] = *(pPatchDatatemp - 2);
        temp[2] = *(pPatchDatatemp - 3);
        avl_int_to_3bytes(dest_addr, pPatchDatatemp1);

        r |= avl_bms_write(pAVL_Chip->chip_pub->i2c_addr, pPatchDatatemp1, (uint32_t)(length + 3));

        *pPatchDatatemp1 = temp[2];
        *(pPatchDatatemp1 + 1) = temp[1];
        *(pPatchDatatemp1 + 2) = temp[0];

#else
        for (uint32_t i = 0; i < length; i++)
        {
          //FIXME: make this a burst write
          uint32_t tdata = patch_read32_AVL62X1(patch_data, &src_data_offset);
          r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, dest_addr + i * 4, tdata);
        }
#endif

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet

        break;
      }

      case PATCH_CMD_LD_TO_DEVICE_IMM: //7
      {
        length = patch_read32_AVL62X1(patch_data, &patch_idx); //in bytes
        dest_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        data = patch_read32_AVL62X1(patch_data, &patch_idx);

        if (length == 4)
        {
          r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, dest_addr, data);
        }
        else if (length == 2)
        {
          r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, dest_addr, data);
        }
        else if (length == 1)
        {
          r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, dest_addr, data);
        }

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet
        break;
      }
      case PATCH_CMD_LD_TO_DEVICE_PACKED:
      {
        length = patch_read32_AVL62X1(patch_data, &patch_idx); //in words
        src_data_offset = patch_read32_AVL62X1(patch_data, &patch_idx);

        src_data_offset += data_section_offset; //add in base offset to make it absolute
        src_data_offset *= 4;                   //convert to byte offset
        length *= 4;                            //Convert to byte length

        src_data_offset += 2;                                                               //skip over address offset length. assume it's 2 for now!
        num_records = (patch_data[src_data_offset] << 8) + patch_data[src_data_offset + 1]; //number of records B.E.
        src_data_offset += 2;
        dest_addr = (patch_data[src_data_offset] << 24) + (patch_data[src_data_offset + 1] << 16) +
                    (patch_data[src_data_offset + 2] << 8) + (patch_data[src_data_offset + 3] << 0); //destination address B.E.
        src_data_offset += 4;

        //AVL_puchar pRecordData = new uint8_t[(1<<16)+3];
        for (record_cnt = 0; record_cnt < num_records; record_cnt++)
        {
          addr_offset = (patch_data[src_data_offset] << 8) + patch_data[src_data_offset + 1]; //address offset B.E.
          src_data_offset += 2;
          record_length = (patch_data[src_data_offset] << 8) + patch_data[src_data_offset + 1]; //data len B.E.
          src_data_offset += 2;

          //temporarily save patch data that will be overwritten
          temp[0] = patch_data[src_data_offset - 3];
          temp[1] = patch_data[src_data_offset - 2];
          temp[2] = patch_data[src_data_offset - 1];

          //break address into 3 bytes and put in patch array right in front of data
          avl_int_to_3bytes(dest_addr + addr_offset, &(patch_data[src_data_offset - 3]));
          r |= avl_bms_write(pAVL_Chip->chip_pub->i2c_addr, &(patch_data[src_data_offset - 3]), record_length + 3);

          //restore patch data
          patch_data[src_data_offset - 3] = temp[0];
          patch_data[src_data_offset - 2] = temp[1];
          patch_data[src_data_offset - 1] = temp[2];

          src_data_offset += record_length;
        }
        num_rvs2 = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs2); //no RV's defined yet
        break;
      }
      case PATCH_CMD_RD_FROM_DEVICE: //8 8
      {
        length = patch_read32_AVL62X1(patch_data, &patch_idx); //in bytes
        src_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        rv0_idx = patch_read32_AVL62X1(patch_data, &patch_idx);

        if (length == 4)
        {
          r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, src_addr, &data);
          variable_array[rv0_idx] = data;
        }
        else if (length == 2)
        {
          r |= avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, src_addr, &data1);
          variable_array[rv0_idx] = data1;
        }
        else if (length == 1)
        {
          r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, src_addr, &data2);
          variable_array[rv0_idx] = data2;
        }
        patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's
        break;
      }
      case PATCH_CMD_DMA: //3
      {
        descr_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        num_descr = patch_read32_AVL62X1(patch_data, &patch_idx);

        pPatchDatatemp = patch_data + patch_idx;
        pPatchDatatemp1 = pPatchDatatemp - 3;
        temp[0] = *(pPatchDatatemp - 1);
        temp[1] = *(pPatchDatatemp - 2);
        temp[2] = *(pPatchDatatemp - 3);
        avl_int_to_3bytes(descr_addr, pPatchDatatemp1);

        r |= avl_bms_write(pAVL_Chip->chip_pub->i2c_addr, pPatchDatatemp1, (uint32_t)(num_descr * 3 * 4));
        *pPatchDatatemp1 = temp[2];
        *(pPatchDatatemp1 + 1) = temp[1];
        *(pPatchDatatemp1 + 2) = temp[0];
        patch_idx += 12 * num_descr;

        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_DMD_fw_command_args_addr_iaddr, descr_addr);
        r |= IBase_SendRxOP_AVL62X1(CMD_DMA, pAVL_Chip);

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet
        break;
      }
      case PATCH_CMD_EXTRACT: //4
      {
        type = patch_read32_AVL62X1(patch_data, &patch_idx);
        src_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        dest_addr = patch_read32_AVL62X1(patch_data, &patch_idx);

        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_DMD_fw_command_args_addr_iaddr, args_addr);
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, args_addr + 0, type);
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, args_addr + 4, src_addr);
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, args_addr + 8, dest_addr);

        r |= IBase_SendRxOP_AVL62X1(CMD_DECOMPRESS, pAVL_Chip);

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet
        break;
      }
      case PATCH_CMD_ASSERT_CPU_RESET: //5
      {
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, hw_AVL62X1_cpucore_top_srst, 1);

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet
        break;
      }
      case PATCH_CMD_RELEASE_CPU_RESET: //6
      {
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, hw_AVL62X1_cpucore_top_srst, 0);

        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet
        break;
      }
      case PATCH_CMD_DMA_HW: //9
      {
        descr_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        num_descr = patch_read32_AVL62X1(patch_data, &patch_idx);

        temp[0] = *(patch_data + patch_idx - 1);
        temp[1] = *(patch_data + patch_idx - 2);
        temp[2] = *(patch_data + patch_idx - 3);
        avl_int_to_3bytes(descr_addr, patch_data + patch_idx - 3);

        if (num_descr > 0)
        {
          r |= avl_bms_write(pAVL_Chip->chip_pub->i2c_addr, patch_data + patch_idx - 3, num_descr * 12 + 3);
        }

        *(patch_data + patch_idx - 1) = temp[0];
        *(patch_data + patch_idx - 2) = temp[1];
        *(patch_data + patch_idx - 3) = temp[2];

        patch_idx += num_descr * 3 * 4;
        dma_tries = 0;
        dma_max_tries = 20;
        do
        {
          if (dma_tries > dma_max_tries)
          {
            return AVL_EC_GENERAL_FAIL; //FIXME return a value to check instead, and load the bootstrap
          }
          avl_bsp_delay(10); //ms
          r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, hw_AVL62X1_dma_sys_status, &ready);
          //System::Console::WriteLine("num_dma_tries pre: {0}",dma_tries);
          dma_tries++;
        } while (!(0x01 & ready));

        if (0x01 & ready)
        {
          r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, hw_AVL62X1_dma_sys_cmd, descr_addr); //Trigger DMA
        }

        dma_tries = 0;
        dma_max_tries = 20;
        do
        {
          if (dma_tries > dma_max_tries)
          {
            return AVL_EC_GENERAL_FAIL; //FIXME return a value to check instead, and load the bootstrap
          }
          avl_bsp_delay(10); //ms
          r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, hw_AVL62X1_dma_sys_status, &ready);
          //System::Console::WriteLine("num_dma_tries pre: {0}",dma_tries);
          dma_tries++;
        } while (0x100 & ready);

        //Add return value later
        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet
        break;
      }

      case PATCH_CMD_SET_COND_IMM: //10
      {
        rv = patch_read32_AVL62X1(patch_data, &patch_idx);
        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        rv0_idx = patch_read32_AVL62X1(patch_data, &patch_idx);
        variable_array[rv0_idx] = rv;
        patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's
        break;
      }
      case PATCH_CMD_EXIT:
      {
        got_cmd_exit = 1;
        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx = 4 * (script_start_idx + script_len); //skip over any extraneous RV's
        break;
      }
      case PATCH_CMD_POLL_WAIT:
      {
        length = patch_read32_AVL62X1(patch_data, &patch_idx); //in bytes
        src_addr = patch_read32_AVL62X1(patch_data, &patch_idx);
        match_value = patch_read32_AVL62X1(patch_data, &patch_idx);
        max_polls = patch_read32_AVL62X1(patch_data, &patch_idx);
        polls = 0;
        do
        {
          if (length == 4)
          {
            r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, src_addr, &data);
            if (data == match_value)
              break;
          }
          else if (length == 2)
          {
            r = avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, src_addr, &data1);
            if (data1 == match_value)
              break;
          }
          else if (length == 1)
          {
            r = avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, src_addr, &data2);
            if (data2 == match_value)
              break;
          }
          avl_bsp_delay(10); //ms
          polls += 1;
        } while (polls < max_polls);
        num_rvs = patch_read32_AVL62X1(patch_data, &patch_idx);
        patch_idx += 4 * (num_rvs); //no RV's defined yet
        break;
      }
      } //switch cmd
    }
    else
    {
      patch_idx = next_cmd_idx; //jump to next command
      continue;
    }
  }

  return (r);
}

uint8_t patch_read8_AVL62X1(uint8_t *pPatchBuf, uint32_t *idx)
{
  uint8_t tmp = 0;
  tmp = pPatchBuf[*idx];
  *idx += 1;
  return tmp;
}
uint16_t patch_read16_AVL62X1(uint8_t *pPatchBuf, uint32_t *idx)
{
  uint16_t tmp = 0;
  tmp = (pPatchBuf[*idx + 0] << 8) | (pPatchBuf[*idx + 1]);
  *idx += 2;
  return tmp;
}
uint32_t patch_read32_AVL62X1(uint8_t *pPatchBuf, uint32_t *idx)
{
  uint32_t tmp = 0;
  tmp = (pPatchBuf[*idx + 0] << 24) | (pPatchBuf[*idx + 1] << 16) | (pPatchBuf[*idx + 2] << 8) | pPatchBuf[*idx + 3];
  *idx += 4;
  return tmp;
}


//XLFSR read32 from s_AVL62X1_S2X_PLScramKey_iaddr
uint32_t Convert_XLFSRToN_AVL62X1(uint32_t XLFSR)
{
  uint32_t i = 0;
  uint32_t reg_data = 0x0001; //Initial FLSR is 0x0001;
  XLFSR &= ~(1 << 18);          //clear x(18) just to be safe
  reg_data &= ~(1 << 18);       //clear x(18) just to be safe

  for (i = 0; i <= 262141; i++)
  {
    if (reg_data == XLFSR)
    {
      return i;
    }
    reg_data |= ((reg_data & 0x1) ^ ((reg_data >> 7) & 0x1)) << 18; //XOR x(0) and x(7), assign to x(18)
    reg_data >>= 1;                                                 //shift everything towards x(0).  x(18) becomes x(17)
  }

  return AVL_EC_ConvertXLFSRToN_FAIL; // error ,no find PLS code
}

// example -> Convert_NToXLFSR_AVL62X1(1,1310701),XLFSR=16416 ---> Gold=131070,Root is 16416;
uint32_t Convert_NToXLFSR_AVL62X1(uint32_t reg_data, uint32_t offset)
{

  reg_data &= ~(1 << 18); //clear x(18) just to be safe
  while (offset-- > 0)
  {
    reg_data |= ((reg_data & 0x1) ^ ((reg_data >> 7) & 0x1)) << 18; //XOR x(0) and x(7), assign to x(18)
    reg_data >>= 1;                                                 //shift everything towards x(0).  x(18) becomes x(17)
  }

  return reg_data;
}
