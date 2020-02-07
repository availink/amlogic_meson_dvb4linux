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

#include "AVL62X1_API.h"


uint16_t AVL62X1_GetChipID(uint16_t slave_addr, uint32_t *pChipId)
{
  uint16_t r = AVL_EC_OK;

  r = avl_bms_read32(slave_addr, 0x40000, pChipId);
  return (r);
}

uint16_t AVL62X1_Initialize(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint16_t i = 0;
  uint16_t uiMaxRetries = 100;
  uint16_t delay_unit_ms = 50; //Time out window is uiMaxRetries * delay_unit_ms = 5000ms;

  r |= IBase_Initialize_AVL62X1(pAVL_Chip); //download, boot, load defaults
  while (AVL_EC_OK != IBase_CheckChipReady_AVL62X1(pAVL_Chip))
  {
    if (i++ >= uiMaxRetries)
    {
      r |= AVL_EC_GENERAL_FAIL;
      return (r);
    }
    avl_bsp_delay(delay_unit_ms);
  }

  r |= IBase_Initialize_TunerI2C_AVL62X1(pAVL_Chip); //config i2c repeater
  r |= IRx_Initialize_AVL62X1(pAVL_Chip);
  r |= IRx_SetTunerPola_AVL62X1(pAVL_Chip->chip_pub->tuner_pol, pAVL_Chip);
  r |= IRx_DriveAGC_AVL62X1(AVL62X1_ON, pAVL_Chip);
  r |= IRx_SetMpegMode_AVL62X1(pAVL_Chip);
  r |= IRx_DriveMpegOutput_AVL62X1(AVL62X1_ON, pAVL_Chip);

  return (r);
}

uint16_t AVL62X1_LockTP(struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_stream_info *pStreamInfo, avl_bool_t blind_sym_rate, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

#ifdef _AVL_LINUX_DVB_
  static uint8_t AGC_enabled = 0;

  r |= IBase_SendRxOP_AVL62X1(CMD_HALT, pAVL_Chip);

  if (AGC_enabled == 0)
  {
    r |= IRx_DriveAGC_AVL62X1(AVL62X1_ON, pAVL_Chip);
    AGC_enabled = 1;
  }
#endif

  if (blind_sym_rate)
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_blind_sym_rate_enable_caddr, 1);
  }
  else
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_blind_sym_rate_enable_caddr, 0);
  }

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_blind_cfo_enable_caddr, 1);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_nom_symbol_rate_Hz_iaddr, pCarrierInfo->m_symbol_rate_Hz);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_nom_carrier_freq_Hz_iaddr, pCarrierInfo->m_carrier_freq_offset_Hz);

  //r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr, pCarrierInfo->m_PL_scram_key);//the register 0x85c default value is "AVL62X1_PL_SCRAM_AUTO"

  if (pStreamInfo != nullptr)
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr, pStreamInfo->m_stream_type);
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr, pStreamInfo->m_ISI);
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_PLP_ID_caddr, pStreamInfo->m_PLP_ID);
    r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr_cur, pStreamInfo->m_T2MI_PID); // Must set the T2MI PID for T2MI signal ,general value :0x1000
  }
  else
  {
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr, AVL62X1_UNDETERMINED);
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr, 0);
  }

  r |= IBase_SendRxOP_AVL62X1(CMD_ACQUIRE, pAVL_Chip);

  return (r);
}

uint16_t AVL62X1_GetLockStatus(enum avl62x1_lock_status *eLockStat, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t ucLockStatus = 0;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_sp_lock_caddr, &ucLockStatus);
  if (ucLockStatus != 0)
  {
    *eLockStat = AVL62X1_STATUS_LOCK;
  }
  else
  {
    *eLockStat = AVL62X1_STATUS_UNLOCK;
  }

  return (r);
}

uint16_t AVL62X1_GetLostLockStatus(enum avl62x1_lost_lock_status *eLostLockStat, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t ucLostLock = 0;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_lost_lock_caddr, &ucLostLock);
  if (ucLostLock != 0)
  {
    *eLostLockStat = AVL62X1_Lost_Lock_Yes;
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_lost_lock_caddr, 0);
  }
  else
  {
    *eLostLockStat = AVL62X1_Lost_Lock_No;
  }

  return (r);
}

uint16_t AVL62X1_DiscoverStreams(struct avl62x1_carrier_info *pCarrierInfo, avl_bool_t blind_sym_rate, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  r |= AVL62X1_LockTP(pCarrierInfo, nullptr, blind_sym_rate, pAVL_Chip);
  return (r);
}

uint16_t AVL62X1_GetDiscoveryStatus(enum avl62x1_discovery_status *eDiscoveryStat, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t ucDiscStatus = 0;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_stream_discover_done_caddr, &ucDiscStatus);
  if (ucDiscStatus != 0)
  {
    *eDiscoveryStat = AVL62X1_DISCOVERY_FINISHED;
  }
  else
  {
    *eDiscoveryStat = AVL62X1_DISCOVERY_RUNNING;
  }
  return (r);
}

uint16_t AVL62X1_GetStreamNumber(uint8_t *pStreamNum, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_NumStreams_cur_TP_caddr, pStreamNum);

  return (r);
}

uint16_t AVL62X1_GetStreamList(struct avl62x1_stream_info *pStreams, const uint8_t max_num_streams, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t stream_list_ptr = 0;
  uint32_t stream_list_ptr_2 = 0;
  uint8_t num_streams = 0;
  uint8_t stream = 0;
  uint8_t tmp8 = 0;
  uint16_t tmp16 = 0;
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_DVB_STREAM_addr_iaddr, &stream_list_ptr);
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_DVB_STREAM2_addr_iaddr, &stream_list_ptr_2);

  r |= AVL62X1_GetStreamNumber(&num_streams, pAVL_Chip);
  if (num_streams > max_num_streams)
  {
    num_streams = max_num_streams;
  }
  for (stream = 0; stream < num_streams; stream++)
  {
    //TODO: this could probably be optimized to a single I2C burst read
    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, stream_list_ptr + stream * AVL62X1_DVB_STREAM_struct_size + AVL62X1_DVB_STREAM_CarrierIndex_caddr, &tmp8);
    pStreams[stream].m_carrier_index = tmp8;

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, stream_list_ptr + stream * AVL62X1_DVB_STREAM_struct_size + AVL62X1_DVB_STREAM_StreamType_caddr, &tmp8);
    pStreams[stream].m_stream_type = (avl62x1_dvb_stream_type)tmp8;

    if (pStreams[stream].m_stream_type == AVL62X1_T2MI)
    {
      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, stream_list_ptr + stream * AVL62X1_DVB_STREAM_struct_size + AVL62X1_DVB_STREAM_PLP_ID_caddr, &tmp8);
      pStreams[stream].m_PLP_ID = 0; //tmp8 is invalid

      r |= avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, stream_list_ptr_2 + stream * 4, &tmp16);
      pStreams[stream].m_T2MI_PID = tmp16; // T2MI PID for every ISI when T2MI Stream
    }
    else
    {
      pStreams[stream].m_PLP_ID = 0;
      pStreams[stream].m_T2MI_PID = 0;
    }
    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, stream_list_ptr + stream * AVL62X1_DVB_STREAM_struct_size + AVL62X1_DVB_STREAM_ISI_caddr, &tmp8);
    pStreams[stream].m_ISI = tmp8;
  }

  return (r);
}

uint16_t AVL62X1_SwitchStream(struct avl62x1_stream_info *pStreamInfo, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t uilockstatus = 0;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_fec_lock_caddr, &uilockstatus);
  if (uilockstatus != 0)
  {
    r |= IBase_SendSPOP_AVL62X1(SP_CMD_HALT, pAVL_Chip);
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr, pStreamInfo->m_stream_type);
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr, pStreamInfo->m_ISI);
    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_PLP_ID_caddr, pStreamInfo->m_PLP_ID);
    r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr_cur, pStreamInfo->m_T2MI_PID); // Must set the T2MI PID for T2MI signal ,general value :0x1000

    r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_sp_lock_caddr, 0);
    r |= IBase_SendSPOP_AVL62X1(SP_CMD_ACQUIRE, pAVL_Chip);
  }
  else
  {
    r |= AVL_EC_GENERAL_FAIL; // demod isn't locked.
  }

  return (r);
}

uint16_t AVL62X1_GetSNR(int16_t *piSNR_x100db, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_snr_dB_x100_saddr, (uint16_t *)piSNR_x100db);

  return (r);
}

uint16_t AVL62X1_GetSignalInfo(struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t ucTemp = 0;
  uint32_t uiTemp = 0;

  //If blind carrier freq search was performed, this is the carrier freq as determined
  //  by the blind search algorithm. Otherwise, it is just the nominal carrier freq from the config.
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_carrier_freq_Hz_iaddr, &uiTemp);
  pCarrierInfo->m_carrier_freq_offset_Hz = (int32_t)uiTemp;

  //Difference, in Hertz, between nominal carrier freq and current freq as indicated by the frequency loop.
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_carrier_freq_err_Hz_iaddr, &uiTemp);
  pCarrierInfo->m_carrier_freq_offset_Hz += (int32_t)uiTemp;
  pCarrierInfo->m_carrier_freq_offset_Hz -= pAVL_Chip->chip_priv->carrier_freq_offset_hz;

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_symbol_rate_Hz_iaddr, &uiTemp);
  pCarrierInfo->m_symbol_rate_Hz = uiTemp;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_signal_type_caddr, &ucTemp);
  pCarrierInfo->m_signal_type = (avl62x1_standard)(ucTemp);

  if (pCarrierInfo->m_signal_type == AVL62X1_DVBS)
  {
    pCarrierInfo->m_modulation = AVL62X1_QPSK;

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_dvbs_code_rate_caddr, &ucTemp);
    pCarrierInfo->m_coderate.m_dvbs_code_rate = (avl62x1_dvbs_code_rate)(ucTemp);

    pCarrierInfo->m_roll_off = AVL62X1_RollOff_35;
  }
  else
  {
    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_s2_pilot_on_caddr, &ucTemp);
    pCarrierInfo->m_pilot = (avl62x1_pilot)(ucTemp);

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_s2_fec_len_caddr, &ucTemp);
    pCarrierInfo->m_dvbs2_fec_length = (avl62x1_fec_length)(ucTemp);

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_s2_modulation_caddr, &ucTemp);
    pCarrierInfo->m_modulation = (avl62x1_modulation_mode)(ucTemp);

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_s2_code_rate_caddr, &ucTemp);
    pCarrierInfo->m_coderate.m_dvbs2_code_rate = (avl62x1_dvbs2_code_rate)(ucTemp);

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_ccm1_acm0_caddr, &ucTemp);
    if (ucTemp == 0)
    {
      pCarrierInfo->m_dvbs2_ccm_acm = AVL62X1_DVBS2_ACM;
      pCarrierInfo->m_PLS_ACM = 0;
    }
    else
    {
      pCarrierInfo->m_dvbs2_ccm_acm = AVL62X1_DVBS2_CCM;
      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_ccm_pls_mode_caddr, &ucTemp);
      pCarrierInfo->m_PLS_ACM = ucTemp;
    }

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_alpha_caddr, &ucTemp);
    pCarrierInfo->m_roll_off = (avl62x1_rolloff)(ucTemp);

    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_PLScramKey_iaddr, &uiTemp);
    pCarrierInfo->m_PL_scram_key = uiTemp;

    //r |= AVL_AVL62X1_GetStreamNumber(&pCarrierInfo->m_num_streams, pAVL_Chip);
    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_SIS_MIS_caddr, &ucTemp);
    pCarrierInfo->m_SIS_MIS = (avl62x1_sis_mis)(ucTemp);
  }
  return (r);
}

uint16_t AVL62X1_GetSignalStrength(uint16_t *puiSignalStrength, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint16_t uiTemp = 0;

  r = avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_DMD_rfagc_gain_saddr, &uiTemp);
  if (uiTemp <= 25000)
  {
    *puiSignalStrength = 100;
  }
  else if (uiTemp >= 55000)
  {
    *puiSignalStrength = 10;
  }
  else
  {
    *puiSignalStrength = (55000 - uiTemp) * 90 / 30000 + 10;
  }

  return (r);
}

uint16_t AVL62X1_GetSignalQuality(uint16_t *puiSignalQuality, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  int16_t iTemp = 0;

  r = AVL62X1_GetSNR(&iTemp, pAVL_Chip);
  if (iTemp >= 2500)
  {
    *puiSignalQuality = 100;
  }
  else if (iTemp <= 0)
  {
    *puiSignalQuality = 10;
  }
  else
  {
    *puiSignalQuality = iTemp * 90 / 2500 + 10;
  }

  return (r);
}


uint16_t AVL62X1_ResetPER(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t uiTemp = 0;

  pAVL_Chip->chip_priv->error_stats.m_SwCntNumPkts.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_SwCntNumPkts.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_SwCntPktErrors.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_SwCntPktErrors.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_NumPkts.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_NumPkts.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_PktErrors.high_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_PktErrors.low_word = 0;
  pAVL_Chip->chip_priv->error_stats.m_PER = 0;

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &uiTemp);
  uiTemp |= 0x00000001;
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &uiTemp);
  uiTemp |= 0x00000008;
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
  uiTemp |= 0x00000001;
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
  uiTemp &= 0xFFFFFFFE;
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);

  return (r);
}

uint16_t AVL62X1_GetPER(uint32_t *puiPER_x1e9, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  enum avl62x1_lock_status lock_status = AVL62X1_STATUS_UNLOCK;
  uint32_t uiHwCntPktErrors = 0;
  uint32_t uiHwCntNumPkts = 0;
  uint32_t uiTemp = 0;
  struct avl_uint64 uiTemp64 = {0, 0};

  r |= AVL62X1_GetLockStatus(&lock_status, pAVL_Chip);

  //record the lock status before return the PER
  if (AVL62X1_STATUS_LOCK == lock_status)
  {
    pAVL_Chip->chip_priv->error_stats.m_LostLock = 0;
  }
  else
  {
    pAVL_Chip->chip_priv->error_stats.m_LostLock = 1;
    return *puiPER_x1e9 = AVL_CONSTANT_10_TO_THE_9TH;
  }

  r = avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__packet_err_cnt, &uiHwCntPktErrors);
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__packet_num, &uiHwCntNumPkts);

  if (uiHwCntNumPkts > (1 << 30))
  {
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &uiTemp);
    uiTemp |= 0x00000001;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__packet_err_cnt, &uiHwCntPktErrors);
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__packet_num, &uiHwCntNumPkts);
    uiTemp &= 0xFFFFFFFE;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, uiTemp);
    avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_SwCntNumPkts, uiHwCntNumPkts);
    avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_SwCntPktErrors, uiHwCntPktErrors);
    uiHwCntNumPkts = 0;
    uiHwCntPktErrors = 0;
  }

  pAVL_Chip->chip_priv->error_stats.m_NumPkts.high_word = pAVL_Chip->chip_priv->error_stats.m_SwCntNumPkts.high_word;
  pAVL_Chip->chip_priv->error_stats.m_NumPkts.low_word = pAVL_Chip->chip_priv->error_stats.m_SwCntNumPkts.low_word;
  avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_NumPkts, uiHwCntNumPkts);
  pAVL_Chip->chip_priv->error_stats.m_PktErrors.high_word = pAVL_Chip->chip_priv->error_stats.m_SwCntPktErrors.high_word;
  pAVL_Chip->chip_priv->error_stats.m_PktErrors.low_word = pAVL_Chip->chip_priv->error_stats.m_SwCntPktErrors.low_word;
  avl_add_32to64(&pAVL_Chip->chip_priv->error_stats.m_PktErrors, uiHwCntPktErrors);

  // Compute the PER
  avl_mult_32to64(&uiTemp64, pAVL_Chip->chip_priv->error_stats.m_PktErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
  pAVL_Chip->chip_priv->error_stats.m_PER = avl_divide_64(pAVL_Chip->chip_priv->error_stats.m_NumPkts, uiTemp64);
  //keep the PER user wanted
  *puiPER_x1e9 = pAVL_Chip->chip_priv->error_stats.m_PER;

  return (r);
}

/************************************************************************/
/* Diseqc                                                               */
/************************************************************************/
#define Diseqc_delay 20

uint16_t AVL62X1_IDiseqc_ReadModulationData(uint8_t *pucBuff, uint8_t *pucSize, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;
  uint32_t i2 = 0;
  uint8_t pucBuffTemp[4] = {0};

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_rx_st, &i1);
  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i2);
  if ((i2 >> 8) & 0x01)
  {
    pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_InModulation;
  }
  if (AVL62X1_DOS_InModulation == pAVL_Chip->chip_priv->diseqc_op_status)
  {
    // In modulation mode
    if ((!((i2 >> 8) & 0x01) && (0x00000004 == (i1 & 0x00000004))) || (((i2 >> 8) & 0x01) && (0x00000004 != (i1 & 0x00000004))))
    {
      *pucSize = (uint8_t)((i1 & 0x00000078) >> 3);
      //Receive data
      for (i1 = 0; i1 < *pucSize; i1++)
      {
        r |= avl_bms_read(pAVL_Chip->chip_pub->i2c_addr, diseqc__rx_fifo, pucBuffTemp, 4);
        pucBuff[i1] = pucBuffTemp[3];
      }
    }
    else
    {
      r = AVL_EC_GENERAL_FAIL;
    }
  }
  else
  {
    r = AVL_EC_GENERAL_FAIL;
  }

  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));

  return (r);
}

uint16_t AVL62X1_IDiseqc_SendModulationData(const uint8_t *pucBuff, uint8_t ucSize, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;
  uint32_t i2 = 0;
  uint8_t pucBuffTemp[8] = {0};
  uint8_t Continuousflag = 0;
  uint16_t uiTempOutTh = 0;

  if (ucSize > 8)
  {
    r = AVL_EC_MemoryRunout;
  }
  else
  {
    r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
    r |= IDiseqc_IsSafeToSwitchMode_AVL62X1(pAVL_Chip);
    if (AVL_EC_OK == r)
    {
      if (pAVL_Chip->chip_priv->diseqc_op_status == AVL62X1_DOS_InContinuous)
      {
        r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
        if ((i1 >> 10) & 0x01)
        {
          Continuousflag = 1;
          i1 &= 0xfffff3ff;
          r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
          r |= avl_bsp_delay(Diseqc_delay); //delay 20ms
        }
      }
      //reset rx_fifo
      r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_rx_cntrl, &i2);
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_rx_cntrl, (i2 | 0x01));
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_rx_cntrl, (i2 & 0xfffffffe));

      r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
      i1 &= 0xfffffff8; //set to modulation mode and put it to FIFO load mode
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);

      //trunk address
      avl_int_to_3bytes(diseqc__tx_fifo, pucBuffTemp);
      pucBuffTemp[3] = 0;
      pucBuffTemp[4] = 0;
      pucBuffTemp[5] = 0;
      for (i2 = 0; i2 < ucSize; i2++)
      {
        pucBuffTemp[6] = pucBuff[i2];

        r |= avl_bms_write(pAVL_Chip->chip_pub->i2c_addr, pucBuffTemp, 7);
      }
      i1 |= (1 << 2); //start fifo transmit.
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);

      if (AVL_EC_OK == r)
      {
        pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_InModulation;
      }
      do
      {
        r |= avl_bsp_delay(1);
        if (++uiTempOutTh > 500)
        {
          r |= AVL_EC_TimeOut;
          r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
          return (r);
        }
        r = avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_st, &i1);
      } while (1 != ((i1 & 0x00000040) >> 6));

      r = avl_bsp_delay(Diseqc_delay); //delay 20ms
      if (Continuousflag == 1)          //resume to send out wave
      {
        //No data in FIFO
        r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
        i1 &= 0xfffffff8;
        i1 |= 0x03; //switch to continuous mode
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);

        //start to send out wave
        i1 |= (1 << 10);
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
        if (AVL_EC_OK == r)
        {
          pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_InContinuous;
        }
      }
    }
    r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  }

  return (r);
}

uint16_t AVL62X1_IDiseqc_GetTxStatus(struct avl62x1_diseqc_tx_status *pTxStatus, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  if ((AVL62X1_DOS_InModulation == pAVL_Chip->chip_priv->diseqc_op_status) || (AVL62X1_DOS_InTone == pAVL_Chip->chip_priv->diseqc_op_status))
  {
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_st, &i1);
    pTxStatus->m_TxDone = (uint8_t)((i1 & 0x00000040) >> 6);
    pTxStatus->m_TxFifoCount = (uint8_t)((i1 & 0x0000003c) >> 2);
  }
  else
  {
    r |= AVL_EC_GENERAL_FAIL;
  }
  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));

  return (r);
}

uint16_t AVL62X1_IDiseqc_GetRxStatus(struct avl62x1_diseqc_rx_status *pRxStatus, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  if (AVL62X1_DOS_InModulation == pAVL_Chip->chip_priv->diseqc_op_status)
  {
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_rx_st, &i1);
    pRxStatus->m_RxDone = (uint8_t)((i1 & 0x00000004) >> 2);
    pRxStatus->m_RxFifoCount = (uint8_t)((i1 & 0x000000078) >> 3);
  }
  else
  {
    r |= AVL_EC_GENERAL_FAIL;
  }
  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));

  return (r);
}

uint16_t AVL62X1_IDiseqc_SendTone(uint8_t ucTone, uint8_t ucCount, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;
  uint32_t i2 = 0;
  uint8_t pucBuffTemp[8];
  uint8_t Continuousflag = 0;
  uint16_t uiTempOutTh = 0;

  if (ucCount > 8)
  {
    r = AVL_EC_MemoryRunout;
  }
  else
  {
    r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
    r |= IDiseqc_IsSafeToSwitchMode_AVL62X1(pAVL_Chip);

    if (AVL_EC_OK == r)
    {
      if (pAVL_Chip->chip_priv->diseqc_op_status == AVL62X1_DOS_InContinuous)
      {
        r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
        if ((i1 >> 10) & 0x01)
        {
          Continuousflag = 1;
          i1 &= 0xfffff3ff;
          r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
          r |= avl_bsp_delay(Diseqc_delay); //delay 20ms
        }
      }
      //No data in the FIFO.
      r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
      i1 &= 0xfffffff8; //put it into the FIFO load mode.
      if (0 == ucTone)
      {
        i1 |= 0x01;
      }
      else
      {
        i1 |= 0x02;
      }
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);

      //trunk address
      avl_int_to_3bytes(diseqc__tx_fifo, pucBuffTemp);
      pucBuffTemp[3] = 0;
      pucBuffTemp[4] = 0;
      pucBuffTemp[5] = 0;
      pucBuffTemp[6] = 1;

      for (i2 = 0; i2 < ucCount; i2++)
      {
        r |= avl_bms_write(pAVL_Chip->chip_pub->i2c_addr, pucBuffTemp, 7);
      }

      i1 |= (1 << 2); //start fifo transmit.
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
      if (AVL_EC_OK == r)
      {
        pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_InTone;
      }
      do
      {
        r |= avl_bsp_delay(1);
        if (++uiTempOutTh > 500)
        {
          r |= AVL_EC_TimeOut;
          r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
          return (r);
        }
        r = avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_st, &i1);
      } while (1 != ((i1 & 0x00000040) >> 6));

      r = avl_bsp_delay(Diseqc_delay); //delay 20ms
      if (Continuousflag == 1)          //resume to send out wave
      {
        //No data in FIFO
        r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
        i1 &= 0xfffffff8;
        i1 |= 0x03; //switch to continuous mode
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);

        //start to send out wave
        i1 |= (1 << 10);
        r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
        if (AVL_EC_OK == r)
        {
          pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_InContinuous;
        }
      }
    }
    r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  }

  return (r);
}

uint16_t AVL62X1_IDiseqc_Start22K(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  r |= IDiseqc_IsSafeToSwitchMode_AVL62X1(pAVL_Chip);

  if (AVL_EC_OK == r)
  {
    //No data in FIFO
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
    i1 &= 0xfffffff8;
    i1 |= 0x03; //switch to continuous mode
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);

    //start to send out wave
    i1 |= (1 << 10);
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
    if (AVL_EC_OK == r)
    {
      pAVL_Chip->chip_priv->diseqc_op_status = AVL62X1_DOS_InContinuous;
    }
  }
  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));

  return (r);
}

uint16_t AVL62X1_IDiseqc_Stop22K(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t i1 = 0;

  r = avl_bsp_wait_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));
  if (AVL62X1_DOS_InContinuous == pAVL_Chip->chip_priv->diseqc_op_status)
  {
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, &i1);
    i1 &= 0xfffff3ff;
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, diseqc__diseqc_tx_cntrl, i1);
  }

  r |= avl_bsp_release_semaphore(&(pAVL_Chip->chip_priv->m_semDiseqc));

  return (r);
}

/************************************************************************/
/* Tuner I2C                                                            */
/************************************************************************/
uint16_t AVL62X1_OpenTunerI2C(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl, 0x07);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl, 0x07);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl, 0x07);

  return (r);
}

uint16_t AVL62X1_CloseTunerI2C(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl, 0x06);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl, 0x06);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl, 0x06);

  return (r);
}

/************************************************************************/
/* GPIO                                                                 */
/************************************************************************/
uint16_t AVL62X1_SetGPIODir(enum avl62x1_gpio_pin ePin, enum avl62x1_gpio_pin_dir eDir, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  switch (ePin)
  {
  case AVL62X1_GPIO_Pin_TUNER_SDA:
    if (eDir == AVL62X1_GPIO_DIR_OUTPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_data2_sel, (uint32_t)(AVL62X1_GPIO_VALUE_LOGIC_0));
    }
    else if (eDir == AVL62X1_GPIO_DIR_INPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_data2_sel, (uint32_t)(AVL62X1_GPIO_VALUE_HIGH_Z));
    }
    else
    {
      return AVL_EC_GENERAL_FAIL;
    }
    break;
  case AVL62X1_GPIO_Pin_TUNER_SCL:
    if (eDir == AVL62X1_GPIO_DIR_OUTPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_clk2_sel, (uint32_t)(AVL62X1_GPIO_VALUE_LOGIC_0));
    }
    else if (eDir == AVL62X1_GPIO_DIR_INPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_clk2_sel, (uint32_t)(AVL62X1_GPIO_VALUE_HIGH_Z));
    }
    else
    {
      return AVL_EC_GENERAL_FAIL;
    }
    break;
  case AVL62X1_GPIO_Pin_S_AGC2:
    if (eDir == AVL62X1_GPIO_DIR_OUTPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__agc2_sel, (uint32_t)(AVL62X1_GPIO_VALUE_LOGIC_0));
    }
    else if (eDir == AVL62X1_GPIO_DIR_INPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__agc2_sel, (uint32_t)(AVL62X1_GPIO_VALUE_HIGH_Z));
    }
    else
    {
      return AVL_EC_GENERAL_FAIL;
    }
    break;
  case AVL62X1_GPIO_Pin_LNB_PWR_EN:
    if (eDir == AVL62X1_GPIO_DIR_OUTPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_0_sel, (uint32_t)(AVL62X1_GPIO_VALUE_LOGIC_0));
    }
    else if (eDir == AVL62X1_GPIO_DIR_INPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_0_sel, (uint32_t)(AVL62X1_GPIO_VALUE_HIGH_Z));
    }
    else
    {
      return AVL_EC_GENERAL_FAIL;
    }
    break;
  case AVL62X1_GPIO_Pin_LNB_PWR_SEL:
    if (eDir == AVL62X1_GPIO_DIR_OUTPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_1_sel, (uint32_t)(AVL62X1_GPIO_VALUE_LOGIC_0));
    }
    else if (eDir == AVL62X1_GPIO_DIR_INPUT)
    {
      r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_1_sel, (uint32_t)(AVL62X1_GPIO_VALUE_HIGH_Z));
    }
    else
    {
      return AVL_EC_GENERAL_FAIL;
    }
    break;
  default:
    return AVL_EC_GENERAL_FAIL;
  }
  return (r);
}

uint16_t AVL62X1_SetGPIOVal(enum avl62x1_gpio_pin ePin, enum avl62x1_gpio_pin_value eVal, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  if ((eVal != AVL62X1_GPIO_VALUE_LOGIC_0) && (eVal != AVL62X1_GPIO_VALUE_LOGIC_1) && (eVal != AVL62X1_GPIO_VALUE_HIGH_Z))
    return AVL_EC_GENERAL_FAIL;

  switch (ePin)
  {
  case AVL62X1_GPIO_Pin_TUNER_SDA:
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_data2_sel, (uint32_t)(eVal));
    break;
  case AVL62X1_GPIO_Pin_TUNER_SCL:
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_clk2_sel, (uint32_t)(eVal));
    break;
  case AVL62X1_GPIO_Pin_S_AGC2:
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__agc2_sel, (uint32_t)(eVal));
    break;
  case AVL62X1_GPIO_Pin_LNB_PWR_EN:
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_0_sel, (uint32_t)(eVal));
    break;
  case AVL62X1_GPIO_Pin_LNB_PWR_SEL:
    r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_1_sel, (uint32_t)(eVal));
    break;
  default:
    return AVL_EC_GENERAL_FAIL;
  }
  return (r);
}

uint16_t AVL62X1_GetGPIOVal(enum avl62x1_gpio_pin ePin, enum avl62x1_gpio_pin_value *peVal, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t tmp = 0;

  switch (ePin)
  {
  case AVL62X1_GPIO_Pin_TUNER_SDA:
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_data2_i, &tmp);
    break;
  case AVL62X1_GPIO_Pin_TUNER_SCL:
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__i2c_clk2_i, &tmp);
    break;
  case AVL62X1_GPIO_Pin_S_AGC2:
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__agc2_i, &tmp);
    break;
  case AVL62X1_GPIO_Pin_LNB_PWR_EN:
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_0_i, &tmp);
    break;
  case AVL62X1_GPIO_Pin_LNB_PWR_SEL:
    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, gpio_debug__lnb_cntrl_1_i, &tmp);
    break;
  default:
    return AVL_EC_GENERAL_FAIL;
  }

  *peVal = (avl62x1_gpio_pin_value)tmp;
  return (r);
}

/************************************************************************/
/* BlindScan                                                            */
/************************************************************************/
//Start the blind scan operation on a single tuner step
uint16_t AVL62X1_BlindScan_Start(struct avl62x1_blind_scan_params *pBSParams, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint32_t samp_rate_Hz = 0;
  uint16_t samp_rate_ratio = 0;

  //r |= IBase_SendRxOP_AVL62X1(CMD_ACQUIRE, pAVL_Chip);

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_sample_rate_Hz_iaddr, &samp_rate_Hz);
  if (samp_rate_Hz == 0)
  {
    samp_rate_ratio = (1 << 15);
  }
  else
  {
    //TunerLPF is single-sided, ratio based on double-sided
    samp_rate_ratio = (uint16_t)(((uint32_t)(2 * pBSParams->m_TunerLPF_100kHz) * (1 << 15)) / (samp_rate_Hz / 100000));
  }

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_nom_carrier_freq_Hz_iaddr, 0);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr, AVL62X1_PL_SCRAM_AUTO);
  r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_BWfilt_to_Rsamp_ratio_saddr, samp_rate_ratio);
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_search_range_percent_caddr, 90); //TODO
  r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_blind_scan_min_sym_rate_kHz_saddr, pBSParams->m_MinSymRate_kHz);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_bs_cent_freq_tuner_Hz_iaddr, pBSParams->m_TunerCenterFreq_100kHz * 100 * 1000);

  r |= IBase_SendRxOP_AVL62X1(CMD_BLIND_SCAN, pAVL_Chip);

  return (r);
}

//Cancel blind scan process
uint16_t AVL62X1_BlindScan_Cancel(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= IBase_SendRxOP_AVL62X1(CMD_HALT, pAVL_Chip);

  return (r);
}

//Get the status of a currently-running blind scan operation (single tuner step)
uint16_t AVL62X1_BlindScan_GetStatus(struct avl62x1_blind_scan_info *pBSInfo, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t tmp8 = 0;
  uint8_t tmp8_1 = 0;
  uint32_t tmp32 = 0;
  enum avl62x1_functional_mode func_mode;

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_current_bs_pair_index_caddr, &tmp8);
  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_bs_num_carrier_candidates_caddr, &tmp8_1);
  if (tmp8_1 == 0)
  {
    pBSInfo->m_ScanProgress = 0;
  }
  else
  {
    pBSInfo->m_ScanProgress = (uint8_t)((100 * (uint32_t)tmp8) / (uint32_t)tmp8_1);
  }

  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_bs_num_confirmed_carriers_caddr, &tmp8);
  if (tmp8 == 255)
  {
    pBSInfo->m_NumCarriers = 0;
  }
  else
  {
    pBSInfo->m_NumCarriers = tmp8;
  }

  r |= IBase_GetFunctionMode_AVL62X1(&func_mode, pAVL_Chip);
  pBSInfo->m_BSFinished = (uint8_t)((func_mode == AVL62X1_FuncMode_Idle) && (tmp8 != 255)); //idle and num confirmed carriers not init value

  pBSInfo->m_NumStreams = 0; //DEPRECATED

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_bs_next_start_freq_Hz_iaddr, &tmp32);
  pBSInfo->m_NextFreqStep_Hz = tmp32;

  return (r);
}

uint16_t AVL62X1_BlindScan_GetCarrierList(const struct avl62x1_blind_scan_params *pBSParams, struct avl62x1_blind_scan_info *pBSInfo, struct avl62x1_carrier_info *pCarriers, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t carrier = 0;
  uint8_t tmp8 = 0;
  uint16_t tmp16 = 0;
  uint32_t carrier_list_ptr = 0;
  uint32_t tmp32 = 0;

  AVL62X1_BlindScan_GetStatus(pBSInfo, pAVL_Chip);
  if (pBSInfo->m_BSFinished == 0)
  {
    r = AVL_EC_RUNNING;
    return (r);
  }

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_bs_carrier_list_address_iaddr, &carrier_list_ptr);

  for (carrier = 0; carrier < pBSInfo->m_NumCarriers; carrier++)
  {
    //TODO: this could probably be optimized to a single I2C burst read
    pCarriers[carrier].m_carrier_index = carrier;
    pCarriers[carrier].m_roll_off = AVL62X1_RollOff_UNKNOWN;

    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_CarrierFreqHz_iaddr, &tmp32);
    pCarriers[carrier].m_carrier_freq_offset_Hz = 0;
    pCarriers[carrier].m_rf_freq_kHz = (uint32_t)(pBSParams->m_TunerCenterFreq_100kHz * 100 + ((int32_t)tmp32 + 500) / 1000);

    r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_SymbolRateHz_iaddr, &tmp32);
    pCarriers[carrier].m_symbol_rate_Hz = tmp32;

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_SignalType_caddr, &tmp8);
    pCarriers[carrier].m_signal_type = (avl62x1_standard)tmp8;

    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_PLScramKeyMSBs_caddr, &tmp8);
    pCarriers[carrier].m_PL_scram_key = tmp8;
    pCarriers[carrier].m_PL_scram_key <<= 16;

    r |= avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_PLScramKeyLSBs_saddr, &tmp16);
    pCarriers[carrier].m_PL_scram_key |= tmp16;

    pCarriers[carrier].m_num_streams = 0;
    pCarriers[carrier].m_SIS_MIS = AVL62X1_SIS_MIS_UNKNOWN;

    r |= avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_SNR_dB_x100_saddr, &tmp16);
    pCarriers[carrier].m_SNR_dB_x100 = tmp16;

    if (pCarriers[carrier].m_signal_type == AVL62X1_DVBS2)
    {
      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_PLS_ACM_caddr, &tmp8);
      pCarriers[carrier].m_PLS_ACM = tmp8;
      pCarriers[carrier].m_dvbs2_ccm_acm = (avl62x1_dvbs2_ccm_acm)(pCarriers[carrier].m_PLS_ACM != 0);

      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_Mod_caddr, &tmp8);
      pCarriers[carrier].m_modulation = (avl62x1_modulation_mode)tmp8;

      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_Pilot_caddr, &tmp8);
      pCarriers[carrier].m_pilot = (avl62x1_pilot)tmp8;

      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_FECLen_caddr, &tmp8);
      pCarriers[carrier].m_dvbs2_fec_length = (avl62x1_fec_length)tmp8;

      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_CodeRate_caddr, &tmp8);
      pCarriers[carrier].m_coderate.m_dvbs2_code_rate = (avl62x1_dvbs2_code_rate)tmp8;
    }
    else if (pCarriers[carrier].m_signal_type == AVL62X1_DVBS)
    {
      pCarriers[carrier].m_PLS_ACM = 16; //QPSK 1/2
      pCarriers[carrier].m_dvbs2_ccm_acm = AVL62X1_DVBS2_CCM;
      pCarriers[carrier].m_modulation = AVL62X1_QPSK;
      pCarriers[carrier].m_pilot = AVL62X1_Pilot_OFF;

      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, carrier_list_ptr + carrier * AVL62X1_SAT_CARRIER_struct_size + AVL62X1_SAT_CARRIER_CodeRate_caddr, &tmp8);
      pCarriers[carrier].m_coderate.m_dvbs_code_rate = (avl62x1_dvbs_code_rate)tmp8;
    }
  } //for carriers

  return (r);
}

uint16_t AVL62X1_BlindScan_GetStreamList(struct avl62x1_carrier_info *pCarrier, struct avl62x1_stream_info *pStreams, const uint8_t max_num_streams, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t tmp8;

  r |= AVL62X1_GetStreamNumber(&tmp8, pAVL_Chip);
  if (pCarrier != nullptr)
  {
    pCarrier->m_num_streams = tmp8;
  }

  r |= AVL62X1_GetStreamList(pStreams, max_num_streams, pAVL_Chip);

  if (pCarrier != nullptr)
  {
    r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_SIS_MIS_caddr, &tmp8);
    pCarrier->m_SIS_MIS = (avl62x1_sis_mis)(tmp8);
  }

  return (r);
}

uint16_t AVL62X1_BlindScan_ConfirmCarrier(const struct avl62x1_blind_scan_params *pBSParams, struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  int32_t cfo_Hz = 0;
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_blind_sym_rate_enable_caddr, 0);
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_blind_cfo_enable_caddr, 0);
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_nom_symbol_rate_Hz_iaddr, pCarrierInfo->m_symbol_rate_Hz);

  //back-calculate CFO from BS RF freq result
  cfo_Hz = ((int32_t)(pCarrierInfo->m_rf_freq_kHz) - (int32_t)(pBSParams->m_TunerCenterFreq_100kHz) * 100) * 1000;
  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_nom_carrier_freq_Hz_iaddr, (uint32_t)(cfo_Hz));

  r |= avl_bms_write32(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr, pCarrierInfo->m_PL_scram_key);
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr, AVL62X1_UNDETERMINED);
  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr, 0);

  r |= IBase_SendRxOP_AVL62X1(CMD_ACQUIRE, pAVL_Chip);
  return (r);
}

//When calling this function in blindscan mode, either set
//  pTuner->ucBlindScanMode = 1, pCarrierInfo = nullptr, or pCarrierInfo->m_symbol_rate_Hz = 0xFFFFFFFF
uint16_t AVL62X1_Optimize_Carrier(struct AVL_Tuner *pTuner, struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  const uint32_t sym_rate_error_Hz = 5 * 1000 * 1000;
  uint32_t carrier_BW_Hz = 0;
  uint32_t maxLPF_Hz = 0;
  uint32_t minLPF_Hz = 0;
  int32_t IF_Hz = 0;
  uint32_t tuner_step_size_Hz = 0;
  uint32_t LPF_step_size_Hz = 0;

  if (pTuner->fpGetRFFreqStepSize == nullptr)
  {
    tuner_step_size_Hz = 250000;
  }
  else
  {
    pTuner->fpGetRFFreqStepSize(pTuner, &tuner_step_size_Hz);
  }
  if (pTuner->fpGetMaxLPF == nullptr)
  {
    maxLPF_Hz = 34000000;
  }
  else
  {
    pTuner->fpGetMaxLPF(pTuner, &maxLPF_Hz);
  }
  if (pTuner->fpGetMinLPF == nullptr)
  {
    minLPF_Hz = 10000000;
  }
  else
  {
    pTuner->fpGetMinLPF(pTuner, &minLPF_Hz);
  }
  if (pTuner->fpGetLPFStepSize == nullptr)
  {
    LPF_step_size_Hz = 1000000;
  }
  else
  {
    pTuner->fpGetLPFStepSize(pTuner, &LPF_step_size_Hz);
  }

  if (pCarrierInfo == nullptr)
  {
    pTuner->ucBlindScanMode = 1;
  }
  else if (pCarrierInfo->m_symbol_rate_Hz == 0xFFFFFFFF)
  {
    pTuner->ucBlindScanMode = 1;
  }
  if (pTuner->ucBlindScanMode == 1)
  {
    //Set tuner LPF wide open
    pTuner->uiLPFHz = maxLPF_Hz;
  }
  else
  {
    //double-sided carrier BW
    carrier_BW_Hz = (pCarrierInfo->m_symbol_rate_Hz / 100) * 135; //35% rolloff
    carrier_BW_Hz += sym_rate_error_Hz + tuner_step_size_Hz / 2;

    if (pCarrierInfo->m_symbol_rate_Hz < AVL62X1_IF_SHIFT_MAX_SR_HZ)
    {
      //remove apriori CFO
      pCarrierInfo->m_rf_freq_kHz += pCarrierInfo->m_carrier_freq_offset_Hz / 1000;
      //adjust by IF
      IF_Hz = (carrier_BW_Hz / 2);
      pCarrierInfo->m_rf_freq_kHz -= IF_Hz / 1000;
      pCarrierInfo->m_carrier_freq_offset_Hz = IF_Hz;
      pTuner->uiRFFrequencyHz = pCarrierInfo->m_rf_freq_kHz * 1000;
      carrier_BW_Hz *= 2;
    }

    pAVL_Chip->chip_priv->carrier_freq_offset_hz = pCarrierInfo->m_carrier_freq_offset_Hz;
    //Set tuner LPF so that carrier edge is an octave from the LPF 3dB point
    pTuner->uiLPFHz = avl_min_32(carrier_BW_Hz + LPF_step_size_Hz / 2, maxLPF_Hz);
    pTuner->uiLPFHz = avl_max_32(pTuner->uiLPFHz, minLPF_Hz);
  }

  return (r);
}

/* AVL62X1_LockTP() or AVL62X1_SwitchStream() must be called after calling either of these functions
 * in order for them to take effect.
 */
uint16_t AVL62X1_Enable_T2MIRawMode(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_raw_t2mi_mode_caddr, 1);

  return (r);
}

uint16_t AVL62X1_Disable_T2MIRawMode(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write8(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_raw_t2mi_mode_caddr, 0);

  return (r);
}

uint16_t AVL62X1_GetPLSXLFSRValue(uint32_t *pXLFSRValue, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_read32(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_S2X_PLScramKey_iaddr, pXLFSRValue);
  *pXLFSRValue &= ~(1 << 18); // Clear bit18( Indicator Bit) for PLS Root Value
  return (r);
}
/* AVL62X1_LockTP() or AVL62X1_SwitchStream() must be called after calling either of these functions
* in order for them to take effect.,if the T2MI TS PID is not 0x1000, please set it .
*/
uint16_t AVL62X1_Manual_Set_T2MI_PID(uint16_t T2MI_PID, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr, T2MI_PID); //default value 0x1000 //Set T2MI ID

  return (r);
}

uint16_t AVL62X1_Manual_Set_T2MI_PID_1(uint16_t T2MI_PID, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr_1, T2MI_PID); //default value 0x40 //Set T2MI ID

  return (r);
}

uint16_t AVL62X1_AutoDetect_T2MI_PID_Enable(struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_detect_auto_en, 1); //default value 0

  return (r);
}

// Get T2MI PID value ,and save it after lock TP without isi id ,plp id stream type parameter with AVL62X1_LockTP(&CarrierInfo, nullptr, AVL_FALSE, pAVL_Chip)
uint16_t AVL62X1_Get_Current_Stream_T2MI_PID(uint16_t *puiT2MI_PID, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_read16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr_cur, puiT2MI_PID); //Get current T2MI stream T2MI PID

  return (r);
}

uint16_t AVL62X1_Set_Current_Stream_T2MI_PID(uint16_t uiT2MI_PID, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr_cur, uiT2MI_PID); //Set current T2MI stream T2MI PID

  return (r);
}

//set collect frame num to chek MPLP ID ,it is related to time for get PLP ID
uint16_t AVL62X1_Set_T2MI_MPLP_id_ScanTime(uint16_t Frame_Num, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;

  r |= avl_bms_write16(pAVL_Chip->chip_pub->i2c_addr, c_AVL62X1_SP_S2X_sp_t2mi_mplp_id_scan_time_caddr, Frame_Num); //default value 10

  return (r);
}

uint16_t AVL62X1_Get_T2MI_PLPid(struct avl62x1_t2mi_mplp *PLP_List, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t PLP_Num = 0, i = 0;
  uint8_t TempPLPID = 0;
  const uint16_t uiTimeDelay = 20;
  const uint16_t uiMaxRetries = 100;
  enum avl62x1_lock_status lock_status = AVL62X1_STATUS_UNLOCK;
  enum avl62x1_dvb_stream_type StreamType = AVL62X1_TRANSPORT;
  r |= AVL62X1_GetLockStatus(&lock_status, pAVL_Chip);
  r |= AVL62X1_Get_StreamType(&StreamType, pAVL_Chip);
  if ((lock_status == AVL62X1_STATUS_LOCK) && (AVL62X1_T2MI == StreamType))
  {
    do
    {
      r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_t2mi_mplp_id_num_caddr, &PLP_Num);
      if (uiMaxRetries < i++)
      {
        break;
      }
      avl_bsp_delay(uiTimeDelay);
    } while (0xff == PLP_Num);

    if (PLP_Num != 0xff) // scan PLP id done ,and get PLP num and PLP id list
    {
      PLP_List->PLP_Num = PLP_Num;
      if (PLP_Num > 16)
      {
        PLP_List->PLP_Num = 16; //Max only support 16 PLPID
      }
      for (i = 0; i < PLP_List->PLP_Num; i++)
      {
        r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_sp_mplp_id_list_iaddr + i, &TempPLPID);
        PLP_List->PLPid_List[i] = TempPLPID;
      }
    }
    else
    {
      PLP_List->PLP_Num = 0;
    }
  }
  else
  {
    PLP_List->PLP_Num = 0;
  }
  return (r);
}

uint16_t AVL62X1_Get_StreamType(enum avl62x1_dvb_stream_type *StreamType, struct avl62x1_chip *pAVL_Chip)
{
  uint16_t r = AVL_EC_OK;
  uint8_t Temp = 0;
  r |= avl_bms_read8(pAVL_Chip->chip_pub->i2c_addr, s_AVL62X1_SP_S2X_DetectedStreamType_caddr, &Temp);
  *StreamType = (avl62x1_dvb_stream_type)Temp;

  return (r);
}
