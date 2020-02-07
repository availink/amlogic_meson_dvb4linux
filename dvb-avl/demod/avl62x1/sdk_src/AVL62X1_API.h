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

#ifndef _AVL62X1_API_H_
#define _AVL62X1_API_H_

#include "AVL62X1_DVBSx.h"

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

  uint16_t AVL62X1_GetChipID(uint16_t slave_addr, uint32_t *pChipId);
  uint16_t AVL62X1_Initialize(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_LockTP(struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_stream_info *pStreamInfo, avl_bool_t blind_sym_rate, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetLockStatus(enum avl62x1_lock_status *eLockStat, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetLostLockStatus(enum avl62x1_lost_lock_status *eLostLockStat, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetSNR(int16_t *piSNR_x100db, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetSignalInfo(struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetSignalStrength(uint16_t *puiSignalStrength, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetSignalQuality(uint16_t *puiSignalQuality, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_ResetPER(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetPER(uint32_t *puiPER_x1e9, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_DiscoverStreams(struct avl62x1_carrier_info *pCarrierInfo, avl_bool_t blind_sym_rate, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetDiscoveryStatus(enum avl62x1_discovery_status *eDiscoveryStat, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetStreamNumber(uint8_t *pStreamNum, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetStreamList(struct avl62x1_stream_info *pStreams, const uint8_t max_num_streams, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_SwitchStream(struct avl62x1_stream_info *pStreamInfo, struct avl62x1_chip *pAVL_Chip);

  uint16_t AVL62X1_IDiseqc_ReadModulationData(uint8_t *pucBuff, uint8_t *pucSize, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_IDiseqc_SendModulationData(const uint8_t *pucBuff, uint8_t ucSize, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_IDiseqc_GetTxStatus(struct avl62x1_diseqc_tx_status *pTxStatus, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_IDiseqc_GetRxStatus(struct avl62x1_diseqc_rx_status *pRxStatus, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_IDiseqc_SendTone(uint8_t ucTone, uint8_t ucCount, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_IDiseqc_Start22K(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_IDiseqc_Stop22K(struct avl62x1_chip *pAVL_Chip);

  uint16_t AVL62X1_OpenTunerI2C(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_CloseTunerI2C(struct avl62x1_chip *pAVL_Chip);

  uint16_t AVL62X1_SetGPIODir(enum avl62x1_gpio_pin ePin, enum avl62x1_gpio_pin_dir eDir, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_SetGPIOVal(enum avl62x1_gpio_pin ePin, enum avl62x1_gpio_pin_value eVal, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetGPIOVal(enum avl62x1_gpio_pin ePin, enum avl62x1_gpio_pin_value *peVal, struct avl62x1_chip *pAVL_Chip);

  uint16_t AVL62X1_BlindScan_Start(struct avl62x1_blind_scan_params *pBSParams, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_BlindScan_Cancel(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_BlindScan_GetStatus(struct avl62x1_blind_scan_info *pBSInfo, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_BlindScan_GetCarrierList(const struct avl62x1_blind_scan_params *pBSParams, struct avl62x1_blind_scan_info *pBSInfo, struct avl62x1_carrier_info *pCarriers, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_BlindScan_ConfirmCarrier(const struct avl62x1_blind_scan_params *pBSParams, struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_BlindScan_GetStreamList(struct avl62x1_carrier_info *pCarrier, struct avl62x1_stream_info *pStreams, const uint8_t max_num_streams, struct avl62x1_chip *pAVL_Chip);

  uint16_t AVL62X1_Optimize_Carrier(struct AVL_Tuner *pTuner, struct avl62x1_carrier_info *pCarrierInfo, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Enable_T2MIRawMode(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Disable_T2MIRawMode(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_GetPLSXLFSRValue(uint32_t *pXLFSRValue, struct avl62x1_chip *pAVL_Chip);

  uint16_t AVL62X1_Manual_Set_T2MI_PID(uint16_t T2MI_PID, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Manual_Set_T2MI_PID_1(uint16_t T2MI_PID, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_AutoDetect_T2MI_PID_Enable(struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Get_Current_Stream_T2MI_PID(uint16_t *puiT2MI_PID, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Set_Current_Stream_T2MI_PID(uint16_t uiT2MI_PID, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Set_T2MI_MPLP_id_ScanTime(uint16_t Frame_Num, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Get_T2MI_PLPid(struct avl62x1_t2mi_mplp *PLP_List, struct avl62x1_chip *pAVL_Chip);
  uint16_t AVL62X1_Get_StreamType(enum avl62x1_dvb_stream_type *StreamType, struct avl62x1_chip *pAVL_Chip);
#ifdef AVL_CPLUSPLUS
}
#endif

#endif
