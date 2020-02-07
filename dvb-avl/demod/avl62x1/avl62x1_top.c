/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/types.h>

#include "avl62x1.h"
#include "AVL62X1_API.h"
#include "AVL_Tuner.h"

#define dbg_avl(fmt, args...)                                           \
	do                                                              \
	{                                                               \
		if (debug)                                              \
			printk("AVL: %s: " fmt "\n", __func__, ##args); \
	} while (0);

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "\n\t\t Enable debug");

struct AVL_Tuner default_avl_tuner = {
    .ucBlindScanMode = 0,
    .vpMorePara = NULL,
    .fpInitializeFunc = NULL,
    .fpLockFunc = NULL,
    .fpGetLockStatusFunc = NULL,
    .fpGetRFStrength = NULL,
    .fpGetMaxLPF = NULL,
    .fpGetMinLPF = NULL,
    .fpGetLPFStepSize = NULL,
    .fpGetAGCSlope = NULL,
    .fpGetMinGainVoltage = NULL,
    .fpGetMaxGainVoltage = NULL,
    .fpGetRFFreqStepSize = NULL};

int init_error_stat(struct avl62x1_priv *priv)
{
	uint16_t r = AVL_EC_OK;
	struct avl62x1_error_stats_config stErrorStatConfig;
	struct avl62x1_ber_config stBERConfig;

	stErrorStatConfig.eErrorStatMode = AVL62X1_ERROR_STATS_AUTO;
	stErrorStatConfig.eAutoErrorStatType = AVL62X1_ERROR_STATS_TIME;
	stErrorStatConfig.uiTimeThresholdMs = 3000;
	stErrorStatConfig.uiNumberThresholdByte = 0;

	r = IRx_ErrorStatMode_AVL62X1(&stErrorStatConfig, priv->chip);

	stBERConfig.eBERTestPattern = AVL62X1_TEST_LFSR_23;
	stBERConfig.eBERFBInversion = AVL62X1_LFSR_FB_INVERTED;
	stBERConfig.uiLFSRSynced = 0;
	stBERConfig.uiLFSRStartPos = 4;
	r |= IRx_ResetBER_AVL62X1(&stBERConfig, priv->chip);

	return r;
}

static int avl62x1_init_dvbs(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_diseqc_params Diseqc_para;
	uint16_t r = AVL_EC_OK;

	Diseqc_para.uiToneFrequencyKHz = 22;
	Diseqc_para.eTXGap = AVL62X1_DTXG_15ms;
	Diseqc_para.eTxWaveForm = AVL62X1_DWM_Normal;
	Diseqc_para.eRxTimeout = AVL62X1_DRT_150ms;
	Diseqc_para.eRxWaveForm = AVL62X1_DWM_Normal;

	r |= IDiseqc_Initialize_AVL62X1(&Diseqc_para, priv->chip);
	if (AVL_EC_OK != r)
	{
		dbg_avl("Diseqc Init failed !\n");
	}

	r |= (int)AVL62X1_SetGPIODir(AVL62X1_GPIO_Pin_LNB_PWR_EN,
				    AVL62X1_GPIO_DIR_OUTPUT,
				    priv->chip);
	r |= (int)AVL62X1_SetGPIODir(AVL62X1_GPIO_Pin_LNB_PWR_SEL,
				     AVL62X1_GPIO_DIR_OUTPUT,
				     priv->chip);

	return r;
}

static int avl62x1_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;

	dbg_avl("%d\n", enable);

	if(priv->chip == NULL) {
		dev_err(&priv->i2c->dev, KBUILD_MODNAME ": NULL fe->demodulator_priv->chip");
	}

	if (enable)
	{
		ret = AVL62X1_OpenTunerI2C(priv->chip);
	}
	else
	{
		ret = AVL62X1_CloseTunerI2C(priv->chip);
	}
	return ret;
}

static int avl62x1_set_dvbs(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_carrier_info CarrierInfo;
	struct avl62x1_stream_info StreamInfo;
	dbg_avl("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

	CarrierInfo.m_carrier_index = 0;
	CarrierInfo.m_rf_freq_kHz = c->frequency;
	//carrier frequency offset (from RF freq) in Hz
	CarrierInfo.m_carrier_freq_offset_Hz = 0;
	CarrierInfo.m_symbol_rate_Hz = c->symbol_rate;
	CarrierInfo.m_roll_off = 0;
	CarrierInfo.m_signal_type = 0;
	CarrierInfo.m_PL_scram_key = AVL62X1_PL_SCRAM_AUTO;
	CarrierInfo.m_PLS_ACM = 0; //PLS if CCM, 0 if ACM
	CarrierInfo.m_SIS_MIS = 0;
	//number of supported streams (that can be output)
	CarrierInfo.m_num_streams = 0;
	CarrierInfo.m_SNR_dB_x100 = 0;
	CarrierInfo.m_modulation = 0;
	CarrierInfo.m_pilot = 0;
	CarrierInfo.m_dvbs2_fec_length = 0;
	CarrierInfo.m_coderate.m_dvbs2_code_rate = 0;
	CarrierInfo.m_dvbs2_ccm_acm = 0; //1:CCM, 0:ACM
	//index of carrier (avl62x1_carrier_info.m_CarrierIndex) that this stream is in
	StreamInfo.m_carrier_index = 0;
	StreamInfo.m_stream_type = AVL62X1_TRANSPORT;
	StreamInfo.m_ISI = c->stream_id;
	StreamInfo.m_PLP_ID = 0;	 // use when lock TP
	StreamInfo.PLP_List.PLP_Num = 0; // save scan out the PLP list for T2MI ISI

	r = AVL62X1_LockTP(&CarrierInfo, &StreamInfo, AVL_FALSE, priv->chip);

	return r;
}

static int avl62x1_set_dvbmode(struct dvb_frontend *fe,
			       enum fe_delivery_system delsys)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_ver_info ver_info;

	/* already in desired mode */
	if (priv->delivery_system == delsys)
		return 0;

	dbg_avl("initing demod for delsys=%d", delsys);

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
		if (delsys == SYS_DVBS2)
			return 0;
		break;
	case SYS_DVBS2:
		if (delsys == SYS_DVBS)
			return 0;
		break;
	default:
		break;
	}
	priv->delivery_system = delsys;

	//Reset Demod
	r = avl_bsp_reset();
	if (AVL_EC_OK != r)
	{
		dbg_avl("Failed to Resed demod via BSP!\n");
		return 0;
	}

	// boot the firmware here
	r |= AVL62X1_Initialize(priv->chip);
	if (AVL_EC_OK != r)
	{
		dbg_avl("AVL_AVL62X1_Initialize failed !\n");
		return (r);
	}

	r |= IBase_GetVersion_AVL62X1(&ver_info, priv->chip);
	if (AVL_EC_OK != r)
	{
		dbg_avl("IBase_GetVersion_AVL62X1 failed\n");
		return (r);
	}
	dbg_avl("FW version %d.%d.%d\n",
		ver_info.m_Patch.m_Major,
		ver_info.m_Patch.m_Minor,
		ver_info.m_Patch.m_Build);
	dbg_avl("SDK version %d.%d.%d\n",
		ver_info.m_API.m_Major,
		ver_info.m_API.m_Minor,
		ver_info.m_API.m_Build);

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
	case SYS_DVBS2:
	default:
		ret |= avl62x1_init_dvbs(fe);
		break;
	}

	ret |= init_error_stat(priv);

	if (ret)
	{
		dev_err(&priv->i2c->dev, KBUILD_MODNAME ": demod init failed");
	}

	return ret;
}

uint16_t diseqc_send_cmd(struct avl62x1_priv *priv,
			 uint8_t *cmd,
			 uint8_t cmdsize)
{
	uint16_t r = AVL_EC_OK;
	struct avl62x1_diseqc_tx_status TxStatus;
	dbg_avl(" %*ph", cmdsize, cmd);

	r = AVL62X1_IDiseqc_SendModulationData(cmd, cmdsize, priv->chip);
	if (r != AVL_EC_OK)
	{
		printk("diseqc_send_cmd failed !\n");
	}
	else
	{
		do
		{
			msleep(5);
			r |= AVL62X1_IDiseqc_GetTxStatus(&TxStatus, priv->chip);
		} while (TxStatus.m_TxDone != 1);
		if (r == AVL_EC_OK)
		{
		}
		else
		{
			printk("diseqc_send_cmd Err. !\n");
		}
	}
	return (int)(r);
}

static int avl62x1_send_master_cmd(struct dvb_frontend *fe,
			  struct dvb_diseqc_master_cmd *cmd)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;

	return diseqc_send_cmd(priv, cmd->msg, cmd->msg_len);
}

static int avl62x1_send_burst(struct dvb_frontend *fe,
enum fe_sec_mini_cmd burst)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;
	uint8_t tone = burst == SEC_MINI_A ? 1 : 0;
	uint8_t count = 1;
	ret = (int)AVL62X1_IDiseqc_SendTone(tone, count, priv->chip);

	return ret;
}

static int avl62x1_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int r = AVL_EC_OK;

	dbg_avl("tone: %d", tone);
	switch (tone)
	{
	case SEC_TONE_ON:
		if (priv->chip->chip_priv->diseqc_op_status !=
		    AVL62X1_DOS_InContinuous)
		{
			r = (int)AVL62X1_IDiseqc_Stop22K(priv->chip);
		}
		break;
	case SEC_TONE_OFF:
		if (priv->chip->chip_priv->diseqc_op_status ==
		    AVL62X1_DOS_InContinuous)
		{
			r = (int)AVL62X1_IDiseqc_Start22K(priv->chip);
		}
		break;
	default:
		return -EINVAL;
	}
	return r;
}

static int avl62x1_set_voltage(struct dvb_frontend *fe,
			       enum fe_sec_voltage voltage)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_gpio_pin_value pwr, vol;
	int ret;

	dbg_avl("volt: %d", voltage);

	switch (voltage)
	{
	case SEC_VOLTAGE_OFF:
		pwr = AVL62X1_GPIO_VALUE_LOGIC_0;
		vol = AVL62X1_GPIO_VALUE_LOGIC_0;
		break;
	case SEC_VOLTAGE_13:
		//power on
		pwr = AVL62X1_GPIO_VALUE_LOGIC_1;
		vol = AVL62X1_GPIO_VALUE_LOGIC_0;
		break;
	case SEC_VOLTAGE_18:
		//power on
		pwr = AVL62X1_GPIO_VALUE_LOGIC_1;
		vol = AVL62X1_GPIO_VALUE_HIGH_Z;
		break;
	default:
		return -EINVAL;
	}
	ret = (int)AVL62X1_SetGPIOVal(AVL62X1_GPIO_Pin_LNB_PWR_EN,
				      pwr,
				      priv->chip);
	ret |= (int)AVL62X1_SetGPIOVal(AVL62X1_GPIO_Pin_LNB_PWR_SEL,
				       vol,
				       priv->chip);
	return ret;
}

static int avl62x1_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;
	avl62x1_lock_status lock = 0;
	int16_t SNR_x100db = 0;
	int16_t SignalStrength = 0;

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
	case SYS_DVBS2:
		ret = AVL62X1_GetLockStatus(&lock, priv->chip);
		//dbg_avl("lock: %d", lock);
		if (lock == AVL62X1_STATUS_LOCK)
		{
			ret |= AVL62X1_GetSNR(&SNR_x100db, priv->chip);
			dbg_avl("SNR_x100db: %d", SNR_x100db);
			if (ret || SNR_x100db > 10000)
				SNR_x100db = 0;
		}
		else
		{
			*status = 0;
			return ret;
		}
		break;
	default:
		*status = 0;
		return 1;
	}

	if (ret)
	{
		*status = 0;
		return ret;
	}
	*status = FE_HAS_SIGNAL;

	//dbg_avl("%s", read_stdout(priv->chip));

	ret = AVL62X1_GetSignalStrength(&SignalStrength, priv->chip);

	c->strength.len = 2;

	c->strength.stat[1].scale = FE_SCALE_RELATIVE;
	c->strength.stat[1].uvalue = (SignalStrength * 65535) / 100;

	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = -80 + SignalStrength / 2;

	if (lock == AVL62X1_STATUS_LOCK)
	{
		*status |= FE_HAS_CARRIER |
			   FE_HAS_VITERBI |
			   FE_HAS_SYNC |
			   FE_HAS_LOCK;
		c->cnr.len = 2;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		c->cnr.stat[0].svalue = SNR_x100db * 10;
		c->cnr.stat[1].scale = FE_SCALE_RELATIVE;
		c->cnr.stat[1].uvalue = ((SNR_x100db + 300) / 10) * 250;
		if (c->cnr.stat[1].uvalue > 0xffff)
			c->cnr.stat[1].uvalue = 0xffff;
	}
	else
	{
		c->cnr.len = 1;
		c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}
	return ret;
}

static int avl62x1_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	*strength = 0;
	for (i = 0; i < c->strength.len; i++)
		if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (u16)c->strength.stat[i].uvalue;

	return 0;
}

static int avl62x1_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	*snr = 0;
	for (i = 0; i < c->cnr.len; i++)
		if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
			*snr = (u16)c->cnr.stat[i].uvalue;

	return 0;
}

static int avl62x1_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;

	*ber = 10e7;
	ret = (int)AVL62X1_GetPER(ber, priv->chip);
	if (!ret)
		*ber /= 100;

	return ret;
}

static int avl62x1_fe_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

//static  struct dtv_frontend_properties _last_dtv;

static int avl62x1_set_frontend(struct dvb_frontend *fe)
{
	int ret;
	//struct avl62x1_priv *priv = fe->demodulator_priv;

	/* setup tuner */
	if (fe->ops.tuner_ops.set_params)
	{
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);
		ret = fe->ops.tuner_ops.set_params(fe);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
		if (ret)
			return ret;
	}

	dbg_avl("ACQUIRE");
	ret = avl62x1_set_dvbs(fe);

	return ret;
}

static int avl62x1_tune(struct dvb_frontend *fe,
			bool re_tune,
			unsigned int mode_flags,
			unsigned int *delay,
			enum fe_status *status)
{
	*delay = HZ / 5;
	if (re_tune)
	{
		int ret = avl62x1_set_frontend(fe);
		if (ret)
			return ret;
	}
	return avl62x1_read_status(fe, status);
}

static int avl62x1_init(struct dvb_frontend *fe)
{
	return 0;
}

static int avl62x1_sleep(struct dvb_frontend *fe)
{
	return 0;
}

static void avl62x1_release(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	kfree(priv->chip->chip_pub);
	kfree(priv->chip->chip_priv);
	kfree(priv->chip);
	kfree(priv);
}

static struct dvb_frontend_ops avl62x1_ops = {
    .delsys = {SYS_DVBS, SYS_DVBS2},
    .info = {
	.name = "Availink avl62x1",
	.frequency_min_hz = 950000000,
	.frequency_max_hz = 2150000000,
	.frequency_stepsize_hz = 0,
	.frequency_tolerance_hz = 0,
	.symbol_rate_min = 1000000,
	.symbol_rate_max = 55000000,
	.caps =
	    FE_CAN_FEC_1_2 |
	    FE_CAN_FEC_2_3 |
	    FE_CAN_FEC_3_4 |
	    FE_CAN_FEC_4_5 |
	    FE_CAN_FEC_5_6 |
	    FE_CAN_FEC_6_7 |
	    FE_CAN_FEC_7_8 |
	    FE_CAN_FEC_AUTO |
	    FE_CAN_QPSK |
	    FE_CAN_QAM_16 |
	    FE_CAN_QAM_32 |
	    FE_CAN_QAM_64 |
	    FE_CAN_QAM_AUTO |
	    FE_CAN_TRANSMISSION_MODE_AUTO |
	    FE_CAN_MUTE_TS |
	    FE_CAN_2G_MODULATION |
	    FE_CAN_MULTISTREAM |
	    FE_CAN_INVERSION_AUTO |
	    FE_CAN_RECOVER},

    .release = avl62x1_release,
    .init = avl62x1_init,

    .sleep = avl62x1_sleep,
    .i2c_gate_ctrl = avl62x1_i2c_gate_ctrl,

    .read_status = avl62x1_read_status,
    .read_signal_strength = avl62x1_read_signal_strength,
    .read_snr = avl62x1_read_snr,
    .read_ber = avl62x1_read_ber,
    .set_tone = avl62x1_set_tone,
    .set_voltage = avl62x1_set_voltage,
    .diseqc_send_master_cmd = avl62x1_send_master_cmd,
    .diseqc_send_burst = avl62x1_send_burst,
    .get_frontend_algo = avl62x1_fe_algo,
    .tune = avl62x1_tune,
    .set_frontend = avl62x1_set_frontend,
};

struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
				    struct i2c_adapter *i2c)
{
	struct avl62x1_priv *priv;
	uint16_t ret;
	u32 id;
	int fw_status;
	unsigned int fw_maj, fw_min, fw_build;

	dbg_avl("enter %s()",__FUNCTION__);

	priv = kzalloc(sizeof(struct avl62x1_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;

	priv->chip = kzalloc(sizeof(struct avl62x1_chip), GFP_KERNEL);
	if (priv->chip == NULL)
		goto err1;

	priv->chip->chip_priv = kzalloc(sizeof(struct avl62x1_chip_priv),
					GFP_KERNEL);
	if (priv->chip->chip_priv == NULL)
		goto err2;

	priv->chip->chip_pub = kzalloc(sizeof(struct avl62x1_chip_pub),
				       GFP_KERNEL);
	if (priv->chip->chip_pub == NULL)
		goto err3;

	memcpy(&priv->frontend.ops, &avl62x1_ops,
	       sizeof(struct dvb_frontend_ops));

	priv->frontend.demodulator_priv = priv;
	priv->i2c = i2c;
	priv->delivery_system = -1;

	/* copy (ephemeral?) public part of chip config into alloc'd area */
	memcpy(priv->chip->chip_pub,
	       config->chip_pub,
	       sizeof(struct avl62x1_chip_pub));
	
	priv->chip->chip_pub->pTuner = &default_avl_tuner;

	dbg_avl("Demod %d, I2C addr 0x%x",
		(priv->chip->chip_pub->i2c_addr >> 8) & 0x7,
		priv->chip->chip_pub->i2c_addr & 0xFF);

	// associate demod ID with i2c_adapter
	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);

	/* get chip id */
	ret = AVL62X1_GetChipID(priv->chip->chip_pub->i2c_addr, &id);
	if (ret)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": attach failed reading id");
		goto err4;
	}

	dbg_avl("chip_id= 0x%x\n", id);

	if (id != AVL62X1_CHIP_ID)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": attach failed, id mismatch");
		goto err4;
	}

	dev_info(&priv->i2c->dev, KBUILD_MODNAME ": found AVL62x1 id=0x%x", id);

	fw_status = request_firmware(&priv->fw,
				     "availink/avl62x1.patch",
				     i2c->dev.parent);
	if (fw_status != 0)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": firmware file not found");
		goto err4;
	}
	else
	{
		priv->chip->chip_priv->patch_data = (unsigned char *)(priv->fw->data);
		fw_maj = priv->chip->chip_priv->patch_data[24]; //major rev
		fw_min = priv->chip->chip_priv->patch_data[25]; //SDK-FW API rev
		fw_build = (priv->chip->chip_priv->patch_data[26] << 8) |
			   priv->chip->chip_priv->patch_data[27]; //internal rev
		if (fw_min != AVL62X1_API_VER_MINOR) //SDK-FW API rev must match
		{
			dev_err(&priv->i2c->dev,
				KBUILD_MODNAME ": Firmware version %d.%d.%d incompatible with this driver version",
				fw_maj, fw_min, fw_build);
			dev_err(&priv->i2c->dev,
				KBUILD_MODNAME ": Firmware minor version must be %d",
				AVL62X1_API_VER_MINOR);
			goto err5;
		}
		else
		{
			dev_info(&priv->i2c->dev,
				 KBUILD_MODNAME ": Firmware version %d.%d.%d found",
				 fw_maj, fw_min, fw_build);
		}
	}

	if (!avl62x1_set_dvbmode(&priv->frontend, SYS_DVBS2))
	{
		dev_info(&priv->i2c->dev,
			 KBUILD_MODNAME ": Firmware booted");
		release_firmware(priv->fw);
		return &priv->frontend;
	}

err5:
	release_firmware(priv->fw);
err4:
	kfree(priv->chip->chip_pub);
err3:
	kfree(priv->chip->chip_priv);
err2:
	kfree(priv->chip);
err1:
	kfree(priv);
err:
	return NULL;
} /* end avl62x1_attach() */


EXPORT_SYMBOL_GPL(avl62x1_attach);

MODULE_DESCRIPTION("Availink AVL62X1 DVB-S/S2/S2X demodulator driver");
MODULE_AUTHOR("Availink, Inc. (opensource@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL62X1_VERSION);
