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

#ifndef _AVL62X1_DVBSX_H_
#define _AVL62X1_DVBSX_H_

#include "AVL62X1_Reg.h"
#include "AVL_Tuner.h"
#include "avl_lib.h"

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

#define AVL62X1_API_VER_MAJOR 1  //Public API rev
#define AVL62X1_API_VER_MINOR 8  //SDK-FW API rev
#define AVL62X1_API_VER_BUILD 23 //internal rev

#define AVL62X1_PL_SCRAM_XSTATE (0x040000)
#define AVL62X1_PL_SCRAM_AUTO (0x800000)
#define AVL62X1_PL_SCRAM_LFSR_MASK (0x03FFFF)

#define AVL62X1_IF_SHIFT_MAX_SR_HZ (15 * 1000 * 1000)

  /* command enumeration definitions */
#define CMD_IDLE 0
#define CMD_LD_DEFAULT 1
#define CMD_ACQUIRE 2
#define CMD_HALT 3
#define CMD_DEBUG 4
#define CMD_SLEEP 7
#define CMD_WAKE 8
#define CMD_BLIND_SCAN 9
#define CMD_ROM_CRC 10
#define CMD_DEBUG_FW 15
#define CMD_ADC_TEST 20
#define CMD_DMA 21
#define CMD_CALC_CRC 22
#define CMD_PING 23
#define CMD_DECOMPRESS 24
#define CMD_CAPTURE_IQ 25
#define CMD_FAILED 255

#define SP_CMD_IDLE 0
#define SP_CMD_LD_DEFAULT 1
#define SP_CMD_ACQUIRE 2
#define SP_CMD_HALT 3
#define SP_CMD_BLIND_SCAN_CLR 4

  /*
	* Patch file stuff
	*/
#define PATCH_CMD_VALIDATE_CRC 0
#define PATCH_CMD_PING 1
#define PATCH_CMD_LD_TO_DEVICE 2
#define PATCH_CMD_DMA 3
#define PATCH_CMD_EXTRACT 4
#define PATCH_CMD_ASSERT_CPU_RESET 5
#define PATCH_CMD_RELEASE_CPU_RESET 6
#define PATCH_CMD_LD_TO_DEVICE_IMM 7
#define PATCH_CMD_RD_FROM_DEVICE 8
#define PATCH_CMD_DMA_HW 9
#define PATCH_CMD_SET_COND_IMM 10
#define PATCH_CMD_EXIT 11
#define PATCH_CMD_POLL_WAIT 12
#define PATCH_CMD_LD_TO_DEVICE_PACKED 13

#define PATCH_CMP_TYPE_ZLIB 0
#define PATCH_CMP_TYPE_ZLIB_NULL 1
#define PATCH_CMP_TYPE_GLIB 2
#define PATCH_CMP_TYPE_NONE 3

#define PATCH_VAR_ARRAY_SIZE 32

  //Addr modes 2 bits
#define PATCH_OP_ADDR_MODE_VAR_IDX 0
#define PATCH_OP_ADDR_MODE_IMMIDIATE 1

  //Unary operators 6 bits
#define PATCH_OP_UNARY_NOP 0
#define PATCH_OP_UNARY_LOGICAL_NEGATE 1
#define PATCH_OP_UNARY_BITWISE_NEGATE 2
#define PATCH_OP_UNARY_BITWISE_AND 3
#define PATCH_OP_UNARY_BITWISE_OR 4

  //Binary operators 1 Byte
#define PATCH_OP_BINARY_LOAD 0
#define PATCH_OP_BINARY_AND 1
#define PATCH_OP_BINARY_OR 2
#define PATCH_OP_BINARY_BITWISE_AND 3
#define PATCH_OP_BINARY_BITWISE_OR 4
#define PATCH_OP_BINARY_EQUALS 5
#define PATCH_OP_BINARY_STORE 6
#define PATCH_OP_BINARY_NOT_EQUALS 7

#define PATCH_COND_EXIT_AFTER_LD 8

#define PATCH_STD_DVBC 0
#define PATCH_STD_DVBSx 1
#define PATCH_STD_DVBTx 2
#define PATCH_STD_ISDBT 3

  typedef enum avl62x1_slave_addr
  {
    AVL62X1_SA_0 = 0x14,
    AVL62X1_SA_1 = 0x15
  } avl62x1_slave_addr;

  typedef enum avl62x1_xtal
  {
    AVL62X1_RefClk_16M,
    AVL62X1_RefClk_27M,
    AVL62X1_RefClk_30M,
  } avl62x1_xtal;

  // Defines the device functional mode.
  typedef enum avl62x1_functional_mode
  {
    AVL62X1_FuncMode_Idle = 0,
    AVL62X1_FuncMode_Demod = 1,    // The device is in demod mode.
    AVL62X1_FuncMode_BlindScan = 2 // The device is in blind scan mode.
  } avl62x1_functional_mode;

  /// Defines the device spectrum polarity setting.
  typedef enum avl62x1_spectrum_polarity
  {
    AVL62X1_Spectrum_Normal = 0, ///< = 0 The received signal spectrum is not inverted.
    AVL62X1_Spectrum_Invert = 1  ///< = 1 The received signal spectrum is inverted.
  } avl62x1_spectrum_polarity;

  // Defines demod lock status
  typedef enum avl62x1_lock_status
  {
    AVL62X1_STATUS_UNLOCK = 0, // demod isn't locked
    AVL62X1_STATUS_LOCK = 1    // demod is in locked state.
  } avl62x1_lock_status;

  typedef enum avl62x1_lost_lock_status
  {
    AVL62X1_Lost_Lock_No = 0, // demod has not lost lock since last check
    AVL62X1_Lost_Lock_Yes = 1 // demod has lost lock since last check
  } avl62x1_lost_lock_status;

  // Defines demod stream discovery status
  typedef enum avl62x1_discovery_status
  {
    AVL62X1_DISCOVERY_RUNNING = 0, // stream discovery is still running
    AVL62X1_DISCOVERY_FINISHED = 1 // stream discovery has finished
  } avl62x1_discovery_status;

  // Defines the ON/OFF options for the AVL62X1 device.
  typedef enum avl62x1_switch
  {
    AVL62X1_ON = 0, // switched on
    AVL62X1_OFF = 1 // switched off
  } avl62x1_switch;

  typedef enum avl62x1_standard
  {
    AVL62X1_DVBS = 0,  // DVBS standard
    AVL62X1_DVBS2 = 1, // DVBS2 standard
    AVL62X1_QAM_Carrier = 2,
    AVL62X1_Edge_Pair = 3
  } avl62x1_standard;

  typedef enum avl62x1_modulation_mode
  {
    AVL62X1_BPSK = 1,
    AVL62X1_QPSK = 2,
    AVL62X1_8PSK = 3,
    AVL62X1_16APSK = 4,
    AVL62X1_32APSK = 5,
    AVL62X1_64APSK = 6,
    AVL62X1_128APSK = 7,
    AVL62X1_256APSK = 8
  } avl62x1_modulation_mode;

  typedef enum avl62x1_dvbs_code_rate
  {
    AVL62X1_DVBS_CR_1_2 = 0,
    AVL62X1_DVBS_CR_2_3 = 1,
    AVL62X1_DVBS_CR_3_4 = 2,
    AVL62X1_DVBS_CR_5_6 = 3,
    AVL62X1_DVBS_CR_7_8 = 4
  } avl62x1_dvbs_code_rate;

  typedef enum avl62x1_dvbs2_code_rate
  {
    AVL62X1_DVBS2_CR_1_4 = 0,
    AVL62X1_DVBS2_CR_1_3 = 1,
    AVL62X1_DVBS2_CR_2_5 = 2,
    AVL62X1_DVBS2_CR_1_2 = 3,
    AVL62X1_DVBS2_CR_3_5 = 4,
    AVL62X1_DVBS2_CR_2_3 = 5,
    AVL62X1_DVBS2_CR_3_4 = 6,
    AVL62X1_DVBS2_CR_4_5 = 7,
    AVL62X1_DVBS2_CR_5_6 = 8,
    AVL62X1_DVBS2_CR_8_9 = 9,
    AVL62X1_DVBS2_CR_9_10 = 10,
    AVL62X1_DVBS2_CR_2_9 = 11,
    AVL62X1_DVBS2_CR_13_45 = 12,
    AVL62X1_DVBS2_CR_9_20 = 13,
    AVL62X1_DVBS2_CR_90_180 = 14,
    AVL62X1_DVBS2_CR_96_180 = 15,
    AVL62X1_DVBS2_CR_11_20 = 16,
    AVL62X1_DVBS2_CR_100_180 = 17,
    AVL62X1_DVBS2_CR_104_180 = 18,
    AVL62X1_DVBS2_CR_26_45 = 19,
    AVL62X1_DVBS2_CR_18_30 = 20,
    AVL62X1_DVBS2_CR_28_45 = 21,
    AVL62X1_DVBS2_CR_23_36 = 22,
    AVL62X1_DVBS2_CR_116_180 = 23,
    AVL62X1_DVBS2_CR_20_30 = 24,
    AVL62X1_DVBS2_CR_124_180 = 25,
    AVL62X1_DVBS2_CR_25_36 = 26,
    AVL62X1_DVBS2_CR_128_180 = 27,
    AVL62X1_DVBS2_CR_13_18 = 28,
    AVL62X1_DVBS2_CR_132_180 = 29,
    AVL62X1_DVBS2_CR_22_30 = 30,
    AVL62X1_DVBS2_CR_135_180 = 31,
    AVL62X1_DVBS2_CR_140_180 = 32,
    AVL62X1_DVBS2_CR_7_9 = 33,
    AVL62X1_DVBS2_CR_154_180 = 34,
    AVL62X1_DVBS2_CR_11_45 = 35,
    AVL62X1_DVBS2_CR_4_15 = 36,
    AVL62X1_DVBS2_CR_14_45 = 37,
    AVL62X1_DVBS2_CR_7_15 = 38,
    AVL62X1_DVBS2_CR_8_15 = 39,
    AVL62X1_DVBS2_CR_32_45 = 40
  } avl62x1_dvbs2_code_rate;

  typedef enum avl62x1_pilot
  {
    AVL62X1_Pilot_OFF = 0, // Pilot off
    AVL62X1_Pilot_ON = 1   // Pilot on
  } avl62x1_pilot;

  typedef enum avl62x1_rolloff
  {
    AVL62X1_RollOff_35 = 0,
    AVL62X1_RollOff_25 = 1,
    AVL62X1_RollOff_20 = 2,
    AVL62X1_RollOff_15 = 3,
    AVL62X1_RollOff_10 = 4,
    AVL62X1_RollOff_05 = 5,
    AVL62X1_RollOff_UNKNOWN = 6
  } avl62x1_rolloff;

  typedef enum avl62x1_fec_length
  {
    AVL62X1_DVBS2_FEC_SHORT = 0,
    AVL62X1_DVBS2_FEC_MEDIUM = 2,
    AVL62X1_DVBS2_FEC_LONG = 1
  } avl62x1_fec_length;

  typedef enum avl62x1_dvbs2_ccm_acm
  {
    AVL62X1_DVBS2_ACM = 0,
    AVL62X1_DVBS2_CCM = 1
  } avl62x1_dvbs2_ccm_acm;

  typedef enum avl62x1_dvb_stream_type
  {
    AVL62X1_GENERIC_PACKETIZED = 0,
    AVL62X1_GENERIC_CONTINUOUS = 1,
    AVL62X1_GSE_HEM = 2,
    AVL62X1_TRANSPORT = 3,
    AVL62X1_UNKNOWN = 4, //stream type unknown/don't care. will output BB frames directly over TS
    AVL62X1_GSE_LITE = 5,
    AVL62X1_GSE_HEM_LITE = 6,
    AVL62X1_T2MI = 7,
    AVL62X1_UNDETERMINED = 255 //don't know stream type. demod will scan for streams then output first one found
  } avl62x1_dvb_stream_type;

  /// The MPEG output format. The default value in the Availink device is \a AVL_DVBSx_MPF_TS
  typedef enum avl62x1_mpeg_format
  {
    AVL62X1_MPF_TS = 0, ///< = 0  Transport stream format.
    AVL62X1_MPF_TSP = 1 ///< = 1  Transport stream plus parity format.
  } avl62x1_mpeg_format;

  /// The MPEG output mode. The default value in the Availink device is \a AVL_DVBSx_MPM_Parallel
  typedef enum avl62x1_mpeg_mode
  {
    AVL62X1_MPM_Parallel = 0,  ///< = 0  Output MPEG data in parallel mode
    AVL62X1_MPM_Serial = 1,    ///< = 0  Output MPEG data in serial mode
    AVL62X1_MPM_2BitSerial = 2 //output in 2bit serial mode
  } avl62x1_mpeg_mode;

  /// The MPEG output clock polarity. The clock polarity should be configured to meet the back end device's requirement.
  /// The default value in the Availink device is \a AVL_DVBSx_MPCP_Rising
  typedef enum avl62x1_mpeg_clock_polarity
  {
    AVL62X1_MPCP_Falling = 0, ///<  = 0  The MPEG data is valid on the falling edge of the clock.
    AVL62X1_MPCP_Rising = 1   ///<  = 1  The MPEG data is valid on the rising edge of the clock.
  } avl62x1_mpeg_clock_polarity;

  // The phase of the MPEG clock edge relative to the data transition.
  // Applies to parallel mode only.
  // 0,1,2,3 will delay the MPEG clock edge by 0,1,2, or 3 m_MPEGFrequency_Hz clock periods
  typedef enum avl62x1_mpeg_clock_phase
  {
    AVL62X1_MPCP_Phase_0 = 0, /// no clock edge delay
    AVL62X1_MPCP_Phase_1 = 1, /// delay clock edge by 1
    AVL62X1_MPCP_Phase_2 = 2, /// delay clock edge by 2
    AVL62X1_MPCP_Phase_3 = 3  /// delay clock edge by 3
  } avl62x1_mpeg_clock_phase;

  // Applies to serial mode only.
  typedef enum avl62x1_mpeg_clock_adaptation
  {
    AVL62X1_MPCA_Fixed = 0,   /// no adaptation - fixed frequency (m_MPEGFrequency_Hz)
    AVL62X1_MPCA_Adaptive = 1 /// adapt clock frequency to data rate
  } avl62x1_mpeg_clock_adaptation;

  // Define MPEG bit order
  typedef enum avl62x1_mpeg_bit_order
  {
    AVL62X1_MPBO_LSB = 0, /// least significant bit first when serial transport mode is used
    AVL62X1_MPBO_MSB = 1  /// most significant bit first when serial transport mode is used
  } avl62x1_mpeg_bit_order;

  ///
  /// Defines the pin on which the Availink device outputs the MPEG data when the MPEG interface has been configured to operate
  /// in serial mode.
  typedef enum avl62x1_mpeg_serial_pin
  {
    AVL62X1_MPSP_DATA0 = 0, ///< = 0 Serial data is output on pin MPEG_DATA_0
    AVL62X1_MPSP_DATA7 = 1  ///< = 1 Serial data is output on pin MPEG_DATA_7
  } avl62x1_mpeg_serial_pin;

  // Defines the polarity of the MPEG error signal.
  typedef enum avl62x1_mpeg_err_polarity
  {
    AVL62X1_MPEP_Normal = 0, // The MPEG error signal is high during the payload of a packet which contains uncorrectable error(s).
    AVL62X1_MPEP_Invert = 1  // The MPEG error signal is low during the payload of a packet which contains uncorrectable error(s).
  } avl62x1_mpeg_err_polarity;

  // Defines whether the feeback bit of the LFSR used to generate the BER/PER test pattern is inverted.
  typedef enum avl62x1_lfsr_fb_bit
  {
    AVL62X1_LFSR_FB_NOT_INVERTED = 0, // LFSR feedback bit isn't inverted
    AVL62X1_LFSR_FB_INVERTED = 1      // LFSR feedback bit is inverted
  } avl62x1_lfsr_fb_bit;

  // Defines the test pattern being used for BER/PER measurements.
  typedef enum avl62x1_test_pattern
  {
    AVL62X1_TEST_LFSR_15 = 0, // BER test pattern is LFSR15
    AVL62X1_TEST_LFSR_23 = 1  // BER test pattern is LFSR23
  } avl62x1_test_pattern;

  // Defines the type of auto error statistics
  typedef enum avl62x1_auto_error_stats_type
  {
    AVL62X1_ERROR_STATS_BYTE = 0, // error statistics will be reset according to the number of received bytes.
    AVL62X1_ERROR_STATS_TIME = 1  // error statistics will be reset according to time interval.
  } avl62x1_auto_error_stats_type;

  // Defines Error statistics mode
  typedef enum avl62x1_error_stats_mode
  {
    AVL62X1_ERROR_STATS_MANUAL = 0,
    AVL62X1_ERROR_STATS_AUTO = 1
  } avl62x1_error_stats_mode;

  // Defines the DiSEqC status
  typedef enum avl62x1_diseqc_status
  {
    AVL62X1_DOS_Uninitialized = 0, // DiSEqC has not been initialized yet.
    AVL62X1_DOS_Initialized = 1,   // DiSEqC has been initialized.
    AVL62X1_DOS_InContinuous = 2,  // DiSEqC is in continuous mode.
    AVL62X1_DOS_InTone = 3,        // DiSEqC is in tone burst mode.
    AVL62X1_DOS_InModulation = 4   // DiSEqC is in modulation mode.
  } avl62x1_diseqc_status;

  typedef enum avl62x1_diseqc_tx_gap
  {
    AVL62X1_DTXG_15ms = 0, // The gap is 15 ms.
    AVL62X1_DTXG_20ms = 1, // The gap is 20 ms.
    AVL62X1_DTXG_25ms = 2, // The gap is 25 ms.
    AVL62X1_DTXG_30ms = 3  // The gap is 30 ms.
  } avl62x1_diseqc_tx_gap;

  typedef enum avl62x1_diseqc_tx_mode
  {
    AVL62X1_DTM_Modulation = 0, // Use modulation mode.
    AVL62X1_DTM_Tone0 = 1,      // Send out tone 0.
    AVL62X1_DTM_Tone1 = 2,      // Send out tone 1.
    AVL62X1_DTM_Continuous = 3  // Continuously send out pulses.
  } avl62x1_diseqc_tx_mode;

  typedef enum avl62x1_diseqc_rx_time
  {
    AVL62X1_DRT_150ms = 0, // Wait 150 ms for receive data and then close the input FIFO.
    AVL62X1_DRT_170ms = 1, // Wait 170 ms for receive data and then close the input FIFO.
    AVL62X1_DRT_190ms = 2, // Wait 190 ms for receive data and then close the input FIFO.
    AVL62X1_DRT_210ms = 3  // Wait 210 ms for receive data and then close the input FIFO.
  } avl62x1_diseqc_rx_time;

  typedef enum avl62x1_diseqc_waveform_mode
  {
    AVL62X1_DWM_Normal = 0,  // Normal waveform mode
    AVL62X1_DWM_Envelope = 1 // Envelope waveform mode
  } avl62x1_diseqc_waveform_mode;

  //GPIO pins by number and name
  typedef enum avl62x1_gpio_pin
  {
    AVL62X1_GPIO_Pin_10 = 0,
    AVL62X1_GPIO_Pin_TUNER_SDA = 0,
    AVL62X1_GPIO_Pin_11 = 1,
    AVL62X1_GPIO_Pin_TUNER_SCL = 1,
    AVL62X1_GPIO_Pin_12 = 2,
    AVL62X1_GPIO_Pin_S_AGC2 = 2,
    AVL62X1_GPIO_Pin_37 = 3,
    AVL62X1_GPIO_Pin_LNB_PWR_EN = 3,
    AVL62X1_GPIO_Pin_38 = 4,
    AVL62X1_GPIO_Pin_LNB_PWR_SEL = 4
  } avl62x1_gpio_pin;

  typedef enum avl62x1_gpio_pin_dir
  {
    AVL62X1_GPIO_DIR_OUTPUT = 0,
    AVL62X1_GPIO_DIR_INPUT = 1
  } avl62x1_gpio_pin_dir;

  typedef enum avl62x1_gpio_pin_value
  {
    AVL62X1_GPIO_VALUE_LOGIC_0 = 0,
    AVL62X1_GPIO_VALUE_LOGIC_1 = 1,
    AVL62X1_GPIO_VALUE_HIGH_Z = 2
  } avl62x1_gpio_pin_value;

  typedef enum avl62x1_sis_mis
  {
    AVL62X1_MIS = 0,
    AVL62X1_SIS = 1,
    AVL62X1_SIS_MIS_UNKNOWN = 2
  } avl62x1_sis_mis;

  struct avl62x1_carrier_info
  {
    uint8_t m_carrier_index;          //index of this carrier
    uint32_t m_rf_freq_kHz;           //RF frequency of the carrier in kHz
    int32_t m_carrier_freq_offset_Hz; //carrier frequency offset (from RF freq) in Hz

    //When locking with blind_sym_rate=false, this is the nominal symbol rate
    //When locking with blind_sym_rate=true, this is the max symbol rate to consider
    uint32_t m_symbol_rate_Hz;

    enum avl62x1_rolloff m_roll_off;
    enum avl62x1_standard m_signal_type;

    //AVL62X1_PL_SCRAM_AUTO: turn on/off automatic PL scrambling detection
    //AVL62X1_PL_SCRAM_XSTATE: 1 - LSB's represent the initial state of the x(i) LFSR
    //AVL62X1_PL_SCRAM_XSTATE: 0 - LSB's represent the sequence shift of the x(i) sequence in the Gold code, defined as the "code number n" in the DVB-S2 standard
    uint32_t m_PL_scram_key;

    uint8_t m_PLS_ACM; //PLS if CCM, 0 if ACM
    enum avl62x1_sis_mis m_SIS_MIS;
    uint8_t m_num_streams; //number of supported streams (that can be output)
    int16_t m_SNR_dB_x100; //
    enum avl62x1_modulation_mode m_modulation;
    enum avl62x1_pilot m_pilot;
    enum avl62x1_fec_length m_dvbs2_fec_length;
    union {
      enum avl62x1_dvbs_code_rate m_dvbs_code_rate;
      enum avl62x1_dvbs2_code_rate m_dvbs2_code_rate;
    } m_coderate;
    enum avl62x1_dvbs2_ccm_acm m_dvbs2_ccm_acm; //1:CCM, 0:ACM
  };

  //structure to save Multistream T2MI MPLP infromation
  struct avl62x1_t2mi_mplp
  {
    uint8_t PLP_Num;        //PLP NUM
    uint8_t PLPid_List[16]; //PLP_Id list
  };

  struct avl62x1_stream_info
  {
    uint8_t m_carrier_index; //index of carrier (avl62x1_carrier_info.m_CarrierIndex) that this stream is in
    enum avl62x1_dvb_stream_type m_stream_type;
    uint8_t m_ISI;
    uint8_t m_PLP_ID;                // use when lock TP
    uint16_t m_T2MI_PID;             // use when lock TP
    struct avl62x1_t2mi_mplp PLP_List; // save scan out the PLP list for T2MI ISI
  };

  struct avl62x1_error_stats
  {
    uint16_t m_LFSR_Sync;          // Indicates whether the receiver is synchronized with the transmitter generating the BER test pattern.
    uint16_t m_LostLock;           // Indicates whether the receiver has lost lock since the BER/PER measurement was started.
    struct avl_uint64 m_SwCntNumBits;   // A software counter which stores the number of bits which have been received.
    struct avl_uint64 m_SwCntBitErrors; // A software counter which stores the number of bit errors which have been detected.
    struct avl_uint64 m_NumBits;        // The total number of bits which have been received.
    struct avl_uint64 m_BitErrors;      // The total number of bit errors which have been detected.
    struct avl_uint64 m_SwCntNumPkts;   // A software counter which stores the number of packets which have been received.
    struct avl_uint64 m_SwCntPktErrors; // A software counter which stores the number of packet errors which have been detected.
    struct avl_uint64 m_NumPkts;        // The total number of packets which have been received.
    struct avl_uint64 m_PktErrors;      // The total number of packet errors which have been detected.
    uint32_t m_BER;                // The bit error rate scaled by 1e9.
    uint32_t m_PER;                // The packet error rate scaled by 1e9.
  };

  // Contains variables for storing error statistics used in the BER and PER calculations.
  struct avl62x1_error_stats_config
  {
    enum avl62x1_error_stats_mode eErrorStatMode;         // indicates the error statistics mode.
    enum avl62x1_auto_error_stats_type eAutoErrorStatType; // indicates the MPEG data sampling clock mode.
    uint32_t uiTimeThresholdMs;                       // used to set time interval for auto error statistics.
    uint32_t uiNumberThresholdByte;                   // used to set the received byte number threshold for auto error statistics.
  };

  // Contains variables for storing error statistics used in the BER and PER calculations.
  struct avl62x1_ber_config
  {
    enum avl62x1_test_pattern eBERTestPattern; // indicates the pattern of LFSR.
    enum avl62x1_lfsr_fb_bit eBERFBInversion;  // indicates LFSR feedback bit inversion.
    uint32_t uiLFSRSynced;                  // indicates the LFSR synchronization status.
    uint32_t uiLFSRStartPos;                //set LFSR start byte positon
  };

  // Stores DiSEqC operation parameters
  struct avl62x1_diseqc_params
  {
    uint16_t uiToneFrequencyKHz;                // The DiSEqC bus speed in units of kHz. Normally, it is 22kHz.
    enum avl62x1_diseqc_tx_gap eTXGap;             // Transmit gap
    enum avl62x1_diseqc_waveform_mode eTxWaveForm; // Transmit waveform format
    enum avl62x1_diseqc_rx_time eRxTimeout;        // Receive time frame window
    enum avl62x1_diseqc_waveform_mode eRxWaveForm; // Receive waveform format
  };

  /// Stores the DiSEqC transmitter status.
  ///
  struct avl62x1_diseqc_tx_status
  {
    uint8_t m_TxDone;      ///< Indicates whether the transmission is complete (1 - transmission is finished, 0 - transmission is still in progress).
    uint8_t m_TxFifoCount; ///< The number of bytes remaining in the transmit FIFO
  };

  /// Stores the DiSEqC receiver status
  ///
  struct avl62x1_diseqc_rx_status
  {
    uint8_t m_RxFifoCount; ///< The number of bytes in the DiSEqC receive FIFO.
    uint8_t m_RxDone;      ///< 1 if the receiver window is turned off, 0 if it is still in receiving state.
  };

  // The Availink version structure.
  struct avl_ver_info
  {
    uint8_t m_Major;  // The major version number.
    uint8_t m_Minor;  // The minor version number.
    uint16_t m_Build; // The build version number.
  };

  // Stores AVL62X1 device version information.
  struct avl62x1_ver_info
  {
    uint32_t m_Chip;   // Hardware version information. 0xPPPPSSSS (P:part number (6261), S:SVN revision in hex (23720))
    struct avl_ver_info m_API;   // SDK version information.
    struct avl_ver_info m_Patch; // The version of the internal patch.
  };

  struct avl62x1_chip
  {
    uint16_t usI2CAddr;
    enum avl62x1_xtal e_Xtal; // Reference clock
    uint8_t *pPatchData;

    struct AVL_Tuner *pTuner;                 // Pointer to AVL_Tuner struct instance
    enum avl62x1_spectrum_polarity e_TunerPol; // Tuner spectrum polarity (e.g. I/Q input swapped)

    enum avl62x1_mpeg_mode e_Mode;
    enum avl62x1_mpeg_clock_polarity e_ClkPol;
    enum avl62x1_mpeg_clock_phase e_ClkPhase;
    enum avl62x1_mpeg_clock_adaptation e_ClkAdapt;
    enum avl62x1_mpeg_format e_Format;
    enum avl62x1_mpeg_serial_pin e_SerPin;

    struct avl62x1_error_stats error_stats;

    //The desired MPEG clock frequency in units of Hz.
    //It is updated with the exact value after the demod has initialized
    uint32_t m_MPEGFrequency_Hz;
    uint32_t m_CoreFrequency_Hz; // The internal core clock frequency in units of Hz.
    uint32_t m_FECFrequency_Hz;  // FEC clk in Hz

    enum avl62x1_diseqc_status m_Diseqc_OP_Status;

    avl_sem_t m_semRx;     // A semaphore used to protect the receiver command channel.
    avl_sem_t m_semDiseqc; // A semaphore used to protect DiSEqC operation.

    uint32_t m_variable_array[PATCH_VAR_ARRAY_SIZE];
  };

  //structure to configure blind scan for a SINGLE TUNER STEP
  struct avl62x1_blind_scan_params
  {
    uint16_t m_TunerCenterFreq_100kHz;
    uint16_t m_TunerLPF_100kHz;
    uint16_t m_MinSymRate_kHz;
  };

  //structure to capture the information from blind scan of a SINGLE TUNER STEP
  struct avl62x1_blind_scan_info
  {
    uint8_t m_ScanProgress;     //approx progress in percent
    uint8_t m_BSFinished;       //!0 when BS completed
    uint8_t m_NumCarriers;      //number of confirmed S or S2/X carriers
    uint8_t m_NumStreams;       //DEPRECATED number of confirmed DVB output streams (e.g. TS, GSE)
    uint32_t m_NextFreqStep_Hz; //amount to move tuner (relative to its current position) for next BS step
  };

  extern int32_t carrier_freq_offset_hz;

  uint16_t Init_AVL62X1_ChipObject(struct avl62x1_chip *pAVL_ChipObject);

  uint16_t IBase_CheckChipReady_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_Initialize_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_GetVersion_AVL62X1(struct avl62x1_ver_info *pVer_info, struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_GetFunctionMode_AVL62X1(enum avl62x1_functional_mode *pFuncMode, struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_GetRxOPStatus_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_SendRxOP_AVL62X1(uint8_t ucOpCmd, struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_GetSPOPStatus_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_SendSPOP_AVL62X1(uint8_t ucOpCmd, struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_Halt_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_Sleep_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_Wakeup_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IBase_Initialize_TunerI2C_AVL62X1(struct avl62x1_chip *pAVL_Chip);

  uint16_t IRx_Initialize_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_GetTunerPola_AVL62X1(enum avl62x1_spectrum_polarity *pTuner_Pol, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_SetTunerPola_AVL62X1(enum avl62x1_spectrum_polarity enumTuner_Pol, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_DriveAGC_AVL62X1(enum avl62x1_switch enumOn_Off, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_GetCarrierFreqOffset_AVL62X1(int32_t *piFreqOffsetHz, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_GetSROffset_AVL62X1(int32_t *piSROffsetPPM, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_ErrorStatMode_AVL62X1(struct avl62x1_error_stats_config *stErrorStatConfig, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_ResetBER_AVL62X1(struct avl62x1_ber_config *pBERConfig, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_GetBER_AVL62X1(uint32_t *puiBER_x1e9, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_GetAcqRetries_AVL62X1(uint8_t *pucRetryNum, struct avl62x1_chip *pAVL_Chip);

  uint16_t IRx_SetMpegMode_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_SetMpegBitOrder_AVL62X1(enum avl62x1_mpeg_bit_order e_BitOrder, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_SetMpegErrorPolarity_AVL62X1(enum avl62x1_mpeg_err_polarity e_ErrorPol, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_SetMpegValidPolarity_AVL62X1(enum avl62x1_mpeg_err_polarity e_ValidPol, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_EnableMpegContiClock_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_DisableMpegContiClock_AVL62X1(struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_DriveMpegOutput_AVL62X1(enum avl62x1_switch enumOn_Off, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_ConfigPLS_AVL62X1(uint32_t uiShiftValue, struct avl62x1_chip *pAVL_Chip);
  uint16_t IRx_SetPLSAutoDetect_AVL62X1(struct avl62x1_chip *pAVL_Chip);

  uint16_t IDiseqc_Initialize_AVL62X1(struct avl62x1_diseqc_params *pDiseqcPara, struct avl62x1_chip *pAVL_Chip);
  uint16_t IDiseqc_IsSafeToSwitchMode_AVL62X1(struct avl62x1_chip *pAVL_Chip);

  uint16_t IBase_DownloadPatch_AVL62X1(struct avl62x1_chip *pAVL_Chip);

  uint8_t patch_read8_AVL62X1(uint8_t *pPatchBuf, uint32_t *idx);
  uint16_t patch_read16_AVL62X1(uint8_t *pPatchBuf, uint32_t *idx);
  uint32_t patch_read32_AVL62X1(uint8_t *pPatchBuf, uint32_t *idx);
  uint32_t Convert_XLFSRToN_AVL62X1(uint32_t XLFSR);

#ifdef AVL_CPLUSPLUS
}
#endif

#endif
