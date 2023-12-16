#ifndef __SAMPLE_H_
#define __SAMPLE_H_
#define OTP_LEN 512
#define SOFTWAREVERSION 0
#define TESTINGTIMEYEAR 1
#define TESTINGWEEK 2
#define BINNUMBER 3
#define TESTINGTIMEWEEKDAY 4
#define TESTINGTIMEHOURS 5
#define TESTINGTIMEMINUTES 6
#define PACKAGEHOUSEVENDORID 7
#define PACKAGEHOUSETESTMACHINEID1 8
#define PACKAGEHOUSETESTMACHINEID2 9
#define FT_RESULT_LEN 10

#define NOISE_CALC_BIN_OFFSET 9
#define NOISE_CALC_HIST_START 1
#define NOISE_CALC_HIST_SIZE 4

#define NOISE0_FIFO_OUT_BUF_INDEX 0
#define NOISE1_FIFO_OUT_BUF_INDEX 1
#define QLF_FIFO_OUT_BUF_INDEX 2
#define FLASHN_FIFO_OUT_BUF_INDEX 3
#define MP0_FIFO_OUT_BUF_INDEX 4
#define MV0_FIFO_OUT_BUF_INDEX 5
#define SP0_FIFO_OUT_BUF_INDEX 6
#define SV0_FIFO_OUT_BUF_INDEX 7
#define TP0_FIFO_OUT_BUF_INDEX 8
#define TV0_FIFO_OUT_BUF_INDEX 9
#define MP1_FIFO_OUT_BUF_INDEX 10
#define MV1_FIFO_OUT_BUF_INDEX 11
#define SP1_FIFO_OUT_BUF_INDEX 12
#define SV1_FIFO_OUT_BUF_INDEX 13
#define TP1_FIFO_OUT_BUF_INDEX 14
#define TV1_FIFO_OUT_BUF_INDEX 15

#define CHIP_ID_REG 0x00
#define I2C_ADDR_REG 0x01
#define CFG1_REG 0x02
#define CFG2_REG 0x03
#define CFG3_REG 0x04
#define CFG4_REG 0x05
#define PULSE_NUM_REG 0x06    //PULSE_NUM_REG,max flashes /16
#define ROOF_HGM_REG  0x07    //ROOF_HGM_REG,threshold to stop histogram accumulation
#define QLF_APL0_REG  0x08
#define QLF_APL1_REG  0x09
#define QLF_APH0_REG  0x0A
#define QLF_APH1_REG  0x0B
#define QLF_SPL0_REG  0x0C
#define QLF_SPL1_REG  0x0D
#define QLF_SPH0_REG  0x0E
#define QLF_SPH1_REG  0x0F

#define FDLY0_REG     0x10
#define FDLY1_REG     0x11
#define TDLY1_REG     0x13

#define STATUS_REG 0x1B
#define DTOF_REG_PTCO_H0 0x1C
#define DTOF_REG_PTCO_H1 0x1D
#define DTOF_REG_PTCO_H2 0x1E

#define DTOF_REG_OFFSET_PIXEL0 0x70
#define DTOF_REG_OFFSET_PIXEL1 0x71
#define DTOF_REG_OFFSET_PIXEL2 0x72
#define DTOF_REG_OFFSET_PIXEL3 0x73
#define DTOF_REG_OFFSET_PIXEL4 0x74
#define DTOF_REG_OFFSET_PIXEL5 0x75
#define DTOF_REG_OFFSET_PIXEL6 0x76
#define DTOF_REG_OFFSET_PIXEL7 0x77
#define DTOF_REG_OFFSET_PIXEL8 0x78
#define DTOF_REG_OFFSET_PIXEL9 0x79
#define DTOF_REG_OFFSET_PIXEL10 0x7A
#define DTOF_REG_OFFSET_PIXEL11 0x7B
#define DTOF_REG_OFFSET_PIXEL12 0x7C
#define DTOF_REG_OFFSET_PIXEL13 0x7D
#define DTOF_REG_OFFSET_PIXEL14 0x7E
#define DTOF_REG_OFFSET_PIXEL15 0x7F

#define DTOF_REG_NE_XT0      0x80
#define DTOF_REG_NE_XT1      0x81
#define DTOF_REG_NE_XT2      0x82
#define DTOF_REG_NE_XT3      0x83
#define DTOF_REG_NE_XT4      0x84
#define DTOF_REG_NE_XT5      0x85
#define DTOF_REG_NE_XT6      0x86
#define DTOF_REG_NE_XT7      0x87
#define DTOF_REG_NE_XT8      0x88
#define DTOF_REG_NE_XT9      0x89
#define DTOF_REG_NE_XT10     0x8A
#define DTOF_REG_NE_XT11     0x8B
#define DTOF_REG_NE_XT12     0x8C
#define DTOF_REG_NE_XT13     0x8D
#define DTOF_REG_NE_XT14     0x8E
#define DTOF_REG_NE_XT15     0x8F
#define DTOF_REG_NE_XT_BSTR  0x90
#define DTOF_REG_OSC_TRIM 0xA1
#define DTOF_REG_PLL_CTRL 0xA2
#define DTOF_REG_TGT_CNT 0xA7
#define DTOF_REG_BGR_VTRIM 0xA8
#define DTOF_REG_OSC_CAL_CTRL 0xA3
#define DTOF_REG_OSC_CNT 0xA4
#define DTOF_REG_PVT_CNT 0xA6
#define DTOF_REG_LDD_CTRL0_REG 0xB0
#define DTOF_REG_DRV_FREQCTR 0xB1

#define DTOF_CFG5_REG     0xD0
#define DTOF_DFT_RSV_REG  0xD1
#define DTOF_GPIO_CFG1_REG 0xD2
#define DTOF_GPIO_CFG2_REG 0xD3
#define DTOF_REG_SPD_MSK0 0xD4
#define DTOF_REG_SPD_MSK1 0xD5
#define DTOF_REG_SPD_MSK2 0xD6
#define DTOF_REG_SPD_MSK3 0xD7
#define DTOF_REG_SPD_MSK4 0xD8
#define DTOF_REG_SPD_MSK5 0xD9
#define DTOF_REG_SPD_MSK6 0xDA
#define DTOF_REG_SPD_MSK7 0xDB
#define DTOF_REG_SPD_MSK8 0xDC
#define DTOF_REG_SPD_MSK9 0xDD

#define DTOF_REG_PTC_VLDH 0xF0
#define DTOF_REG_PTCO_H3 0xF1
#define DTOF_REG_PTCO_H4 0xF2
#define DTOF_REG_PTCO_H5 0xF3
#define DTOF_REG_PTCO_H6 0xF4
#define DTOF_REG_PTCO_H7 0xF5
#define DTOF_REG_PTCO_H8 0xF6
#define DTOF_REG_PTCO_H9 0xF7
#define DTOF_REG_PTCO_H10 0xF8
#define DTOF_REG_PTCO_H11 0xF9
#define DTOF_REG_PTCO_H12 0xFA
#define DTOF_REG_OTP_CFG1 0xFB
#define DTOF_REG_OTP_CFG2 0xFC

#define DTOF_REG_RAM_STARTA 0xFE
#define DTOF_REG_RAM_DATA 0xFF

#define DTOF_RESULT_SUCCESS 0
#define DTOF_RESULT_ERROR -1

#define DTOF_REG_PULSE_NUM 0x06

#define CROSS_TALK_HISG_BIN_SIZE 32
#define CROSS_TALK_AC_DATA_SIZE 16
#define CROSS_TALK_CALIB_FLASH_CNT 0x4000
#define CROSS_TALK_CALIB_FRAME_MAX_SIZE 100
#define CROSS_TALK_CALIB_AC_FACTOR 1050
#define AGC_MAX_FLASH_NUM 65536
#define VAL_BOOST_MULTIPLE 10

#define I2C_CHIP_ADDR 0x10
#define UUID_CP_OTP_ADDR 0x02
#define UUID_FT_OTP_ADDR 0x0e
#define SPAD_MASK_OTP_ADDR 0x12
#define TRIM1_OTP_ADDR 0x26
#define TRIM_OSC_TCSEL_OTP_ADDR 0x2b
#define TRIM2_OTP_ADDR 0x2c
#define BIN_OFFSET_OTP_ADDR 0x30
#define DRV_PULSE_WIDTH_OTP_ADDR 0x31
#define SPAD_MSK_FT_OTP_ADDR 0x32
#define OSCTRIM_OFFSET_OTP_ADDR 0X46
#define FT_RESULT_OTP_ADDR 0X48
#define RTEMP_CHECKSUM_OTP_ADDR 0x50
#define HTEMP_CHECKSUM_OTP_ADDR 0x52
#define OSC_FREQVSTEMP_OTP_ADDR 0x5a
#define OSC_FREQVSTCCODE_OTP_ADDR 0x6a
#define BANK_CHECK_SUM_1ST_OTP_ADDR 0x7E
#define PACKAGE_CROSSTALK_DATA_OTP_ADDR 0x80

#define I2C_CHIP_ADDR_OTP_LEN 2
#define UUID_CP_OTP_LEN 12
#define UUID_FT_OTP_LEN 4
#define SPAD_MASK_OTP_LEN 20
#define TRIM1_OTP_LEN 5
#define TRIM_OSC_TCSEL_OTP_LEN 1
#define TRIM2_OTP_LEN 5
#define BIN_OFFSET_OTP_LEN 1
#define SPAD_MSK_FT_OTP_LEN 4
#define DRV_PULSE_WIDTH_OTP_LEN 1
#define OSCTRIM_OFFSET_OTP_LEN 2
#define FT_RESULT_OTP_LEN 8
#define RTEMP_CHECKSUM_OTP_LEN 2
#define HTEMP_CHECKSUM_OTP_LEN 2
#define OSC_FREQVSTEMP_OTP_LEN 12
#define OSC_FREQVSTCCODE_OTP_LEN 20
#define BANK_CHECK_SUM_1ST_OTP_LEN 2
#define PACKAGE_CROSSTALK_DATA_OTP_LEN 17

#define CUPCAKE_MODULE 0X0001
#define DTOF_MODULE CUPCAKE_MODULE

#define ECO1 0xa11b
#define ECO2 0xa118

void sample_init();
void sample_hold_run();
void sample_get_filter_distance();
void sample_calibration();

int dtof_hold_run_once();
void dtof_set_threshold_parameter(float distance_max, float distance_min, float confidence_max, float confidence_min);
int dtof_calculate_noise(uint16_t *hist, uint16_t size, uint16_t *noise);
void dtof_lib_debug(uint16_t *fifo, float *snr, float *dis);
int dtof_get_distance(uint16_t *fifo, uint16_t noise, float *distance);
int dtof_cross_talk_sample_data(uint16_t frame_size, uint16_t *cross_talk_calib_data);
int dtof_cross_talk_calibrate_calculate(const uint16_t *cross_talk_hisg, uint16_t size, uint16_t *cross_talk_ac_data, uint16_t *cross_talk_dc_data, uint16_t *ne_xt_bstr);
int dtof_update_cross_talk_ac_data_reg(uint16_t *value, uint16_t size);
int dtof_update_cross_talk_dc_data_reg(uint16_t value);
int dtof_cross_talk_get_frame_data(uint16_t offset, uint16_t *out_buf, uint16_t len);
int prepare_get_one_frame();
int dtof_clear_pcgi_intr(); 
int dtof_clear_ram();
int dtof_block();
int dtof_read_fifo(uint16_t *out_buf);

int read_ram(uint16_t* out_buf, int offset, int len);
void read_otp(uint16_t* out_buf, int offset, int len);

void otp_mode_init();
int update_otp_param_to_reg(uint16_t chip_id);
int update_spad_mask_otp_to_reg(const uint8_t* otp_data);
int read_otp_from_ram(const uint8_t* opt_data, uint16_t* out_buf, int offset, int len);
int update_trim1_otp_to_reg(const uint8_t* otp_data);
int update_trim_osc_tcsel_otp_to_reg(const uint8_t* otp_data, uint16_t chip_id);
int update_trim2_otp_to_reg(const uint8_t* otp_data, uint16_t chip_id);
int update_bin_offset_otp_to_reg(const uint8_t* otp_data);
int update_spad_msk_ft_otp_to_reg(const uint8_t* otp_data);
int update_osc_trim_offset_value_otp(const uint8_t* otp_data);
int update_ft_result_otp_to_reg(const uint8_t* otp_data);
int update_osc_freqvstemp_otp_to_reg(const uint8_t* otp_data, uint16_t chip_id);
int update_osc_freqvstccode_otp_to_reg(const uint8_t* otp_data, uint16_t chip_id);
int update_bank_check_sum_1st_otp_to_reg(const uint8_t* otp_data);
int update_package_crosstalk_data_otp_to_reg(const uint8_t* otp_data);
bool reg_readback(uint8_t reg_addr, uint8_t offset, uint16_t otp_val, uint16_t otp_mask);

void dump_regs();
float calculateConfidence(uint16_t noise0, uint16_t mv0);
float calculateDistance(uint16_t peak, uint16_t ref_index);
int modify_bit(uint8_t addr, uint16_t mask, uint16_t val);
uint32_t value_round_up(uint32_t value, uint32_t multiple);
int dtof_write_reg(uint8_t dtof_reg_addr, uint16_t dtof_reg_data);
int dtof_read_reg(uint8_t dtof_reg_addr, uint16_t *dtof_val);
int write_reg(uint8_t reg_addr, uint16_t reg_val);
int read_reg(uint8_t reg_addr, uint16_t *reg_val);

#endif
