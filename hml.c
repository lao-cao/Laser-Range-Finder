#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "I2Cdev.h"
#include "hml.h"

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#define swap16bit_big_little(val) ((((val)&0x00FF) << 8) | ((val)&0xFF00) >> 8)

typedef struct OtpCheckSum_ {
    bool otp_check_sum_flag;          // 0:disable otp check sum功能, 1:enable
    bool otp_rtemp_check_sum_result;  // 0:fail 1:pass
    bool otp_htemp_check_sum_result;
    bool otp_bank_check_sum_1st_result;
    uint16_t otp_rtemp_check_sum_value_from_cal;  //计算出来的值
    uint16_t otp_htemp_check_sum_value_from_cal;
    uint16_t otp_bank_check_sum_1st_value_from_cal;
    uint16_t otp_rtemp_check_sum_value;  //直接读otp得到的值
    uint16_t otp_htemp_check_sum_value;
    uint16_t otp_bank_check_sum_1st_value;
} OtpCheckSum;

typedef struct lib_param_st_{
    float distance_max;
    float distance_min;
    float confidence_max;
    float confidence_min;
    float bin_width;
    float offset;
} lib_param_st;

typedef struct reg_node {
    uint8_t reg_addr;
    uint16_t reg_value;
} reg_node_st;

static uint8_t otp_data[OTP_LEN] = {0};
static uint8_t ft_result[FT_RESULT_LEN] = {0};
static uint16_t main_hist[384];
static int16_t s_oscTrim_off = 0;
static uint16_t bank_check_sum_1st = 0;
static lib_param_st s_lib_param = {
    .distance_max = 6000.0f,
    .distance_min = 0.0f,
    .confidence_max = 200.0f,
    .confidence_min = 0.0f,
    .bin_width = 18.75f,
    .offset = 187.5f,
};
static OtpCheckSum otpCheckSum;

static const reg_node_st s_defalut_setting[] = {
    {0xb1, 0x8141},
    {0xb0, 0x03b3},
    {0x07, 0x7f00},
    {0x10, 0xe40a},
    {0x11, 0x0007},
    {0xa1, 0x3480},
    {0xa3, 0x011f},
    {0x20, 0xffff},
    {0x06, 0x8000},
    {0x02, 0xB611},
    {0x03, 0x000f},
    {0x04, 0xF790},
    {0xa8, 0x6210}};

static const reg_node_st s_kernel_setting[] = {
    {0x30, 0x02E6},
    {0x31, 0x070B},
    {0x32, 0x0B3B},
    {0x33, 0x0E5D},
    {0x34, 0x0FEE},
    {0x35, 0x1006},
    {0x36, 0x0EF3},
    {0x37, 0x0CEC},
    {0x38, 0x0A2D},
    {0x39, 0x0726},
    {0x3a, 0x047D},
    {0x3b, 0x02B3},
    {0x3c, 0x01DA},
    {0x3d, 0x01A1},
    {0x3e, 0x019D},
    {0x3f, 0x0192},
    {0x40, 0x0178},
    {0x4f, 0x02E6},
    {0x50, 0x070B},
    {0x51, 0x0B3B},
    {0x52, 0x0E5D},
    {0x53, 0x0FEE},
    {0x54, 0x1006},
    {0x55, 0x0EF3},
    {0x56, 0x0CEC},
    {0x57, 0x0A2D},
    {0x58, 0x0726},
    {0x59, 0x047D},
    {0x5a, 0x02B3},
    {0x5b, 0x01DA},
    {0x5c, 0x01A1},
    {0x5d, 0x019D},
    {0x5e, 0x0192},
    {0x5f, 0x0178},
};

void sample_init()
{
    uint16_t reg;
    uint16_t chipid;

    printf("-----------------read chipid-----------------\n");
		read_reg(CHIP_ID_REG, &chipid);
		
		printf("Chip_id = 0x%x \r\n",chipid);

    otp_mode_init();
    printf("-----------------wait pll lock-----------------\n");
    write_reg(DTOF_REG_PLL_CTRL, 0x37D3);
    while((reg & 0x08) == 0)
    {
        read_reg(STATUS_REG, &reg);
    }
    write_reg(DTOF_CFG5_REG, 0x0008);

    printf("-----------------set default-----------------\n");

    for (int i = 0; i < sizeof(s_defalut_setting) / sizeof(reg_node_st); i++)
    {
        write_reg(s_defalut_setting[i].reg_addr, s_defalut_setting[i].reg_value);
    }

    printf("-----------------MF-----------------\n");

    for (int i = 0; i < sizeof(s_kernel_setting) / sizeof(reg_node_st); i++)
    {
        write_reg(s_kernel_setting[i].reg_addr, s_kernel_setting[i].reg_value);
    }

    update_otp_param_to_reg(chipid);

    printf("-----------------set cross talk-----------------\n");
#if 1
    write_reg(DTOF_REG_NE_XT0, 0x0001);
    write_reg(DTOF_REG_NE_XT1, 0x0001);
    write_reg(DTOF_REG_NE_XT2, 0x0001);
    write_reg(DTOF_REG_NE_XT3, 0x0001);
    write_reg(DTOF_REG_NE_XT4, 0x0001);
    write_reg(DTOF_REG_NE_XT5, 0x0001);
    write_reg(DTOF_REG_NE_XT6, 0x0001);
    write_reg(DTOF_REG_NE_XT7, 0x0001);
    write_reg(DTOF_REG_NE_XT8, 0x0001);
    write_reg(DTOF_REG_NE_XT9, 0x0001);
    write_reg(DTOF_REG_NE_XT10, 0x0001);
    write_reg(DTOF_REG_NE_XT11, 0x0001);
    write_reg(DTOF_REG_NE_XT12, 0x0001);
    write_reg(DTOF_REG_NE_XT13, 0x0001);
    write_reg(DTOF_REG_NE_XT14, 0x0001);
    write_reg(DTOF_REG_NE_XT15, 0x0001);
    write_reg(DTOF_REG_NE_XT_BSTR, 0x0001);
#else
    write_reg(DTOF_REG_NE_XT0, 0x0021);
    write_reg(DTOF_REG_NE_XT1, 0x0064);
    write_reg(DTOF_REG_NE_XT2, 0x00A3);
    write_reg(DTOF_REG_NE_XT3, 0x00AB);
    write_reg(DTOF_REG_NE_XT4, 0x0062);
    write_reg(DTOF_REG_NE_XT5, 0x005A);
    write_reg(DTOF_REG_NE_XT6, 0x0045);
    write_reg(DTOF_REG_NE_XT7, 0x0043);
    write_reg(DTOF_REG_NE_XT8, 0x003D);
    write_reg(DTOF_REG_NE_XT9, 0x0042);
    write_reg(DTOF_REG_NE_XT10, 0x0037);
    write_reg(DTOF_REG_NE_XT11, 0x0037);
    write_reg(DTOF_REG_NE_XT12, 0x0033);
    write_reg(DTOF_REG_NE_XT13, 0x0039);
    write_reg(DTOF_REG_NE_XT14, 0x0030);
    write_reg(DTOF_REG_NE_XT15, 0x0031);
    write_reg(DTOF_REG_NE_XT_BSTR, 0x0002);
#endif
    printf("-----------------set main ref spad mask-----------------\n");
    // main
    write_reg(DTOF_REG_SPD_MSK0, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK1, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK2, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK3, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK4, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK5, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK6, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK7, 0xFFFF);
    // ref
    write_reg(DTOF_REG_SPD_MSK8, 0xFFFF);
    write_reg(DTOF_REG_SPD_MSK9, 0x0000);
    printf("-----------------bin offset-----------------\n");
    //通过第2 3 4 5几个点获取底噪，计算snr
    write_reg(DTOF_REG_OFFSET_PIXEL0, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL1, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL2, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL3, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL4, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL5, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL6, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL7, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL8, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL9, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL10, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL11, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL12, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL13, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL14, NOISE_CALC_BIN_OFFSET);
    write_reg(DTOF_REG_OFFSET_PIXEL15, NOISE_CALC_BIN_OFFSET);
    printf("-----------------range mode----------------\n"); 
    write_reg(DTOF_REG_PLL_CTRL, 0x37D3);
}

/**
 * @brief 
 * sample code
 */
void sample_hold_run(void)
{
    uint16_t fifo_data[16];
    while(1)
    {
        dtof_clear_pcgi_intr();
        // 清ram
        dtof_clear_ram();
        dtof_hold_run_once();
        dtof_block();
        dtof_read_fifo(fifo_data);

        printf("mp0 = %d, mp1 = %d, D=%fmm\n",
            fifo_data[MP0_FIFO_OUT_BUF_INDEX],
            fifo_data[MP1_FIFO_OUT_BUF_INDEX],
            (fifo_data[MP0_FIFO_OUT_BUF_INDEX] - fifo_data[MP1_FIFO_OUT_BUF_INDEX]) * s_lib_param.bin_width + s_lib_param.offset);//only work while distance >= 37.5mm
        usleep(1000);
    }
}
/**
 * @brief 
 * sample code
 */

void sample_get_filter_distance(void)
{
    float distance = 0.0f;
    uint16_t fifo_data[16];
    int ret;
    uint16_t noise;
    
    //dump_regs();
    
    while(1)
    {
        dtof_clear_pcgi_intr();
        // 清ram
        dtof_clear_ram();
        dtof_hold_run_once();
        dtof_block();
        dtof_read_fifo(fifo_data);
        read_ram(main_hist, 0, 384);
        dtof_calculate_noise(&main_hist[NOISE_CALC_HIST_START], NOISE_CALC_HIST_SIZE, &noise);

        float snr[3];
        float dis[3];
        dtof_lib_debug(fifo_data, snr, dis);
        printf("fifo data:mp0=%d, mv0=%d, sp0=%d, sv0=%d, tp0=%d, tv0=%d, mp1=%d, noise0=%d\r\n",
            fifo_data[MP0_FIFO_OUT_BUF_INDEX], fifo_data[MV0_FIFO_OUT_BUF_INDEX],
            fifo_data[SP0_FIFO_OUT_BUF_INDEX], fifo_data[SV0_FIFO_OUT_BUF_INDEX],
            fifo_data[TP0_FIFO_OUT_BUF_INDEX], fifo_data[TV0_FIFO_OUT_BUF_INDEX],
            fifo_data[MP1_FIFO_OUT_BUF_INDEX], noise);
        printf("snr:%f | %f | %f;  dis:%f | %f | %f;\r\n", snr[0], snr[1], snr[2], dis[0], dis[1], dis[2]);

        /*
        int snr_index = 1;
        float max_snr = snr[0];
        float max_snr_dis = dis[0];
        while(snr_index <= 2)
        {
        	if(max_snr < snr[snr_index])
        	{
        		max_snr_dis = dis[snr_index];
        	}
        	
        	snr_index = snr_index + 1;
        }
        
        printf("distance with max snr:%f \r\n", max_snr_dis);
        */
        ret = dtof_get_distance(fifo_data, noise, &distance);
        if (1 == ret) {
            log_printf("get valid distance = %f !\r\n", distance);
        } else {
            log_printf("not get valid distance,ret=%d !\r\n", ret);
        }
        usleep(1000);
    }
}

/**
 * @brief 
 * sample code
 */
void sample_calibration(void)
{
    uint16_t original_data[CROSS_TALK_HISG_BIN_SIZE];
    uint16_t reg_80_8f[CROSS_TALK_HISG_BIN_SIZE/2];
    uint16_t ratio;
    uint16_t reg_90;
    int ret;
    
    printf("-----------------crosstal calibrate----------------\n");
    // 初始化
    
    // 采样cross talk数据
    ret = dtof_cross_talk_sample_data(10, original_data);
    if (DTOF_RESULT_SUCCESS != ret) {
        printf("sample data fail!\r\n");
        return ;
    }

    // 计算cross talk校准参数
    ret = dtof_cross_talk_calibrate_calculate(original_data, CROSS_TALK_HISG_BIN_SIZE, reg_80_8f, &reg_90, &ratio);
    if (DTOF_RESULT_SUCCESS != ret) {
        printf("cross talk calibrate calculate fail!\r\n");
        return ;
    }

    // 更新cross talk校准寄存器
    // Write calib param(80-8f) into reg
    ret = dtof_update_cross_talk_ac_data_reg(reg_80_8f, CROSS_TALK_HISG_BIN_SIZE / 2);
    if (DTOF_RESULT_SUCCESS != ret) {
        printf("update cross talk ac data reg fail!\r\n");
        return ;
    }

    // Write calib param(90) into reg
    ret = dtof_update_cross_talk_dc_data_reg(reg_90);
    if (DTOF_RESULT_SUCCESS != ret) {
        printf("update cross talk dc data reg fail!\r\n");
        return ;
    }
    
    printf("-------------cross talk param init: calibrate data----------\r\n");
    for (int i = 0; i < CROSS_TALK_HISG_BIN_SIZE; i++) {
        printf("%d : %d\n", i, original_data[i]);
    }
    printf("-------------cross talk param init: ac data----------\r\n");
    for (int i = 0; i < CROSS_TALK_HISG_BIN_SIZE / 2; i++) {
        printf("0x%x : 0x%x\n", i+0x80, reg_80_8f[i]);
    }
    printf("-------------cross talk param init: dc data----------\r\n");
    printf("dc=%d ne_xt_bstr=%d\n", reg_90, ratio);

    return;
}

int dtof_hold_run_once() {
    write_reg(CFG1_REG, 0x7612);
    write_reg(CFG1_REG, 0x3612);

    return 0;
}

int dtof_calculate_noise(uint16_t *hist, uint16_t size, uint16_t *noise) {
    uint32_t sum = 0;
    int i;

    if(!hist || !noise) {
        return -1;
    }
    for (i=0;i<size;i++) {
        sum += hist[i];
    }
    *noise = sum/size;
    return 0;
}

void dtof_lib_debug(uint16_t *fifo, float *snr, float *dis) {

    snr[0] = calculateConfidence(fifo[NOISE0_FIFO_OUT_BUF_INDEX], fifo[MV0_FIFO_OUT_BUF_INDEX]);
    dis[0] = calculateDistance(fifo[MP0_FIFO_OUT_BUF_INDEX], fifo[MP1_FIFO_OUT_BUF_INDEX]);


    snr[1] = calculateConfidence(fifo[NOISE0_FIFO_OUT_BUF_INDEX], fifo[SV0_FIFO_OUT_BUF_INDEX]);
    dis[1] = calculateDistance(fifo[SP0_FIFO_OUT_BUF_INDEX], fifo[MP1_FIFO_OUT_BUF_INDEX]);


    snr[2] = calculateConfidence(fifo[NOISE0_FIFO_OUT_BUF_INDEX], fifo[TV0_FIFO_OUT_BUF_INDEX]);
    dis[2] = calculateDistance(fifo[TP0_FIFO_OUT_BUF_INDEX], fifo[MP1_FIFO_OUT_BUF_INDEX]);
}

int dtof_get_distance(uint16_t *fifo, uint16_t noise, float *distance) {
    float Confidence = (float)0x8000;
    float temp = (float)0x8000;
    float local_dis = (float)0x8000;
    uint16_t peak = 0;
    uint16_t ref_index = 0;

    if(!fifo || !distance) {
        return -1;
    }
    local_dis = (float)0x8000;

    Confidence = calculateConfidence(noise, fifo[MV0_FIFO_OUT_BUF_INDEX]);
    if (Confidence >= s_lib_param.confidence_min && Confidence <= s_lib_param.confidence_max) {
        temp = calculateDistance(fifo[MP0_FIFO_OUT_BUF_INDEX], fifo[MP1_FIFO_OUT_BUF_INDEX]);
        if(temp >= s_lib_param.distance_min && temp <= s_lib_param.distance_max) {
            local_dis = temp;
        }
    }

    Confidence = calculateConfidence(noise, fifo[SV0_FIFO_OUT_BUF_INDEX]);
    if (Confidence >= s_lib_param.confidence_min && Confidence <= s_lib_param.confidence_max) {
        temp = calculateDistance(fifo[SP0_FIFO_OUT_BUF_INDEX], fifo[MP1_FIFO_OUT_BUF_INDEX]);
        if(temp >= s_lib_param.distance_min && temp <= s_lib_param.distance_max) {
            if(local_dis > (0x8000 - 1) || temp < local_dis) {
                local_dis = temp;
            }
        }
    }


    Confidence = calculateConfidence(noise, fifo[TV0_FIFO_OUT_BUF_INDEX]);
    if (Confidence >= s_lib_param.confidence_min && Confidence <= s_lib_param.confidence_max) {
        temp = calculateDistance(fifo[TP0_FIFO_OUT_BUF_INDEX], fifo[MP1_FIFO_OUT_BUF_INDEX]);
        if(temp >= s_lib_param.distance_min && temp <= s_lib_param.distance_max) {
            if(local_dis > (0x8000 - 1) || temp < local_dis) {
                local_dis = temp;
            }
        }
    }


    if(local_dis > (0x8000 - 1)) {
        return 0;
    }

    *distance = local_dis;

    return 1;
}

int dtof_cross_talk_sample_data(
    uint16_t frame_size, uint16_t *cross_talk_calib_data) {
    uint16_t repeat_n = 0;
    uint16_t captured_frm_cnt = 0;
    uint32_t calib_hsg[CROSS_TALK_HISG_BIN_SIZE];
    uint16_t reg_06;
    int i;
    int ret;

    if ((0 == frame_size || CROSS_TALK_CALIB_FRAME_MAX_SIZE < frame_size) ||
        !cross_talk_calib_data) {
        return DTOF_RESULT_ERROR;
        
    }

    memset(calib_hsg, 0, sizeof(calib_hsg));

    // step1:record reg06 and set reg06=0x4000
    ret = read_reg(DTOF_REG_PULSE_NUM, &reg_06);
    if (ret == 0) {

        return DTOF_RESULT_ERROR;
    }
    ret = write_reg(DTOF_REG_PULSE_NUM, CROSS_TALK_CALIB_FLASH_CNT);
    if (ret == 0) {
        return DTOF_RESULT_ERROR;
    }

    // step2:calculate the hist based on 65536 pulse number
    repeat_n = (AGC_MAX_FLASH_NUM / CROSS_TALK_CALIB_FLASH_CNT) * frame_size;

    // step3:start loop for getting main histogram calc data
    while (repeat_n > captured_frm_cnt) {
        // Read the first 32 bin from RAM
        if (DTOF_RESULT_SUCCESS !=
            dtof_cross_talk_get_frame_data(0, cross_talk_calib_data,
                                           CROSS_TALK_HISG_BIN_SIZE)) {
            return DTOF_RESULT_ERROR;
        }
        // And the sum of the first 32 bin
        for (i = 0; i < CROSS_TALK_HISG_BIN_SIZE; i++) {
            calib_hsg[i] += cross_talk_calib_data[i];
        }
        ++captured_frm_cnt;
    }

    // step4:get the average base 65536
    for (i = 0; i < CROSS_TALK_HISG_BIN_SIZE; i++) {
        // Filter out bin0
        if (0 == i) {
            cross_talk_calib_data[i] = 0;
            continue;
        }
        cross_talk_calib_data[i] = (calib_hsg[i] / frame_size);
    }
    cross_talk_calib_data[0] = 0;

    // step5:restore reg06
    ret = write_reg(DTOF_REG_PULSE_NUM, reg_06);
    if (ret == 0) {
        return DTOF_RESULT_ERROR;
    }

    return DTOF_RESULT_SUCCESS;
}

int dtof_cross_talk_calibrate_calculate(
    const uint16_t *cross_talk_hisg,
    uint16_t size,
    uint16_t *cross_talk_ac_data,
    uint16_t *cross_talk_dc_data,
    uint16_t *ne_xt_bstr)
{
    int i;
    uint16_t maxBin = 0;
    uint16_t tempdata = 0;
    uint16_t reg_06;
    int ret;

    if (!cross_talk_hisg || (CROSS_TALK_HISG_BIN_SIZE != size) ||
        !cross_talk_ac_data || !cross_talk_dc_data || !ne_xt_bstr) {
        return DTOF_RESULT_ERROR;
    }

    // step 1. The average value of 32 BIN pairs is calculated to obtain 16 BIN
    // data Calculate the value of reg:0x80-0x8f
    for (i = 0; i < size; i += 2) {
        // Filter out bin0: The count number of cross_talk bin0 may be greater
        // than the normal peak value
        if (0 == i) {
            tempdata = cross_talk_hisg[1];
        } else {
            tempdata = (cross_talk_hisg[i] + cross_talk_hisg[i + 1]) / 2;
        }

        if (tempdata > 0) {
            cross_talk_ac_data[i / 2] = ((uint16_t)(value_round_up(
                (tempdata * CROSS_TALK_CALIB_AC_FACTOR), 1000)));
           
        } else {
            cross_talk_ac_data[i / 2] = 1;  // 0x80-0x8f min = 1
        }

        if (cross_talk_ac_data[i / 2] > maxBin) {
            maxBin = cross_talk_ac_data[i / 2];
        }
    }

    // step 2. Calculate the ratio of reg:0x90
    if (0x0100 > maxBin) {
        *ne_xt_bstr = 1;
    } else {
        *ne_xt_bstr = ((maxBin / 0x0100) + 1);
    }

    if (*ne_xt_bstr > 1) {
        for (i = 0; i < size / 2; ++i) {
            cross_talk_ac_data[i] /= (*ne_xt_bstr);
        }
    }
    ret = read_reg(DTOF_REG_PULSE_NUM, &reg_06);
    if (ret == 0) {
        return DTOF_RESULT_ERROR;
    }

    *cross_talk_dc_data =
        *ne_xt_bstr * VAL_BOOST_MULTIPLE * reg_06 / AGC_MAX_FLASH_NUM;
    *cross_talk_dc_data =
        ((uint16_t)(value_round_up(*cross_talk_dc_data, VAL_BOOST_MULTIPLE)));


    return DTOF_RESULT_SUCCESS;
}

int dtof_update_cross_talk_ac_data_reg(uint16_t *value, uint16_t size) {
    uint16_t i;
    int ret;

    if (!value || (CROSS_TALK_HISG_BIN_SIZE / 2 != size)) {
        return DTOF_RESULT_ERROR;
    }

    for (i = 0; i < size; i++) {
        ret = write_reg((uint8_t)(DTOF_REG_NE_XT0 + i), value[i]);
        if (ret == 0) {
            return DTOF_RESULT_ERROR;
        }
    }
    return DTOF_RESULT_SUCCESS;
}

int dtof_update_cross_talk_dc_data_reg(uint16_t value) {
    int ret;

    ret = write_reg((uint8_t)(DTOF_REG_NE_XT_BSTR), value);
    if (ret == 0) {
        return DTOF_RESULT_ERROR;
    }
    return DTOF_RESULT_SUCCESS;
}

int dtof_cross_talk_get_frame_data(uint16_t offset,
                                                  uint16_t *out_buf,
                                                  uint16_t len) {
    int ret;

    // uint16_t tmp = 0;
    if (!out_buf || 0 == len) {
        return DTOF_RESULT_ERROR;
    }

    ret = prepare_get_one_frame();
    if (DTOF_RESULT_SUCCESS != ret) {
        return DTOF_RESULT_ERROR;
    }
    ret = read_ram(out_buf, offset, len);
    if (ret == 0) {
        return DTOF_RESULT_ERROR;
    }

    return DTOF_RESULT_SUCCESS;
}

int prepare_get_one_frame() {
    int ret;
    ret = dtof_clear_ram();
    if (ret) {
        printf("clear ram fail\n");
        return -1;
    }
    ret = dtof_hold_run_once();
    if (ret) {
        printf("hold run once fail\n");
        return -1;
    }
    ret = dtof_block();
    if (ret) {
        printf("block fail\n");
        return -1;
    }
    ret = dtof_clear_pcgi_intr();
    if (ret) {
        printf("clear interrupt fail\n");
        return -1;
    }
    return 0;
}

int dtof_clear_pcgi_intr()
{
    //uint16_t val;
    //read_reg(DTOF_REG_PTC_VLDH, &val);
    return 0;
}

int dtof_clear_ram() {
    write_reg(TDLY1_REG, 0x0801);
    write_reg(PULSE_NUM_REG, 0x0001);
    write_reg(ROOF_HGM_REG, 0x0001);

    write_reg(TDLY1_REG, 0x0001);
    write_reg(ROOF_HGM_REG, 0x7F00);
    write_reg(PULSE_NUM_REG, 0x4C00);

    return 0;
}

int dtof_block() {
    int ret = 0;
    uint16_t tmp;
    ret = read_reg(STATUS_REG, &tmp);
    while ((tmp & 0x0007) >= 0x0003) {
        ret = read_reg(STATUS_REG, &tmp);
        if (ret == 0) {
            printf("read reg 0x%02x failed.\n", 0x1b);
            return 1;
        }
    }
    return 0;
}

int dtof_read_fifo(uint16_t *out_buf)
{

    if (out_buf == NULL)
    {
        return -1;
    }
    uint16_t fifo_val[13] = {0};
    
    for (int i = 0; i < 3; i++)
    {
        read_reg(((uint8_t)(DTOF_REG_PTCO_H0 + i)), &fifo_val[i]);
    }

    for (int i = 3; i < 13; i++)
    {
        read_reg(((uint8_t)(DTOF_REG_PTCO_H3 + i - 3)), &fifo_val[i]);
    }
    
    printf("fifo src:");
    for (int i = 0; i < 13; i++) {
        printf("%d,", fifo_val[i]);
    }
    printf("\r\n");

    out_buf[NOISE0_FIFO_OUT_BUF_INDEX] = fifo_val[0] & 0x0fff;
    out_buf[NOISE1_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[1] & 0x00ff) << 4 | (fifo_val[0] & 0xf000) >> 12));
    out_buf[QLF_FIFO_OUT_BUF_INDEX] = (fifo_val[1] >> 9) & 0x003f;
    out_buf[FLASHN_FIFO_OUT_BUF_INDEX] = ((uint16_t)((fifo_val[3] & 0x0003) << 17 |
                                                     fifo_val[2] << 1 |
                                                     (fifo_val[1] & 0x8000) >> 15));
    out_buf[MP0_FIFO_OUT_BUF_INDEX] = (fifo_val[3] >> 2) & 0x01ff;
    out_buf[MV0_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[4] & 0x07ff) << 5 | (fifo_val[3] & 0xf800) >> 11));
    out_buf[SP0_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[5] & 0x000f) << 5 | (fifo_val[4] & 0xf800) >> 11));
    out_buf[SV0_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[6] & 0x000f) << 12 | (fifo_val[5] & 0xfff0) >> 4));
    out_buf[TP0_FIFO_OUT_BUF_INDEX] = (fifo_val[6] >> 4) & 0x01ff;
    out_buf[TV0_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[7] & 0x1fff) << 3 | (fifo_val[6] & 0xe000) >> 13));
    out_buf[MP1_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[8] & 0x003f) << 3 | (fifo_val[7] & 0xe000) >> 13));
    out_buf[MV1_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[9] & 0x003f) << 10 | (fifo_val[8] & 0xffc0) >> 6));
    out_buf[SP1_FIFO_OUT_BUF_INDEX] = (fifo_val[9] >> 6) & 0x01ff;
    out_buf[SV1_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[10] & 0x7fff) << 1 | (fifo_val[9] & 0x8000) >> 15));
    out_buf[TP1_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[11] & 0x00ff) << 1 | (fifo_val[10] & 0x8000) >> 15));
    out_buf[TV1_FIFO_OUT_BUF_INDEX] =
        ((uint16_t)((fifo_val[12] & 0x00ff) << 8 | (fifo_val[11] & 0xff00) >> 8));

    return 0;
}

int read_ram(uint16_t* out_buf, int offset, int len) {
    uint16_t val;
    int i;
    if (!out_buf) {
        return -1;
    }

    write_reg(DTOF_REG_RAM_STARTA, ((uint16_t)offset));

    for (i = 0; i < len; i++) {
        read_reg(DTOF_REG_RAM_DATA, &val);
        out_buf[i] = val;
    }
    return 1;
}



void read_otp(uint16_t* out_buf, int offset, int len) {
    uint16_t val;
    if (!out_buf || offset >= 512 || len > 512 ||
        (offset + len) > 512) {
        printf("read otp info:param error!!!offset=%d len=%d\r\n", offset,
                len);
    }

    write_reg(DTOF_REG_OTP_CFG1, 0x0010);

		write_reg(DTOF_REG_RAM_STARTA, ((uint16_t)(0x0800 + offset)));

    for (int i = 0; i < len; i++) {
        read_reg(DTOF_REG_RAM_DATA, &val);
        out_buf[i] = val & 0xFF;
    }
    
    write_reg(DTOF_REG_OTP_CFG1, 0x00);
}

void otp_mode_init() {
    uint16_t reg_value;
    int i;
    
    for (i = 0; i < OTP_LEN; i++) {
        read_otp(&reg_value, i, 1);
        otp_data[i] = (uint8_t)(reg_value & 0x00FF);
    }
}

int update_otp_param_to_reg(uint16_t chip_id) {
    int ret = 0;
    printf("-----------------update otp data-----------------\r\n");

    if ((otpCheckSum.otp_check_sum_flag &&
         otpCheckSum.otp_rtemp_check_sum_result) ||
        (!otpCheckSum.otp_check_sum_flag)) {
        ret = update_spad_mask_otp_to_reg(otp_data);
        if (ret) {
            printf("[error]update spad_mask_otp to reg failed.\n");
            return ret;
        }
        ret = update_trim1_otp_to_reg(otp_data);
        if (ret) {
            printf("[error]update trim1_otp to reg failed.\n");
            return ret;
        }
    } else {
        printf(
            "[error]otp_rtemp_check_sum_result: false. "
            "update_spad_mask_otp_to_reg and update_trim1_otp_to_reg "
            "failed.\n");
    }

    if ((otpCheckSum.otp_check_sum_flag &&
         otpCheckSum.otp_htemp_check_sum_result) ||
        (!otpCheckSum.otp_check_sum_flag)) {
        ret = update_trim_osc_tcsel_otp_to_reg(otp_data, chip_id);
        if (ret) {
            printf("[error]update trim_osc_tcsel_otp to reg failed.\n");
            return ret;
        }
        ret = update_trim2_otp_to_reg(otp_data, chip_id);
        if (ret) {
            printf("[error]update trim2_otp to reg failed.\n");
            return ret;
        }
    } else {
        printf(
            "[error]otp_htemp_check_sum_result: false. "
            "update_trim_osc_tcsel_otp_to_reg and update_trim2_otp_to_reg "
            "failed.\n");
    }

    if ((otpCheckSum.otp_check_sum_flag &&
         otpCheckSum.otp_bank_check_sum_1st_result) ||
        (!otpCheckSum.otp_check_sum_flag)) {
        if (CUPCAKE_MODULE == DTOF_MODULE) {
            ret = update_bin_offset_otp_to_reg(otp_data);
            if (ret) {
                printf("[error]update_bin_offset_otp_to_reg failed.\n");
                return ret;
            }
        }

        ret = update_spad_msk_ft_otp_to_reg(otp_data);
        if (ret) {
            printf("[error]update spad_msk_ft_otp to reg failed.\n");
            return ret;
        }
        ret = update_osc_trim_offset_value_otp(otp_data);
        if (ret) {
            printf("update osc trim offset value failed.\n");
            return ret;
        }
        ret = update_ft_result_otp_to_reg(otp_data);
        if (ret) {
            printf("update_ft_result_otp_to_reg failed.\n");
            return ret;
        }
        ret = update_osc_freqvstemp_otp_to_reg(otp_data, chip_id);
        if (ret) {
            printf("update osc_freqvstemp_otp to reg failed.\n");
            return ret;
        }
        ret = update_osc_freqvstccode_otp_to_reg(otp_data, chip_id);
        if (ret) {
            printf("update osc_freqvstccode_otp to reg failed.\n");
            return ret;
        }
        ret = update_bank_check_sum_1st_otp_to_reg(otp_data);
        if (ret) {
            printf("update_bank_check_sum_1st_otp_to_reg failed.\n");
            return ret;
        }
        ret = update_package_crosstalk_data_otp_to_reg(otp_data);
        if (ret) {
            printf("update package_crosstalk_data_otp to reg failed.\n");
            return ret;
        }
    } else {
        printf("otp_bank_check_sum_1st_result: false. update ... failed.\n");
    }
    return ret;
}

int update_spad_mask_otp_to_reg(const uint8_t* otp_data) {
    int ret = 0;
    uint16_t spadmask;
    int j;
    uint16_t otpValue[SPAD_MASK_OTP_LEN];

    ret = read_otp_from_ram(otp_data, otpValue, SPAD_MASK_OTP_ADDR,
                            SPAD_MASK_OTP_LEN);
    if (ret) {
        printf("read spad mask otp failed,ret=%d\n", ret);
        return ret;
    }

    for (j = 0; j < (SPAD_MASK_OTP_LEN / 2 - 2); j++) {
        spadmask = ((uint16_t)(otpValue[j * 2] | (otpValue[j * 2 + 1] << 8)));
        printf("spad%d: 0x%04x\n", j, spadmask);
        write_reg(((uint8_t)(DTOF_REG_SPD_MSK0 + j)), spadmask);
 
        if (!reg_readback(DTOF_REG_SPD_MSK0 + j, 0, spadmask, 0xffff)) {
            ret = -1;
        }
        if (ret) {
            printf("readback reg 0x%02x failed.\n", DTOF_REG_SPD_MSK0 + j);
            return ret;
        } else {
            printf("readback reg 0x%02x successful.\n", DTOF_REG_SPD_MSK0 + j);
        }
    }
    return ret;
}

int read_otp_from_ram(const uint8_t* opt_data, uint16_t* out_buf, int offset,
                      int len) {
    int i = 0;


    if (NULL == out_buf || OTP_LEN < offset || 0 > offset || OTP_LEN < len ||
        0 >= len || OTP_LEN < (offset + len)) {
        printf("read otp from ram info: param error! offset=%d len=%d\r\n",
                offset, len);
        return -1;
    }

    for (i = 0; i < len; i++) {
        out_buf[i] = opt_data[i + offset];
    }
    return 0;
}

int update_trim1_otp_to_reg(const uint8_t* otp_data) {
    int ret = 0;
    uint16_t otpValue[TRIM1_OTP_LEN];

    ret = read_otp_from_ram(otp_data, otpValue, TRIM1_OTP_ADDR, TRIM1_OTP_LEN);
    if (ret) {
        printf("read trim1 otp failed,ret=%d\n", ret);
        return ret;
    }

    if (otpValue[0] != 0xff) {
        ret = modify_bit(DTOF_REG_BGR_VTRIM, 0x1f, (otpValue[0] & 0x1f));
        if (ret) {
            printf("write reg 0x%02x failed.\n", DTOF_REG_BGR_VTRIM);
            return ret;
        }
        if (!reg_readback(DTOF_REG_BGR_VTRIM, 0, otpValue[0], 0x1f)) {
            ret = -1;
        }
        if (ret) {
            printf("readback reg 0x%02x failed.\n", DTOF_REG_BGR_VTRIM);
            return ret;
        } else {
            printf("readback reg 0x%02x successful.\n", DTOF_REG_BGR_VTRIM);
        }
    }
    if (otpValue[1] != 0xff) {
        ret = modify_bit(DTOF_REG_BGR_VTRIM, 0x3e0,
                         ((uint16_t)((otpValue[1] & 0x1f) << 5)));
        if (ret) {
            printf("write reg 0x%02x failed.\n", DTOF_REG_BGR_VTRIM);
            return ret;
        }
        if (!reg_readback(DTOF_REG_BGR_VTRIM, 5, otpValue[1], 0x1f)) {
            ret = -1;
        }
        if (ret) {
            printf("readback reg 0x%02x failed.\n", DTOF_REG_BGR_VTRIM);
            return ret;
        } else {
            printf("readback reg 0x%02x successful.\n", DTOF_REG_BGR_VTRIM);
        }
    }

    if (otpValue[2] != 0xff) {
        ret = modify_bit(DTOF_REG_OSC_TRIM, 0xff, (otpValue[2] & 0xff));
        if (ret) {
            printf("write reg 0x%02x failed.\n", DTOF_REG_OSC_TRIM);
            return ret;
        }

        if (!reg_readback(DTOF_REG_OSC_TRIM, 0, otpValue[2], 0xff)) {
            ret = -1;
        }
        if (ret) {
            printf("readback reg 0x%02x failed.\n", DTOF_REG_OSC_TRIM);
            return ret;
        } else {
            printf("readback reg 0x%02x successful.\n", DTOF_REG_OSC_TRIM);
        }
    }

    if (otpValue[3] != 0xff) {
        otpValue[3] = (otpValue[3] & 0x1F) + 10;
        if (otpValue[3] > 31) {
            otpValue[3] = 31;
        }
        ret = modify_bit(DTOF_REG_OSC_CAL_CTRL, 0x1f, (otpValue[3] & 0x1f));
        if (ret) {
            printf("write reg 0x%02x failed.\n", DTOF_REG_OSC_CAL_CTRL);
            return ret;
        }

        if (!reg_readback(DTOF_REG_OSC_CAL_CTRL, 0, otpValue[3], 0x1f)) {
            ret = -1;
        }
        if (ret) {
            printf("readback reg 0x%02x failed.\n", DTOF_REG_OSC_CAL_CTRL);
            return ret;
        } else {
            printf("readback reg 0x%02x successful.\n", DTOF_REG_OSC_CAL_CTRL);
        }
    }

    if (otpValue[4] != 0xff) {
        ret = modify_bit(DTOF_REG_OSC_CAL_CTRL, 0x1e00,
                         ((uint16_t)((otpValue[4] & 0xf) << 9)));
        if (ret) {
            printf("write reg 0x%02x failed.\n", DTOF_REG_OSC_CAL_CTRL);
            return ret;
        }
        if (!reg_readback(DTOF_REG_OSC_CAL_CTRL, 9, otpValue[4], 0xf)) {
            ret = -1;
        }
        if (ret) {
            printf("readback reg 0x%02x failed.\n", DTOF_REG_OSC_CAL_CTRL);
            return ret;
        } else {
            printf("readback reg 0x%02x successful.\n", DTOF_REG_OSC_CAL_CTRL);
        }
    }
    return ret;
}

int update_trim_osc_tcsel_otp_to_reg(const uint8_t* otp_data,
                                            uint16_t chip_id) {
    int ret = 0;
    uint16_t otpValue[TRIM_OSC_TCSEL_OTP_LEN];

    printf("chip_id 0x%04x\n", chip_id);
    if (chip_id != ECO2) {
        return ret;
    }

    ret = read_otp_from_ram(otp_data, otpValue, TRIM_OSC_TCSEL_OTP_ADDR,
                            TRIM_OSC_TCSEL_OTP_LEN);
    if (ret) {
        printf("read trim osc tcsel otp failed,ret=%d\n", ret);
        return ret;
    }

    if (otpValue[0] != 0xff) {
        ret = modify_bit(DTOF_REG_OSC_TRIM, 0x700,
                         ((uint16_t)((otpValue[0] & 0x7) << 8)));
        if (ret) {
            printf("write reg 0x%02x failed.\n", DTOF_REG_OSC_TRIM);
            return ret;
        }
        if (!reg_readback(DTOF_REG_OSC_TRIM, 8, otpValue[0], 0x7)) {
            ret = -1;
        }
        if (ret) {
            printf("readback reg 0x%02x failed.\n", DTOF_REG_OSC_TRIM);
            return ret;
        } else {
            printf("readback reg 0x%02x successful.\n", DTOF_REG_OSC_TRIM);
        }
    }
    return ret;
}

int update_trim2_otp_to_reg(const uint8_t* otp_data, uint16_t chip_id) {
    int ret = 0;
    uint16_t otpValue[TRIM2_OTP_LEN];

    ret = read_otp_from_ram(otp_data, otpValue, TRIM2_OTP_ADDR, TRIM2_OTP_LEN);
    if (ret) {
        printf("read trim2 otp failed,ret=%d\n", ret);
        return ret;
    }

    uint16_t hvpp_tc_trim = 0x2;
    if (chip_id == ECO2) {
        hvpp_tc_trim = otpValue[0];
    }
    ret = modify_bit(DTOF_REG_OSC_CAL_CTRL, 0x1e0,
                     ((uint16_t)(hvpp_tc_trim << 5)));
    if (ret) {
        printf("write reg 0x%02x failed\n", DTOF_REG_OSC_CAL_CTRL);
        return ret;
    }
    // }

    if (!reg_readback(DTOF_REG_OSC_CAL_CTRL, 5, ((uint16_t)(hvpp_tc_trim)),
                      0xF)) {
        ret = -1;
    }
    if (ret) {
        printf("readback reg 0x%02x failed.\n", DTOF_REG_OSC_CAL_CTRL);
        return ret;
    } else {
        printf("readback reg 0x%02x successful.\n", DTOF_REG_OSC_CAL_CTRL);
    }

    ret = modify_bit(DTOF_REG_BGR_VTRIM, 0xfc00, (0x18 << 10));
    if (ret) {
        printf("write reg 0x%02x failed\n", DTOF_REG_BGR_VTRIM);
        return ret;
    }
    if (!reg_readback(DTOF_REG_BGR_VTRIM, 10, ((uint16_t)(0x18)), 0x3f)) {
        ret = -1;
    }
    if (ret) {
        printf("readback reg 0x%02x failed.\n", DTOF_REG_BGR_VTRIM);
        return ret;
    } else {
        printf("readback reg 0x%02x successful.\n", DTOF_REG_BGR_VTRIM);
    }
    return ret;
}

int update_bin_offset_otp_to_reg(const uint8_t* otp_data) {
    int ret = 0;
    int i = 0;
    uint16_t otpValue[BIN_OFFSET_OTP_LEN];

    ret = read_otp_from_ram(otp_data, otpValue, BIN_OFFSET_OTP_ADDR,
                            BIN_OFFSET_OTP_LEN);
    if (ret) {
        printf("read bin offset otp failed,ret=%d\n", ret);
        return ret;
    }

    if (0xff != otpValue[0]) {
        for (i = DTOF_REG_OFFSET_PIXEL0; i <= DTOF_REG_OFFSET_PIXEL15; i++) {
            ret = modify_bit(i, 0xff, ((uint16_t)(otpValue[0] & 0xff)));
            if (ret) {
                printf("write reg 0x%02x failed\n", i);
                return ret;
            }
            if (!reg_readback(i, 0, ((uint16_t)(otpValue[0] & 0xff)), 0xff)) {
                ret = -1;
            }
            if (ret) {
                printf("readback reg 0x%02x failed.\n", i);
                return ret;
            } else {
                printf("readback reg 0x%02x successful.\n", i);
            }
        }
    }
    return ret;
}

int update_spad_msk_ft_otp_to_reg(const uint8_t* otp_data) {
    int ret = 0;
    int i = 0;
    uint16_t otpValue[SPAD_MSK_FT_OTP_LEN];
    uint16_t ref_spad_msk = 0;

    ret = read_otp_from_ram(otp_data, otpValue, SPAD_MSK_FT_OTP_ADDR,
                            SPAD_MSK_FT_OTP_LEN);
    if (ret) {
        printf("read spad_msk_ft otp failed,ret=%d\n", ret);
        return ret;
    }
    for (i = 0; i < 2; i++) {
        if (0xFF != otpValue[i * 2] || 0xFF != otpValue[i * 2 + 1]) {
            ref_spad_msk = otpValue[i * 2] | (otpValue[i * 2 + 1] << 8);
            write_reg(((uint8_t)(DTOF_REG_SPD_MSK8 + i)), ref_spad_msk);

            if (!reg_readback(DTOF_REG_SPD_MSK8 + i, 0, ref_spad_msk, 0xffff)) {
                ret = -1;
            }
            if (ret) {
                printf("readback reg 0x%02x failed.\n", DTOF_REG_SPD_MSK8 + i);
                return ret;
            } else {
                printf("readback reg 0x%02x successful.\n",
                        DTOF_REG_SPD_MSK8 + i);
            }
        }
    }
    return ret;
}

int update_osc_trim_offset_value_otp(const uint8_t* otp_data) {
    int ret = 0;
    uint16_t otpValue[OSCTRIM_OFFSET_OTP_LEN];

    ret = read_otp_from_ram(otp_data, otpValue, OSCTRIM_OFFSET_OTP_ADDR,
                            OSCTRIM_OFFSET_OTP_LEN);
    if (ret) {
        printf("read osc trim offset value otp failed,ret=%d\n", ret);
        return ret;
    }

    // osc trim offset value is defaut value in otp
    if (0xff == otpValue[0] && 0xff == otpValue[1]) {
        s_oscTrim_off = 0;
    } else {
        s_oscTrim_off = (int16_t)((otpValue[1] << 8) | otpValue[0]);
    }

    return 0;
}

int update_ft_result_otp_to_reg(const uint8_t* otp_data) {
    int ret = 0;
    // ft_result
    uint16_t otpValue[FT_RESULT_OTP_LEN];
    ret = read_otp_from_ram(otp_data, otpValue, FT_RESULT_OTP_ADDR,
                            FT_RESULT_OTP_LEN);
    if (ret) {
        printf("read osc ft_result otp failed,ret=%d\n", ret);
        return ret;
    }
    ft_result[SOFTWAREVERSION] = (uint8_t)otpValue[0];
    ft_result[TESTINGTIMEYEAR] = (uint8_t)otpValue[1] & 0X03;
    ft_result[TESTINGWEEK] = (uint8_t)((otpValue[1] >> 2) & 0X3F);
    ft_result[BINNUMBER] = (uint8_t)otpValue[2];
    ft_result[TESTINGTIMEWEEKDAY] = (uint8_t)otpValue[3] & 0X07;
    ft_result[TESTINGTIMEHOURS] = (uint8_t)((otpValue[3] >> 3) & 0X1F);
    ft_result[TESTINGTIMEMINUTES] = (uint8_t)otpValue[4] & 0X1F;
    ft_result[PACKAGEHOUSEVENDORID] = (uint8_t)otpValue[5];
    ft_result[PACKAGEHOUSETESTMACHINEID1] = (uint8_t)otpValue[6];
    ft_result[PACKAGEHOUSETESTMACHINEID2] = (uint8_t)otpValue[7];
    return 0;
}

int update_osc_freqvstemp_otp_to_reg(const uint8_t* otp_data,
                                            uint16_t chip_id) {
    int ret = 0;
    int j = 0;
    uint16_t osc_freqvstemp;
    uint16_t otpValue[OSC_FREQVSTEMP_OTP_LEN];

    if (chip_id == ECO2) {
        return ret;
    }

    ret = read_otp_from_ram(otp_data, otpValue, OSC_FREQVSTEMP_OTP_ADDR,
                            OSC_FREQVSTEMP_OTP_LEN);
    if (ret) {
        printf("read trim2 otp failed,ret=%d\n", ret);
        return ret;
    }
    for (j = 0; j < OSC_FREQVSTEMP_OTP_LEN / 2; j++) {
        osc_freqvstemp =
            ((uint16_t)(otpValue[j * 2] | (otpValue[j * 2 + 1] << 8)));
        printf("osc_freq vs temp%d: 0x%04x\n", j, osc_freqvstemp);
    }
    return ret;
}

int update_osc_freqvstccode_otp_to_reg(const uint8_t* otp_data,
                                              uint16_t chip_id) {
    int ret = 0;
    uint16_t osc_freqvstccode;
    int j = 0;
    uint16_t otpValue[OSC_FREQVSTCCODE_OTP_LEN];

    if (chip_id != ECO2) {
        return ret;
    }

    ret = read_otp_from_ram(otp_data, otpValue, OSC_FREQVSTCCODE_OTP_ADDR,
                            OSC_FREQVSTCCODE_OTP_LEN);
    if (ret) {
        printf("read trim2 otp failed,ret=%d\n", ret);
        return ret;
    }
    for (j = 0; j < OSC_FREQVSTCCODE_OTP_LEN / 2; j++) {
        osc_freqvstccode =
            ((uint16_t)(otpValue[j * 2] | (otpValue[j * 2 + 1] << 8)));
        printf("osc_freq vs tccode%d: 0x%04x\n", j, osc_freqvstccode);
    }
    return ret;
}

int update_bank_check_sum_1st_otp_to_reg(const uint8_t* otp_data) {
    int ret = 0;

    uint16_t otpValue[BANK_CHECK_SUM_1ST_OTP_LEN];

    ret = read_otp_from_ram(otp_data, otpValue, BANK_CHECK_SUM_1ST_OTP_ADDR,
                            BANK_CHECK_SUM_1ST_OTP_LEN);
    if (ret) {
        printf("read bank_check_sum_1st otp failed,ret=%d\n", ret);
        return ret;
    }
    bank_check_sum_1st = (otpValue[1] << 8) | otpValue[0];

    return ret;
}

int update_package_crosstalk_data_otp_to_reg(const uint8_t* otp_data) {
    int j = 0;
    int ret = 0;
    int count = 0;
    uint16_t package_crosstalk_data;
    uint16_t otpValue[PACKAGE_CROSSTALK_DATA_OTP_LEN];

    ret = read_otp_from_ram(otp_data, otpValue, PACKAGE_CROSSTALK_DATA_OTP_ADDR,
                            PACKAGE_CROSSTALK_DATA_OTP_LEN);
    if (ret) {
        printf("read trim2 otp failed,ret=%d\n", ret);
        return ret;
    }
    for (j = 0; j < PACKAGE_CROSSTALK_DATA_OTP_LEN; j++) {
        if (0xff == otpValue[j]) {
            count++;
        }
    }
    if (3 >= count) {
        for (j = 0; j < PACKAGE_CROSSTALK_DATA_OTP_LEN; j++) {
            package_crosstalk_data = (uint16_t)otpValue[j];
            printf("package_crosstalk_data%d: 0x%04x\n", j,
                     package_crosstalk_data);
            write_reg(((uint8_t)(DTOF_REG_NE_XT0 + j)),
                            package_crosstalk_data);

            if (!reg_readback(DTOF_REG_NE_XT0 + j, 0, package_crosstalk_data,
                              0xffff)) {
                ret = -1;
            }
            if (ret) {
                printf("readback failed reg 0x%02x.\n", DTOF_REG_NE_XT0 + j);
                return ret;
            } else {
                printf("readback successful reg 0x%02x .\n",
                        DTOF_REG_NE_XT0 + j);
            }
        }
    }
    return ret;
}

bool reg_readback(uint8_t reg_addr, uint8_t offset, uint16_t otp_val, uint16_t otp_mask) {
    uint16_t reg_val = 0;
    read_reg(reg_addr, &reg_val);
    reg_val = reg_val >> offset;
    reg_val &= otp_mask;
    otp_val &= otp_mask;

    if (reg_val == otp_val) {
        printf("readback successful reg:%02x, reg_val:%04x, otp_val:%04x, offset:%d\n",
            reg_addr, reg_val, otp_val, offset);
        return true;
    } else {
        printf("readback failed reg:%02x, reg_val:%04x, otp_val:%04x, offset:%d\n",
            reg_addr, reg_val, otp_val, offset);
        return false;
    }
}

void dump_regs() {
    uint16_t val;
    int i,j;
    printf("-----------------dump all regs-----------------\n\r");
    for(j=0; j<16; j++) {
        printf("%02x:", j*16);
        for(i=0; i<16; i++) {
            read_reg(j*16 + i, &val);
            printf("%04x,", val);
        }
        printf("\r\n");
    }
    printf("\r\n");
}

float calculateConfidence(uint16_t noise0, uint16_t mv0) {
    float avrg0, peak_count0, fSNR = 0.0f;

    avrg0 = (float)noise0;
    peak_count0 = (float)(mv0 + 1);
    fSNR = (peak_count0 - avrg0) / (((sqrtf(avrg0)) > (0.3f)) ? (sqrtf(avrg0)) : (0.3f));

    fSNR = 10 * log10f(fSNR);

    return fSNR;
}

float calculateDistance(uint16_t peak, uint16_t ref_index) {
    return ((peak - ref_index) * s_lib_param.bin_width + s_lib_param.offset);
}

int modify_bit(uint8_t addr, uint16_t mask, uint16_t val) {
    uint16_t temp = 0;
    read_reg(addr, &temp);

    temp &= ~mask;
    temp |= val & mask;
    write_reg(addr, temp);

    return 0;
}

uint32_t value_round_up(uint32_t value, uint32_t multiple) {
    if (!value) {
        value = 1;
    } else {
        if (!multiple) {
            value = 1;
        } else {
            // round up
            if (0 < (value % multiple)) {
                value = (value / multiple) + 1;
            } else {
                value = value / multiple;
            }
        }
    }
    // printf(" (%d|%d) ", value, multiple);
    return value;
}

int write_reg(uint8_t reg_addr, uint16_t reg_val)
{
	return I2Cdev::writeWord(I2C_CHIP_ADDR, reg_addr, reg_val);
}

int read_reg(uint8_t reg_addr, uint16_t *reg_val)
{
	I2Cdev::readWord(I2C_CHIP_ADDR, reg_addr, reg_val);
	*reg_val = swap16bit_big_little(*reg_val);
	return 1;
}