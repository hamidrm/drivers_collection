/*
 * aic3212_driver.c
 *
 *  Created on: Jan 14, 2019
 *      Author: mehrabian
 */


#include <msfw.h>
#include LAYER(hld)
#include "../inc/aic3212_driver.h"

/***********************************************************************/
/*              LOCAL VARIABLES                                        */
/***********************************************************************/
static const float aic3212_mixer_amp_gain_lut[AIC_MIXER_AMP_GAIN_LUT_CNT] = {0.0,-0.4,-0.9,-1.3,-1.8,-2.3,
                                                -2.9,-3.3,-3.9,-4.3,-4.8,-5.2,
                                                -5.8,-6.3,-6.6,-7.2,-7.8,-8.2,
                                                -8.5,-9.3,-9.7,-10.1,-10.6,-11.0,
                                                -11.5,-12.0,-12.6,-13.2,-13.8,-14.5,
                                                -15.3,-16.1,-17.0,-18.1,-19.2,-20.6,
                                                -22.1,-24.1,-26.6,-30.1,-36.1};

static const float aic3212_lo_hp_gain_lut[AIC_LO_HP_GAIN_LUT_CNT] = { 0.00,-0.5,-1.0,-1.5,-2.0,-2.5,-3.0,-3.5,-4.0,-4.5,-5.0,
                                                -5.5,-6.0,-6.5,-7.0,-7.5,-8.0,-8.5,-9.0,-9.5,-10.0,-10.5,
                                                -11.0,-11.5,-12.0,-12.5,-13.0,-13.5,-14.1,-14.6,-15.1,-15.6,
                                                -16.0,-16.5,-17.1,-17.5,-18.1,-18.6,-19.1,-19.6,-20.1,-20.6,
                                                -21.1,-21.6,-22.1,-22.6,-23.1,-23.6,-24.1,-24.6,-25.1,-25.6,
                                                -26.1,-26.6,-27.1,-27.6,-28.1,-28.6,-29.1,-29.6,-30.1,-30.6,
                                                -31.1,-31.6,-32.1,-32.7,-33.1,-33.6,-34.1,-34.6,-35.2,-35.7,
                                                -36.1,-36.7,-37.1,-37.7,-38.2,-38.7,-39.2,-39.7,-40.2,-40.7,
                                                -41.2,-41.8,-42.1,-42.7,-43.2,-43.8,-44.3,-44.8,-45.2,-45.8,
                                                -46.2,-46.7,-47.4,-47.9,-48.2,-48.7,-49.3,-50.0,-50.3,-51.0,
                                                -51.4,-51.8,-52.3,-52.7,-53.7,-54.2,-55.4,-56.7,-58.3,-60.2,
                                                -62.7,-64.3,-66.2,-68.7,-72.3,-78.3};

/***********************************************************************/
/*              LOCAL FUNCTION PROTOTYPES.                            */
/***********************************************************************/
static  uint32_t    aic3212_spi_init(aic3212_dev_t * dev);
static  uint32_t    aic3212_spi_transceiver(aic3212_dev_t * dev, uint8_t * tx_buff, uint8_t * rx_buff, size_t len);
static  uint32_t    aic3212_spi_remove(aic3212_dev_t * dev);
static  void   aic3212_spi_cb(spi_instance ins, uint8_t * buffer_tx, uint8_t * buffer_rx, size_t len, void * arg);
static void aic3212_tx_completed(aic3212_instance dev_ins);
static void aic3212_rx_completed(aic3212_instance dev_ins);

/***********************************************************************/
/*              LOCAL FUNCTION DEFINITIONS.                            */
/***********************************************************************/
static  void   aic3212_spi_cb(spi_instance ins, uint8_t * buffer_tx, uint8_t * buffer_rx, size_t len, void * arg){
    Semaphore_post(arg);
}

static  uint32_t    aic3212_spi_init(aic3212_dev_t * dev){
    aic3212_ins_t * aic3212 = (aic3212_ins_t *)dev->ins;
    aic3212_software_reset(aic3212);
    aic3212_config_power(aic3212, AIC_ANALOG_ON, AIC_DVDD_AVDD_CONNECTION_DISABLE,
                     AIC_MIC_QC_R1_1_L0_5, AIC_VREF_FC_10_30_MS, AIC_CHIP_REF_AUTO,
                     AIC_FCT_POW_MODE);
    return 0;

}

static  uint32_t    aic3212_spi_transceiver(aic3212_dev_t * dev, uint8_t * tx_buff, uint8_t * rx_buff, size_t len){
    aic3212_ins_t * aic3212 = (aic3212_ins_t *)dev->ins;
    spi_transceiver_sub_channel_start(aic3212->spi_instance, aic3212->spi_channel_id, tx_buff, rx_buff, len);
    Semaphore_pend(aic3212->sem, BIOS_WAIT_FOREVER);
    return 0;
}

static  uint32_t    aic3212_spi_remove(aic3212_dev_t * dev){
    return 0;
}

static void aic3212_rx_completed(aic3212_instance dev_ins){
    aic3212_ins_t * dev = (aic3212_ins_t *)dev_ins;

    if(dev->aic3212_rx_done_fn)
        dev->aic3212_rx_done_fn(dev->rx_done_arg, MCASP_CURRENT_RX_BUFFER(dev->mcasp_instance), ((mcasp_ins_t *)(dev->mcasp_instance))->mcasp_config_rx.samples_number);
}

static void aic3212_tx_completed(aic3212_instance dev_ins){
    aic3212_ins_t * dev = (aic3212_ins_t *)dev_ins;

    if(dev->aic3212_tx_done_fn)
        dev->aic3212_tx_done_fn(dev->tx_done_arg, MCASP_CURRENT_TX_BUFFER(dev->mcasp_instance), ((mcasp_ins_t *)(dev->mcasp_instance))->mcasp_config_tx.samples_number);
}

static uint8_t aic3212_get_vol_index(const float * lut,uint8_t num,uint8_t expected_percent){
    uint8_t vol_index = 0;
    float max = lut[0];
    float min = lut[num-1];
    float expected_vol = expected_percent;
    expected_vol = min + (expected_vol/100.0f) * (max - min);

    while(lut[vol_index] > expected_vol && vol_index<num)
        vol_index++;

    return vol_index;
}

/***********************************************************************/
/*              API FUNCTION DEFINITIONS.                              */
/***********************************************************************/

/**
 * \brief    This function creates an AD9864 Instance.
 *
 * \param    spi    Instance of SPI
 * \param    spi_sub_channel_id Sub-channel id of the AD5689 (leave it zero in the cases that CS does not multiplexed)
 * \param    sem    A semaphore handle for internal usage
 *
 *
 * \return   ad5689_instance    Created instance for specified AD5689.
 *
 */
aic3212_instance aic3212_create(spi_instance spi, mcasp_instance mcasp, uint8_t spi_sub_channel_id, Semaphore_Handle spi_sem,
                                aic3212_rx_done_cb rx_done, void *rx_done_arg, aic3212_tx_done_cb tx_done, void *tx_done_arg
                                , void *rx_buff_audio_ping, void *rx_buff_audio_pong
                                , void *tx_buff_audio_ping, void *tx_buff_audio_pong){
    aic3212_instance aic3212_ins;
    struct aic3212_init_param param;
    aic3212_ins_t * aic3212;

    aic3212_ins = malloc(sizeof(aic3212_ins_t));
    aic3212 = (aic3212_ins_t *)aic3212_ins;

    spi_transceiver_assign_sub_channel(spi, spi_sub_channel_id, aic3212_spi_cb, 0, spi_sem);
    aic3212->spi_instance = spi;
    aic3212->sem = spi_sem;
    aic3212->mcasp_instance = mcasp;
    aic3212->spi_channel_id = spi_sub_channel_id;
    param.ins = aic3212_ins;

    param.aic3212_spi_init = aic3212_spi_init;
    param.aic3212_spi_remove = aic3212_spi_remove;
    param.aic3212_spi_transceiver = aic3212_spi_transceiver;

    aic3212->aic3212_rx_done_fn = rx_done;
    aic3212->rx_done_arg = rx_done_arg;

    aic3212->aic3212_tx_done_fn = tx_done;
    aic3212->tx_done_arg = tx_done_arg;


    mcasp_mgr_set_rx_callback(mcasp, aic3212_rx_completed, aic3212);
    mcasp_mgr_set_tx_callback(mcasp, aic3212_tx_completed, aic3212);
    mcasp_mgr_configure_rx(mcasp, rx_buff_audio_ping, rx_buff_audio_pong);
    mcasp_mgr_configure_tx(mcasp, tx_buff_audio_ping, tx_buff_audio_pong);
    if(aic3212_setup(&aic3212->dev, &param) == 0){

        return aic3212_ins;
    }else{
        free(aic3212_ins);
        return NULL;
    }
}

void aic3212_start(aic3212_instance aic3212){
    mcasp_mgr_start(((aic3212_ins_t *)aic3212)->mcasp_instance, 0);
}

void aic3212_set_processing_block(aic3212_instance aic3212, uint8_t adc_pb,uint8_t dac_pb){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_DAC_PROCESSING_BLOCK_CONTROL,dac_pb & 0x1F);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_PROCESSING_BLOCK_CONTROL,adc_pb & 0x1F);
}

void aic3212_adc_power_tune(aic3212_instance aic3212, AicAdcPTM ptm,AicAdcInput l_input,AicAdcInput r_input){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_POWERTUNE_CONFIGURATION_REGISTER,((ptm & 3) << 6) | ((l_input & 1) << 3) | ((r_input & 1) << 2));
}

void aic3212_dac_power_tune(aic3212_instance aic3212, AicDacPTM left_ptm,AicDacPTM right_ptm){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_DAC_POWERTUNE_CONFIGURATION_REGISTER,((left_ptm & 7) << 2));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_DAC_POWERTUNE_CONFIGURATION_REGISTER,((right_ptm & 7) << 2));
}

void aic3212_config_power(aic3212_instance aic3212, AicPowConfigAnalog pow_analog,AicPowConfigDvddAvddConnection d_a_conn,
                      AicMicPgaPower mic_pow,AicVrefFastCharge vref_fc,AicChipRefPow chip_ref,
                      AicFineCoarsePowMode f_c_pm){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_POWER_CONFIGURATION_REGISTER,pow_analog | d_a_conn);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_REFERENCE_POWERUP_DELAY_REGISTER,vref_fc | chip_ref | f_c_pm);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_MICROPHONE_PGA_POWER_UP_CONTROL_REGISTER,mic_pow);
    AIC3212_SLEEP(100);
}

void aic3212_software_reset(aic3212_instance aic3212){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SOFTWARE_RESET_REGISTER,AIC3212_RESET);
    AIC3212_SLEEP(10);
}

int aic3212_power_on_pll(aic3212_instance aic3212, AicPllInput pll_in,uint32_t freq_in,uint32_t freq_out){
    uint32_t pll_div=1;
    uint32_t pll_p=1;
    uint32_t pll_j=1;
    uint32_t pll_r=1;
    uint32_t pll_d=0;
    uint32_t pll_jr = 1;
    uint32_t pll_pdiv = 1;
    uint32_t t = 1,i,j;
    uint32_t t_in,t_out;
    uint8_t reg_val;
    float d_f;
    uint32_t org_freq_in = freq_in;
    uint32_t org_pll_jr = 1;
    uint32_t org_pll_pdiv = 1;

    if((freq_out % freq_in) == 0){
        pll_jr = freq_out / freq_in;
    }else if((freq_in % freq_out) == 0){
        pll_pdiv = freq_in / freq_out;
    }else{
        t_in=freq_in;
        t_out=freq_out;
        for(i=2;i<(t_in/2);i++)
            while(((t_in % i) == 0) && ((t_out % i) == 0)){
                t *= i;
                t_in /= i;
                t_out /= i;
            }
        freq_in /= t;
        freq_out /= t;

        if(freq_in < (128 * 8))
            pll_pdiv = freq_in;
        else
            return AIC_ERROR;

        if(freq_out < (16 * 64))
            pll_jr = freq_out;
        else
            return AIC_ERROR;
    }
    i=8;
    j=16;
    org_pll_jr = pll_jr;
    org_pll_pdiv = pll_pdiv;
    do{
        pll_jr = org_pll_jr;
        pll_pdiv = org_pll_pdiv;
        if(pll_pdiv > 128){
            for(;i>1;i--)
                if(((pll_pdiv % i) == 0) && ((pll_pdiv / i) < 128)){
                    pll_div = pll_pdiv / i;
                    pll_p = i;
                    break;
                }
            if(i==1)
                return AIC_ERROR;
        }else
            pll_div = pll_pdiv;

        if(pll_jr > 64){
            for(;j>1;j--)
                if(((pll_jr % j) == 0) && ((pll_jr / j) < 64)){
                    pll_j = pll_jr / j;
                    pll_r = j;
                    break;
                }
            if(j==1)
                return AIC_ERROR;
        }else
            pll_j = pll_jr;

        d_f = (((float)(freq_out) * ((float)(pll_div * pll_p))) / (((float)(pll_r)) * freq_in)) - pll_j;

        pll_d = (uint32_t)(d_f * 10000.0f);

    }while(((org_freq_in / (pll_p*pll_div)) >= (20 MHz)) || ((org_freq_in / (pll_p*pll_div)) <= ((pll_d == 0) ? 512 KHz : 10 MHz)));


    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_2_PLL_INPUT_MULTIPLEXER,pll_in);

    reg_val = AIC3212_PLL_POW_ON | ((pll_p<<AIC3212_PLL_P_SHIFT)&AIC3212_PLL_P_MASK) |
            ((pll_r<<AIC3212_PLL_R_SHIFT)&AIC3212_PLL_R_MASK);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_3_PLL_P_AND_R_VALUES,reg_val);

    reg_val = AIC3212_PLL_J_BASE_VAL | ((pll_j<<AIC3212_PLL_J_SHIFT)&AIC3212_PLL_J_MASK);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_4_PLL_J_VALUE,reg_val);

    reg_val = AIC3212_PLL_D_MSB_BASE_VAL | (((pll_d>>8)<<AIC3212_PLL_D_MSB_SHIFT)&AIC3212_PLL_D_MSB_MASK);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_5_PLL_D_VALUES_MSB,reg_val);

    reg_val = (((pll_d & 0xFF)<<AIC3212_PLL_D_LSB_SHIFT)&AIC3212_PLL_D_LSB_MASK);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_6_PLL_D_VALUES_LSB,reg_val);

    reg_val = AIC3212_PLL_DIV_BASE_VAL | ((pll_div<<AIC3212_PLL_DIV_SHIFT)&AIC3212_PLL_DIV_MASK);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_7_PLL_CLKIN_DIVIDER,reg_val);

    return AIC_OK;
}

void aic3212_config_fs_clk(aic3212_instance aic3212, AicClkInput adc_clk_src,AicClkInput dac_clk_src,
                       uint8_t ndac,uint8_t mdac,uint16_t dac_osr,
                       uint8_t nadc,uint8_t madc,uint8_t adc_osr){

    uint8_t reg_val;

    reg_val = ((adc_clk_src & 3)<<AIC3212_CLK_IN_MUX_ADC_SHIFT) |
              ((dac_clk_src & 3)<<AIC3212_CLK_IN_MUX_DAC_SHIFT);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_1_CLOCK_INPUT_MULTIPLEXERS,reg_val);

    reg_val = (ndac?AIC3212_NDAC_ON:0) | (((ndac < 128 ? ndac : 0) & AIC_NDAC_VAL_MASK)<<AIC_NDAC_VAL_SHIFT);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_8_NDAC_DIVIDER_VALUES,reg_val);

    reg_val = (mdac?AIC3212_MDAC_ON:0) | (((mdac < 128 ? mdac : 0) & AIC_MDAC_VAL_MASK)<<AIC_MDAC_VAL_SHIFT);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_9_MDAC_DIVIDER_VALUES,reg_val);

    dac_osr = (dac_osr < 1024 ? dac_osr : 0);

    reg_val = AIC3212_DAC_OSR_MSB_BASE_VAL | (((dac_osr>>8) & AIC_DAC_OSR_MSB_MASK)<<AIC_DAC_OSR_MSB_SHIFT);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_DAC_OSR_CONTROL_REGISTER_1_MSB_VALUE,reg_val);

    reg_val = (dac_osr & 0xFF) & AIC_DAC_OSR_LSB_MASK;
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_DAC_OSR_CONTROL_REGISTER_2_LSB_VALUE,reg_val);



    reg_val = (nadc?AIC3212_NADC_ON:0) | (((nadc < 128 ? nadc : 0) & AIC_NADC_VAL_MASK)<<AIC_NADC_VAL_SHIFT);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_10_NADC_VALUES,reg_val);

    reg_val = (madc?AIC3212_MADC_ON:0) | (((madc < 128 ? madc : 0) & AIC_MADC_VAL_MASK)<<AIC_MADC_VAL_SHIFT);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_11_MADC_VALUES,reg_val);

    reg_val = (((adc_osr < 256 ? adc_osr : 0) & AIC_ADC_OSR_MASK)<<AIC_ADC_OSR_SHIFT);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_OVERSAMPLING_AOSR_REGISTER,reg_val);
}


void aic3212_config_clkout(aic3212_instance aic3212, AicClkOutInput clkout_mux,uint8_t divder){
    uint8_t reg_val;
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLKOUT_MUX,clkout_mux);

    reg_val = (divder?AIC3212_CLKOUT_DIVIDER_ON:0) | ((divder<128 ? divder : 0) & AIC3212_CLKOUT_DIVIDER_VAL_MASK);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_CLOCK_CONTROL_REGISTER_12_CLKOUT_M_DIVIDER_VALUE,reg_val);




    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_GPO1_PIN_CONTROL,6);
}

void aic3212_config_sar(aic3212_instance aic3212, aic3212_sar_adc_config_t * config){

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_SAR_ADC_CONTROL,(config->resulation << 5) | (config->clk_divider << 3) | (config->filter << 2) | (config->avg));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_SAR_ADC_CONVERSION_MODE,(config->conv_mode << 2) | (config->interrupt_control));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SAR_REFERENCE_CONTROL,(config->internal_ref << 7) | (config->auto_power << 5) | (config->refrence_stabilization_time << 2));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SAR_ADC_BUFFER_MODE_CONTROL,(config->buffer_data << 7) | (config->buffer_mode << 6) | (config->trig_lvl << 3));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SCAN_MODE_TIMER_CONTROL,(config->auto_measurement << 3) | (config->timer_delay));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SAR_ADC_CLOCK_CONTROL,(config->clk_src << 7) | (config->clk_divider));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SAR_ADC_BUFFER_MODE_DATA_READ_CONTROL,config->update_mode << 5);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SAR_ADC_MEASUREMENT_CONTROL,config->adc_measurement_control);

    config->in1l_max &= 0xFFF;
    config->in1l_min &= 0xFFF;
    config->in1r_max &= 0xFFF;
    config->in1r_min &= 0xFFF;
    config->temp_max &= 0xFFF;
    config->temp_max &= 0xFFF;

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1L_MAX_THRESHOLD_CHECK_CONTROL_1,(config->in1l_max >> 4) | (config->in1l_max_threshold << 4));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1L_MIN_THRESHOLD_CHECK_CONTROL_1,(config->in1l_min >> 4) | (config->in1l_min_threshold << 4));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1R_MAX_THRESHOLD_CHECK_CONTROL_1,(config->in1r_max >> 4) | (config->in1r_max_threshold << 4));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1R_MIN_THRESHOLD_CHECK_CONTROL_1,(config->in1r_min >> 4) | (config->in1r_min_threshold << 4));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP_MAX_THRESHOLD_CHECK_CONTROL_1,(config->temp_max >> 4) | (config->temp_max_threshold << 4));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP_MIN_THRESHOLD_CHECK_CONTROL_1,(config->temp_min >> 4) | (config->temp_min_threshold << 4));

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1L_MAX_THRESHOLD_CHECK_CONTROL_2,config->in1l_max & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1L_MIN_THRESHOLD_CHECK_CONTROL_2,config->in1l_min & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1R_MAX_THRESHOLD_CHECK_CONTROL_2,config->in1r_max & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1R_MIN_THRESHOLD_CHECK_CONTROL_2,config->in1r_min & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP_MAX_THRESHOLD_CHECK_CONTROL_2,config->temp_max & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP_MIN_THRESHOLD_CHECK_CONTROL_2,config->temp_min & 0xFF);
}


uint16_t aic3212_get_sar_temp1_value(aic3212_instance aic3212){
    uint16_t val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP1_MEASUREMENT_DATA_MSB);
    val = (val << 8) | aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP1_MEASUREMENT_DATA_LSB);
    return val;
}

uint16_t aic3212_get_sar_temp2_value(aic3212_instance aic3212){
    uint16_t val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP2_MEASUREMENT_DATA_MSB);
    val = (val << 8) | aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_TEMP2_MEASUREMENT_DATA_LSB);
    return val;
}

uint16_t aic3212_get_sar_vbat_value(aic3212_instance aic3212){
    uint16_t val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_VBAT_MEASUREMENT_DATA_MSB);
    val = (val << 8) | aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_VBAT_MEASUREMENT_DATA_LSB);
    return val;
}

uint16_t aic3212_get_sar_in1l_value(aic3212_instance aic3212){
    uint16_t val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1L_MEASUREMENT_DATA_MSB);
    val = (val << 8) | aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1L_MEASUREMENT_DATA_LSB);
    return val;
}

uint16_t aic3212_get_sar_in1r_value(aic3212_instance aic3212){
    uint16_t val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1R_MEASUREMENT_DATA_MSB);
    val = (val << 8) | aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_IN1R_MEASUREMENT_DATA_LSB);
    return val;
}

void aic3212_sar_convert(aic3212_instance aic3212, aic3212_sar_adc_config_t * config){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_SAR_ADC_CONVERSION_MODE,(config->conv_mode << 2) | (config->interrupt_control));
}

void aic3212_config_left_pga(aic3212_instance aic3212, aic3212_left_pga_config_t * config){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_INPUT_SELECT_1_FOR_LEFT_MICROPHONE_PGA_P_TERMINAL,(config->in1l_p << 6) | (config->in2l_p << 4) | (config->in3l_p << 2) | config->in1r_p);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_INPUT_SELECT_2_FOR_LEFT_MICROPHONE_PGA_P_TERMINAL,(config->in4l_p << 5) | (config->in4r_m << 4));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_INPUT_SELECT_FOR_LEFT_MICROPHONE_PGA_M_TERMINAL,(config->cm1_m << 6) | (config->cm2_m) | (config->in2r_m << 4) | (config->in3r_m << 2));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_MICROPHONE_PGA_CONTROL,AIC_PGA_GAIN(config->gain));
}

void aic3212_config_right_pga(aic3212_instance aic3212, aic3212_right_pga_config_t * config){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_INPUT_SELECT_1_FOR_RIGHT_MICROPHONE_PGA_P_TERMINAL,(config->in1r_p << 6) | (config->in2r_p << 4) | (config->in3r_p << 2) |  config->in2l_p);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_INPUT_SELECT_2_FOR_RIGHT_MICROPHONE_PGA_P_TERMINAL,(config->in4l_m << 4) | (config->in4r_p << 5));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_INPUT_SELECT_FOR_RIGHT_MICROPHONE_PGA_M_TERMINAL,(config->cm1_m << 6) | (config->cm2_m) | (config->in1l_m << 4) | (config->in3l_m << 2));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_MICROPHONE_PGA_CONTROL,AIC_PGA_GAIN(config->gain));
}

void aic3212_config_mixer_amp(aic3212_instance aic3212, AicConfigType type,uint8_t power,AicRouteStatus in1_route,float weakening){
    uint8_t gain_index=0;
    uint8_t aic3212_mixer_amp_control_value;
    while(aic3212_mixer_amp_gain_lut[gain_index]>weakening && gain_index<AIC_MIXER_AMP_GAIN_LUT_CNT)
        gain_index++;

    aic3212_mixer_amp_control_value = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_MIXER_AMPLIFIER_CONTROL);
    if(type == AIC_CHANNEL_LEFT){
        aic3212_mixer_amp_control_value &= 0xD7;
        aic3212_mixer_amp_control_value |= ((power & 1) << 3) | ((in1_route & 1) << 5);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_MIXER_AMPLIFIER_CONTROL,aic3212_mixer_amp_control_value);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_PGA_TO_LEFT_MIXER_AMPLIFIER_MAL_VOLUME_CONTROL,gain_index);
    }else if(type == AIC_CHANNEL_RIGHT){
        aic3212_mixer_amp_control_value &= 0xEB;
        aic3212_mixer_amp_control_value |= ((power & 1) << 2) | ((in1_route & 1) << 4);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_MIXER_AMPLIFIER_CONTROL,aic3212_mixer_amp_control_value);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_PGA_TO_LEFT_MIXER_AMPLIFIER_MAL_VOLUME_CONTROL,gain_index);
    }
}

int aic3212_config_lol(aic3212_instance aic3212, uint8_t power,AicRouteStatus mal_route,AicRouteStatus ldacm_route,
                    AicRouteStatus rdacp_route,AicRouteStatus in1l_route,
                    float in1l_weakening){
    uint8_t aic3212_line_out_amp_control_1_value;
    uint8_t aic3212_line_out_amp_control_2_value;
    uint8_t in1l_weakening_val = (uint8_t)(in1l_weakening/-6.0);

    if((in1l_weakening > 0.0) ||  (in1l_weakening < -12.0))
        return AIC_ERROR;
    aic3212_line_out_amp_control_1_value = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_1);
    aic3212_line_out_amp_control_2_value = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2);

    aic3212_line_out_amp_control_1_value &= 0x5D;
    aic3212_line_out_amp_control_2_value &= 0x67;
    in1l_weakening_val = in1l_route ? ((in1l_weakening_val > 2) ? 3 : (in1l_weakening_val + 1)) : 0;

    aic3212_line_out_amp_control_1_value |= ((power & 1) << 1) | ((ldacm_route & 1) << 7) | ((rdacp_route & 1) << 5);
    aic3212_line_out_amp_control_2_value |= ((mal_route & 1) << 7) | ((in1l_weakening_val & 3) << 3);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_1,aic3212_line_out_amp_control_1_value);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2,aic3212_line_out_amp_control_2_value);
    return AIC_OK;
}

int aic3212_config_lor(aic3212_instance aic3212, uint8_t power,AicRouteStatus lol_route,
                    AicRouteStatus rdacm_route,AicRouteStatus mar_route,
                    AicRouteStatus in1r_route,float in1r_weakening){
    uint8_t aic3212_line_out_amp_control_1_value;
    uint8_t aic3212_line_out_amp_control_2_value;
    uint8_t in1r_weakening_val = (uint8_t)(in1r_weakening/-6.0);

    if((in1r_weakening > 0.0) ||  (in1r_weakening < -12.0))
        return AIC_ERROR;
    aic3212_line_out_amp_control_1_value = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_1);
    aic3212_line_out_amp_control_2_value = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2);

    aic3212_line_out_amp_control_1_value &= 0xBA;
    aic3212_line_out_amp_control_2_value &= 0xBC;
    in1r_weakening_val = in1r_route ? ((in1r_weakening_val > 2) ? 3 : (in1r_weakening_val + 1)) : 0;

    aic3212_line_out_amp_control_1_value |= (power & 1) | ((lol_route & 1) << 2) | ((rdacm_route & 1) << 6);
    aic3212_line_out_amp_control_2_value |= ((mar_route & 1) << 6) | (in1r_weakening_val & 3);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_1,aic3212_line_out_amp_control_1_value);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2,aic3212_line_out_amp_control_2_value);
    return AIC_OK;
}

int aic3212_config_hpl(aic3212_instance aic3212, uint8_t power,AicHeadphoneDriverConfig hp_mode,uint8_t mute,
                   int8_t gain,AicRouteStatus mal_route,AicRouteStatus ldacp_route,
                   AicRouteStatus lol_route,float lol_weakening){

    int8_t gain_val = ((mute == 0) ? gain : -7) & 0x3F;
    uint8_t lol_gain_index=0;
    uint8_t hpl_config_val;

    if(gain > 14 || gain < -6)
        return AIC_ERROR;
    if(lol_weakening > 0 || lol_weakening < -78.3)
        return AIC_ERROR;

    while(aic3212_lo_hp_gain_lut[lol_gain_index]>lol_weakening && lol_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        lol_gain_index++;

    hpl_config_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_1);

    hpl_config_val &= 0x5D;

    hpl_config_val |= ((power & 1) << 1) | ((ldacp_route & 1) << 5) | ((mal_route & 1) << 7);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_1,hpl_config_val);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_2,(lol_route?lol_gain_index:0x7F));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HPL_DRIVER_VOLUME_CONTROL,gain_val | ((hp_mode & 1)<<7));
    return AIC_OK;
}

int aic3212_config_hpr(aic3212_instance aic3212, uint8_t power,AicHeadphoneDriverConfig hp_mode,uint8_t mute,
                    int8_t gain,AicRouteStatus mar_route,AicRouteStatus ldacm_route,
                    AicRouteStatus rdacp_route,uint8_t same_l_r_vol,
                    AicRouteStatus lor_route,float lor_weakening){

    int8_t gain_val = ((mute == 0) ? gain : -7) & 0x3F;
    uint8_t lor_gain_index=0;
    uint8_t hpr_config_val;

    if(gain > 14 || gain < -6)
        return AIC_ERROR;
    if(lor_weakening > 0 || lor_weakening < -78.3)
        return AIC_ERROR;

    while(aic3212_lo_hp_gain_lut[lor_gain_index]>lor_weakening && lor_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        lor_gain_index++;

    hpr_config_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_1);

    hpr_config_val &= 0xAA;

    hpr_config_val |= (power & 1) | ((ldacm_route & 1) << 2) | ((mar_route & 1) << 6) | ((rdacp_route & 1) << 4);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_1,hpr_config_val);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_3,((same_l_r_vol & 1)<<7) | (lor_route?lor_gain_index:0x7F));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HPR_DRIVER_VOLUME_CONTROL,((same_l_r_vol & 1)<<7) | gain_val);
    return AIC_OK;
}

int aic3212_config_spkl(aic3212_instance aic3212, uint8_t power,uint8_t gain,uint8_t mute,
                    AicRouteStatus mal_route,uint8_t is_mono,
                    AicRouteStatus lol_route,float lol_weakening){
    int8_t gain_val = ((mute == 0) ? gain / 6 : 0) & 0x07;
    uint8_t lol_gain_index=0;
    uint8_t spk_control_val;
    uint8_t spk_amplifier_vol_val;

    if(gain > 30 || gain < 6)
        return AIC_ERROR;
    if(lol_weakening > 0 || lol_weakening < -78.3)
        return AIC_ERROR;

    while(aic3212_lo_hp_gain_lut[lol_gain_index]>lol_weakening && lol_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        lol_gain_index++;

    spk_control_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_1);
    spk_amplifier_vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS);

    spk_control_val &= 0x79;
    spk_amplifier_vol_val &= 0x8F;

    spk_control_val |= ((power & 1)<<1) | ((is_mono & 1) << 2) | ((mal_route & 1) << 7);
    spk_amplifier_vol_val |= gain_val << 4;

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_1,spk_control_val);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_2,(lol_route?lol_gain_index:0x7F));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS,spk_amplifier_vol_val);
    return AIC_OK;
}

int aic3212_config_spkr(aic3212_instance aic3212, uint8_t power,uint8_t gain,uint8_t mute,
                    AicRouteStatus mar_route,uint8_t r_l_same_vol,
                    AicRouteStatus lor_route,float lor_weakening){
    int8_t gain_val = ((mute == 0) ? gain / 6 : 0) & 0x07;
    uint8_t lor_gain_index=0;
    uint8_t spk_control_val;
    uint8_t spk_amplifier_vol_val;

    if(gain > 30 || gain < 6)
        return AIC_ERROR;
    if(lor_weakening > 0 || lor_weakening < -78.3)
        return AIC_ERROR;

    while(aic3212_lo_hp_gain_lut[lor_gain_index]>lor_weakening && lor_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        lor_gain_index++;

    spk_control_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_1);
    spk_amplifier_vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS);

    spk_control_val &= 0xBE;
    spk_amplifier_vol_val &= 0xF8;

    spk_control_val |= (power & 1) | ((mar_route & 1) << 6);
    spk_amplifier_vol_val |= gain_val;

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_1,spk_control_val);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_3,(r_l_same_vol << 7) | (lor_route?lor_gain_index:0x7F));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS,spk_amplifier_vol_val);
    return AIC_OK;
}

int aic3212_config_rec(aic3212_instance aic3212, aic3212_rec_config_t * config,uint8_t p_mute,uint8_t m_mute,uint8_t r_l_same_vol){
    uint8_t lol_gain_index=0;
    uint8_t in1l_gain_index=0;
    uint8_t in1r_gain_index=0;
    uint8_t lor_gain_index=0;

    int8_t gain_m_val = ((m_mute == 0) ? config->gain_m : -7) & 0x3F;
    int8_t gain_p_val = ((p_mute == 0) ? config->gain_p : -7) & 0x3F;

    if(config->gain_m > 29 || config->gain_m < -6)
        return AIC_ERROR;
    if(config->gain_p > 29 || config->gain_p < -6)
        return AIC_ERROR;

    if(config->lol_gain > 0 || config->lol_gain < -78.3)
        return AIC_ERROR;

    if(config->in1l_gain > 0 || config->in1l_gain < -78.3)
        return AIC_ERROR;

    if(config->in1r_gain > 0 || config->in1r_gain < -78.3)
        return AIC_ERROR;

    if(config->lor_gain > 0 || config->lor_gain < -78.3)
        return AIC_ERROR;

    while(aic3212_lo_hp_gain_lut[lol_gain_index] > config->lol_gain && lol_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        lol_gain_index++;

    while(aic3212_lo_hp_gain_lut[in1l_gain_index] > config->lol_gain && in1l_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        in1l_gain_index++;

    while(aic3212_lo_hp_gain_lut[in1r_gain_index] > config->lol_gain && in1r_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        in1r_gain_index++;

    while(aic3212_lo_hp_gain_lut[lor_gain_index] > config->lol_gain && lor_gain_index<AIC_LO_HP_GAIN_LUT_CNT)
        lor_gain_index++;


    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_1,config->lol_route ? lol_gain_index : 0x7F);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_2,config->lor_route ? lor_gain_index : 0x7F);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_3,config->in1l_route ? in1l_gain_index : 0x7F);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_4,config->in1r_route ? in1r_gain_index : 0x7F);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_5,((config->pow_p & 1) << 7) | ((config->pow_m & 1) << 6) | gain_p_val);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_6,((r_l_same_vol & 1) << 7) | gain_m_val);
    // REC Calibration (default values)
    //aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_7,...);

    return AIC_OK;
}


int aic3212_adc_config(aic3212_instance aic3212, AicConfigType type,uint8_t pow,AicSoftSteppingVol soft_stepping_vol,
                    AicAdcDigitalMic digital_mic,uint8_t is_mute,double gain){

    uint8_t fine_gain;
    uint8_t adc_power;


    int32_t gain_int = (int32_t)(gain * 10.0f);
    int32_t gain_coarse;
    int32_t gain_fine;

    if(gain_int < - 129 || gain_int > 200)
        return AIC_ERROR;

    if (gain_int <= 0){
        gain_coarse = gain_int / 5;
        gain_coarse = gain_coarse * 5;
    }else{
        gain_coarse = (gain_int - 1) / 5;
        gain_coarse = (gain_coarse + 1) * 5;
    }

    gain_fine = gain_coarse - gain_int;
    gain_coarse /= 5;


    gain_fine = 7 - (gain_fine - 1);

    adc_power = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_CHANNEL_POWER_CONTROL);
    fine_gain = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL);

    if(type == AIC_CHANNEL_LEFT){

        adc_power &= 0x4C;
        fine_gain &= 0x0F;

        adc_power |= ((pow & 1)<<7) | ((digital_mic & 1)<<4) | (soft_stepping_vol & 3);
        fine_gain |= ((is_mute & 1) << 7) | ((gain_fine & 7) << 4);

        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_CHANNEL_POWER_CONTROL,adc_power);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL,fine_gain);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_VOLUME_CONTROL,gain_coarse);

    }else if(type == AIC_CHANNEL_RIGHT){

        adc_power &= 0xB0;
        fine_gain &= 0xF0;

        adc_power |= ((pow & 1)<<6) | ((digital_mic & 1)<<2) | (soft_stepping_vol & 3);
        fine_gain |= ((is_mute & 1) << 3) | (gain_fine & 7);

        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_CHANNEL_POWER_CONTROL,adc_power);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL,fine_gain);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_ADC_VOLUME_CONTROL,gain_coarse);
    }

    return AIC_OK;
}

int aic3212_dac_config(aic3212_instance aic3212, AicConfigType type,AicDacPower pow,AicSoftSteppingVol soft_stepping_vol,
                         AicVolChannelsMode vol_mode,AicAutoMuteMode auto_mute,uint8_t is_mute,double gain){

    uint8_t dac_primary_control;
    uint8_t dac_vol_config;
    int32_t vol_int = (int32_t)(gain * 10.0);

    if(gain < -63.5 || gain > 24.0)
        return AIC_ERROR;

    dac_primary_control = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_POWER_AND_SOFT_STEPPING_CONTROL);
    dac_vol_config = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_MASTER_VOLUME_CONFIGURATION);

    vol_int /= 5;
    if(type == AIC_CHANNEL_LEFT){
        dac_primary_control &= 0x7C;
        dac_vol_config &= 0x84;

        dac_primary_control |= ((pow & 1) << 7) | (soft_stepping_vol & 3);
        dac_vol_config |= (vol_mode & 3) | ((is_mute & 1)<<3) | ((auto_mute & 7)<<4);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_LEFT_VOLUME_CONTROL_SETTING,vol_int & 0xFF);

    }else if(type == AIC_CHANNEL_RIGHT){
        dac_primary_control &= 0xBC;
        dac_vol_config &= 0x08;

        dac_primary_control |= ((pow & 1) << 6) | (soft_stepping_vol & 3);
        dac_vol_config |= (vol_mode & 3) | ((is_mute & 1)<<2) | ((auto_mute & 7)<<4) | ((pow & 2)<<6);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_RIGHT_VOLUME_CONTROL_SETTING,vol_int & 0xFF);
    }

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_POWER_AND_SOFT_STEPPING_CONTROL,dac_primary_control);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_MASTER_VOLUME_CONFIGURATION,dac_vol_config);

    return AIC_OK;
}

void aic3212_beep_config(aic3212_instance aic3212, int8_t left_vol,int8_t right_vol,AicVolChannelsMode vol_mode,uint32_t length,uint16_t sin_freq,uint16_t cos_freq){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_GENERATOR_REGISTER_1,left_vol & 0x3F);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_GENERATOR_REGISTER_2,((vol_mode & 3)<<6) | right_vol & 0x3F);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_GENERATOR_REGISTER_3,(length >> 16) & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_GENERATOR_REGISTER_4,(length >> 8) & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_GENERATOR_REGISTER_5,length & 0xFF);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_SIN_X_LSB,(sin_freq >> 8) & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_SIN_X_MSB,(sin_freq) & 0xFF);

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_COS_X_LSB,(cos_freq >> 8) & 0xFF);
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_COS_X_MSB,(cos_freq) & 0xFF);
}

void aic3212_beep(aic3212_instance aic3212){
    uint8_t beep_gen_reg_1 = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_GENERATOR_REGISTER_1);
    beep_gen_reg_1 |= 1<<7;
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_BEEP_GENERATOR_REGISTER_1,beep_gen_reg_1);
}


void aic3212_config_asi(aic3212_instance aic3212, AicAsiType type,aic3212_asi_config_t * config){
    switch(type){
        case AIC_ASI_1:
        {
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_AUDIO_BUS_FORMAT_CONTROL_REGISTER,
                      (config->dout_high_impendance & 1) | ((config->data_word_len & 3) << 3) | ((config->type & 7) << 5));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_LEFT_CH_OFFSET_1_CONTROL_REGISTER,
                      config->left_channel_offset_1);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_RIGHT_CH_OFFSET_2_CONTROL_REGISTER,
                      config->right_channel_offset_2);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_CHANNEL_SETUP_REGISTER,
                      (config->adc_status & 3) | ((config->dac_status & 3) << 2));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_ADC_INPUT_CONTROL,
                      config->input_mux & 7);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_DAC_OUTPUT_CONTROL,
                      (config->time_slot_mode & 1) | ((config->adc_swap_mode & 1) << 1) | ((config->dac_data_path_right & 3) << 4) | ((config->dac_data_path_left & 3) << 6));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_CONTROL_REGISTER_9_ADC_SLOT_TRISTATE_CONTROL,
                      (config->adc_right_tri_state & 1) | ((config->adc_left_tri_state & 1) << 4));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_WCLK_AND_BCLK_CONTROL_REGISTER,
                      (config->bclk_wclk_pow_mode & 1) | ((config->clk_polarity & 1) << 1) | ((config->wclk_dir & 7) << 5) | ((config->bclk_dir & 7) << 2));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_BIT_CLOCK_N_DIVIDER_INPUT_CONTROL,
                      config->bclk_divider_input & 0x03);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_BIT_CLOCK_N_DIVIDER,
                      (config->bclk_divider_value == 128 ? 0 : (config->bclk_divider_value & 0x7F)) | ((config->bclk_divider_power & 1) << 7));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_WORD_CLOCK_N_DIVIDER,
                      (config->wclk_divider_value == 128 ? 0 : (config->wclk_divider_value & 0x7F)) | ((config->wclk_divder_power & 1) << 7));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_BCLK_AND_WCLK_OUTPUT,
                      (config->wclk_output & 7) | ((config->bclk_output & 7) << 4));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_DATA_OUTPUT,
                      config->data_output_route & 3);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_1_ADC_WCLK_AND_BCLK_CONTROL,
                      (config->adc_bclk & 7) | ((config->adc_wclk & 7) << 4));
        }
        break;
        case AIC_ASI_2:
        {
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_AUDIO_BUS_FORMAT_CONTROL_REGISTER,
                      (config->dout_high_impendance & 1) | ((config->data_word_len & 3) << 3) | ((config->type & 7) << 5));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_DATA_OFFSET_CONTROL_REGISTER,
                      config->left_channel_offset_1);


            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_ADC_INPUT_CONTROL,
                      config->input_mux & 7);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_DAC_OUTPUT_CONTROL,
                      ((config->dac_data_path_right & 3) << 4) | ((config->dac_data_path_left & 3) << 6));

            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_WCLK_AND_BCLK_CONTROL_REGISTER,
                      (config->bclk_wclk_pow_mode & 1) | ((config->clk_polarity & 1) << 1) | ((config->wclk_dir & 1) << 5) | ((config->bclk_dir & 1) << 2));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_BIT_CLOCK_N_DIVIDER_INPUT_CONTROL,
                      config->bclk_divider_input & 0x03);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_BIT_CLOCK_N_DIVIDER,
                      (config->bclk_divider_value == 128 ? 0 : (config->bclk_divider_value & 0x7F)) | ((config->bclk_divider_power & 1) << 7));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_WORD_CLOCK_N_DIVIDER,
                      (config->wclk_divider_value == 128 ? 0 : (config->wclk_divider_value & 0x7F)) | ((config->wclk_divder_power & 1) << 7));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_BCLK_AND_WCLK_OUTPUT,
                      (config->wclk_output & 7) | ((config->bclk_output & 7) << 4));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_DATA_OUTPUT,
                      config->data_output_route & 3);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_2_ADC_WCLK_AND_BCLK_CONTROL,
                      (config->adc_bclk & 7) | ((config->adc_wclk & 7) << 4));
        }
        break;
        case AIC_ASI_3:
        {
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_AUDIO_BUS_FORMAT_CONTROL_REGISTER,
                      (config->dout_high_impendance & 1) | ((config->data_word_len & 3) << 3) | ((config->type & 7) << 5));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_DATA_OFFSET_CONTROL_REGISTER,
                      config->left_channel_offset_1);


            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_ADC_INPUT_CONTROL,
                      config->input_mux & 7);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_DAC_OUTPUT_CONTROL,
                      ((config->dac_data_path_right & 3) << 4) | ((config->dac_data_path_left & 3) << 6));

            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_WCLK_AND_BCLK_CONTROL_REGISTER,
                      (config->bclk_wclk_pow_mode & 1) | ((config->clk_polarity & 1) << 1) | ((config->wclk_dir & 1) << 5) | ((config->bclk_dir & 1) << 2));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_BIT_CLOCK_N_DIVIDER_INPUT_CONTROL,
                      config->bclk_divider_input & 0x03);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_BIT_CLOCK_N_DIVIDER,
                      (config->bclk_divider_value == 128 ? 0 : (config->bclk_divider_value & 0x7F)) | ((config->bclk_divider_power & 1) << 7));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_WORD_CLOCK_N_DIVIDER,
                      (config->wclk_divider_value == 128 ? 0 : (config->wclk_divider_value & 0x7F)) | ((config->wclk_divder_power & 1) << 7));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_BCLK_AND_WCLK_OUTPUT,
                      (config->wclk_output & 7) | ((config->bclk_output & 7) << 4));
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_DATA_OUTPUT,
                      config->data_output_route & 3);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_AUDIO_SERIAL_INTERFACE_3_ADC_WCLK_AND_BCLK_CONTROL,
                      (config->adc_bclk & 7) | ((config->adc_wclk & 7) << 4));
        }
        break;
    }
}

int aic3212_config_drc(aic3212_instance aic3212, uint8_t left_drc,uint8_t right_drc,int8_t drc_thresh,int8_t drc_hysteresis,
                    uint8_t hold_factor,uint8_t max_gain_change_rate,
                    uint8_t attack_rate_factor,uint8_t decay_rate_factor){

    if(drc_thresh > -3 || drc_thresh < -24)
        return AIC_ERROR;

    if(drc_hysteresis > 3 || drc_hysteresis < 0)
        return AIC_ERROR;

    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_DRC_CONTROL_REGISTER_1,
              drc_hysteresis | ((left_drc & 1) << 6) | ((right_drc & 1) << 5) | (drc_thresh << 2));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_DRC_CONTROL_REGISTER_2,
              (max_gain_change_rate & 7) | ((hold_factor & 15) << 3));
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_DRC_CONTROL_REGISTER_3,
              (decay_rate_factor & 15) | ((attack_rate_factor & 15) << 4));

    return AIC_OK;
}

int aic3212_config_agc(aic3212_instance aic3212, AicConfigType type,uint8_t status,aic3212_agc_config_t * config){
    int8_t max_gain_setting = config->maximum_gain * 2;
    int8_t agc_gain = config->gain * 2;

    if(config->gain > 63.5 || config->gain < -12.0)
        return AIC_ERROR;

    if(config->maximum_gain > 63.5 || config->maximum_gain < 0.0)
        return AIC_ERROR;

    if(type == AIC_CHANNEL_LEFT){
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_CONTROL_1,
                  ((status & 1)<<7) | ((config->trg_lvl & 7) << 4) | (config->gain_hysteresis & 3));
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_CONTROL_2,
                  ((config->hysteresis & 3)<<6) | ((config->noise_threshold & 31) << 1));
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_CONTROL_3,
                  max_gain_setting & 127);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_ATTACK_TIME,
                  ((config->attack_time_x32_wclk & 31)<<3) | (config->attack_time_scale_factor & 7));
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_NOISE_DEBOUNCE,
                  config->noise_debounce_time_factor & 31);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_SIGNAL_DEBOUNCE,
                  config->signal_debounce_time_factor & 31);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_GAIN,
                  agc_gain);
    }else if(type == AIC_CHANNEL_RIGHT){
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_CONTROL_1,
                  ((status & 1)<<7) | ((config->trg_lvl & 7) << 4) | (config->gain_hysteresis & 3));
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_CONTROL_2,
                  ((config->hysteresis & 3)<<6) | ((config->noise_threshold & 31) << 1));
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_CONTROL_3,
                  max_gain_setting & 127);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_ATTACK_TIME,
                  ((config->attack_time_x32_wclk & 31)<<3) | (config->attack_time_scale_factor & 7));
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_NOISE_DEBOUNCE,
                  config->noise_debounce_time_factor & 31);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_SIGNAL_DEBOUNCE,
                  config->signal_debounce_time_factor & 31);
        aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_GAIN,
                  agc_gain);
    }

    return AIC_OK;
}



void aic3212_set_vol(aic3212_instance aic3212, AicVolumeType type,uint8_t vol){
    uint8_t vol_val=0;
    vol = vol > 100 ? 100 : vol;

    switch(type){
        case AIC_VOL_PGA_MAL:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_PGA_TO_LEFT_MIXER_AMPLIFIER_MAL_VOLUME_CONTROL);
            vol_val &= 0xC0;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_mixer_amp_gain_lut,AIC_MIXER_AMP_GAIN_LUT_CNT,100 - vol) : 0x3F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_PGA_TO_LEFT_MIXER_AMPLIFIER_MAL_VOLUME_CONTROL,vol_val);
        }
        break;
        case AIC_VOL_PGA_MAR:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_PGA_TO_LEFT_MIXER_AMPLIFIER_MAL_VOLUME_CONTROL);
            vol_val &= 0xC0;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_mixer_amp_gain_lut,AIC_MIXER_AMP_GAIN_LUT_CNT,100 - vol) : 0x3F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_ADC_PGA_TO_RIGHT_MIXER_AMPLIFIER_MAR_VOLUME_CONTROL,vol_val);
        }
        break;
        case AIC_VOL_IN1L_LOL:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2);
            vol_val &= 0xE7;
            vol = 100 - vol;
            vol /= 34;
            vol_val |= vol ? ((vol + 1) & 0x03) : 0x00;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2,vol_val);
        }
        break;
        case AIC_VOL_IN1R_LOR:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2);
            vol_val &= 0xFC;
            vol = 100 - vol;
            vol /= 34;
            vol_val |= vol ? ((vol + 1) & 0x03) : 0x00;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LINEOUT_AMPLIFIER_CONTROL_2,vol_val);
        }
        break;
        case AIC_VOL_LOL_HPL:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_2);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_2,vol_val);
        }
        break;
        case AIC_VOL_LOR_HPR:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_2);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HEADPHONE_AMPLIFIER_CONTROL_2,vol_val);
        }
        break;
        case AIC_VOL_LOL_SPL:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_2);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_2,vol_val);
        }
        break;
        case AIC_VOL_LOR_SPR:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_3);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_CONTROL_3,vol_val);
        }
        break;
        case AIC_VOL_LOL_REC:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_1);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_1,vol_val);
        }
        break;
        case AIC_VOL_LOR_REC:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_2);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_2,vol_val);
        }
        break;
        case AIC_VOL_IN1L_REC:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_3);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_3,vol_val);
        }
        break;
        case AIC_VOL_IN1R_REC:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_4);
            vol_val &= 0x80;
            vol_val |= vol ? aic3212_get_vol_index(aic3212_lo_hp_gain_lut,AIC_LO_HP_GAIN_LUT_CNT,100 - vol) : 0x7F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_4,vol_val);
        }
        break;
        case AIC_VOL_HPL_MAIN:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_HPL_DRIVER_VOLUME_CONTROL);
            vol_val &= 0xC0;
            vol /= 5;
            vol_val |= (vol ? vol - 6 : -7) & 0x3F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HPL_DRIVER_VOLUME_CONTROL,vol_val);
        }
        break;
        case AIC_VOL_HPR_MAIN:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_HPR_DRIVER_VOLUME_CONTROL);
            vol_val &= 0xC0;
            vol /= 5;
            vol_val |= (vol ? vol - 6 : -7) & 0x3F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_HPR_DRIVER_VOLUME_CONTROL,vol_val);
        }
        break;
        case AIC_VOL_REC_P_MAIN:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_5);
            vol_val &= 0xC0;
            vol = (uint8_t)(((float)vol) / 2.8f);
            vol_val |= (vol ? vol - 6 : -7) & 0x3F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_5,vol_val);
        }
        break;
        case AIC_VOL_REC_M_MAIN:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_6);
            vol_val &= 0xC0;
            vol = (uint8_t)(((float)vol) / 2.8f);
            vol_val |= (vol ? vol - 6 : -7) & 0x3F;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RECEIVER_AMPLIFIER_CONTROL_6,vol_val);
        }
        break;
        case AIC_VOL_SPK_L_MAIN:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS);
            vol_val &= 0x8F;
            vol /= 25;
            vol_val |= ((vol ? vol + 1 : 0) & 0x07)<<4;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS,vol_val);
        }
        case AIC_VOL_SPK_R_MAIN:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS);
            vol_val &= 0xF8;
            vol /= 25;
            vol_val |= (vol ? vol + 1 : 0) & 0x07;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS,vol_val);
        }
        break;
        case AIC_VOL_PGA_LEFT:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_MICROPHONE_PGA_CONTROL);
            vol_val &= 0x7F;
            vol = (uint8_t)(((float)vol) / 1.05f);
            vol_val |= vol;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_MICROPHONE_PGA_CONTROL,vol_val);
        }
        break;
        case AIC_VOL_PGA_RIGHT:
        {
            vol_val = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_MICROPHONE_PGA_CONTROL);
            vol_val &= 0x7F;
            vol = (uint8_t)(((float)vol) / 1.05f);
            vol_val |= vol;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_MICROPHONE_PGA_CONTROL,vol_val);
        }
        break;
        case AIC_VOL_ADC_LEFT:
        {
            double vol_double = vol;
            vol_double = ((vol_double / 100.0f) * 32.0f) - 12.0f;
            int32_t gain_int = (int32_t)(vol_double * 10.0f);
            int32_t gain_coarse;
            int32_t gain_fine;

            int8_t fine_gain,coarse_gain;

            if (gain_int < 0){
                gain_coarse = gain_int / 5;
                gain_coarse = gain_coarse * 5;
            }else{
                gain_coarse = (gain_int - 1) / 5;
                gain_coarse = (gain_coarse + 1) * 5;
            }
            gain_fine = gain_coarse - gain_int;
            gain_coarse /= 5;

            gain_fine = 7 - (gain_fine - 1);
            fine_gain = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL);
            fine_gain &= 0x0F;
            fine_gain |= (vol==0?1<<7:0) | ((gain_fine & 7) << 4);
            coarse_gain = gain_coarse & 0x7F;

            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL,fine_gain);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_VOLUME_CONTROL,coarse_gain);
        }
        break;
        case AIC_VOL_ADC_RIGHT:
        {
            double vol_double = vol;
            vol_double = ((vol_double / 100.0f) * 32.0f) - 12.0f;
            int32_t gain_int = (int32_t)(vol_double * 10.0f);
            int32_t gain_coarse;
            int32_t gain_fine;

            int8_t fine_gain,coarse_gain;

            if (gain_int < 0){
                gain_coarse = gain_int / 5;
                gain_coarse = gain_coarse * 5;
            }else{
                gain_coarse = (gain_int - 1) / 5;
                gain_coarse = (gain_coarse + 1) * 5;
            }
            gain_fine = gain_coarse - gain_int;
            gain_coarse /= 5;

            gain_fine = 7 - (gain_fine - 1);
            fine_gain = aic3212_read(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL);
            fine_gain &= 0xF0;
            fine_gain |= (vol==0?1<<3:0) | (gain_fine & 7);
            coarse_gain = gain_coarse & 0x7F;

            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL,fine_gain);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_ADC_VOLUME_CONTROL,coarse_gain);
        }
        break;

        case AIC_VOL_DAC_LEFT:
        {
            int32_t vol_int;
            double vol_double = vol;
            vol_double = ((vol_double * 87.5f) / 100.0f) - 63.5;
            vol_int = (int32_t)(vol_double * 10.0);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_LEFT_VOLUME_CONTROL_SETTING,vol_int & 0xFF);
        }
        break;

        case AIC_VOL_DAC_RIGHT:
        {
            int32_t vol_int;
            double vol_double = vol;
            vol_double = ((vol_double * 87.5f) / 100.0f) - 63.5;
            vol_int = (int32_t)(vol_double * 10.0);
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_PRIMARY_DAC_RIGHT_VOLUME_CONTROL_SETTING,vol_int & 0xFF);
        }
        break;

        case AIC_VOL_AGC_LEFT:
        {
            int8_t vol_int = ((int8_t)((vol * 151) / 100)) - 24;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_LEFT_AGC_GAIN,
                      vol_int);
        }
        break;
        case AIC_VOL_AGC_RIGHT:
        {
            int8_t vol_int = ((int8_t)((vol * 151) / 100)) - 24;
            aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_RIGHT_AGC_GAIN,
                      vol_int);
        }
        break;
    }
}

void aic3212_set_dac_adc_data_port(aic3212_instance aic3212, AicAdcSrcPort adc_src,AicDacSrcPort dac_src){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_ADC_DAC_DATA_PORT_CONTROL,((adc_src & 1)<<6) | ((dac_src & 3)<<4));
}

void aic3212_config_sync_engine(aic3212_instance aic3212, AicSyncEngine adc_sync,AicSyncEngine dac_sync){
    aic3212_write(((aic3212_ins_t *)aic3212)->dev, AIC3212_DIGITAL_AUDIO_ENGINE_SYNCHRONIZATION_CONTROL,((adc_sync & 3)<<4) | ((dac_sync & 3)<<6));
}
