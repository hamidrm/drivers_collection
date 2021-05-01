/*
 * aic3212_driver.h
 *
 *  Created on: Jan 14, 2019
 *      Author: mehrabian
 */

#ifndef BSP_AIC3212_DRIVER_H_
#define BSP_AIC3212_DRIVER_H_

#ifdef  __cplusplus
extern "C" {
#endif

/***********************************************************************/
/*                     MACRO DEFINITIONS.                              */
/***********************************************************************/

#define AIC3212_SLEEP(x)    Task_sleep(x)

#define AIC_ENABLE  1
#define AIC_DISABLE  0


#define AIC_RESET_CONFIG(x) memset(&x,0,sizeof(x))

#define AIC_PGA_GAIN(x) (((uint8_t)(x * 2.0f)) & 0x7F)


#define AIC_MIXER_AMP_GAIN_LUT_CNT  41
#define AIC_LO_HP_GAIN_LUT_CNT 118


/*
 * -------------------------------------
 * Page 0 registers fields
 * -------------------------------------
 * */
/* Reset Register - 1 */
#define AIC3212_RESET   1

/* Clock Control Register 1, Clock Input Multiplexers - 4 */
#define AIC3212_CLK_IN_MUX_ADC_SHIFT 0
#define AIC3212_CLK_IN_MUX_ADC_MASK 0x0F
#define AIC3212_CLK_IN_MUX_DAC_SHIFT 4
#define AIC3212_CLK_IN_MUX_DAC_MASK 0xF0
#define AIC3212_CLK_IN_MCLK1   0
#define AIC3212_CLK_IN_BCLK1   1
#define AIC3212_CLK_IN_GPIO1   2
#define AIC3212_CLK_IN_PLL     3
#define AIC3212_CLK_IN_BCLK2   4
#define AIC3212_CLK_IN_GPI1    5
#define AIC3212_CLK_IN_HF_REF_CLK   6
#define AIC3212_CLK_IN_HF_OSC_CLK   7
#define AIC3212_CLK_IN_MCLK2   8
#define AIC3212_CLK_IN_GPIO2   9
#define AIC3212_CLK_IN_GPI2    10


/* Clock Control Register 2, PLL Input Multiplexer - 5 */
#define AIC3212_PLL_INPUT_BASE_VAL   0x00
#define AIC3212_PLL_IN_MCLK1   (AIC3212_PLL_INPUT_BASE_VAL | (0<<2))
#define AIC3212_PLL_IN_BCLK1   (AIC3212_PLL_INPUT_BASE_VAL | (1<<2))
#define AIC3212_PLL_IN_GPIO1   (AIC3212_PLL_INPUT_BASE_VAL | (2<<2))
#define AIC3212_PLL_IN_DIN1   (AIC3212_PLL_INPUT_BASE_VAL | (3<<2))
#define AIC3212_PLL_IN_BCLK2   (AIC3212_PLL_INPUT_BASE_VAL | (4<<2))
#define AIC3212_PLL_IN_GPI1   (AIC3212_PLL_INPUT_BASE_VAL | (5<<2))
#define AIC3212_PLL_IN_HF_REF_CLK   (AIC3212_PLL_INPUT_BASE_VAL | (6<<2))
#define AIC3212_PLL_IN_GPIO2   (AIC3212_PLL_INPUT_BASE_VAL | (7<<2))
#define AIC3212_PLL_IN_GPI2   (AIC3212_PLL_INPUT_BASE_VAL | (8<<2))
#define AIC3212_PLL_IN_MCLK2   (AIC3212_PLL_INPUT_BASE_VAL | (9<<2))
#define AIC3212_PLL_HIGH_RANGE (AIC3212_PLL_INPUT_BASE_VAL | (1<<6))
#define AIC3212_PLL_LOW_RANGE (AIC3212_PLL_INPUT_BASE_VAL | (0<<6))

/* Clock Control Register 3, PLL P and R Values - 6 */
#define AIC3212_PLL_P_R_BASE_VAL   0x00
#define AIC3212_PLL_POW_ON   (AIC3212_PLL_P_R_BASE_VAL | (1<<7))
#define AIC3212_PLL_POW_OFF   (AIC3212_PLL_P_R_BASE_VAL | (0<<7))
#define AIC3212_PLL_R_SHIFT 0
#define AIC3212_PLL_P_SHIFT 4
#define AIC3212_PLL_R_MASK 0x0F
#define AIC3212_PLL_P_MASK 0x70

/* Clock Control Register 4, PLL J Value - 7 */
#define AIC3212_PLL_J_BASE_VAL   0x00
#define AIC3212_PLL_J_SHIFT 0
#define AIC3212_PLL_J_MASK 0x3F

/* Clock Control Register 5, PLL D Values (MSB) - 8 */
#define AIC3212_PLL_D_MSB_BASE_VAL   0x00
#define AIC3212_PLL_D_MSB_SHIFT 0
#define AIC3212_PLL_D_MSB_MASK 0x3F

/* Clock Control Register 6, PLL D Values (LSB) - 9 */
#define AIC3212_PLL_D_LSB_SHIFT 0
#define AIC3212_PLL_D_LSB_MASK 0xFF

/* Clock Control Register 7, PLL_CLKIN Divider - 10  */
#define AIC3212_PLL_DIV_BASE_VAL   0x00
#define AIC3212_PLL_DIV_SHIFT 0
#define AIC3212_PLL_DIV_MASK 0x7F

/* Clock Control Register 8, NDAC Divider Values  - 11 */
#define AIC3212_NDAC_ON  1<<7
#define AIC3212_NDAC_OFF 0<<7
#define AIC_NDAC_VAL_SHIFT  0
#define AIC_NDAC_VAL_MASK  0x7F

/* Clock Control Register 9, MDAC Divider Values  - 12 */
#define AIC3212_MDAC_ON  1<<7
#define AIC3212_MDAC_OFF 0<<7
#define AIC_MDAC_VAL_SHIFT  0
#define AIC_MDAC_VAL_MASK  0x7F

/* DAC OSR Control Register 1, MSB Value - 13 */
#define AIC3212_DAC_OSR_MSB_BASE_VAL   0x00
#define AIC_DAC_OSR_MSB_SHIFT  0
#define AIC_DAC_OSR_MSB_MASK  0x3

/* DAC OSR Control Register 2, LSB Value - 14 */
#define AIC_DAC_OSR_LSB_SHIFT  0
#define AIC_DAC_OSR_LSB_MASK  0xFF

/* 15-17 Reserved */

/* Clock Control Register 10, NADC Values - 18 */
#define AIC3212_NADC_ON  1<<7
#define AIC3212_NADC_OFF 0<<7
#define AIC_NADC_VAL_SHIFT  0
#define AIC_NADC_VAL_MASK  0x7F

/* Clock Control Register 11, MADC Values - 19 */
#define AIC3212_MADC_ON  1<<7
#define AIC3212_MADC_OFF 0<<7
#define AIC_MADC_VAL_SHIFT  0
#define AIC_MADC_VAL_MASK  0x7F


/* ADC Over-Sampling (AOSR) Register - 20 */
#define AIC_ADC_OSR_SHIFT  0
#define AIC_ADC_OSR_MASK  0xFF

/* CLKOUT MUX - 21 */
#define AIC3212_CLK_OUT_BASE_VAL   0x00
#define AIC3212_CLK_OUT_MCLK1  0
#define AIC3212_CLK_OUT_BCLK1  1
#define AIC3212_CLK_OUT_DIN1   2
#define AIC3212_CLK_OUT_PLL     3
#define AIC3212_CLK_OUT_DAC_CLK 4
#define AIC3212_CLK_OUT_DAC_MOD_CLK 5
#define AIC3212_CLK_OUT_ADC_CLK     6
#define AIC3212_CLK_OUT_ADC_MOD_CLK     7
#define AIC3212_CLK_OUT_BCLK2    8
#define AIC3212_CLK_OUT_GPI1    9
#define AIC3212_CLK_OUT_HF_REF_CLK  10
#define AIC3212_CLK_OUT_HF_OSC_CLK  11
#define AIC3212_CLK_OUT_MCLK2   12
#define AIC3212_CLK_OUT_GPI2    13

/* Clock Control Register 12, CLKOUT M Divider Value - 22 */
#define AIC3212_CLKOUT_DIVIDER_ON  1<<7
#define AIC3212_CLKOUT_DIVIDER_OFF 0<<7
#define AIC3212_CLKOUT_DIVIDER_VAL_MASK 0x7F

/* ADC Flag Register - 36 */
#define ADC_FLAG_L_ADC_PGA_STATUS_SHIFT  7
#define ADC_FLAG_L_ADC_POW_STATUS_SHIFT  6
#define ADC_FLAG_L_ADC_AGC_GAIN_STATUS_SHIFT  5
#define ADC_FLAG_R_ADC_PGA_STATUS_SHIFT  3
#define ADC_FLAG_R_ADC_POW_STATUS_SHIFT  2
#define ADC_FLAG_R_ADC_AGC_GAIN_STATUS_SHIFT  1

#define ADC_FLAG_L_ADC_PGA_MASK  (1<<ADC_FLAG_L_ADC_PGA_STATUS_SHIFT)
#define ADC_FLAG_L_ADC_POW_MASK  (1<<ADC_FLAG_L_ADC_POW_STATUS_SHIFT)
#define ADC_FLAG_L_ADC_AGC_GAIN_MASK  (1<<ADC_FLAG_L_ADC_AGC_GAIN_STATUS_SHIFT)
#define ADC_FLAG_R_ADC_PGA_MASK  (1<<ADC_FLAG_R_ADC_PGA_STATUS_SHIFT)
#define ADC_FLAG_R_ADC_POW_MASK  (1<<ADC_FLAG_R_ADC_POW_STATUS_SHIFT)
#define ADC_FLAG_R_ADC_AGC_GAIN_MASK  (1<<ADC_FLAG_R_ADC_AGC_GAIN_STATUS_SHIFT)


/* DAC Flag Register 1 - 37 */
#define DAC_FLAG_1_L_DAC_POW_STATUS_SHIFT  7
#define DAC_FLAG_1_JACK_STATUS_SHIFT  4
#define DAC_FLAG_1_R_DAC_POW_STATUS_SHIFT  3
#define DAC_FLAG_1_HEAD_SET_STATUS_SHIFT  0

#define DAC_FLAG_1_L_DAC_POW_MASK  (1<<DAC_FLAG_1_L_DAC_POW_STATUS_SHIFT)
#define DAC_FLAG_1_JACK_MASK  (3<<DAC_FLAG_1_JACK_STATUS_SHIFT)
#define DAC_FLAG_1_R_DAC_POW_MASK  (1<<DAC_FLAG_1_R_DAC_POW_STATUS_SHIFT)
#define DAC_FLAG_1_HEAD_SET_MASK  (3<<DAC_FLAG_1_HEAD_SET_STATUS_SHIFT)


/* DAC Flag Register 2 - 38 */
#define DAC_FLAG_2_SEC_L_DAC_PGA_MUTE_STATUS_SHIFT  7
#define DAC_FLAG_2_PRI_L_DAC_PGA_MUTE_STATUS_SHIFT  6
#define DAC_FLAG_2_SEC_L_DAC_PGA_GAIN_STATUS_SHIFT  5
#define DAC_FLAG_2_PRI_L_DAC_PGA_GAIN_STATUS_SHIFT  4
#define DAC_FLAG_2_SEC_R_DAC_PGA_MUTE_STATUS_SHIFT  3
#define DAC_FLAG_2_PRI_R_DAC_PGA_MUTE_STATUS_SHIFT  2
#define DAC_FLAG_2_SEC_R_DAC_PGA_GAIN_STATUS_SHIFT  1
#define DAC_FLAG_2_PRI_R_DAC_PGA_GAIN_STATUS_SHIFT  0

#define DAC_FLAG_2_SEC_L_DAC_PGA_MUTE_MASK  (1<<DAC_FLAG_2_SEC_L_DAC_PGA_MUTE_STATUS_SHIFT)
#define DAC_FLAG_2_PRI_L_DAC_PGA_MUTE_MASK  (1<<DAC_FLAG_2_PRI_L_DAC_PGA_MUTE_STATUS_SHIFT)
#define DAC_FLAG_2_SEC_L_DAC_PGA_GAIN_MASK  (1<<DAC_FLAG_2_SEC_L_DAC_PGA_GAIN_STATUS_SHIFT)
#define DAC_FLAG_2_PRI_L_DAC_PGA_GAIN_MASK  (1<<DAC_FLAG_2_PRI_L_DAC_PGA_GAIN_STATUS_SHIFT)
#define DAC_FLAG_2_SEC_R_DAC_PGA_MUTE_MASK  (1<<DAC_FLAG_2_SEC_R_DAC_PGA_MUTE_STATUS_SHIFT)
#define DAC_FLAG_2_PRI_R_DAC_PGA_MUTE_MASK  (1<<DAC_FLAG_2_PRI_R_DAC_PGA_MUTE_STATUS_SHIFT)
#define DAC_FLAG_2_SEC_R_DAC_PGA_GAIN_MASK  (1<<DAC_FLAG_2_SEC_R_DAC_PGA_GAIN_STATUS_SHIFT)
#define DAC_FLAG_2_PRI_R_DAC_PGA_GAIN_MASK  (1<<DAC_FLAG_2_PRI_R_DAC_PGA_GAIN_STATUS_SHIFT)


/* 39-41 Reserved */

/* Sticky Flag Register 1 - 42 */
#define STICKY_FLAG_REG_1_L_DAC_OVF_STATUS_SHIFT    7
#define STICKY_FLAG_REG_1_R_DAC_OVF_STATUS_SHIFT    6
#define STICKY_FLAG_REG_1_L_ADC_OVF_STATUS_SHIFT    3
#define STICKY_FLAG_REG_1_R_ADC_OVF_STATUS_SHIFT    2

#define STICKY_FLAG_REG_1_L_DAC_OVF_MASK    (1<<STICKY_FLAG_REG_1_L_DAC_OVF_STATUS_SHIFT)
#define STICKY_FLAG_REG_1_R_DAC_OVF_MASK    (1<<STICKY_FLAG_REG_1_R_DAC_OVF_STATUS_SHIFT)
#define STICKY_FLAG_REG_1_L_ADC_OVF_MASK    (1<<STICKY_FLAG_REG_1_L_ADC_OVF_STATUS_SHIFT)
#define STICKY_FLAG_REG_1_R_ADC_OVF_MASK    (1<<STICKY_FLAG_REG_1_R_ADC_OVF_STATUS_SHIFT)


/* Interrupt Flag Register 1  - 43 */
#define INT_FLAG_REG_1_L_DAC_OVF_STATUS_SHIFT    7
#define INT_FLAG_REG_1_R_DAC_OVF_STATUS_SHIFT    6
#define INT_FLAG_REG_1_L_ADC_OVF_STATUS_SHIFT    3
#define INT_FLAG_REG_1_R_ADC_OVF_STATUS_SHIFT    2

#define INT_FLAG_REG_1_L_DAC_OVF_MASK    (1<<INT_FLAG_REG_1_L_DAC_OVF_STATUS_SHIFT)
#define INT_FLAG_REG_1_R_DAC_OVF_MASK    (1<<INT_FLAG_REG_1_R_DAC_OVF_STATUS_SHIFT)
#define INT_FLAG_REG_1_L_ADC_OVF_MASK    (1<<INT_FLAG_REG_1_L_ADC_OVF_STATUS_SHIFT)
#define INT_FLAG_REG_1_R_ADC_OVF_MASK    (1<<INT_FLAG_REG_1_R_ADC_OVF_STATUS_SHIFT)


/* Sticky Flag Register 2 - 44 */
#define STICKY_FLAG_REG_2_L_SHORT_CIRCUITE_STATUS_SHIFT   7
#define STICKY_FLAG_REG_2_R_SHORT_CIRCUITE_STATUS_SHIFT   6
#define STICKY_FLAG_REG_2_HEADSET_BUTTON_STATUS_SHIFT   5
#define STICKY_FLAG_REG_2_HEADSET_STATUS_SHIFT   4
#define STICKY_FLAG_REG_2_L_DRC_STATUS_SHIFT   3
#define STICKY_FLAG_REG_2_R_DRC_STATUS_SHIFT   2

#define STICKY_FLAG_REG_2_L_SHORT_CIRCUITE_MASK   (1<<STICKY_FLAG_REG_2_L_SHORT_CIRCUITE_STATUS_SHIFT)
#define STICKY_FLAG_REG_2_R_SHORT_CIRCUITE_MASK   (1<<STICKY_FLAG_REG_2_R_SHORT_CIRCUITE_STATUS_SHIFT)
#define STICKY_FLAG_REG_2_HEADSET_BUTTON_MASK  (1<<STICKY_FLAG_REG_2_HEADSET_BUTTON_STATUS_SHIFT)
#define STICKY_FLAG_REG_2_HEADSET_MASK   (1<<STICKY_FLAG_REG_2_HEADSET_STATUS_SHIFT)
#define STICKY_FLAG_REG_2_L_DRC_MASK   (1<<STICKY_FLAG_REG_2_L_DRC_STATUS_SHIFT)
#define STICKY_FLAG_REG_2_R_DRC_MASK   (1<<STICKY_FLAG_REG_2_R_DRC_STATUS_SHIFT)

/* Sticky Flag Register 3 - 45 */
#define STICKY_FLAG_REG_3_SPK_OVER_TEMP_STATUS_SHIFT   7
#define STICKY_FLAG_REG_3_L_AGC_NOISE_STATUS_SHIFT   6
#define STICKY_FLAG_REG_3_R_AGC_NOISE_STATUS_SHIFT   5
#define STICKY_FLAG_REG_3_L_ADC_DATA_AVLBLE_STATUS_SHIFT   2
#define STICKY_FLAG_REG_3_R_ADC_DATA_AVLBLE_STATUS_SHIFT   1

#define STICKY_FLAG_REG_3_SPK_OVER_TEMP_MASK   (1<<STICKY_FLAG_REG_3_SPK_OVER_TEMP_STATUS_SHIFT)
#define STICKY_FLAG_REG_3_L_AGC_NOISE_MASK   (1<<STICKY_FLAG_REG_3_L_AGC_NOISE_STATUS_SHIFT)
#define STICKY_FLAG_REG_3_R_AGC_NOISE_MASK   (1<<STICKY_FLAG_REG_3_R_AGC_NOISE_STATUS_SHIFT)
#define STICKY_FLAG_REG_3_L_ADC_DATA_AVLBLE_MASK   (1<<STICKY_FLAG_REG_3_L_ADC_DATA_AVLBLE_STATUS_SHIFT)
#define STICKY_FLAG_REG_3_R_ADC_DATA_AVLBLE_MASK   (1<<STICKY_FLAG_REG_3_R_ADC_DATA_AVLBLE_STATUS_SHIFT)

/* Interrupt Flag Register 2 - 46 */
#define INT_FLAG_REG_2_L_SHORT_CIRCUITE_STATUS_SHIFT   7
#define INT_FLAG_REG_2_R_SHORT_CIRCUITE_STATUS_SHIFT   6
#define INT_FLAG_REG_2_HEADSET_BUTTON_STATUS_SHIFT   5
#define INT_FLAG_REG_2_HEADSET_STATUS_SHIFT   4
#define INT_FLAG_REG_2_L_DRC_STATUS_SHIFT   3
#define INT_FLAG_REG_2_R_DRC_STATUS_SHIFT   2

#define INT_FLAG_REG_2_L_SHORT_CIRCUITE_MASK   (1<<INT_FLAG_REG_2_L_SHORT_CIRCUITE_STATUS_SHIFT)
#define INT_FLAG_REG_2_R_SHORT_CIRCUITE_MASK   (1<<INT_FLAG_REG_2_R_SHORT_CIRCUITE_STATUS_SHIFT)
#define INT_FLAG_REG_2_HEADSET_BUTTON_MASK  (1<<INT_FLAG_REG_2_HEADSET_BUTTON_STATUS_SHIFT)
#define INT_FLAG_REG_2_HEADSET_MASK   (1<<INT_FLAG_REG_2_HEADSET_STATUS_SHIFT)
#define INT_FLAG_REG_2_L_DRC_MASK   (1<<INT_FLAG_REG_2_L_DRC_STATUS_SHIFT)
#define INT_FLAG_REG_2_R_DRC_MASK   (1<<INT_FLAG_REG_2_R_DRC_STATUS_SHIFT)

/* Interrupt Flag Register 3 - 47 */
#define INT_FLAG_REG_3_SPK_OVER_TEMP_STATUS_SHIFT   7
#define INT_FLAG_REG_3_L_AGC_NOISE_STATUS_SHIFT   6
#define INT_FLAG_REG_3_R_AGC_NOISE_STATUS_SHIFT   5
#define INT_FLAG_REG_3_L_ADC_DATA_AVLBLE_STATUS_SHIFT   2
#define INT_FLAG_REG_3_R_ADC_DATA_AVLBLE_STATUS_SHIFT   1

#define INT_FLAG_REG_3_SPK_OVER_TEMP_MASK   (1<<INT_FLAG_REG_3_SPK_OVER_TEMP_STATUS_SHIFT)
#define INT_FLAG_REG_3_L_AGC_NOISE_MASK   (1<<INT_FLAG_REG_3_L_AGC_NOISE_STATUS_SHIFT)
#define INT_FLAG_REG_3_R_AGC_NOISE_MASK   (1<<INT_FLAG_REG_3_R_AGC_NOISE_STATUS_SHIFT)
#define INT_FLAG_REG_3_L_ADC_DATA_AVLBLE_MASK   (1<<INT_FLAG_REG_3_L_ADC_DATA_AVLBLE_STATUS_SHIFT)
#define INT_FLAG_REG_3_R_ADC_DATA_AVLBLE_MASK   (1<<INT_FLAG_REG_3_R_ADC_DATA_AVLBLE_STATUS_SHIFT)

/* INT1 Interrupt Control - 48 */
#define INT1_CONTROL_BASE_VAL   0x00
#define INT1_HEADSET_INSERTION_EVT_SHIFT 7
#define INT1_BUTTON_EVT_SHIFT 6
#define INT1_DRC_EVT_SHIFT 5
#define INT1_AGC_EVT_SHIFT 4
#define INT1_OVER_CURRENT_EVT_SHIFT 3
#define INT1_OVF_EVT_SHIFT 2
#define INT1_SPK_OVER_TEMP_EVT_SHIFT 1

#define INT1_HEADSET_INSERTION_EVT_ON (INT1_CONTROL_BASE_VAL | (1<<INT1_HEADSET_INSERTION_EVT_SHIFT))
#define INT1_HEADSET_INSERTION_EVT_OFF (INT1_CONTROL_BASE_VAL | (0<<INT1_HEADSET_INSERTION_EVT_SHIFT))
#define INT1_BUTTON_EVT_EVT_ON (INT1_CONTROL_BASE_VAL | (1<<INT1_BUTTON_EVT_SHIFT))
#define INT1_BUTTON_EVT_OFF (INT1_CONTROL_BASE_VAL | (0<<INT1_BUTTON_EVT_SHIFT))
#define INT1_DRC_EVT_ON (INT1_CONTROL_BASE_VAL | (1<<INT1_DRC_EVT_SHIFT))
#define INT1_DRC_EVT_OFF (INT1_CONTROL_BASE_VAL | (0<<INT1_DRC_EVT_SHIFT))
#define INT1_AGC_EVT_ON (INT1_CONTROL_BASE_VAL | (1<<INT1_AGC_EVT_SHIFT))
#define INT1_AGC_EVT_OFF (INT1_CONTROL_BASE_VAL | (0<<INT1_AGC_EVT_SHIFT))
#define INT1_OVER_CURRENT_EVT_ON (INT1_CONTROL_BASE_VAL | (1<<INT1_OVER_CURRENT_EVT_SHIFT))
#define INT1_OVER_CURRENT_EVT_OFF (INT1_CONTROL_BASE_VAL | (0<<INT1_OVER_CURRENT_EVT_SHIFT))
#define INT1_OVF_EVT_ON (INT1_CONTROL_BASE_VAL | (1<<INT1_OVF_EVT_SHIFT))
#define INT1_OVF_EVT_OFF (INT1_CONTROL_BASE_VAL | (0<<INT1_OVF_EVT_SHIFT))
#define INT1_SPK_OVER_TEMP_EVT_ON (INT1_CONTROL_BASE_VAL | (1<<INT1_SPK_OVER_TEMP_EVT_SHIFT))
#define INT1_SPK_OVER_TEMP_EVT_OFF (INT1_CONTROL_BASE_VAL | (0<<INT1_SPK_OVER_TEMP_EVT_SHIFT))

/* INT2 Interrupt Control - 49 */
#define INT2_CONTROL_BASE_VAL   0x00
#define INT2_HEADSET_INSERTION_EVT_SHIFT 7
#define INT2_BUTTON_EVT_SHIFT 6
#define INT2_DRC_EVT_SHIFT 5
#define INT2_AGC_EVT_SHIFT 4
#define INT2_OVER_CURRENT_EVT_SHIFT 3
#define INT2_OVF_EVT_SHIFT 2
#define INT2_SPK_OVER_TEMP_EVT_SHIFT 1

#define INT2_HEADSET_INSERTION_EVT_ON (INT2_CONTROL_BASE_VAL | (1<<INT2_HEADSET_INSERTION_EVT_SHIFT))
#define INT2_HEADSET_INSERTION_EVT_OFF (INT2_CONTROL_BASE_VAL | (0<<INT2_HEADSET_INSERTION_EVT_SHIFT))
#define INT2_BUTTON_EVT_EVT_ON (INT2_CONTROL_BASE_VAL | (1<<INT2_BUTTON_EVT_SHIFT))
#define INT2_BUTTON_EVT_OFF (INT2_CONTROL_BASE_VAL | (0<<INT2_BUTTON_EVT_SHIFT))
#define INT2_DRC_EVT_ON (INT2_CONTROL_BASE_VAL | (1<<INT2_DRC_EVT_SHIFT))
#define INT2_DRC_EVT_OFF (INT2_CONTROL_BASE_VAL | (0<<INT2_DRC_EVT_SHIFT))
#define INT2_AGC_EVT_ON (INT2_CONTROL_BASE_VAL | (1<<INT2_AGC_EVT_SHIFT))
#define INT2_AGC_EVT_OFF (INT2_CONTROL_BASE_VAL | (0<<INT2_AGC_EVT_SHIFT))
#define INT2_OVER_CURRENT_EVT_ON (INT2_CONTROL_BASE_VAL | (1<<INT2_OVER_CURRENT_EVT_SHIFT))
#define INT2_OVER_CURRENT_EVT_OFF (INT2_CONTROL_BASE_VAL | (0<<INT2_OVER_CURRENT_EVT_SHIFT))
#define INT2_OVF_EVT_ON (INT2_CONTROL_BASE_VAL | (1<<INT2_OVF_EVT_SHIFT))
#define INT2_OVF_EVT_OFF (INT2_CONTROL_BASE_VAL | (0<<INT2_OVF_EVT_SHIFT))
#define INT2_SPK_OVER_TEMP_EVT_ON (INT2_CONTROL_BASE_VAL | (1<<INT2_SPK_OVER_TEMP_EVT_SHIFT))
#define INT2_SPK_OVER_TEMP_EVT_OFF (INT2_CONTROL_BASE_VAL | (0<<INT2_SPK_OVER_TEMP_EVT_SHIFT))

/*
 * -------------------------------------
 * Page 1 registers fields
 * -------------------------------------
 * */
/* Power Configuration Register - 1 */
#define AIC3212_POW_CONTROL_BASE_VAL   0x00
#define AIC3212_PC_ANALOG_POWER_ON   (AIC3212_POW_CONTROL_BASE_VAL | (0<<2))
#define AIC3212_PC_ANALOG_POWER_OFF  (AIC3212_POW_CONTROL_BASE_VAL | (1<<2))
#define AIC3212_PC_DVDD_AVDD_CONNECTION_ENABLE  (AIC3212_POW_CONTROL_BASE_VAL | (1<<3))
#define AIC3212_PC_DVDD_AVDD_CONNECTION_DISABLE (AIC3212_POW_CONTROL_BASE_VAL | (0<<3))


/* Microphone PGA Power-Up Control Register - 121  */
#define AIC3212_PGA_POWER_UP_CONTROL_BASE_VAL   0x30
#define AIC3212_MIC_PGA_QUICK_CHARGE_R_0_MS_L_0_MS       (AIC3212_PGA_POWER_UP_CONTROL_BASE_VAL | (0<<0))
#define AIC3212_MIC_PGA_QUICK_CHARGE_R_2_2_MS_L_0_9_MS   (AIC3212_PGA_POWER_UP_CONTROL_BASE_VAL | (1<<0))
#define AIC3212_MIC_PGA_QUICK_CHARGE_R_5_5_MS_L_0_9_MS   (AIC3212_PGA_POWER_UP_CONTROL_BASE_VAL | (2<<0))
#define AIC3212_MIC_PGA_QUICK_CHARGE_R_1_1_MS_L_0_5_MS   (AIC3212_PGA_POWER_UP_CONTROL_BASE_VAL | (3<<0))

/* Reference Power-Up Delay Register - 122   */
#define AIC3212_REF_POWER_UP_DELAY_BASE_VAL   0x00
#define AIC3212_VREF_FAST_CHARGE_DISABLED   (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (0<<0))
#define AIC3212_VREF_FAST_CHARGE_10_30_MS   (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (1<<0))
#define AIC3212_VREF_FAST_CHARGE_20_60_MS   (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (2<<0))
#define AIC3212_VREF_FAST_CHARGE_30_90_MS   (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (3<<0))
#define AIC3212_CHIP_REF_POW_AUTO       (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (0<<2))
#define AIC3212_CHIP_REF_POW_FORCE_ON   (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (1<<2))
#define AIC3212_FINE_CT_POWERED_UP       (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (0<<3))
#define AIC3212_COARSE_CT_POWERED_UP   (AIC3212_REF_POWER_UP_DELAY_BASE_VAL | (1<<3))

/*
 * -------------------------------------
 * Page 3 registers fields
 * -------------------------------------
 * */

/* SAR ADC Flags Register 1 - 9 */
#define AIC3212_SAR_ADC_FLAGS_REG_1_ADC_BUSY_SHIFT   6
#define AIC3212_SAR_ADC_FLAGS_REG_1_ADC_NEW_DATA_SHIFT   5
#define AIC3212_SAR_ADC_FLAGS_REG_1_ADC_BUSY_MASK    (1<<AIC3212_SAR_ADC_FLAGS_REG_1_ADC_BUSY_SHIFT)
#define AIC3212_SAR_ADC_FLAGS_REG_1_ADC_NEW_DATA_MASK    (1<<AIC3212_SAR_ADC_FLAGS_REG_1_ADC_NEW_DATA_SHIFT)

/* SAR ADC Flags Register 2 - 10 */
#define AIC3212_SAR_ADC_FLAGS_REG_2_IN1L_DATA_READY_SHIFT   7
#define AIC3212_SAR_ADC_FLAGS_REG_2_IN1R_DATA_READY_SHIFT   6
#define AIC3212_SAR_ADC_FLAGS_REG_2_VBAT_DATA_READY_SHIFT   5
#define AIC3212_SAR_ADC_FLAGS_REG_2_TEMP1_DATA_READY_SHIFT   1
#define AIC3212_SAR_ADC_FLAGS_REG_2_TEMP2_DATA_READY_SHIFT   0
#define AIC3212_SAR_ADC_FLAGS_REG_2_IN1L_DATA_READY_MASK   (1<<AIC3212_SAR_ADC_FLAGS_REG_2_IN1L_DATA_READY_SHIFT)
#define AIC3212_SAR_ADC_FLAGS_REG_2_IN1R_DATA_READY_MASK   (1<<AIC3212_SAR_ADC_FLAGS_REG_2_IN1R_DATA_READY_SHIFT)
#define AIC3212_SAR_ADC_FLAGS_REG_2_VBAT_DATA_READY_MASK   (1<<AIC3212_SAR_ADC_FLAGS_REG_2_VBAT_DATA_READY_SHIFT)
#define AIC3212_SAR_ADC_FLAGS_REG_2_TEMP1_DATA_READY_MASK   (1<<AIC3212_SAR_ADC_FLAGS_REG_2_TEMP1_DATA_READY_SHIFT)
#define AIC3212_SAR_ADC_FLAGS_REG_2_TEMP2_DATA_READY_MASK   (1<<AIC3212_SAR_ADC_FLAGS_REG_2_TEMP2_DATA_READY_SHIFT)

/* SAR ADC Measurement Control - 19 */
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_SHIFT 7
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_SHIFT 6
#define AIC3212_SAR_ADC_AUTO_MEASURE_VBAT_SHIFT 5
#define AIC3212_SAR_ADC_AUTO_MEASURE_TEMP_SHIFT 4
#define AIC3212_SAR_ADC_AUTO_MEASURE_TEMP_SELECT_SHIFT 3
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_R_OR_V_SHIFT 2
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_R_OR_V_SHIFT 1
#define AIC3212_SAR_ADC_AUTO_MEASURE_RES_MEASURE_BIAS_SHIFT 0

#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_ENABLE    (1<<AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_ENABLE    (1<<AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_VBAT_ENABLE    (1<<AIC3212_SAR_ADC_AUTO_MEASURE_VBAT_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_TEMP_1_ENABLE    (1<<AIC3212_SAR_ADC_AUTO_MEASURE_TEMP_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_TEMP_2_ENABLE    ((1<<AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_SHIFT) | (1<<AIC3212_SAR_ADC_AUTO_MEASURE_TEMP_SELECT_SHIFT))
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_VOLT_ENABLE    (0<<AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_R_OR_V_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_RES_ENABLE    (1<<AIC3212_SAR_ADC_AUTO_MEASURE_IN1L_R_OR_V_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_VOLT_ENABLE    (0<<AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_R_OR_V_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_RES_ENABLE    (1<<AIC3212_SAR_ADC_AUTO_MEASURE_IN1R_R_OR_V_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_RES_MEASURE_BIAS_INT_ENABLE    (0<<AIC3212_SAR_ADC_AUTO_MEASURE_RES_MEASURE_BIAS_SHIFT)
#define AIC3212_SAR_ADC_AUTO_MEASURE_RES_MEASURE_BIAS_EXT_ENABLE    (1<<AIC3212_SAR_ADC_AUTO_MEASURE_RES_MEASURE_BIAS_SHIFT)


/* SAR ADC Measurement Threshold Flags - 21 */
#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1L_MAX_THRESH_SHIFT 5
#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1L_MIN_THRESH_SHIFT 4
#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1R_MAX_THRESH_SHIFT 3
#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1R_MIN_THRESH_SHIFT 2
#define AIC3212_SAR_ADC_MEASURE_THRESH_TEMP_MAX_THRESH_SHIFT 1
#define AIC3212_SAR_ADC_MEASURE_THRESH_TEMP_MIN_THRESH_SHIFT 0

#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1L_MAX_MASK (1<<AIC3212_SAR_ADC_MEASURE_THRESH_IN1L_MAX_THRESH_SHIFT)
#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1L_MIN_MASK (1<<AIC3212_SAR_ADC_MEASURE_THRESH_IN1L_MIN_THRESH_SHIFT)
#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1R_MAX_MASK (1<<AIC3212_SAR_ADC_MEASURE_THRESH_IN1R_MAX_THRESH_SHIFT)
#define AIC3212_SAR_ADC_MEASURE_THRESH_IN1R_MIN_MASK (1<<AIC3212_SAR_ADC_MEASURE_THRESH_IN1R_MIN_THRESH_SHIFT)
#define AIC3212_SAR_ADC_MEASURE_THRESH_TEMP_MAX_MASK (1<<AIC3212_SAR_ADC_MEASURE_THRESH_TEMP_MAX_THRESH_SHIFT)
#define AIC3212_SAR_ADC_MEASURE_THRESH_TEMP_MIN_MASK (1<<AIC3212_SAR_ADC_MEASURE_THRESH_TEMP_MIN_THRESH_SHIFT)

/* SAR ADC Controls */
#define AIC3212_SAR_ADC_FILTER_MEAN    0
#define AIC3212_SAR_ADC_BUFFER_MEDIAN    1

#define AIC3212_SAR_ADC_BUFFER_CONTINUOU_MODE    0
#define AIC3212_SAR_ADC_BUFFER_SINGLE_SHOT_MODE    1

#define AIC3212_SAR_ADC_CLK_SRC_INTERNAL    0
#define AIC3212_SAR_ADC_CLK_SRC_MCLK1    1

#define AIC3212_SAR_ADC_AUTO_HALTED_UPDATE      0
#define AIC3212_SAR_ADC_UPDATE_ALL_TIME    2
#define AIC3212_SAR_ADC_UPDATE_STOPED



/***********************************************************************/
/*                     TYPEDEF DEFINITIONS.                         */
/***********************************************************************/
typedef void * aic3212_instance;
typedef void(* aic3212_rx_done_cb)(void * arg, uint32_t * buffer, size_t samples_number);
typedef void(* aic3212_tx_done_cb)(void * arg, uint32_t * buffer, size_t samples_number);

/***********************************************************************/
/*                     ENUMERATOR DEFINITIONS.                         */
/***********************************************************************/

typedef enum {
    AIC_POWER_DOWN,
    AIC_POWER_UP,
}AicPower;


/*
 * -------------------------------------
 * Page 0
 * -------------------------------------
 * */

typedef enum {
     AIC_CLKIN_MCLK1 = AIC3212_CLK_IN_MCLK1,
     AIC_CLKIN_BCLK1 = AIC3212_CLK_IN_BCLK1,
     AIC_CLKIN_GPIO1 = AIC3212_CLK_IN_GPIO1,
     AIC_CLKIN_PLL = AIC3212_CLK_IN_PLL,
     AIC_CLKIN_BCLK2 = AIC3212_CLK_IN_BCLK2,
     AIC_CLKIN_GPI1 = AIC3212_CLK_IN_GPI1,
     AIC_CLKIN_HF_REF_CLK = AIC3212_CLK_IN_HF_REF_CLK,
     AIC_CLKIN_GPIO2 = AIC3212_CLK_IN_GPIO2,
     AIC_CLKIN_GPI2 = AIC3212_CLK_IN_GPI2,
     AIC_CLKIN_MCLK2 = AIC3212_CLK_IN_MCLK2,
}AicClkInput;

 typedef enum {
    AIC_PLLIN_MCLK1 = AIC3212_PLL_IN_MCLK1,
    AIC_PLLIN_BCLK1 = AIC3212_PLL_IN_BCLK1,
    AIC_PLLIN_GPIO1 = AIC3212_PLL_IN_GPIO1,
    AIC_PLLIN_DIN1 = AIC3212_PLL_IN_DIN1,
    AIC_PLLIN_BCLK2 = AIC3212_PLL_IN_BCLK2,
    AIC_PLLIN_GPI1 = AIC3212_PLL_IN_GPI1,
    AIC_PLLIN_HF_REF_CLK = AIC3212_PLL_IN_HF_REF_CLK,
    AIC_PLLIN_GPIO2 = AIC3212_PLL_IN_GPIO2,
    AIC_PLLIN_GPI2 = AIC3212_PLL_IN_GPI2,
    AIC_PLLIN_MCLK2 = AIC3212_PLL_IN_MCLK2,
}AicPllInput;

typedef enum {
    AIC_CLKOUT_MCLK1 = AIC3212_CLK_OUT_MCLK1,
    AIC_CLKOUT_BCLK1 = AIC3212_CLK_OUT_BCLK1,
    AIC_CLKOUT_DIN1 = AIC3212_CLK_OUT_DIN1,
    AIC_CLKOUT_PLL_CLK = AIC3212_CLK_OUT_PLL,
    AIC_CLKOUT_DAC_CLK = AIC3212_CLK_OUT_DAC_CLK,
    AIC_CLKOUT_DAC_MOD_CLK = AIC3212_CLK_OUT_DAC_MOD_CLK,
    AIC_CLKOUT_ADC_CLK = AIC3212_CLK_OUT_ADC_CLK,
    AIC_CLKOUT_ADC_MOD_CLK = AIC3212_CLK_OUT_ADC_MOD_CLK ,
    AIC_CLKOUT_BCLK2 = AIC3212_CLK_OUT_BCLK2,
    AIC_CLKOUT_GPI1 = AIC3212_CLK_OUT_GPI1,
    AIC_CLKOUT_HF_REF_CLK = AIC3212_CLK_OUT_HF_REF_CLK,
    AIC_CLKOUT_HF_OSC_CLK = AIC3212_CLK_OUT_HF_OSC_CLK,
    AIC_CLKOUT_MCLK2 = AIC3212_CLK_OUT_MCLK2,
    AIC_CLKOUT_GPI2 = AIC3212_CLK_OUT_GPI2,
}AicClkOutInput;


typedef enum {
    AIC_ADC_DIGITAL_MIC_DISABLE,
    AIC_ADC_DIGITAL_MIC_ENABLE,
    AIC_ADC_DAC_CIC_LOOPBACK,
}AicAdcDigitalMic;

typedef enum {
    AIC_SOFT_STEPPING_VOL_1_0_GAIN_PER_WC,
    AIC_SOFT_STEPPING_VOL_0_5_GAIN_PER_WC,
    AIC_SOFT_STEPPING_VOL_DISABLE
}AicSoftSteppingVol;


typedef enum {
    AIC_DAC_POWER_DOWN_ZERO_AS_OUTPUT,
    AIC_DAC_POWER_UP,
    AIC_DAC_POWER_DOWN_INVERT_L_AS_OUTPUT, // Just usable for right DAC
}AicDacPower;


typedef enum {
    AIC_DAC_AUTO_MUTE_DISABLE,
    AIC_DAC_AUTO_MUTE_AFTER_100_SAMPLE,
    AIC_DAC_AUTO_MUTE_AFTER_200_SAMPLE,
    AIC_DAC_AUTO_MUTE_AFTER_400_SAMPLE,
    AIC_DAC_AUTO_MUTE_AFTER_800_SAMPLE,
    AIC_DAC_AUTO_MUTE_AFTER_1600_SAMPLE,
    AIC_DAC_AUTO_MUTE_AFTER_3200_SAMPLE,
    AIC_DAC_AUTO_MUTE_AFTER_6400_SAMPLE,
}AicAutoMuteMode;

typedef enum {
    AIC_VOL_INDEPENDENT,
    AIC_VOL_LEFT_IS_VOL_RIGHT,
    AIC_VOL_RIGHT_IS_VOL_LEFT,
}AicVolChannelsMode;

typedef enum {
    AIC_AGC_TRIG_LVL_N_5_5_DBFS,
    AIC_AGC_TRIG_LVL_N_8_DBFS,
    AIC_AGC_TRIG_LVL_N_10_DBFS,
    AIC_AGC_TRIG_LVL_N_12_DBFS,
    AIC_AGC_TRIG_LVL_N_14_DBFS,
    AIC_AGC_TRIG_LVL_N_17_DBFS,
    AIC_AGC_TRIG_LVL_N_20_DBFS,
    AIC_AGC_TRIG_LVL_N_24_DBFS,
}AicAgcTrigLvl;

typedef enum {
    AIC_AGC_GAIN_HYSTERESIS_0_5_DB,
    AIC_AGC_GAIN_HYSTERESIS_1_0_DB,
    AIC_AGC_GAIN_HYSTERESIS_1_5_DB,
}AicAgcGainHysteresis;

typedef enum {
    AIC_AGC_HYSTERESIS_1_DB,
    AIC_AGC_HYSTERESIS_2_DB,
    AIC_AGC_HYSTERESIS_4_DB,
}AicAgcHysteresis;


/*
 * -------------------------------------
 * Page 1
 * -------------------------------------
 * */

typedef enum {
    AIC_ANALOG_ON = AIC3212_PC_ANALOG_POWER_ON,
    AIC_ANALOG_OFF = AIC3212_PC_ANALOG_POWER_OFF,
}AicPowConfigAnalog;

typedef enum {
    AIC_DVDD_AVDD_CONNECTION_ENABLE = AIC3212_PC_DVDD_AVDD_CONNECTION_ENABLE,
    AIC_DVDD_AVDD_CONNECTION_DISABLE = AIC3212_PC_DVDD_AVDD_CONNECTION_DISABLE,
}AicPowConfigDvddAvddConnection;

typedef enum {
    AIC_MIC_QC_R0_L0 = AIC3212_MIC_PGA_QUICK_CHARGE_R_0_MS_L_0_MS,
    AIC_MIC_QC_R2_2_L0_9 = AIC3212_MIC_PGA_QUICK_CHARGE_R_2_2_MS_L_0_9_MS,
    AIC_MIC_QC_R5_5_L0_9 = AIC3212_MIC_PGA_QUICK_CHARGE_R_5_5_MS_L_0_9_MS,
    AIC_MIC_QC_R1_1_L0_5 = AIC3212_MIC_PGA_QUICK_CHARGE_R_1_1_MS_L_0_5_MS,
}AicMicPgaPower;

typedef enum {
    AIC_VREF_FC_DIS = AIC3212_VREF_FAST_CHARGE_DISABLED,
    AIC_VREF_FC_10_30_MS = AIC3212_VREF_FAST_CHARGE_10_30_MS,
    AIC_VREF_FC_20_60_MS = AIC3212_VREF_FAST_CHARGE_20_60_MS,
    AIC_VREF_FC_30_90_MS = AIC3212_VREF_FAST_CHARGE_30_90_MS,
}AicVrefFastCharge;

typedef enum {
    AIC_CHIP_REF_AUTO = AIC3212_CHIP_REF_POW_AUTO,
    AIC_CHIP_REF_FORCE = AIC3212_CHIP_REF_POW_FORCE_ON,
}AicChipRefPow;

typedef enum {
    AIC_FCT_POW_MODE = AIC3212_FINE_CT_POWERED_UP,
    AIC_CCT_POW_MODE = AIC3212_COARSE_CT_POWERED_UP,
}AicFineCoarsePowMode;

typedef enum {
    AIC_VOL_PGA_MAL,
    AIC_VOL_PGA_MAR,

    AIC_VOL_IN1L_LOL,
    AIC_VOL_IN1R_LOR,

    AIC_VOL_LOL_HPL,
    AIC_VOL_LOR_HPR,

    AIC_VOL_LOL_SPL,
    AIC_VOL_LOR_SPR,

    AIC_VOL_LOL_REC,
    AIC_VOL_LOR_REC,
    AIC_VOL_IN1L_REC,
    AIC_VOL_IN1R_REC,

    AIC_VOL_HPL_MAIN,
    AIC_VOL_HPR_MAIN,
    AIC_VOL_REC_P_MAIN,
    AIC_VOL_REC_M_MAIN,
    AIC_VOL_SPK_L_MAIN,
    AIC_VOL_SPK_R_MAIN,

    AIC_VOL_PGA_LEFT,
    AIC_VOL_PGA_RIGHT,

    AIC_VOL_ADC_LEFT,
    AIC_VOL_ADC_RIGHT,

    AIC_VOL_DAC_LEFT,
    AIC_VOL_DAC_RIGHT,

    AIC_VOL_AGC_LEFT,
    AIC_VOL_AGC_RIGHT,
}AicVolumeType;

typedef enum {
    AIC_IN_DISCONNECTED,
    AIC_IN_10_K,
    AIC_IN_20_K,
    AIC_IN_40_K,
}AicPgaInput;

typedef enum {
    AIC_ADC_PTM_R4,
    AIC_ADC_PTM_R3,
    AIC_ADC_PTM_R2,
    AIC_ADC_PTM_R1,
}AicAdcPTM;

typedef enum {
    AIC_DAC_PTM_P3_P4,
    AIC_DAC_PTM_P2,
    AIC_DAC_PTM_P1,
}AicDacPTM;

typedef enum {
    AIC_ADC_SRC_PGA,
    AIC_ADC_SRC_IN2,
}AicAdcInput;

typedef enum {
    AIC_CHANNEL_LEFT,
    AIC_CHANNEL_RIGHT,
}AicConfigType;

typedef enum {
    AIC_NOT_ROUTED,
    AIC_ROUTED,
}AicRouteStatus;


typedef enum {
    AIC_HP_UNIPOLAR_CAP_COUPLED,
    AIC_HP_GROUND_CENTERED,
}AicHeadphoneDriverConfig;


/*
 * -------------------------------------
 * Page 3
 * -------------------------------------
 * */
typedef enum {
    AIC_SAR_ADC_REULATION_8_BITS = 1,
    AIC_SAR_ADC_REULATION_10_BITS = 2,
    AIC_SAR_ADC_REULATION_12_BITS = 0,
}AicSarResulation;

typedef enum {
    AIC_SAR_ADC_CLK_DIVIDER_1,
    AIC_SAR_ADC_CLK_DIVIDER_2,
    AIC_SAR_ADC_CLK_DIVIDER_4,
    AIC_SAR_ADC_CLK_DIVIDER_8,
}AicSarClkDivider;

typedef enum {
    AIC_SAR_ADC_AVG_4_MEAN_5_MEDIAN,
    AIC_SAR_ADC_AVG_8_MEAN_9_MEDIAN,
    AIC_SAR_ADC_AVG_16_MEAN_15_MEADIAN,
}AicSarAvg;

typedef enum {
    AIC_SAR_ADC_VBAT = 6,
    AIC_SAR_ADC_IN1R = 7,
    AIC_SAR_ADC_IN1L = 8,
    AIC_SAR_ADC_AUTO = 9,
    AIC_SAR_ADC_TEMP1 = 10,
    AIC_SAR_ADC_TEMP2 = 12,
    AIC_SAR_ADC_PORT = 11,
}AicSarConvMode;

typedef enum {
    AIC_SAR_ADC_REF_STABILIZATION_TIME_0_MS,
    AIC_SAR_ADC_REF_STABILIZATION_TIME_1_MS,
    AIC_SAR_ADC_REF_STABILIZATION_TIME_4_MS,
    AIC_SAR_ADC_REF_STABILIZATION_TIME_8_MS,
}AicSarRefrenceStabilizationTime ;

typedef enum {
    AIC_SAR_ADC_BUF_TRIG_LVL_8,
    AIC_SAR_ADC_BUF_TRIG_LVL_16,
    AIC_SAR_ADC_BUF_TRIG_LVL_24,
    AIC_SAR_ADC_BUF_TRIG_LVL_32,
    AIC_SAR_ADC_BUF_TRIG_LVL_40,
    AIC_SAR_ADC_BUF_TRIG_LVL_48,
    AIC_SAR_ADC_BUF_TRIG_LVL_56,
    AIC_SAR_ADC_BUF_TRIG_LVL_64,
}AicSarBufferTrigLvl;

typedef enum {
    AIC_SAR_ADC_TIMER_DELAY_1_12_MIN,
    AIC_SAR_ADC_TIMER_DELAY_3_36_MIN,
    AIC_SAR_ADC_TIMER_DELAY_5_59_MIN,
    AIC_SAR_ADC_TIMER_DELAY_7_83_MIN,
    AIC_SAR_ADC_TIMER_DELAY_10_01_MIN,
    AIC_SAR_ADC_TIMER_DELAY_12_30_MIN,
    AIC_SAR_ADC_TIMER_DELAY_14_54_MIN,
    AIC_SAR_ADC_TIMER_DELAY_16_78_MIN,
}AicSarTimerDelay;

/*
 * -------------------------------------
 * Page 4
 * -------------------------------------
 * */

typedef enum {
    AIC_ASI_1,
    AIC_ASI_2,
    AIC_ASI_3,
} AicAsiType;

typedef enum {
    AIC_ASI_I2S,
    AIC_ASI_DSP,
    AIC_ASI_RJF,
    AIC_ASI_LJF,
    AIC_ASI_MONO_PCM,
} AicAsiInterfaceType;

typedef enum {
    AIC_ASI_NOT_HIGH_IMPENDANCE,
    AIC_ASI_HIGH_IMPENDANCE,
} AicAsiDoutHighImpendance;

typedef enum {
    AIC_ASI_L_ENABLE_R_ENABLE,
    AIC_ASI_L_DISABLE,
    AIC_ASI_R_DISABLE,
    AIC_ASI_L_DISABLE_R_DISABLE,
} AicAsiDacAdcStatus;

typedef enum {
    AIC_ASI_IN_DISABLE,
    AIC_ASI_IN_ADC,
    AIC_ASI_IN_ASI_1,
    AIC_ASI_IN_ASI_2,
    AIC_ASI_IN_ASI_3,
} AicAsiOutputMux;


typedef enum {
    AIC_ASI_DATA_PATH_OFF,
    AIC_ASI_DATA_PATH_LEFT,
    AIC_ASI_DATA_PATH_RIGHT,
    AIC_ASI_DATA_PATH_MONO_MIX,
} AicAsiDacDataPath;

typedef enum {
    AIC_ASI_ADC_SWAP_CHANNELS_DISABLE,
    AIC_ASI_ADC_SWAP_CHANNELS_ENABLE,
} AicAsiAdcSwapMode;

typedef enum {
    AIC_ASI_TIME_SLOT_MODE_DISABLE,
    AIC_ASI_TIME_SLOT_MODE_ENABLE,
} AicAsiTimeSlotMode;

typedef enum {
    AIC_ASI_ADC_TRI_STATE_DISABLE,
    AIC_ASI_ADC_TRI_STATE_ENABLE,
} AicAsiAdcTriStateMode;

typedef enum {
    AIC_ASI_WCLK_DIR_WCLK_INPUT,
    AIC_ASI_WCLK_DIR_WCLK_OUTPUT,
    AIC_ASI_WCLK_DIR_GPIO_INPUT,
    AIC_ASI_WCLK_DIR_GPIO_OUTPUT,
    AIC_ASI_WCLK_DIR_DOUT_OUTPUT,
} AicAsiWclkDir;

typedef enum {
    AIC_ASI_BCLK_DIR_BCLK_INPUT,
    AIC_ASI_BCLK_DIR_BCLK_OUTPUT,
    AIC_ASI_BCLK_DIR_GPIO_INPUT,
    AIC_ASI_BCLK_DIR_GPIO_OUTPUT,
} AicAsiBclkDir;


typedef enum {
    AIC_ASI_BCLK_NORMAL,
    AIC_ASI_BCLK_INVERTED,
} AicAsiBitClkPolarity;

typedef enum {
    AIC_ASI_CLK_POWER_BY_ACTIVITY,
    AIC_ASI_CLK_POWER_ALWAYS_ON,
} AicAsiBclkWclkPowMode;

typedef enum {
    AIC_ASI_BCLK_DIVIDER_IN_N_DAC_CLK,
    AIC_ASI_BCLK_DIVIDER_IN_M_DAC_CLK,
    AIC_ASI_BCLK_DIVIDER_IN_N_ADC_CLK,
    AIC_ASI_BCLK_DIVIDER_IN_M_ADC_CLK,
} AicAsiBclkDividerInput;


typedef enum {
    AIC_ASI_BCLK_MUX_ASI_1_BCLK_DIVIDER,
    AIC_ASI_BCLK_MUX_ASI_1_BCLK_IN,
    AIC_ASI_BCLK_MUX_ASI_2_BCLK_DIVIDER,
    AIC_ASI_BCLK_MUX_ASI_2_BCLK_IN,
    AIC_ASI_BCLK_MUX_ASI_3_BCLK_DIVIDER,
    AIC_ASI_BCLK_MUX_ASI_3_BCLK_IN,
} AicAsiBclkMux;

typedef enum {
    AIC_ASI_WCLK_MUX_ASI_1_DAC_FS,
    AIC_ASI_WCLK_MUX_ASI_1_ADC_FS,
    AIC_ASI_WCLK_MUX_ASI_1_WCLK_DIVIDER,
    AIC_ASI_WCLK_MUX_ASI_1_WCLK_IN,
    AIC_ASI_WCLK_MUX_ASI_2_WCLK_DIVIDER,
    AIC_ASI_WCLK_MUX_ASI_2_WCLK_IN,
    AIC_ASI_WCLK_MUX_ASI_3_WCLK_DIVIDER,
    AIC_ASI_WCLK_MUX_ASI_3_WCLK_IN,
} AicAsiWclkMux;

typedef enum {
    AIC_ASI_DOUT_ASI_OUTPUT,
    AIC_ASI_DOUT_ASI_1_INPUT,
    AIC_ASI_DOUT_ASI_2_INPUT,
    AIC_ASI_DOUT_ASI_3_INPUT,
} AicAsiDataOutControl;

typedef enum {
    AIC_ASI_ADC_WCLK_DAC_WCLK,
    AIC_ASI_ADC_WCLK_GPIO_1,
    AIC_ASI_ADC_WCLK_GPIO_2,
    AIC_ASI_ADC_WCLK_GPI_1,
    AIC_ASI_ADC_WCLK_GPI_2,
    AIC_ASI_ADC_WCLK_GPI_3,
    AIC_ASI_ADC_WCLK_GPI_4,
} AicAsiAdcWclk;

typedef enum {
    AIC_ASI_ADC_BCLK_DAC_BCLK,
    AIC_ASI_ADC_BCLK_GPIO_1,
    AIC_ASI_ADC_BCLK_GPIO_2,
    AIC_ASI_ADC_BCLK_GPI_1,
    AIC_ASI_ADC_BCLK_GPI_2,
    AIC_ASI_ADC_BCLK_GPI_3,
    AIC_ASI_ADC_BCLK_GPI_4,
} AicAsiAdcBclk;

typedef enum {
    AIC_ASI_DATA_WORD_16_BITS,
    AIC_ASI_DATA_WORD_20_BITS,
    AIC_ASI_DATA_WORD_24_BITS,
    AIC_ASI_DATA_WORD_32_BITS,
} AicAsiDataWordLen;


typedef enum {
    AIC_ADC_SRC_PORT_INDEPENDENT,
    AIC_ADC_R_CHANNEL_IS_L_CHANNEL,
}AicAdcSrcPort;

typedef enum {
    AIC_DAC_SRC_ASI1,
    AIC_DAC_SRC_ASI2,
    AIC_DAC_SRC_ASI3,
    AIC_DAC_SRC_ADC,
}AicDacSrcPort;

typedef enum {
    AIC_SYNC_ENGINE_USING_ASI1,
    AIC_SYNC_ENGINE_USING_ASI2,
    AIC_SYNC_ENGINE_USING_ASI3,
    AIC_SYNC_ENGINE_DISABLE,
}AicSyncEngine;

typedef enum {
    AIC_ASI_DIVIDER_POWER_OFF,
    AIC_ASI_DIVIDER_POWER_ON,
} AicAsiDividerPow;

typedef enum {
    AIC_ERROR = -1,
    AIC_OK = 0
} AicStatus;





/***********************************************************************/
/*                     DATA STRUCTURE DEFINITIONS.                     */
/***********************************************************************/

typedef struct aic_sar_adc_config_t{
    AicSarResulation resulation;
    AicSarClkDivider clk_divider;
    AicSarAvg avg;
    AicSarConvMode conv_mode;
    AicSarRefrenceStabilizationTime refrence_stabilization_time;
    AicSarBufferTrigLvl trig_lvl;
    AicSarTimerDelay timer_delay;
    uint8_t filter;
    uint8_t interrupt_control;
    uint8_t internal_ref;
    uint8_t auto_power;
    uint8_t buffer_data;
    uint8_t buffer_mode;
    uint8_t auto_measurement;
    uint8_t clk_src;
    uint8_t update_mode;
    uint8_t adc_measurement_control;
    uint8_t in1l_max_threshold;
    uint8_t in1l_min_threshold;
    uint8_t in1r_max_threshold;
    uint8_t in1r_min_threshold;
    uint8_t temp_max_threshold;
    uint8_t temp_min_threshold;
    uint16_t in1l_max;
    uint16_t in1l_min;
    uint16_t in1r_max;
    uint16_t in1r_min;
    uint16_t temp_max;
    uint16_t temp_min;
} aic3212_sar_adc_config_t;

typedef struct aic_left_pga_config_t{
    AicPgaInput in1l_p;
    AicPgaInput in2l_p;
    AicPgaInput in3l_p;
    AicRouteStatus     in4l_p;
    AicPgaInput in1r_p;

    AicPgaInput in2r_m;
    AicPgaInput in3r_m;
    AicRouteStatus     in4r_m;
    AicPgaInput cm1_m;
    AicPgaInput cm2_m;

    float     gain;
} aic3212_left_pga_config_t;

typedef struct aic_right_pga_config_t{
    AicPgaInput        in1r_p;
    AicPgaInput        in2r_p;
    AicPgaInput        in3r_p;
    AicRouteStatus     in4r_p;
    AicPgaInput        in2l_p;

    AicPgaInput        in1l_m;
    AicPgaInput        in3l_m;
    AicRouteStatus     in4l_m;
    AicPgaInput        cm1_m;
    AicPgaInput        cm2_m;

    float              gain;
} aic3212_right_pga_config_t;


typedef struct aic_rec_config_t{
    AicPgaInput lol_route;
    float lol_gain;
    AicPgaInput in1l_route;
    float in1l_gain;
    AicPgaInput in1r_route;
    float in1r_gain;
    AicPgaInput lor_route;
    float lor_gain;

    int8_t gain_m;
    int8_t gain_p;

    uint8_t pow_m;
    uint8_t pow_p;

} aic3212_rec_config_t;

typedef struct aic_asi_config_t{
    AicAsiInterfaceType type;
    AicAsiDataWordLen data_word_len;
    AicAsiDoutHighImpendance dout_high_impendance;
    uint8_t left_channel_offset_1;
    uint8_t right_channel_offset_2;
    AicAsiDacAdcStatus dac_status;
    AicAsiDacAdcStatus adc_status;
    AicAsiOutputMux input_mux;
    AicAsiDacDataPath dac_data_path_left;
    AicAsiDacDataPath dac_data_path_right;
    AicAsiAdcSwapMode adc_swap_mode;
    AicAsiTimeSlotMode time_slot_mode;
    AicAsiAdcTriStateMode adc_left_tri_state;
    AicAsiAdcTriStateMode adc_right_tri_state;
    AicAsiWclkDir wclk_dir;
    AicAsiBclkDir bclk_dir;
    AicAsiBitClkPolarity clk_polarity;
    AicAsiBclkWclkPowMode bclk_wclk_pow_mode;
    AicAsiBclkDividerInput bclk_divider_input;
    AicPower bclk_divider_power;
    uint8_t bclk_divider_value;
    AicPower wclk_divder_power;
    uint8_t wclk_divider_value;
    AicAsiBclkMux bclk_output;
    AicAsiWclkMux wclk_output;
    AicAsiDataOutControl data_output_route;
    AicAsiAdcWclk adc_wclk;
    AicAsiAdcBclk adc_bclk;
} aic3212_asi_config_t;

typedef struct aic_agc_config_t {
    AicAgcTrigLvl trg_lvl;
    AicAgcGainHysteresis gain_hysteresis;
    AicAgcHysteresis hysteresis;
    int8_t noise_threshold;
    float maximum_gain;
    uint8_t attack_time_x32_wclk;
    uint8_t attack_time_scale_factor;
    uint8_t decay_time_x512_wclk;
    uint8_t decay_time_scale_factor;
    uint8_t noise_debounce_time_factor;
    uint8_t signal_debounce_time_factor;
    float gain;
} aic3212_agc_config_t;


typedef struct {
    struct aic3212_dev * dev;
    void * sem;
    void * spi_instance;
    void * mcasp_instance;
    aic3212_rx_done_cb aic3212_rx_done_fn;
    void * rx_done_arg;
    aic3212_tx_done_cb aic3212_tx_done_fn;
    void * tx_done_arg;
    uint8_t spi_channel_id;
}aic3212_ins_t;

/***********************************************************************/
/*                     FUNCTION PROTOTYPES       .                     */
/***********************************************************************/
aic3212_instance aic3212_create(instance_t spi, void *mcasp, uint8_t spi_sub_channel_id, Semaphore_Handle spi_sem,
                                aic3212_rx_done_cb rx_done, void *rx_done_arg, aic3212_tx_done_cb tx_done, void *tx_done_arg
                                , void *rx_buff_audio_ping, void *rx_buff_audio_pong
                                , void *tx_buff_audio_ping, void *tx_buff_audio_pong);
void aic3212_set_processing_block(aic3212_instance aic3212, uint8_t adc_pb,uint8_t dac_pb);
void aic3212_adc_power_tune(aic3212_instance aic3212, AicAdcPTM ptm,AicAdcInput l_input,AicAdcInput r_input);
void aic3212_dac_power_tune(aic3212_instance aic3212, AicDacPTM left_ptm,AicDacPTM right_ptm);
void aic3212_config_power(aic3212_instance aic3212, AicPowConfigAnalog pow_analog,AicPowConfigDvddAvddConnection d_a_conn,
                      AicMicPgaPower mic_pow,AicVrefFastCharge vref_fc,AicChipRefPow chip_ref,
                      AicFineCoarsePowMode f_c_pm);
void aic3212_software_reset(aic3212_instance aic3212);
int aic3212_power_on_pll(aic3212_instance aic3212, AicPllInput pll_in,uint32_t freq_in,uint32_t freq_out);
void aic3212_config_fs_clk(aic3212_instance aic3212, AicClkInput adc_clk_src,AicClkInput dac_clk_src,
                       uint8_t ndac,uint8_t mdac,uint16_t dac_osr,
                       uint8_t nadc,uint8_t madc,uint8_t adc_osr);
void aic3212_config_clkout(aic3212_instance aic3212, AicClkOutInput clkout_mux,uint8_t divder);
void aic3212_config_sar(aic3212_instance aic3212, aic3212_sar_adc_config_t * config);
uint16_t aic3212_get_sar_temp1_value(aic3212_instance aic3212);
uint16_t aic3212_get_sar_temp2_value(aic3212_instance aic3212);
uint16_t aic3212_get_sar_vbat_value(aic3212_instance aic3212);
uint16_t aic3212_get_sar_in1l_value(aic3212_instance aic3212);
uint16_t aic3212_get_sar_in1r_value(aic3212_instance aic3212);
void aic3212_sar_convert(aic3212_instance aic3212, aic3212_sar_adc_config_t * config);
void aic3212_config_left_pga(aic3212_instance aic3212, aic3212_left_pga_config_t * config);
void aic3212_config_right_pga(aic3212_instance aic3212, aic3212_right_pga_config_t * config);
void aic3212_config_mixer_amp(aic3212_instance aic3212, AicConfigType type,uint8_t power,AicRouteStatus in1_route,float weakening);
int aic3212_config_lol(aic3212_instance aic3212, uint8_t power,AicRouteStatus mal_route,AicRouteStatus ldacm_route,
                    AicRouteStatus rdacp_route,AicRouteStatus in1l_route,
                    float in1l_weakening);
int aic3212_config_lor(aic3212_instance aic3212, uint8_t power,AicRouteStatus lol_route,
                    AicRouteStatus rdacm_route,AicRouteStatus mar_route,
                    AicRouteStatus in1r_route,float in1r_weakening);
int aic3212_config_hpl(aic3212_instance aic3212, uint8_t power,AicHeadphoneDriverConfig hp_mode,uint8_t mute,
                   int8_t gain,AicRouteStatus mal_route,AicRouteStatus ldacp_route,
                   AicRouteStatus lol_route,float lol_weakening);
int aic3212_config_hpr(aic3212_instance aic3212, uint8_t power,AicHeadphoneDriverConfig hp_mode,uint8_t mute,
                    int8_t gain,AicRouteStatus mar_route,AicRouteStatus ldacm_route,
                    AicRouteStatus rdacp_route,uint8_t same_l_r_vol,
                    AicRouteStatus lor_route,float lor_weakening);
int aic3212_config_spkl(aic3212_instance aic3212, uint8_t power,uint8_t gain,uint8_t mute,
                    AicRouteStatus mal_route,uint8_t is_mono,
                    AicRouteStatus lol_route,float lol_weakening);
int aic3212_config_spkr(aic3212_instance aic3212, uint8_t power,uint8_t gain,uint8_t mute,
                    AicRouteStatus mar_route,uint8_t r_l_same_vol,
                    AicRouteStatus lor_route,float lor_weakening);
int aic3212_config_rec(aic3212_instance aic3212, aic3212_rec_config_t * config,uint8_t p_mute,uint8_t m_mute,uint8_t r_l_same_vol);
int aic3212_adc_config(aic3212_instance aic3212, AicConfigType type,uint8_t pow,AicSoftSteppingVol soft_stepping_vol,
                    AicAdcDigitalMic digital_mic,uint8_t is_mute,double gain);
int aic3212_dac_config(aic3212_instance aic3212, AicConfigType type,AicDacPower pow,AicSoftSteppingVol soft_stepping_vol,
                         AicVolChannelsMode vol_mode,AicAutoMuteMode auto_mute,uint8_t is_mute,double gain);
void aic3212_beep_config(aic3212_instance aic3212, int8_t left_vol,int8_t right_vol,AicVolChannelsMode vol_mode,uint32_t length,uint16_t sin_freq,uint16_t cos_freq);
void aic3212_beep(aic3212_instance aic3212);
void aic3212_config_asi(aic3212_instance aic3212, AicAsiType type,aic3212_asi_config_t * config);
int aic3212_config_drc(aic3212_instance aic3212, uint8_t left_drc,uint8_t right_drc,int8_t drc_thresh,int8_t drc_hysteresis,
                    uint8_t hold_factor,uint8_t max_gain_change_rate,
                    uint8_t attack_rate_factor,uint8_t decay_rate_factor);
int aic3212_config_agc(aic3212_instance aic3212, AicConfigType type,uint8_t status,aic3212_agc_config_t * config);
void aic3212_set_vol(aic3212_instance aic3212, AicVolumeType type,uint8_t vol);
void aic3212_set_dac_adc_data_port(aic3212_instance aic3212, AicAdcSrcPort adc_src,AicDacSrcPort dac_src);
void aic3212_config_sync_engine(aic3212_instance aic3212, AicSyncEngine adc_sync,AicSyncEngine dac_sync);
void aic3212_start(aic3212_instance aic3212);

#ifdef __cplusplus
}
#endif
#endif
