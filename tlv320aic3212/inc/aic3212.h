/*
 * aic3212.h
 *
 *  Created on: Jan 15, 2019
 *      Author: mehrabian
 */

#ifndef HLD_INC_AIC3212_SPI_H_
#define HLD_INC_AIC3212_SPI_H_

/***********************************************************************/
/*                     MACRO DEFINITIONS.                              */
/***********************************************************************/
#define AIC_TO_PACKED_ADDR(book,page,reg) (((book&0xFF) << 16) | ((page&0xFF) << 8) | (reg&0xFF))
#define AIC_TO_ADDR(x)  (*(aic_addr *)&x)
#define AIC3212_SELECT_PAGE_REG 0x00
#define AIC3212_SELECT_BOOK_REG 0x7F

/* AIC3212 registers list */
#define AIC3212_PAGE_SELECT_REGISTER AIC_TO_PACKED_ADDR(0,0,0)
#define AIC3212_SOFTWARE_RESET_REGISTER AIC_TO_PACKED_ADDR(0,0,1)
#define AIC3212_CLOCK_CONTROL_REGISTER_1_CLOCK_INPUT_MULTIPLEXERS AIC_TO_PACKED_ADDR(0,0,4)
#define AIC3212_CLOCK_CONTROL_REGISTER_2_PLL_INPUT_MULTIPLEXER AIC_TO_PACKED_ADDR(0,0,5)
#define AIC3212_CLOCK_CONTROL_REGISTER_3_PLL_P_AND_R_VALUES AIC_TO_PACKED_ADDR(0,0,6)
#define AIC3212_CLOCK_CONTROL_REGISTER_4_PLL_J_VALUE AIC_TO_PACKED_ADDR(0,0,7)
#define AIC3212_CLOCK_CONTROL_REGISTER_5_PLL_D_VALUES_MSB AIC_TO_PACKED_ADDR(0,0,8)
#define AIC3212_CLOCK_CONTROL_REGISTER_6_PLL_D_VALUES_LSB AIC_TO_PACKED_ADDR(0,0,9)
#define AIC3212_CLOCK_CONTROL_REGISTER_7_PLL_CLKIN_DIVIDER AIC_TO_PACKED_ADDR(0,0,10)
#define AIC3212_CLOCK_CONTROL_REGISTER_8_NDAC_DIVIDER_VALUES AIC_TO_PACKED_ADDR(0,0,11)
#define AIC3212_CLOCK_CONTROL_REGISTER_9_MDAC_DIVIDER_VALUES AIC_TO_PACKED_ADDR(0,0,12)
#define AIC3212_DAC_OSR_CONTROL_REGISTER_1_MSB_VALUE AIC_TO_PACKED_ADDR(0,0,13)
#define AIC3212_DAC_OSR_CONTROL_REGISTER_2_LSB_VALUE AIC_TO_PACKED_ADDR(0,0,14)
#define AIC3212_CLOCK_CONTROL_REGISTER_10_NADC_VALUES AIC_TO_PACKED_ADDR(0,0,18)
#define AIC3212_CLOCK_CONTROL_REGISTER_11_MADC_VALUES AIC_TO_PACKED_ADDR(0,0,19)
#define AIC3212_ADC_OVERSAMPLING_AOSR_REGISTER AIC_TO_PACKED_ADDR(0,0,20)
#define AIC3212_CLKOUT_MUX AIC_TO_PACKED_ADDR(0,0,21)
#define AIC3212_CLOCK_CONTROL_REGISTER_12_CLKOUT_M_DIVIDER_VALUE AIC_TO_PACKED_ADDR(0,0,22)
#define AIC3212_TIMER_CLOCK AIC_TO_PACKED_ADDR(0,0,23)
#define AIC3212_LOW_FREQUENCY_CLOCK_GENERATION_CONTROL AIC_TO_PACKED_ADDR(0,0,24)
#define AIC3212_HIGH_FREQUENCY_CLOCK_GENERATION_CONTROL_1 AIC_TO_PACKED_ADDR(0,0,25)
#define AIC3212_HIGH_FREQUENCY_CLOCK_GENERATION_CONTROL_2 AIC_TO_PACKED_ADDR(0,0,26)
#define AIC3212_HIGH_FREQUENCY_CLOCK_GENERATION_CONTROL_3 AIC_TO_PACKED_ADDR(0,0,27)
#define AIC3212_HIGH_FREQUENCY_CLOCK_GENERATION_CONTROL_4 AIC_TO_PACKED_ADDR(0,0,28)
#define AIC3212_HIGH_FREQUENCY_CLOCK_TRIM_CONTROL_1 AIC_TO_PACKED_ADDR(0,0,29)
#define AIC3212_HIGH_FREQUENCY_CLOCK_TRIM_CONTROL_2 AIC_TO_PACKED_ADDR(0,0,30)
#define AIC3212_HIGH_FREQUENCY_CLOCK_TRIM_CONTROL_3 AIC_TO_PACKED_ADDR(0,0,31)
#define AIC3212_HIGH_FREQUENCY_CLOCK_TRIM_CONTROL_4 AIC_TO_PACKED_ADDR(0,0,32)
#define AIC3212_ADC_FLAG_REGISTER AIC_TO_PACKED_ADDR(0,0,36)
#define AIC3212_DAC_FLAG_REGISTER_1 AIC_TO_PACKED_ADDR(0,0,37)
#define AIC3212_DAC_FLAG_REGISTER_2 AIC_TO_PACKED_ADDR(0,0,38)
#define AIC3212_STICKY_FLAG_REGISTER_1 AIC_TO_PACKED_ADDR(0,0,42)
#define AIC3212_INTERRUPT_FLAG_REGISTER_1 AIC_TO_PACKED_ADDR(0,0,43)
#define AIC3212_STICKY_FLAG_REGISTER_2 AIC_TO_PACKED_ADDR(0,0,44)
#define AIC3212_STICKY_FLAG_REGISTER_3 AIC_TO_PACKED_ADDR(0,0,45)
#define AIC3212_INTERRUPT_FLAG_REGISTER_2 AIC_TO_PACKED_ADDR(0,0,46)
#define AIC3212_INTERRUPT_FLAG_REGISTER_3    AIC_TO_PACKED_ADDR(0,0,47)
#define AIC3212_INT1_INTERRUPT_CONTROL AIC_TO_PACKED_ADDR(0,0,48)
#define AIC3212_INT2_INTERRUPT_CONTROL AIC_TO_PACKED_ADDR(0,0,49)
#define AIC3212_SAR_CONTROL_1 AIC_TO_PACKED_ADDR(0,0,50)
#define AIC3212_INTERRUPT_FORMAT_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,0,51)
#define AIC3212_DAC_PROCESSING_BLOCK_CONTROL AIC_TO_PACKED_ADDR(0,0,60)
#define AIC3212_ADC_PROCESSING_BLOCK_CONTROL AIC_TO_PACKED_ADDR(0,0,61)
#define AIC3212_PRIMARY_DAC_POWER_AND_SOFT_STEPPING_CONTROL AIC_TO_PACKED_ADDR(0,0,63)
#define AIC3212_PRIMARY_DAC_MASTER_VOLUME_CONFIGURATION AIC_TO_PACKED_ADDR(0,0,64)
#define AIC3212_PRIMARY_DAC_LEFT_VOLUME_CONTROL_SETTING AIC_TO_PACKED_ADDR(0,0,65)
#define AIC3212_PRIMARY_DAC_RIGHT_VOLUME_CONTROL_SETTING AIC_TO_PACKED_ADDR(0,0,66)
#define AIC3212_HEADSET_DETECTION AIC_TO_PACKED_ADDR(0,0,67)
#define AIC3212_DRC_CONTROL_REGISTER_1 AIC_TO_PACKED_ADDR(0,0,68)
#define AIC3212_DRC_CONTROL_REGISTER_2 AIC_TO_PACKED_ADDR(0,0,69)
#define AIC3212_DRC_CONTROL_REGISTER_3 AIC_TO_PACKED_ADDR(0,0,70)
#define AIC3212_BEEP_GENERATOR_REGISTER_1 AIC_TO_PACKED_ADDR(0,0,71)
#define AIC3212_BEEP_GENERATOR_REGISTER_2 AIC_TO_PACKED_ADDR(0,0,72)
#define AIC3212_BEEP_GENERATOR_REGISTER_3 AIC_TO_PACKED_ADDR(0,0,73)
#define AIC3212_BEEP_GENERATOR_REGISTER_4 AIC_TO_PACKED_ADDR(0,0,74)
#define AIC3212_BEEP_GENERATOR_REGISTER_5 AIC_TO_PACKED_ADDR(0,0,75)
#define AIC3212_BEEP_SIN_X_MSB AIC_TO_PACKED_ADDR(0,0,76)
#define AIC3212_BEEP_SIN_X_LSB AIC_TO_PACKED_ADDR(0,0,77)
#define AIC3212_BEEP_COS_X_MSB AIC_TO_PACKED_ADDR(0,0,78)
#define AIC3212_BEEP_COS_X_LSB AIC_TO_PACKED_ADDR(0,0,79)
#define AIC3212_ADC_CHANNEL_POWER_CONTROL AIC_TO_PACKED_ADDR(0,0,81)
#define AIC3212_ADC_FINE_GAIN_VOLUME_CONTROL AIC_TO_PACKED_ADDR(0,0,82)
#define AIC3212_LEFT_ADC_VOLUME_CONTROL AIC_TO_PACKED_ADDR(0,0,83)
#define AIC3212_RIGHT_ADC_VOLUME_CONTROL AIC_TO_PACKED_ADDR(0,0,84)
#define AIC3212_ADC_PHASE_CONTROL AIC_TO_PACKED_ADDR(0,0,85)
#define AIC3212_LEFT_AGC_CONTROL_1 AIC_TO_PACKED_ADDR(0,0,86)
#define AIC3212_LEFT_AGC_CONTROL_2 AIC_TO_PACKED_ADDR(0,0,87)
#define AIC3212_LEFT_AGC_CONTROL_3 AIC_TO_PACKED_ADDR(0,0,88)
#define AIC3212_LEFT_AGC_ATTACK_TIME AIC_TO_PACKED_ADDR(0,0,89)
#define AIC3212_LEFT_AGC_DECAY_TIME AIC_TO_PACKED_ADDR(0,0,90)
#define AIC3212_LEFT_AGC_NOISE_DEBOUNCE AIC_TO_PACKED_ADDR(0,0,91)
#define AIC3212_LEFT_AGC_SIGNAL_DEBOUNCE AIC_TO_PACKED_ADDR(0,0,92)
#define AIC3212_LEFT_AGC_GAIN AIC_TO_PACKED_ADDR(0,0,93)
#define AIC3212_RIGHT_AGC_CONTROL_1 AIC_TO_PACKED_ADDR(0,0,94)
#define AIC3212_RIGHT_AGC_CONTROL_2 AIC_TO_PACKED_ADDR(0,0,95)
#define AIC3212_RIGHT_AGC_CONTROL_3 AIC_TO_PACKED_ADDR(0,0,96)
#define AIC3212_RIGHT_AGC_ATTACK_TIME AIC_TO_PACKED_ADDR(0,0,97)
#define AIC3212_RIGHT_AGC_DECAY_TIME AIC_TO_PACKED_ADDR(0,0,98)
#define AIC3212_RIGHT_AGC_NOISE_DEBOUNCE     AIC_TO_PACKED_ADDR(0,0,99)
#define AIC3212_RIGHT_AGC_SIGNAL_DEBOUNCE AIC_TO_PACKED_ADDR(0,0,100)
#define AIC3212_RIGHT_AGC_GAIN AIC_TO_PACKED_ADDR(0,0,101)
#define AIC3212_ADC_DC_MEASUREMENT_CONTROL_REGISTER_1 AIC_TO_PACKED_ADDR(0,0,102)
#define AIC3212_ADC_DC_MEASUREMENT_CONTROL_REGISTER_2 AIC_TO_PACKED_ADDR(0,0,103)
#define AIC3212_LEFT_CHANNEL_DC_MEASUREMENT_OUTPUT_REGISTER_1_MSB_BYTE AIC_TO_PACKED_ADDR(0,0,104)
#define AIC3212_LEFT_CHANNEL_DC_MEASUREMENT_OUTPUT_REGISTER_2_MIDDLE_BYTE AIC_TO_PACKED_ADDR(0,0,105)
#define AIC3212_LEFT_CHANNEL_DC_MEASUREMENT_OUTPUT_REGISTER_3_LSB_BYTE AIC_TO_PACKED_ADDR(0,0,106)
#define AIC3212_RIGHT_CHANNEL_DC_MEASUREMENT_OUTPUT_REGISTER_1_MSB_BYTE AIC_TO_PACKED_ADDR(0,0,107)
#define AIC3212_RIGHT_CHANNEL_DC_MEASUREMENT_OUTPUT_REGISTER_2_MIDDLE_BYTE AIC_TO_PACKED_ADDR(0,0,108)
#define AIC3212_RIGHT_CHANNEL_DC_MEASUREMENT_OUTPUT_REGISTER_3_LSB_BYTE AIC_TO_PACKED_ADDR(0,0,109)
#define AIC3212_I2C_INTERFACE_MISCELLANEOUS_CONTROL AIC_TO_PACKED_ADDR(0,0,115)
#define AIC3212_BOOK_SELECTION_REGISTER AIC_TO_PACKED_ADDR(0,0,127)
#define AIC3212_POWER_CONFIGURATION_REGISTER AIC_TO_PACKED_ADDR(0,1,1)
#define AIC3212_LEFT_DAC_POWERTUNE_CONFIGURATION_REGISTER AIC_TO_PACKED_ADDR(0,1,3)
#define AIC3212_RIGHT_DAC_POWERTUNE_CONFIGURATION_REGISTER AIC_TO_PACKED_ADDR(0,1,4)
#define AIC3212_COMMON_MODE_REGISTER AIC_TO_PACKED_ADDR(0,1,8)
#define AIC3212_HEADPHONE_OUTPUT_DRIVER_CONTROL AIC_TO_PACKED_ADDR(0,1,9)
#define AIC3212_RECEIVER_OUTPUT_DRIVER_CONTROL AIC_TO_PACKED_ADDR(0,1,10)
#define AIC3212_HEADPHONE_OUTPUT_DRIVER_DE_POP_CONTROL AIC_TO_PACKED_ADDR(0,1,11)
#define AIC3212_RECEIVER_OUTPUT_DRIVER_DE_POP_CONTROL AIC_TO_PACKED_ADDR(0,1,12)
#define AIC3212_MIXER_AMPLIFIER_CONTROL AIC_TO_PACKED_ADDR(0,1,17)
#define AIC3212_LEFT_ADC_PGA_TO_LEFT_MIXER_AMPLIFIER_MAL_VOLUME_CONTROL AIC_TO_PACKED_ADDR(0,1,18)
#define AIC3212_RIGHT_ADC_PGA_TO_RIGHT_MIXER_AMPLIFIER_MAR_VOLUME_CONTROL AIC_TO_PACKED_ADDR(0,1,19)
#define AIC3212_LINEOUT_AMPLIFIER_CONTROL_1 AIC_TO_PACKED_ADDR(0,1,22)
#define AIC3212_LINEOUT_AMPLIFIER_CONTROL_2 AIC_TO_PACKED_ADDR(0,1,23)
#define AIC3212_HEADPHONE_AMPLIFIER_CONTROL_1 AIC_TO_PACKED_ADDR(0,1,27)
#define AIC3212_HEADPHONE_AMPLIFIER_CONTROL_2 AIC_TO_PACKED_ADDR(0,1,28)
#define AIC3212_HEADPHONE_AMPLIFIER_CONTROL_3 AIC_TO_PACKED_ADDR(0,1,29)
#define AIC3212_HPL_DRIVER_VOLUME_CONTROL AIC_TO_PACKED_ADDR(0,1,31)
#define AIC3212_HPR_DRIVER_VOLUME_CONTROL AIC_TO_PACKED_ADDR(0,1,32)
#define AIC3212_CHARGE_PUMP_CONTROL_1 AIC_TO_PACKED_ADDR(0,1,33)
#define AIC3212_CHARGE_PUMP_CONTROL_2    AIC_TO_PACKED_ADDR(0,1,34)
#define AIC3212_CHARGE_PUMP_CONTROL_3 AIC_TO_PACKED_ADDR(0,1,35)
#define AIC3212_RECEIVER_AMPLIFIER_CONTROL_1 AIC_TO_PACKED_ADDR(0,1,36)
#define AIC3212_RECEIVER_AMPLIFIER_CONTROL_2 AIC_TO_PACKED_ADDR(0,1,37)
#define AIC3212_RECEIVER_AMPLIFIER_CONTROL_3 AIC_TO_PACKED_ADDR(0,1,38)
#define AIC3212_RECEIVER_AMPLIFIER_CONTROL_4 AIC_TO_PACKED_ADDR(0,1,39)
#define AIC3212_RECEIVER_AMPLIFIER_CONTROL_5 AIC_TO_PACKED_ADDR(0,1,40)
#define AIC3212_RECEIVER_AMPLIFIER_CONTROL_6 AIC_TO_PACKED_ADDR(0,1,41)
#define AIC3212_RECEIVER_AMPLIFIER_CONTROL_7 AIC_TO_PACKED_ADDR(0,1,42)
#define AIC3212_SPEAKER_AMPLIFIER_CONTROL_1 AIC_TO_PACKED_ADDR(0,1,45)
#define AIC3212_SPEAKER_AMPLIFIER_CONTROL_2 AIC_TO_PACKED_ADDR(0,1,46)
#define AIC3212_SPEAKER_AMPLIFIER_CONTROL_3 AIC_TO_PACKED_ADDR(0,1,47)
#define AIC3212_SPEAKER_AMPLIFIER_VOLUME_CONTROLS AIC_TO_PACKED_ADDR(0,1,48)
#define AIC3212_MICROPHONE_BIAS_CONTROL AIC_TO_PACKED_ADDR(0,1,51)
#define AIC3212_INPUT_SELECT_1_FOR_LEFT_MICROPHONE_PGA_P_TERMINAL AIC_TO_PACKED_ADDR(0,1,52)
#define AIC3212_INPUT_SELECT_2_FOR_LEFT_MICROPHONE_PGA_P_TERMINAL AIC_TO_PACKED_ADDR(0,1,53)
#define AIC3212_INPUT_SELECT_FOR_LEFT_MICROPHONE_PGA_M_TERMINAL AIC_TO_PACKED_ADDR(0,1,54)
#define AIC3212_INPUT_SELECT_1_FOR_RIGHT_MICROPHONE_PGA_P_TERMINAL AIC_TO_PACKED_ADDR(0,1,55)
#define AIC3212_INPUT_SELECT_2_FOR_RIGHT_MICROPHONE_PGA_P_TERMINAL AIC_TO_PACKED_ADDR(0,1,56)
#define AIC3212_INPUT_SELECT_FOR_RIGHT_MICROPHONE_PGA_M_TERMINAL AIC_TO_PACKED_ADDR(0,1,57)
#define AIC3212_INPUT_COMMON_MODE_CONTROL AIC_TO_PACKED_ADDR(0,1,58)
#define AIC3212_LEFT_MICROPHONE_PGA_CONTROL AIC_TO_PACKED_ADDR(0,1,59)
#define AIC3212_RIGHT_MICROPHONE_PGA_CONTROL AIC_TO_PACKED_ADDR(0,1,60)
#define AIC3212_ADC_POWERTUNE_CONFIGURATION_REGISTER AIC_TO_PACKED_ADDR(0,1,61)
#define AIC3212_ADC_ANALOG_PGA_GAIN_FLAG_REGISTER AIC_TO_PACKED_ADDR(0,1,62)
#define AIC3212_DAC_ANALOG_GAIN_FLAGS_REGISTER_1 AIC_TO_PACKED_ADDR(0,1,63)
#define AIC3212_DAC_ANALOG_GAIN_FLAGS_REGISTER_2 AIC_TO_PACKED_ADDR(0,1,64)
#define AIC3212_ANALOG_BYPASS_GAIN_FLAGS_REGISTER AIC_TO_PACKED_ADDR(0,1,65)
#define AIC3212_DRIVER_POWER_UP_FLAGS_REGISTER AIC_TO_PACKED_ADDR(0,1,66)
#define AIC3212_HEADSET_DETECTION_TUNING_REGISTER_1 AIC_TO_PACKED_ADDR(0,1,119)
#define AIC3212_HEADSET_DETECTION_TUNING_REGISTER_2 AIC_TO_PACKED_ADDR(0,1,120)
#define AIC3212_MICROPHONE_PGA_POWER_UP_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,1,121)
#define AIC3212_REFERENCE_POWERUP_DELAY_REGISTER AIC_TO_PACKED_ADDR(0,1,122)
#define AIC3212_PRIMARY_SAR_ADC_CONTROL AIC_TO_PACKED_ADDR(0,3,2)
#define AIC3212_PRIMARY_SAR_ADC_CONVERSION_MODE AIC_TO_PACKED_ADDR(0,3,3)
#define AIC3212_SAR_REFERENCE_CONTROL    AIC_TO_PACKED_ADDR(0,3,6)
#define AIC3212_SAR_ADC_FLAGS_REGISTER_1 AIC_TO_PACKED_ADDR(0,3,9)
#define AIC3212_SAR_ADC_FLAGS_REGISTER_2 AIC_TO_PACKED_ADDR(0,3,10)
#define AIC3212_SAR_ADC_BUFFER_MODE_CONTROL AIC_TO_PACKED_ADDR(0,3,13)
#define AIC3212_SCAN_MODE_TIMER_CONTROL AIC_TO_PACKED_ADDR(0,3,15)
#define AIC3212_SAR_ADC_CLOCK_CONTROL AIC_TO_PACKED_ADDR(0,3,17)
#define AIC3212_SAR_ADC_BUFFER_MODE_DATA_READ_CONTROL AIC_TO_PACKED_ADDR(0,3,18)
#define AIC3212_SAR_ADC_MEASUREMENT_CONTROL AIC_TO_PACKED_ADDR(0,3,19)
#define AIC3212_SAR_ADC_MEASUREMENT_THRESHOLD_FLAGS AIC_TO_PACKED_ADDR(0,3,21)
#define AIC3212_IN1L_MAX_THRESHOLD_CHECK_CONTROL_1 AIC_TO_PACKED_ADDR(0,3,22)
#define AIC3212_IN1L_MAX_THRESHOLD_CHECK_CONTROL_2 AIC_TO_PACKED_ADDR(0,3,23)
#define AIC3212_IN1L_MIN_THRESHOLD_CHECK_CONTROL_1 AIC_TO_PACKED_ADDR(0,3,24)
#define AIC3212_IN1L_MIN_THRESHOLD_CHECK_CONTROL_2 AIC_TO_PACKED_ADDR(0,3,25)
#define AIC3212_IN1R_MAX_THRESHOLD_CHECK_CONTROL_1 AIC_TO_PACKED_ADDR(0,3,26)
#define AIC3212_IN1R_MAX_THRESHOLD_CHECK_CONTROL_2 AIC_TO_PACKED_ADDR(0,3,27)
#define AIC3212_IN1R_MIN_THRESHOLD_CHECK_CONTROL_1 AIC_TO_PACKED_ADDR(0,3,28)
#define AIC3212_IN1R_MIN_THRESHOLD_CHECK_CONTROL_2 AIC_TO_PACKED_ADDR(0,3,29)
#define AIC3212_TEMP_MAX_THRESHOLD_CHECK_CONTROL_1 AIC_TO_PACKED_ADDR(0,3,30)
#define AIC3212_TEMP_MAX_THRESHOLD_CHECK_CONTROL_2 AIC_TO_PACKED_ADDR(0,3,31)
#define AIC3212_TEMP_MIN_THRESHOLD_CHECK_CONTROL_1 AIC_TO_PACKED_ADDR(0,3,32)
#define AIC3212_TEMP_MIN_THRESHOLD_CHECK_CONTROL_2 AIC_TO_PACKED_ADDR(0,3,33)
#define AIC3212_IN1L_MEASUREMENT_DATA_MSB AIC_TO_PACKED_ADDR(0,3,54)
#define AIC3212_IN1L_MEASUREMENT_DATA_LSB AIC_TO_PACKED_ADDR(0,3,55)
#define AIC3212_IN1R_MEASUREMENT_DATA_MSB AIC_TO_PACKED_ADDR(0,3,56)
#define AIC3212_IN1R_MEASUREMENT_DATA_LSB AIC_TO_PACKED_ADDR(0,3,57)
#define AIC3212_VBAT_MEASUREMENT_DATA_MSB AIC_TO_PACKED_ADDR(0,3,58)
#define AIC3212_VBAT_MEASUREMENT_DATA_LSB AIC_TO_PACKED_ADDR(0,3,59)
#define AIC3212_TEMP1_MEASUREMENT_DATA_MSB AIC_TO_PACKED_ADDR(0,3,66)
#define AIC3212_TEMP1_MEASUREMENT_DATA_LSB AIC_TO_PACKED_ADDR(0,3,67)
#define AIC3212_TEMP2_MEASUREMENT_DATA_MSB AIC_TO_PACKED_ADDR(0,3,68)
#define AIC3212_TEMP2_MEASUREMENT_DATA_LSB AIC_TO_PACKED_ADDR(0,3,69)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_AUDIO_BUS_FORMAT_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,1)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_LEFT_CH_OFFSET_1_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,2)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_RIGHT_CH_OFFSET_2_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,3)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_CHANNEL_SETUP_REGISTER AIC_TO_PACKED_ADDR(0,4,4)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_ADC_INPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,7)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_DAC_OUTPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,8)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_CONTROL_REGISTER_9_ADC_SLOT_TRISTATE_CONTROL AIC_TO_PACKED_ADDR(0,4,9)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_WCLK_AND_BCLK_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,10)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_BIT_CLOCK_N_DIVIDER_INPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,11)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_BIT_CLOCK_N_DIVIDER AIC_TO_PACKED_ADDR(0,4,12)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_WORD_CLOCK_N_DIVIDER AIC_TO_PACKED_ADDR(0,4,13)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_BCLK_AND_WCLK_OUTPUT AIC_TO_PACKED_ADDR(0,4,14)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_DATA_OUTPUT AIC_TO_PACKED_ADDR(0,4,15)
#define AIC3212_AUDIO_SERIAL_INTERFACE_1_ADC_WCLK_AND_BCLK_CONTROL AIC_TO_PACKED_ADDR(0,4,16)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_AUDIO_BUS_FORMAT_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,17)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_DATA_OFFSET_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,18)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_ADC_INPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,23)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_DAC_OUTPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,24)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_WCLK_AND_BCLK_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,26)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_BIT_CLOCK_N_DIVIDER_INPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,27)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_BIT_CLOCK_N_DIVIDER AIC_TO_PACKED_ADDR(0,4,28)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_WORD_CLOCK_N_DIVIDER AIC_TO_PACKED_ADDR(0,4,29)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_BCLK_AND_WCLK_OUTPUT AIC_TO_PACKED_ADDR(0,4,30)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_DATA_OUTPUT AIC_TO_PACKED_ADDR(0,4,31)
#define AIC3212_AUDIO_SERIAL_INTERFACE_2_ADC_WCLK_AND_BCLK_CONTROL AIC_TO_PACKED_ADDR(0,4,32)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_AUDIO_BUS_FORMAT_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,33)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_DATA_OFFSET_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,34)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_ADC_INPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,39)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_DAC_OUTPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,40)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_WCLK_AND_BCLK_CONTROL_REGISTER AIC_TO_PACKED_ADDR(0,4,42)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_BIT_CLOCK_N_DIVIDER_INPUT_CONTROL AIC_TO_PACKED_ADDR(0,4,43)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_BIT_CLOCK_N_DIVIDER AIC_TO_PACKED_ADDR(0,4,44)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_WORD_CLOCK_N_DIVIDER AIC_TO_PACKED_ADDR(0,4,45)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_BCLK_AND_WCLK_OUTPUT AIC_TO_PACKED_ADDR(0,4,46)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_DATA_OUTPUT AIC_TO_PACKED_ADDR(0,4,47)
#define AIC3212_AUDIO_SERIAL_INTERFACE_3_ADC_WCLK_AND_BCLK_CONTROL AIC_TO_PACKED_ADDR(0,4,48)
#define AIC3212_WCLK1_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,65)
#define AIC3212_DOUT1_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,67)
#define AIC3212_DIN1_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,68)
#define AIC3212_WCLK2_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,69)
#define AIC3212_BCLK2_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,70)
#define AIC3212_DOUT2_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,71)
#define AIC3212_DIN2_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,72)
#define AIC3212_WCLK3_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,73)
#define AIC3212_BCLK3_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,74)
#define AIC3212_DOUT3_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,75)
#define AIC3212_DIN3_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,76)
#define AIC3212_MCLK2_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,82)
#define AIC3212_GPIO1_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,86)
#define AIC3212_GPIO2_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,87)
#define AIC3212_GPI1_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,91)
#define AIC3212_GPI2_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,92)
#define AIC3212_GPO1_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,96)
#define AIC3212_DIGITAL_MICROPHONE_INPUT_PIN_CONTROL AIC_TO_PACKED_ADDR(0,4,101)
#define AIC3212_ADC_DAC_DATA_PORT_CONTROL AIC_TO_PACKED_ADDR(0,4,118)
#define AIC3212_DIGITAL_AUDIO_ENGINE_SYNCHRONIZATION_CONTROL AIC_TO_PACKED_ADDR(0,4,119)
#define AIC3212_SAR_BUFFER_MODE_DATA_MSB_AND_BUFFER_FLAGS AIC_TO_PACKED_ADDR(0,252,1)
#define AIC3212_SAR_BUFFER_MODE_DATA_LSB AIC_TO_PACKED_ADDR(0,252,2)
#define AIC3212_ADC_ADAPTIVE_CRAM_CONFIGURATION_REGISTER AIC_TO_PACKED_ADDR(40,0,1)



/***********************************************************************/
/*                     ENUMERATOR DEFINITIONS.                         */
/***********************************************************************/

/***********************************************************************/
/*                       Typedef Declarations                          */
/***********************************************************************/

typedef struct aic3212_dev aic3212_dev_t;

typedef uint32_t(* aic3212_spi_init_cb)(aic3212_dev_t * dev);
typedef uint32_t(* aic3212_spi_transceiver_cb)(aic3212_dev_t * dev, uint8_t * tx_buff, uint8_t * rx_buff, size_t tx_len);
typedef uint32_t(* aic3212_spi_remove_cb)(aic3212_dev_t * dev);

/******************************************************************************/
/************************ Data Structure Declarations ******************************/
/******************************************************************************/

#pragma pack(push,1)
typedef struct{
    uint8_t reg;
    uint8_t page;
    uint8_t book;
    uint8_t dummy;
} aic_addr;
#pragma pack(pop)

struct aic3212_dev {
    struct aic3212_ins_t * ins;

    /* Device Settings */
    aic_addr current_address;

    /* Device SPI functions */
    aic3212_spi_init_cb  aic3212_spi_init;
    aic3212_spi_transceiver_cb  aic3212_spi_transceiver;
    aic3212_spi_remove_cb  aic3212_spi_remove;
};

struct aic3212_init_param {
    struct aic3212_ins_t * ins;


    /* Device Settings */
    aic3212_spi_init_cb  aic3212_spi_init;
    aic3212_spi_transceiver_cb  aic3212_spi_transceiver;
    aic3212_spi_remove_cb  aic3212_spi_remove;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

uint32_t    aic3212_setup(struct aic3212_dev **device,
                          struct aic3212_init_param * init_param);
uint8_t     aic3212_read(struct aic3212_dev *dev, uint32_t packed_addr);
void        aic3212_write(struct aic3212_dev *dev, uint32_t packed_addr, uint8_t data);

#endif
