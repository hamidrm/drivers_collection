/*
 * ublox.h
 *
 *  Created on: Feb 25, 2019
 *      Author: mehrabian
 */

#ifndef UBLOX_INC_UBLOX_H_
#define UBLOX_INC_UBLOX_H_

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************/
/*                     MACRO DEFINITIONS.                              */
/***********************************************************************/

#define UBLOX_MSG_AID_ALPSRV_MAX_DATA_SIZE 32
#define UBLOX_MSG_AID_ALPSRV_MAX_SIZE 16
#define UBLOX_MSG_AID_ALP_MAX_SIZE 16
#define UBLOX_MSG_CFG_GNSS_BLOCKS_MAX_SIZE 16
#define UBLOX_MSG_CFG_INFO_BLOCKS_MAX_SIZE 16
#define UBLOX_MSG_CFG_RINV_MAX_SIZE 16
#define UBLOX_MSG_NAV_DGPS_DATA_MAX_SIZE 32
#define UBLOX_MSG_NAV_SBAS_DATA_MAX_SIZE 32
#define UBLOX_MSG_NAV_SVINFO_DATA_MAX_SIZE 32
#define MAX_CFG_MSG_TYPES 256
#define MAX_CFG_MSG_ACK_TIMEOUT_SEC 5
#define UBLOX_MAX_RX_BUFFER_SIZE    64

/* UBX messages format */
#define UBLOX_UBX_SYNC_CHAR_A    0xB5
#define UBLOX_UBX_SYNC_CHAR_B    0x62
#define UBLOX_UBX_CHKSUM_LEN    2
#define UBLOX_UBX_LEN_LEN    2
#define UBLOX_UBX_CLASS_ID_LEN    1
#define UBLOX_UBX_MSG_ID_LEN    1
#define UBLOX_UBX_HEADER_LEN    2

/* UBX messages offset */
#define UBLOX_UBX_SYNC_CHAR_A_OFFSET    0
#define UBLOX_UBX_SYNC_CHAR_B_OFFSET    1
#define UBLOX_UBX_CLASS_ID_OFFSET    2
#define UBLOX_UBX_MSG_ID_OFFSET    3
#define UBLOX_UBX_LEN_OFFSET    4
#define UBLOX_UBX_PAYLOAD_OFFSET    6


/* UBLOX AID flags */
#define UBLOX_UBX_AID_INI_FLAGS_POS (1<<0)
#define UBLOX_UBX_AID_INI_FLAGS_TIME (1<<1)
#define UBLOX_UBX_AID_INI_FLAGS_CLOCK_D (1<<2)
#define UBLOX_UBX_AID_INI_FLAGS_TP (1<<3)
#define UBLOX_UBX_AID_INI_FLAGS_CLOCK_F (1<<4)
#define UBLOX_UBX_AID_INI_FLAGS_LLA (1<<5)
#define UBLOX_UBX_AID_INI_FLAGS_ALT_INV (1<<6)
#define UBLOX_UBX_AID_INI_FLAGS_PREV_TM (1<<7)
#define UBLOX_UBX_AID_INI_FLAGS_UTC (1<<10)

#define UBLOX_UBX_AID_INI_TM_CFG_F_EDGE (1<<1)
#define UBLOX_UBX_AID_INI_TM_CFG_TM_1 (1<<4)
#define UBLOX_UBX_AID_INI_TM_CFG_F_1 (1<<6)

#define UBLOX_UBX_AID_HUI_FLAGS_HEALTH_VALID (1<<0)
#define UBLOX_UBX_AID_HUI_FLAGS_UTC_VALID (1<<1)
#define UBLOX_UBX_AID_HUI_FLAGS_KLOB_VALID (1<<2)


/* UBLOX CFG flags */
#define UBLOX_UBX_CFG_USB_FLAGS_REF_ENUM (1<<0)
#define UBLOX_UBX_CFG_USB_FLAGS_POWER_MODE (1<<1)


#define UBLOX_UBX_CFG_TP5_FLAGS_ACTIVE (1<<0)
#define UBLOX_UBX_CFG_TP5_FLAGS_LOCK_GPS_FREQ (1<<1)
#define UBLOX_UBX_CFG_TP5_FLAGS_LOCKED_OTHER_SET (1<<2)
#define UBLOX_UBX_CFG_TP5_FLAGS_IS_FREQ (1<<3)
#define UBLOX_UBX_CFG_TP5_FLAGS_IS_LENGTH (1<<4)
#define UBLOX_UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW (1<<5)
#define UBLOX_UBX_CFG_TP5_FLAGS_POLARITY (1<<6)
#define UBLOX_UBX_CFG_TP5_FLAGS_GRID_UTC_GPS (1<<7)


#define UBLOX_UBX_CFG_SBAS_SCAN_MODE1(X) (1<<X)

#define UBLOX_UBX_CFG_SBAS_SCAN_MODE2(X) (1<<(X & 0x7F))

#define UBLOX_UBX_CFG_SBAS_USAGE_RANGE (1<<0)
#define UBLOX_UBX_CFG_SBAS_USAGE_DIFF_CORR (1<<1)
#define UBLOX_UBX_CFG_SBAS_USAGE_INTEGRITY (1<<2)

#define UBLOX_UBX_CFG_SBAS_MODE_ENABLED (1<<0)
#define UBLOX_UBX_CFG_SBAS_MODE_TEST (1<<1)

#define UBLOX_UBX_CFG_RST_HOT_START 0x0000
#define UBLOX_UBX_CFG_RST_WARM_START 0x0001
#define UBLOX_UBX_CFG_RST_COLD_START 0xFFFF


#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_EPH (1<<0)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_ALM (1<<1)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_HEALTH (1<<2)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_KLOB (1<<3)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_POS (1<<4)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_CLKD (1<<5)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_OSC (1<<6)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_UTC (1<<7)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_RTC (1<<8)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_SFDR (1<<11)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_VMON (1<<12)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_TCT (1<<13)
#define UBLOX_UBX_CFG_RST_NAV_BBR_MASK_AOP (1<<15)


#define UBLOX_UBX_CFG_RINV_FLAGS_DUMP (1<<0)
#define UBLOX_UBX_CFG_RINV_FLAGS_BINARY (1<<1)


#define UBLOX_UBX_CFG_PRT_IN_MASK_UBX     (1<<0)
#define UBLOX_UBX_CFG_PRT_IN_MASK_NMEA     (1<<1)
#define UBLOX_UBX_CFG_PRT_IN_MASK_RTCM     (1<<2)

#define UBLOX_UBX_CFG_PRT_OUT_MASK_UBX     (1<<0)
#define UBLOX_UBX_CFG_PRT_OUT_MASK_NMEA     (1<<1)

#define UBLOX_UBX_CFG_PRT_DDC_FLAGS_EXT_TX_TIMEOUT (1<<1)

#define UBLOX_UBX_CFG_PRT_DDC_OUT_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_DDC_OUT_PROTO_MASK_NMEA (1<<1)

#define UBLOX_UBX_CFG_PRT_DDC_IN_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_DDC_IN_PROTO_MASK_NMEA (1<<1)
#define UBLOX_UBX_CFG_PRT_DDC_IN_PROTO_MASK_RTCM (1<<1)

#define UBLOX_UBX_CFG_PRT_DDC_MODE_SLAVE_ADDR ((X & 0x7F)<<1)

#define UBLOX_UBX_CFG_PRT_DDC_TXREADY_EN (1<<0)
#define UBLOX_UBX_CFG_PRT_DDC_TXREADY_POL (1<<1)
#define UBLOX_UBX_CFG_PRT_DDC_TXREADY_PIN(X) ((X & 0x1F)<<2)
#define UBLOX_UBX_CFG_PRT_DDC_TXREADY_THRESH(X) ((X & 0x1FF)<<7)

#define UBLOX_UBX_CFG_PRT_SPI_FLAGS_EXT_TX_TIMEOUT (1<<1)

#define UBLOX_UBX_CFG_PRT_SPI_OUT_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_SPI_OUT_PROTO_MASK_NMEA (1<<1)

#define UBLOX_UBX_CFG_PRT_SPI_IN_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_SPI_IN_PROTO_MASK_NMEA (1<<1)
#define UBLOX_UBX_CFG_PRT_SPI_IN_PROTO_MASK_RTCM (1<<1)

#define UBLOX_UBX_CFG_PRT_SPI_MODE_SPI_MODE(X) ((X & 0x03)<<1)
#define UBLOX_UBX_CFG_PRT_SPI_MODE_FLOW_CONTROL (1<<6)
#define UBLOX_UBX_CFG_PRT_SPI_MODE_FFCNT(X) ((X & 0xFF)<<8)

#define UBLOX_UBX_CFG_PRT_SPI_TXREADY_EN (1<<0)
#define UBLOX_UBX_CFG_PRT_SPI_TXREADY_POL (1<<1)
#define UBLOX_UBX_CFG_PRT_SPI_TXREADY_PIN(X) ((X & 0x1F)<<2)
#define UBLOX_UBX_CFG_PRT_SPI_TXREADY_THRESH(X) ((X & 0x1FF)<<7)

#define UBLOX_UBX_CFG_PRT_USB_OUT_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_USB_OUT_PROTO_MASK_NMEA (1<<1)

#define UBLOX_UBX_CFG_PRT_USB_IN_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_USB_IN_PROTO_MASK_NMEA (1<<1)
#define UBLOX_UBX_CFG_PRT_USB_IN_PROTO_MASK_RTCM (1<<1)

#define UBLOX_UBX_CFG_PRT_USB_TXREADY_EN (1<<0)
#define UBLOX_UBX_CFG_PRT_USB_TXREADY_POL (1<<1)
#define UBLOX_UBX_CFG_PRT_USB_TXREADY_PIN(X) ((X & 0x1F)<<2)
#define UBLOX_UBX_CFG_PRT_USB_TXREADY_THRESH(X) ((X & 0x1FF)<<7)

#define UBLOX_UBX_CFG_PRT_UART_FLAGS_EXT_TX_TIMEOUT (1<<1)

#define UBLOX_UBX_CFG_PRT_UART_OUT_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_UART_OUT_PROTO_MASK_NMEA (1<<1)

#define UBLOX_UBX_CFG_PRT_UART_IN_PROTO_MASK_UBX (1<<0)
#define UBLOX_UBX_CFG_PRT_UART_IN_PROTO_MASK_NMEA (1<<1)
#define UBLOX_UBX_CFG_PRT_UART_IN_PROTO_MASK_RTCM (1<<1)

#define UBLOX_UBX_CFG_PRT_UART_MODE_CHAR_LEN(X) ((X & 0x03)<<6)
#define UBLOX_UBX_CFG_PRT_UART_MODE_EVEN_PARITY(X) ((X & 0x07)<<9)
#define UBLOX_UBX_CFG_PRT_UART_MODE_N_STOP_BITS(X) ((X & 0x03)<<12)

#define UBLOX_UBX_CFG_PRT_UART_TXREADY_EN (1<<0)
#define UBLOX_UBX_CFG_PRT_UART_TXREADY_POL (1<<1)
#define UBLOX_UBX_CFG_PRT_UART_TXREADY_PIN(X) ((X & 0x1F)<<2)
#define UBLOX_UBX_CFG_PRT_UART_TXREADY_THRESH(X) ((X & 0x1FF)<<7)


#define UBLOX_UBX_CFG_PM2_FLAGS_EXT_INT1_SELECT (1<<4)
#define UBLOX_UBX_CFG_PM2_FLAGS_EXT_INT_WAKE (1<<5)
#define UBLOX_UBX_CFG_PM2_FLAGS_EXT_INT_BACKUP (1<<6)
#define UBLOX_UBX_CFG_PM2_FLAGS_LIMIT_PEAK_CURR(X) ((X & 0x03)<<8)
#define UBLOX_UBX_CFG_PM2_FLAGS_WAIT_TIME_FIX (1<<10)
#define UBLOX_UBX_CFG_PM2_FLAGS_UPDATE_RTC (1<<11)
#define UBLOX_UBX_CFG_PM2_FLAGS_UPDATE_EPH (1<<12)
#define UBLOX_UBX_CFG_PM2_FLAGS_DO_NOT_ENTER_OFF (1<<16)
#define UBLOX_UBX_CFG_PM2_FLAGS_MODE(X) ((X & 0x03)<<17)


#define UBLOX_UBX_CFG_NMEA_GNSS_FILTER_GPS (1<<0)
#define UBLOX_UBX_CFG_NMEA_GNSS_FILTER_SBAS (1<<1)
#define UBLOX_UBX_CFG_NMEA_GNSS_FILTER_QZSS (1<<4)
#define UBLOX_UBX_CFG_NMEA_GNSS_FILTER_GLONASS (1<<5)

#define UBLOX_UBX_CFG_NMEA_FLAGS_COMPAT (1<<0)
#define UBLOX_UBX_CFG_NMEA_FLAGS_CONSIDER (1<<1)

#define UBLOX_UBX_CFG_NMEA_FILTER_POS (1<<0)
#define UBLOX_UBX_CFG_NMEA_FILTER_MSK_POS (1<<1)
#define UBLOX_UBX_CFG_NMEA_FILTER_TIME (1<<2)
#define UBLOX_UBX_CFG_NMEA_FILTER_DATE (1<<3)
#define UBLOX_UBX_CFG_NMEA_FILTER_GPS_ONLY (1<<4)
#define UBLOX_UBX_CFG_NMEA_FILTER_TRACK (1<<5)


#define UBLOX_UBX_CFG_NAVX_AOP_CFG_USE_AOP (1<<0)

#define UBLOX_UBX_CFG_NAVX_MASK1_MIN_MAX (1<<2)
#define UBLOX_UBX_CFG_NAVX_MASK1_MIN_CNO (1<<3)
#define UBLOX_UBX_CFG_NAVX_MASK1_INITIAL_3D_FIX (1<<6)
#define UBLOX_UBX_CFG_NAVX_MASK1_WKN_ROLL (1<<9)
#define UBLOX_UBX_CFG_NAVX_MASK1_PPP (1<<13)
#define UBLOX_UBX_CFG_NAVX_MASK1_AOP (1<<14)


#define UBLOX_UBX_CFG_NAV5_MASK_DYN (1<<0)
#define UBLOX_UBX_CFG_NAV5_MASK_MIN_EL (1<<1)
#define UBLOX_UBX_CFG_NAV5_MASK_POS_FIX_MODE (1<<2)
#define UBLOX_UBX_CFG_NAV5_MASK_DR_LIM (1<<3)
#define UBLOX_UBX_CFG_NAV5_MASK_POS_MASK (1<<4)
#define UBLOX_UBX_CFG_NAV5_MASK_TIME_MASK (1<<5)
#define UBLOX_UBX_CFG_NAV5_MASK_STATIC_HOLD_MASK (1<<6)
#define UBLOX_UBX_CFG_NAV5_MASK_DGPS_MASK (1<<7)


#define UBLOX_UBX_CFG_LOGFILTER_FLAGS_RECORD_ENABLED (1<<0)
#define UBLOX_UBX_CFG_LOGFILTER_FLAGS_PSM_ONE_PER_WAKUP_ENABLED (1<<1)
#define UBLOX_UBX_CFG_LOGFILTER_FLAGS_APLLY_ALL_FILTERS (1<<2)

#define UBLOX_UBX_CFG_ITFM_CONFIG2_ANT_SETTING(X) ((X & 0x03)<<12)


#define UBLOX_UBX_CFG_ITFM_CONFIG_BB_THRESHOLD (1<<3)
#define UBLOX_UBX_CFG_ITFM_CONFIG_CW_THRESHOLD (1<<8)
#define UBLOX_UBX_CFG_ITFM_CONFIG_ENABLE (1<<31)


#define UBLOX_UBX_CFG_INF_MSG_MASK_ERROR (1<<0)
#define UBLOX_UBX_CFG_INF_MSG_MASK_WARNING (1<<1)
#define UBLOX_UBX_CFG_INF_MSG_MASK_NOTIFIC (1<<2)
#define UBLOX_UBX_CFG_INF_MSG_MASK_DEBUG (1<<3)
#define UBLOX_UBX_CFG_INF_MSG_MASK_TEST (1<<4)

#define UBLOX_UBX_CFG_GNSS_FLAGS_ENABLE (1<<0)


#define UBLOX_UBX_CFG_CFG_DEVICE_MASK_BBR (1<<0)
#define UBLOX_UBX_CFG_CFG_DEVICE_MASK_FLASH (1<<1)
#define UBLOX_UBX_CFG_CFG_DEVICE_MASK_EEPROM (1<<2)
#define UBLOX_UBX_CFG_CFG_DEVICE_MASK_SPI_FLASH (1<<3)

#define UBLOX_UBX_CFG_CFG_CLEAR_MASK_IO_PORT (1<<0)
#define UBLOX_UBX_CFG_CFG_CLEAR_MASK_MSG_CONF (1<<1)
#define UBLOX_UBX_CFG_CFG_CLEAR_MASK_INF_MSG (1<<2)
#define UBLOX_UBX_CFG_CFG_CLEAR_MASK_NAV_CONF (1<<3)
#define UBLOX_UBX_CFG_CFG_CLEAR_MASK_RXM_CONF (1<<4)
#define UBLOX_UBX_CFG_CFG_CLEAR_MASK_RINV_CONF (1<<9)
#define UBLOX_UBX_CFG_CFG_CLEAR_MASK_ANT_CONF (1<<10)


#define UBLOX_UBX_CFG_ANT_PINS_PIN_SWITCH(X) ((X & 0x1F)<<0)
#define UBLOX_UBX_CFG_ANT_PINS_PIN_SCD(X) ((X & 0x1F)<<5)
#define UBLOX_UBX_CFG_ANT_PINS_PIN_OCD(X) ((X & 0x1F)<<10)
#define UBLOX_UBX_CFG_ANT_PINS_RECONFIG (1<<15)

#define UBLOX_UBX_CFG_ANT_FLAGS_SVCS (1<<0)
#define UBLOX_UBX_CFG_ANT_FLAGS_SCD (1<<1)
#define UBLOX_UBX_CFG_ANT_FLAGS_OCD (1<<2)
#define UBLOX_UBX_CFG_ANT_FLAGS_PDWN_ON_SCD (1<<3)
#define UBLOX_UBX_CFG_ANT_FLAGS_RECOVERY (1<<4)


/* UBLOX NAV flags */
#define UBLOX_UBX_NAV_TIMEUTC_VALID_TOW (1<<0)
#define UBLOX_UBX_NAV_TIMEUTC_VALID_WKN (1<<1)
#define UBLOX_UBX_NAV_TIMEUTC_VALID_UTC (1<<2)


#define UBLOX_UBX_NAV_TIMEGPS_VALID_TOW (1<<0)
#define UBLOX_UBX_NAV_TIMEUTC_VALID_WEEK (1<<1)
#define UBLOX_UBX_NAV_TIMEUTC_VALID_LEAP (1<<2)


#define UBLOX_UBX_NAV_SVINFO_QUALITY_IDLE 0
#define UBLOX_UBX_NAV_SVINFO_QUALITY_SEARCHING 1
#define UBLOX_UBX_NAV_SVINFO_QUALITY_SIGNAL_AQUIRED 2
#define UBLOX_UBX_NAV_SVINFO_QUALITY_SIGNAL_UNSTABLE 3
#define UBLOX_UBX_NAV_SVINFO_QUALITY_CODE_LOCK_ON_SIGNAL 4

#define UBLOX_UBX_NAV_SVINFO_FLAGS_SV_USED (1<<0)
#define UBLOX_UBX_NAV_SVINFO_FLAGS_DIFF_CORR (1<<1)
#define UBLOX_UBX_NAV_SVINFO_FLAGS_ORBIT_AVAIL (1<<2)
#define UBLOX_UBX_NAV_SVINFO_FLAGS_ORBIT_EPH (1<<3)
#define UBLOX_UBX_NAV_SVINFO_FLAGS_UNHEALTHLY (1<<4)
#define UBLOX_UBX_NAV_SVINFO_FLAGS_ORBIT_ALM (1<<5)
#define UBLOX_UBX_NAV_SVINFO_FLAGS_ORBIT_AOP (1<<6)
#define UBLOX_UBX_NAV_SVINFO_FLAGS_SMOOTHED (1<<7)

#define UBLOX_UBX_NAV_SVINFO_GLOBALFLAG_CHIPGEN_ANTARIS (1<<0)
#define UBLOX_UBX_NAV_SVINFO_GLOBALFLAG_CHIPGEN_UBLOX5 (1<<1)
#define UBLOX_UBX_NAV_SVINFO_GLOBALFLAG_CHIPGEN_UBLOX6 (1<<2)


#define UBLOX_UBX_NAV_STATUS_FLAG2_ACQUISITION 0
#define UBLOX_UBX_NAV_STATUS_FLAG2_TRACKING 1
#define UBLOX_UBX_NAV_STATUS_FLAG2_POWER_OPTIMIZED_TRACKING 2
#define UBLOX_UBX_NAV_STATUS_FLAG2_INACTIVE 3

#define UBLOX_UBX_NAV_STATUS_FIXSTAT_DGPS_ISTAT (1<<0)
#define UBLOX_UBX_NAV_STATUS_FIXSTAT_MAP_MATCHING_NONE (0<<6)
#define UBLOX_UBX_NAV_STATUS_FIXSTAT_MAP_MATCHING_VALID (1<<6)
#define UBLOX_UBX_NAV_STATUS_FIXSTAT_MAP_MATCHING_USED (2<<6)
#define UBLOX_UBX_NAV_STATUS_FIXSTAT_MAP_MATCHING_DR (3<<6)

#define UBLOX_UBX_NAV_STATUS_FLAGS_FIX_OK (1<<0)
#define UBLOX_UBX_NAV_STATUS_FLAGS_DIFF_SOLN (1<<1)
#define UBLOX_UBX_NAV_STATUS_FLAGS_WKN_SET (1<<2)
#define UBLOX_UBX_NAV_STATUS_FLAGS_TOW_SET (1<<3)


#define UBLOX_UBX_NAV_SOL_FLAGS_FIX_OK (1<<0)
#define UBLOX_UBX_NAV_SOL_FLAGS_DIFF_SOLN (1<<1)
#define UBLOX_UBX_NAV_SOL_FLAGS_WKN_SET (1<<2)
#define UBLOX_UBX_NAV_SOL_FLAGS_TOW_SET (1<<3)


#define UBLOX_UBX_NAV_SBAS_SERVICE_RANGING (1<<0)
#define UBLOX_UBX_NAV_SBAS_SERVICE_CORRECTIONS (1<<1)
#define UBLOX_UBX_NAV_SBAS_SERVICE_INTEGRITY (1<<2)
#define UBLOX_UBX_NAV_SBAS_SERVICE_TEST_MODE (1<<3)


#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_FIX_OK (1<<0)
#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_DIFF_SOLIN (1<<1)
#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE(X) ((X & 0x07)<<2)
#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE_ENABLED UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE(1)
#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE_ACQUISITION UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE(2)
#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE_TRACKING UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE(3)
#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE_POWER_OPTIMIZED_TRACKING UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE(4)
#define UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE_INACTIVE UBLOX_UBX_NAV_PVT_FLAGS_GNSS_PSM_STATE(5)

#define UBLOX_UBX_NAV_PVT_VALID_DATE (1<<0)
#define UBLOX_UBX_NAV_PVT_VALID_TIME (1<<1)
#define UBLOX_UBX_NAV_PVT_FULLY_RESOLVED (1<<2)


#define UBLOX_UBX_NAV_DGPS_FLAGS_DGPS_USED (1<<4)
#define UBLOX_UBX_NAV_DGPS_FLAGS_CHANNEL(X) ((X & 0x0F)<<0)


#define UBLOX_UBX_NAV_AOP_CFG_USE_AOP (1<<0)

/***********************************************************************/
/*                     TYPEDEF DEFINITIONS.                         */
/***********************************************************************/

/* Data types */
typedef uint8_t ublox_msg_id_t;
typedef uint8_t* ublox_ubx_packet_t;
typedef uint8_t u1_t;
typedef uint16_t u2_t;
typedef uint32_t u4_t;
typedef int8_t i1_t;
typedef int16_t i2_t;
typedef int32_t i4_t;
typedef uint8_t x1_t;
typedef uint16_t x2_t;
typedef uint32_t x4_t;
typedef float r4_t;
typedef double r8_t;
typedef char ch_t;
typedef void* ublox_instance;

/* Function Pointers */
typedef void(*ublox_msg_received_cb)(ublox_ubx_class_id_t, ublox_msg_id_t, ublox_ubx_msg_t*);
typedef void(*ublox_uart_send_cb_t)(const uint8_t *, size_t);
typedef void(*ublox_uart_update_params_cb_t)(ublox_uart_baudrate_t,ublox_uart_char_len_t,
        ublox_uart_parity_t,ublox_uart_n_stop_bits_t);
typedef void*(*ublox_malloc_cb_t)(size_t);
typedef void(*ublox_free_cb_t)(void *);

/***********************************************************************/
/*                     ENUMERATOR DEFINITIONS.                         */
/***********************************************************************/
typedef enum {
    UBLOX_PERIODIC_MSG_STATUS_DISABLE,
    UBLOX_PERIODIC_MSG_STATUS_ENABLE,
}ublox_periodic_msg_status_t;

typedef enum {
    UBLOX_TIME_PULSE,
    UBLOX_TIME_PULSE2,
}ublox_time_pulse_id_t;

typedef enum {
    UBLOX_RESET_MODE_HARDWARE_RESET_IMMEDIATELY,
    UBLOX_RESET_MODE_SOFTWARE_RESET,
    UBLOX_RESET_MODE_SOFTWARE_RESET_GNSS_ONLY,
    UBLOX_RESET_MODE_HARDWARE_RESET_AFTER_SHUTDOWN,
    UBLOX_RESET_MODE_GNSS_STOP,
    UBLOX_RESET_MODE_GNSS_START,
}ublox_reset_mode_t;


typedef enum {
    UBLOX_PERIODIC_RATE_REF_UTC_TIME,
    UBLOX_PERIODIC_RATE_REF_GPS_TIME,
}ublox_periodic_rate_time_ref_t;

typedef enum {
    UBLOX_PORT_DDC,
    UBLOX_PORT_UART1,
    UBLOX_PORT_USB,
    UBLOX_PORT_SPI,
}ublox_ports_id_t;

typedef enum {
    UBLOX_UART_BAUDRATE_4800 = 4800,
    UBLOX_UART_BAUDRATE_9600 = 9600,
    UBLOX_UART_BAUDRATE_19200 = 19200,
    UBLOX_UART_BAUDRATE_38400 = 38400,
    UBLOX_UART_BAUDRATE_57600 = 57600,
    UBLOX_UART_BAUDRATE_115200 = 115200,
}ublox_uart_baudrate_t;

typedef enum {
    UBLOX_UART_CHAR_LEN_5_BIT,
    UBLOX_UART_CHAR_LEN_6_BIT,
    UBLOX_UART_CHAR_LEN_7_BIT,
    UBLOX_UART_CHAR_LEN_8_BIT,
}ublox_uart_char_len_t;

typedef enum {
    UBLOX_UART_PARITY_EVEN_PARITY,
    UBLOX_UART_PARITY_ODD_PARITY,
    UBLOX_UART_PARITY_NO_PARITY = 4,
}ublox_uart_parity_t;

typedef enum {
    UBLOX_UART_N_STOP_BITS_1_0,
    UBLOX_UART_N_STOP_BITS_1_5,
    UBLOX_UART_N_STOP_BITS_2_0,
    UBLOX_UART_N_STOP_BITS_0_5,
}ublox_uart_n_stop_bits_t;

typedef enum {
    UBLOX_SPI_MODE_0,
    UBLOX_SPI_MODE_1,
    UBLOX_SPI_MODE_2,
    UBLOX_SPI_MODE_3,
}ublox_spi_mode_t;

typedef enum {
    UBLOX_SPI_FLOW_CONTROL_DISABLED,
    UBLOX_SPI_FLOW_CONTROL_ENABLED,
}ublox_spi_flow_control_t;

typedef enum {
    UBLOX_CFG_REQ_STATUS_IDLE,
    UBLOX_CFG_REQ_STATUS_WAITING,
    UBLOX_CFG_REQ_STATUS_TIMEOUT,
}ublox_cfg_req_status_t;

typedef enum {
    UBLOX_UBX_CLASS_ID_NAV = 0x01,
    UBLOX_UBX_CLASS_ID_RXM = 0x02,
    UBLOX_UBX_CLASS_ID_INF = 0x04,
    UBLOX_UBX_CLASS_ID_ACK = 0x05,
    UBLOX_UBX_CLASS_ID_CFG = 0x06,
    UBLOX_UBX_CLASS_ID_MON = 0x0A,
    UBLOX_UBX_CLASS_ID_AID = 0x0B,
    UBLOX_UBX_CLASS_ID_TIM = 0x0D,
    UBLOX_UBX_CLASS_ID_LOG = 0x21,
    UBLOX_UBX_CLASS_ID_NMEA_STD = 0xF0,
    UBLOX_UBX_CLASS_ID_NMEA_PUBX = 0xF1,
    /*---------------------------------
     * Manager messages
     * --------------------------------*/
    UBLOX_UBX_CLASS_ID_CFG_TIMEOUT = 0xFF,
    UBLOX_UBX_CLASS_ID_CFG_NACK = 0xFE,
    UBLOX_UBX_CLASS_ID_CFG_ACK = 0xFD,
    UBLOX_UBX_CLASS_ID_DISCONNECTED = 0xFC,
}ublox_ubx_class_id_t;

typedef enum {
    UBLOX_UBX_MSG_ID_ACK_ACK = 0x01,
    UBLOX_UBX_MSG_ID_ACK_NAK = 0x00,
}ublox_ubx_msg_ack_t;


typedef enum {
    UBLOX_UBX_MSG_ID_AID_ALM = 0x30,
    UBLOX_UBX_MSG_ID_AID_ALPSRV = 0x32,
    UBLOX_UBX_MSG_ID_AID_ALP = 0x50,
    UBLOX_UBX_MSG_ID_AID_AOP = 0x33,
    UBLOX_UBX_MSG_ID_AID_DATA = 0x10,
    UBLOX_UBX_MSG_ID_AID_EPH = 0x31,
    UBLOX_UBX_MSG_ID_AID_HUI = 0x02,
    UBLOX_UBX_MSG_ID_AID_INI = 0x01,
    UBLOX_UBX_MSG_ID_AID_REQ = 0x00,
}ublox_ubx_msg_aiding_t;

typedef enum {
    UBLOX_UBX_MSG_ID_CFG_ANT = 0x13,
    UBLOX_UBX_MSG_ID_CFG_CFG = 0x09,
    UBLOX_UBX_MSG_ID_CFG_DAT = 0x06,
    UBLOX_UBX_MSG_ID_CFG_GNSS = 0x3E,
    UBLOX_UBX_MSG_ID_CFG_INF = 0x02,
    UBLOX_UBX_MSG_ID_CFG_ITFM = 0x39,
    UBLOX_UBX_MSG_ID_CFG_LOGFILTER = 0x47,
    UBLOX_UBX_MSG_ID_CFG_MSG = 0x01,
    UBLOX_UBX_MSG_ID_CFG_NAV5 = 0x24,
    UBLOX_UBX_MSG_ID_CFG_NAVX5 = 0x23,
    UBLOX_UBX_MSG_ID_CFG_NMEA = 0x17,
    UBLOX_UBX_MSG_ID_CFG_PM2 = 0x3B,
    UBLOX_UBX_MSG_ID_CFG_PRT = 0x00,
    UBLOX_UBX_MSG_ID_CFG_RATE = 0x08,
    UBLOX_UBX_MSG_ID_CFG_RINV = 0x34,
    UBLOX_UBX_MSG_ID_CFG_RST = 0x04,
    UBLOX_UBX_MSG_ID_CFG_RXM = 0x11,
    UBLOX_UBX_MSG_ID_CFG_SBAS = 0x16,
    UBLOX_UBX_MSG_ID_CFG_TP5 = 0x31,
    UBLOX_UBX_MSG_ID_CFG_USB = 0x1B,
}ublox_ubx_msg_cfg_t;

typedef enum {
    UBLOX_UBX_MSG_ID_INF_DEBUG = 0x04,
    UBLOX_UBX_MSG_ID_INF_ERROR = 0x00,
    UBLOX_UBX_MSG_ID_INF_NOTICE = 0x02,
    UBLOX_UBX_MSG_ID_INF_TEST = 0x03,
    UBLOX_UBX_MSG_ID_INF_WARNING = 0x01,
}ublox_ubx_msg_inf_t;

typedef enum {
    UBLOX_UBX_MSG_ID_LOG_CREATE = 0x07,
    UBLOX_UBX_MSG_ID_LOG_ERASE = 0x03,
    UBLOX_UBX_MSG_ID_LOG_FINDTIME = 0x0E,
    UBLOX_UBX_MSG_ID_LOG_INFO = 0x08,
    UBLOX_UBX_MSG_ID_LOG_RETRIEVEPOS = 0x0b,
    UBLOX_UBX_MSG_ID_LOG_RETRIEVESTRING = 0x0d,
    UBLOX_UBX_MSG_ID_LOG_RETRIEVE = 0x09,
    UBLOX_UBX_MSG_ID_LOG_STRING = 0x04,
}ublox_ubx_msg_log_t;

typedef enum {
    UBLOX_UBX_MSG_ID_MON_HW2 = 0x0B,
    UBLOX_UBX_MSG_ID_MON_HW = 0x09,
    UBLOX_UBX_MSG_ID_MON_IO = 0x02,
    UBLOX_UBX_MSG_ID_MON_MSGPP = 0x06,
    UBLOX_UBX_MSG_ID_MON_RXBUF = 0x07,
    UBLOX_UBX_MSG_ID_MON_RXR = 0x21,
    UBLOX_UBX_MSG_ID_MON_TXBUF = 0x08,
    UBLOX_UBX_MSG_ID_MON_VER = 0x04,
}ublox_ubx_msg_mon_t;

typedef enum {
    UBLOX_UBX_MSG_ID_NAV_AOPSTATUS = 0x60,
    UBLOX_UBX_MSG_ID_NAV_CLOCK = 0x22,
    UBLOX_UBX_MSG_ID_NAV_DGPS = 0x31,
    UBLOX_UBX_MSG_ID_NAV_DOP = 0x04,
    UBLOX_UBX_MSG_ID_NAV_POSECEF = 0x01,
    UBLOX_UBX_MSG_ID_NAV_POSLLH = 0x02,
    UBLOX_UBX_MSG_ID_NAV_PVT = 0x07,
    UBLOX_UBX_MSG_ID_NAV_SBAS = 0x32,
    UBLOX_UBX_MSG_ID_NAV_SOL = 0x06,
    UBLOX_UBX_MSG_ID_NAV_STATUS = 0x03,
    UBLOX_UBX_MSG_ID_NAV_SVINFO = 0x30,
    UBLOX_UBX_MSG_ID_NAV_TIMEGPS = 0x20,
    UBLOX_UBX_MSG_ID_NAV_TIMEUTC = 0x21,
    UBLOX_UBX_MSG_ID_NAV_VELECEF = 0x11,
    UBLOX_UBX_MSG_ID_NAV_VELNED = 0x12,
}ublox_ubx_msg_nav_t;

typedef enum {
    UBLOX_UBX_MSG_ID_RXM_ALM = 0x30,
    UBLOX_UBX_MSG_ID_RXM_EPH = 0x31,
    UBLOX_UBX_MSG_ID_RXM_PMREQ = 0x41,
    UBLOX_UBX_MSG_ID_RXM_RAW = 0x10,
    UBLOX_UBX_MSG_ID_RXM_SFRB = 0x11,
    UBLOX_UBX_MSG_ID_RXM_SVSI = 0x20,
}ublox_ubx_msg_rxm_t;

typedef enum {
    UBLOX_UBX_MSG_ID_TIM_TM2 = 0x03,
    UBLOX_UBX_MSG_ID_TIM_TP = 0x01,
    UBLOX_UBX_MSG_ID_TIM_VRFY = 0x06,
}ublox_ubx_msg_tim_t;

typedef enum {
    UBLOX_UBX_MSG_ID_NMEA_STD_DTM = 0x0A,
    UBLOX_UBX_MSG_ID_NMEA_STD_GBS = 0x09,
    UBLOX_UBX_MSG_ID_NMEA_STD_GGA = 0x00,
    UBLOX_UBX_MSG_ID_NMEA_STD_GLL = 0x01,
    UBLOX_UBX_MSG_ID_NMEA_STD_GLQ = 0x43,
    UBLOX_UBX_MSG_ID_NMEA_STD_GNQ = 0x42,
    UBLOX_UBX_MSG_ID_NMEA_STD_GNS = 0x0D,
    UBLOX_UBX_MSG_ID_NMEA_STD_GPQ = 0x40,
    UBLOX_UBX_MSG_ID_NMEA_STD_GRS = 0x06,
    UBLOX_UBX_MSG_ID_NMEA_STD_GSA = 0x02,
    UBLOX_UBX_MSG_ID_NMEA_STD_GST = 0x07,
    UBLOX_UBX_MSG_ID_NMEA_STD_GSV = 0x03,
    UBLOX_UBX_MSG_ID_NMEA_STD_RMC = 0x04,
    UBLOX_UBX_MSG_ID_NMEA_STD_TXT = 0x41,
    UBLOX_UBX_MSG_ID_NMEA_STD_VTG = 0x05,
    UBLOX_UBX_MSG_ID_NMEA_STD_ZDA = 0x08,
}ublox_ubx_msg_nmea_std_t;

typedef enum {
    UBLOX_UBX_MSG_ID_NMEA_PUBX_CONFIG = 0x41,
    UBLOX_UBX_MSG_ID_NMEA_PUBX_POSITION = 0x00,
    UBLOX_UBX_MSG_ID_NMEA_PUBX_RATE = 0x40,
    UBLOX_UBX_MSG_ID_NMEA_PUBX_SVSTATUS = 0x03,
    UBLOX_UBX_MSG_ID_NMEA_PUBX_TIME = 0x04,
}ublox_ubx_msg_nmea_pubx_t;

typedef enum {
    UBLOX_PACKET_FSM_SYNC_CHAR_A,
    UBLOX_PACKET_FSM_SYNC_CHAR_B,
    UBLOX_PACKET_FSM_CLASS_ID,
    UBLOX_PACKET_FSM_MSG_ID,
    UBLOX_PACKET_FSM_LEN_L,
    UBLOX_PACKET_FSM_LEN_H,
    UBLOX_PACKET_FSM_PAYLOAD,
    UBLOX_PACKET_FSM_CK_A,
    UBLOX_PACKET_FSM_CK_B,
} ublox_packet_fsm_states_t;

/***********************************************************************/
/*                     DATA STRUCTURE DEFINITIONS.                     */
/***********************************************************************/
typedef struct {
    u1_t    class_id;
    u1_t    msg_id;
    u2_t    len;
    u1_t *  msg_payload;
    u1_t    ck_a;
    u1_t    ck_b;
} ublox_packet_t;

typedef struct {
    u1_t gnss_id;
    u1_t res_trk_ch;
    u1_t max_trk_ch;
    u1_t reserved_1;
    x4_t flags;
} cfg_gnss_config_block_t;

typedef struct {
    u1_t protocol_id;
    u1_t reserved0;
    u1_t reserved1;
    x1_t inf_msg_mask[6];
} cfg_inf_information_t;

typedef struct {
    u1_t sv_id;
    x1_t flags;
    u2_t age_c;
    r4_t prc;
    r4_t prrc;
} nav_dgps_data_t;

typedef struct {
    u1_t sv_id;
    u1_t flags;
    u1_t udre;
    u1_t sv_sys;
    u1_t sv_service;
    u1_t reserved1;
    i2_t prc;
    u2_t reserved2;
    i2_t ic;
} nav_sbas_data_t;

typedef struct {
    u1_t chn;
    u1_t svid;
    x1_t flags;
    x1_t quality;
    u1_t cno;
    i1_t elev;
    i2_t azim;
    i4_t pr_res;
} nav_sv_info_data_t;

typedef struct {
    union{
        union {
            struct{
                ublox_ubx_class_id_t class_id : 8;
                ublox_msg_id_t msg_id : 8;
            } ack;
            struct {
                ublox_ubx_class_id_t class_id : 8;
                ublox_msg_id_t msg_id : 8;
            } nack;
        }ublox_ubx_msg_ack;

        union {
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    uint8_t sv_id;
                } poll_sv;
                struct {
                    uint32_t sv_id;
                    uint32_t week;
                    struct {
                        uint32_t dwrd[8];
                    } optional;
                } in_out;
            } alm;
            union {
                struct {
                    uint8_t id_size;
                    uint8_t type;
                    uint16_t ofs;
                    uint16_t size;
                    uint16_t field_id;
                    uint16_t data_size;
                    uint8_t id1;
                    uint8_t id2;
                    uint32_t id3;
                } client_req;
                struct {
                    uint8_t id_size;
                    uint8_t type;
                    uint16_t ofs;
                    uint16_t size;
                    uint16_t field_id;
                    uint16_t data_size;
                    uint8_t id1;
                    uint8_t id2;
                    uint32_t id3;
                    uint8_t data[UBLOX_MSG_AID_ALPSRV_MAX_DATA_SIZE];
                } send_to_client;
                struct {
                    uint8_t id_size;
                    uint8_t type;
                    uint16_t ofs;
                    uint16_t size;
                    uint16_t field_id;
                    uint16_t data[UBLOX_MSG_AID_ALPSRV_MAX_SIZE];
                } send_to_server;
            } alpsrv;
            union {
                struct {
                    uint16_t alp_data[UBLOX_MSG_AID_ALP_MAX_SIZE];
                } file_data_transfer;
                struct {
                    uint8_t dummy;
                } end_of_data;
                struct {
                    uint8_t ack_nack;
                } acknowledge;
                struct {
                    uint32_t pred_tow;
                    uint32_t pred_dur;
                    int32_t age;
                    uint16_t pred_wno;
                    uint16_t alm_wno;
                    uint32_t rederved1;
                    uint8_t svs;
                    uint8_t rederved2;
                    uint16_t rederved3;
                } poll_status;
            }alp;
            union {
                struct {
                    /* Empty */
                }poll;
                struct {
                    uint8_t sv_id;
                } poll_for_one_satelite;
                struct {
                    uint8_t sv_id;
                    uint8_t data[59];
                    struct {
                        uint8_t optional0[48];
                        uint8_t optional1[48];
                        uint8_t optional2[48];
                    } optional;
                }data;
            }aop;
            union {
                struct {
                    /* Empty */
                } poll;
            }data;
            union {
                struct {
                    /* Empty */
                }poll;
                struct {
                    uint8_t sv_id;
                } poll_sv;
                struct {
                    uint32_t sv_id;
                    uint32_t how;
                    struct {
                        uint32_t sf1d[8];
                        uint32_t sf2d[8];
                        uint32_t sf3d[8];
                    } optional;
                }in_out;
            }eph;
            union {
                struct {
                    uint32_t health;
                    double utc_a0;
                    double utc_a1;
                    int32_t utc_tow;
                    int16_t utc_wnt;
                    int16_t utc_ls;
                    int16_t utc_wnf;
                    int16_t utc_dn;
                    int16_t utc_lsf;
                    int16_t utc_spare;
                    float klob_a0;
                    float klob_a1;
                    float klob_a2;
                    float klob_a3;
                    float klob_b0;
                    float klob_b1;
                    float klob_b2;
                    float klob_b3;
                    uint32_t flags;
                } poll;
            }hui;
            union {
                struct {
                    /* Empty */
                }poll;
                struct {
                    int32_t ecef_x_or_lat;
                    int32_t ecef_y_or_lon;
                    int32_t ecef_z_or_alt;
                    uint32_t pos_acc;
                    uint16_t tm_cfg;
                    uint16_t wno_or_date;
                    uint32_t tow_or_time;
                    int32_t tow_ns;
                    uint32_t t_acc_ms;
                    uint32_t t_acc_ns;
                    int32_t clk_d_or_freq;
                    uint32_t clk_d_acc_or_freq_acc;
                    uint32_t flags;
                } pos_time_freq_clock_drift;
            }ini;
            union {
                struct {
                    /* Empty */
                }poll;
            }req;
        }ublox_ubx_msg_aid;


        union {
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    x2_t flags;
                    x2_t pins;
                } in_out;
            } ant;
            union {
                struct {
                    x4_t clear_mask;
                    x4_t save_mask;
                    x4_t load_mask;
                    struct {
                        x1_t device_mask;
                    } optional;
                } clear_save_load;
            } cfg;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    r8_t maj_a;
                    r8_t flat;
                    r4_t dx;
                    r4_t dy;
                    r4_t dz;
                    r4_t rot_x;
                    r4_t rot_y;
                    r4_t rot_z;
                    r4_t scale;
                } set_datum;
                struct {
                    u2_t datum_num;
                    ch_t datum_name[6];
                    r8_t maj_a;
                    r8_t flat;
                    r4_t dx;
                    r4_t dy;
                    r4_t dz;
                    r4_t rot_x;
                    r4_t rot_y;
                    r4_t rot_z;
                    r4_t scale;
                } currently_datum;
            } dat;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u1_t msg_ver;
                    u1_t num_trk_ch_hw;
                    u1_t num_trk_ch_use;
                    u1_t num_config_blocks;
                    cfg_gnss_config_block_t cfg_blocks[UBLOX_MSG_CFG_GNSS_BLOCKS_MAX_SIZE];
                } sys_config;
            } gnss;
            union {
                struct {
                    u1_t protocol_id;
                } poll_for_one_protocol;
                struct {
                    cfg_inf_information_t info[UBLOX_MSG_CFG_INFO_BLOCKS_MAX_SIZE];
                } info;
            } inf;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    x4_t config;
                    x4_t config2;
                } jamming_interfernce_cfg;
            } itfm;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u1_t version;
                    x1_t flags;
                    u2_t min_interval;
                    u2_t time_threshold;
                    u2_t speed_threshold;
                    u2_t position_threshold;
                } config;
            } log_filter;
            union {
                struct {
                    u1_t msg_class;
                    u1_t msg_id;
                } poll;
                struct {
                    u1_t msg_class;
                    u1_t msg_id;
                    u1_t rates[6];
                } set_rates;
                struct {
                    u1_t msg_class;
                    u1_t msg_id;
                    u1_t rate;
                } set_rate;
            } msg;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    x2_t mask;
                    u1_t dyn_model;
                    u1_t fix_mode;
                    i4_t fixed_alt;
                    u4_t fixed_alt_var;
                    i1_t min_elev;
                    u1_t dr_limit;
                    u2_t p_dop;
                    u2_t t_dop;
                    u2_t p_acc;
                    u2_t t_acc;
                    u1_t static_hold_thresh;
                    u1_t dgps_timeout;
                    u1_t cno_thresh_num_svs;
                    u1_t cno_thresh;
                    u2_t reserved2;
                    u4_t reserved3;
                    u4_t reserved4;
                } engine_settings;
            } nav5;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u2_t version;
                    x2_t mask1;
                    u4_t reserved0;
                    u1_t reserved1;
                    u1_t reserved2;
                    u1_t min_svs;
                    u1_t max_svs;
                    u1_t min_cno;
                    u1_t reserved5;
                    u1_t ini_fix_3d;
                    u1_t reserved6;
                    u1_t reserved7;
                    u1_t reserved8;
                    u2_t wkn_rollover;
                    u4_t reserved9;
                    u1_t reserved10;
                    u1_t reserved11;
                    u1_t use_ppp;
                    u1_t aop_cfg;
                    u1_t reserved12;
                    u1_t reserved13;
                    u2_t aop_orb_max_err;
                    u1_t reserved14;
                    u1_t reserved15;
                    u2_t reserved3;
                    u4_t reserved4;
                } engine_exp_settings;
            } navx5;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    x1_t filter;
                    u1_t nmea_version;
                    u1_t num_sv;
                    x1_t flags;
                    x4_t gnss_to_filter;
                    u1_t sv_numbering;
                    u1_t main_talker_id;
                    u1_t gsv_talker_id;
                    u1_t reserved;
                } config;
            } nmea;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u1_t version;
                    u1_t reserved1;
                    u1_t reserved2;
                    u1_t reserved3;
                    x4_t flags;
                    u4_t update_period;
                    u4_t search_period;
                    u4_t grid_offset;
                    u2_t on_time;
                    u2_t min_acq_time;
                    u2_t reserved4;
                    u2_t reserved5;
                    u4_t reserved6;
                    u4_t reserved7;
                    u1_t reserved8;
                    u1_t reserved9;
                    u2_t reserved10;
                    u4_t reserved11;
                } ext_pow_management_cfg;
            } pm2;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u1_t port_id;
                } poll_for_one_port;
                struct {
                    u1_t port_id;
                    u1_t reserved0;
                    x2_t tx_ready;
                    x4_t mode;
                    u4_t baud_rate;
                    x2_t in_proto_mask;
                    x2_t out_proto_mask;
                    x2_t flags;
                    u2_t reserved5;
                } config_uart;
                struct {
                    u1_t port_id;
                    u1_t reserved0;
                    x2_t tx_ready;
                    u4_t reserved2;
                    u4_t reserved3;
                    x2_t in_proto_mask;
                    x2_t out_proto_mask;
                    u2_t reserved4;
                    u2_t reserved5;
                } config_usb;
                struct {
                    u1_t port_id;
                    u1_t reserved0;
                    x2_t tx_ready;
                    x4_t mode;
                    u4_t reserved3;
                    x2_t in_proto_mask;
                    x2_t out_proto_mask;
                    x2_t flags;
                    u2_t reserved5;
                } config_spi;
                struct {
                    u1_t port_id;
                    u1_t reserved0;
                    x2_t tx_ready;
                    x4_t mode;
                    u4_t reserved3;
                    x2_t in_proto_mask;
                    x2_t out_proto_mask;
                    x2_t flags;
                    u2_t reserved5;
                } config_ddc;
            } prt;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u2_t meas_rate;
                    u2_t nav_rate;
                    u2_t time_ref;
                } setting;
            } rate;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    x1_t flags;
                    u1_t data[UBLOX_MSG_CFG_RINV_MAX_SIZE];
                } contents;
            } rinv;
            union {
                struct {
                    x2_t nav_bbr_mask;
                    u1_t reset_mode;
                    u1_t reserved1;
                } reset;
            } rst;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u1_t reserved1;
                    u1_t lp_mode;
                } setting;
            } rxm;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    x1_t mode;
                    x1_t usage;
                    u1_t max_sbas;
                    x1_t scan_mode2;
                    x4_t scan_mode1;
                } config;
            } sbas;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u1_t tp_idx;
                } poll_by_idx;
                struct {
                    u1_t tp_idx;
                    u1_t reserved0;
                    u2_t reserved1;
                    i2_t ant_cable_delay;
                    i2_t rf_group_delay;
                    u4_t freq_period;
                    u4_t freq_period_lock;
                    u4_t pulse_len_rato;
                    u4_t pulse_len_rato_lock;
                    i4_t user_config_delay;
                    x4_t flags;
                } time_pulse_parameters;
            } tp5;
            union {
                struct {
                    /* Empty */
                } poll;
                struct {
                    u2_t vendor_id;
                    u2_t product_id;
                    u2_t reserved1;
                    u2_t reserved2;
                    u2_t power_consumption;
                    x2_t flags;
                    ch_t vendor_string[32];
                    ch_t product_string[32];
                    ch_t serial_string[32];
                } config;
            } usb;
        }ublox_ubx_msg_cfg;



        union {
            union {
                struct {
                    u4_t i_tow;
                    u1_t aop_cfg;
                    u1_t status;
                    u1_t reserved0;
                    u1_t reserved1;
                    u4_t avail_gps;
                    u4_t reserved2;
                    u4_t reserved3;
                } status;
            } aop_status;
            union {
                struct {
                    u4_t i_tow;
                    i4_t clk_b;
                    i4_t clk_d;
                    u4_t t_acc;
                    u4_t f_acc;
                } clock;
            } clock;
            union {
                struct {
                    u4_t i_tow;
                    i4_t age;
                    i2_t base_id;
                    i2_t base_health;
                    u1_t num_ch;
                    u1_t status;
                    u2_t reserved1;
                    nav_dgps_data_t data[UBLOX_MSG_NAV_DGPS_DATA_MAX_SIZE];
                } dgps_data;
            } dgps;
            union {
                struct {
                    u4_t i_tow;
                    u2_t g_dop;
                    u2_t p_dop;
                    u2_t t_dop;
                    u2_t v_dop;
                    u2_t h_dop;
                    u2_t n_dop;
                    u2_t e_dop;
                } dilution_of_precision;
            } dop;
            union {
                struct {
                    u4_t i_tow;
                    i4_t ecefe_x;
                    i4_t ecefe_y;
                    i4_t ecefe_z;
                    u4_t p_acc;
                } position_ecefe;
            } pos_ecefe;
            union {
                struct {
                    u4_t i_tow;
                    i4_t lon;
                    i4_t lat;
                    i4_t height;
                    i4_t h_msl;
                    u4_t h_acc;
                    u4_t v_acc;
                } position_llh;
            } pos_llh;
            union {
                struct {
                    u4_t i_tow;
                    u2_t year;
                    u1_t month;
                    u1_t day;
                    u1_t hour;
                    u1_t min;
                    u1_t sec;
                    x1_t valid;
                    u4_t t_acc;
                    i4_t nano;
                    u1_t fix_type;
                    x1_t flags;
                    u1_t reserved1;
                    u1_t num_sv;
                    i4_t lon;
                    i4_t lat;
                    i4_t height;
                    i4_t h_msl;
                    u4_t h_acc;
                    u4_t v_acc;
                    i4_t vel_n;
                    i4_t vel_e;
                    i4_t vel_d;
                    i4_t g_speed;
                    i4_t heading;
                    u4_t s_acc;
                    u4_t heading_acc;
                    u2_t p_dop;
                    x2_t reserved2;
                    u4_t reserved3;
                } position_velocity_time;
            } pos_pvt;
            union {
                struct {
                    u4_t i_tow;
                    u1_t geo;
                    u1_t mode;
                    i1_t sys;
                    x1_t service;
                    u1_t cnt;
                    u1_t reserved0[3];
                    nav_sbas_data_t data[UBLOX_MSG_NAV_SBAS_DATA_MAX_SIZE];
                } sbas_status;
            } sbas;
            union {
                struct {
                    u4_t i_tow;
                    i4_t f_tow;
                    i2_t week;
                    u1_t gps_fix;
                    x1_t flags;
                    i4_t ecefe_x;
                    i4_t ecefe_y;
                    i4_t ecefe_z;
                    u4_t p_acc;
                    i4_t ecefe_v_x;
                    i4_t ecefe_v_y;
                    i4_t ecefe_v_z;
                    u4_t s_acc;
                    u2_t p_dop;
                    u1_t reserved1;
                    u1_t num_sv;
                    u4_t reserved2;
                } sol_info;
            } sol;
            union {
                struct {
                    u4_t i_tow;
                    u1_t gps_fix;
                    x1_t flags;
                    x1_t fix_stat;
                    x1_t flags2;
                    u4_t ttff;
                    u4_t msss;
                } status;
            } status;
            union {
                struct {
                    u4_t i_tow;
                    u1_t num_ch;
                    x1_t global_flags;
                    u2_t reserved2;
                    nav_sv_info_data_t data[UBLOX_MSG_NAV_SVINFO_DATA_MAX_SIZE];
                } sv_info;
            } sv_info;
            union {
                struct {
                    u4_t i_tow;
                    i4_t f_tow;
                    i2_t week;
                    i1_t leap_s;
                    x1_t valid;
                    u4_t t_acc;
                } gps_time;
            } time_gps;
            union {
                struct {
                    u4_t i_tow;
                    u4_t t_acc;
                    i4_t nano;
                    u2_t year;
                    u1_t month;
                    u1_t day;
                    u1_t hour;
                    u1_t min;
                    u1_t sec;
                    x1_t valid;
                } gps_time;
            } time_utc;
            union {
                struct {
                    u4_t i_tow;
                    i4_t ecefe_vx;
                    i4_t ecefe_vy;
                    i4_t ecefe_vz;
                    u4_t s_acc;
                } vel_ecefe;
            } vel_ecefe;
            union {
                struct {
                    u4_t i_tow;
                    i4_t vel_n;
                    i4_t vel_e;
                    i4_t vel_d;
                    u4_t speed;
                    u4_t g_speed;
                    i4_t heading;
                    u4_t s_acc;
                    u4_t c_acc;
                } vel_ned;
            } vel_ned;
        }ublox_ubx_msg_nav;
        union {
            union {
                struct {
                    i1_t ofs_i;
                    u1_t mag_i;
                    i1_t ofs_q;
                    u1_t mag_q;
                    u1_t cfg_source;
                    u1_t reserved0[3];
                    u4_t low_lev_cfg;
                    u4_t reserved1[2];
                    u4_t post_status;
                    u4_t reserved2;
                } hw2;
            } hw2;
            union {
                struct {
                    x4_t pin_sel;
                    x4_t pin_bank;
                    x4_t pin_dir;
                    x4_t pin_val;
                    u2_t noise_per_ms;
                    u2_t agc_cnt;
                    u1_t a_status;
                    u1_t a_power;
                    x1_t flags;
                    u1_t reserved1;
                    x4_t used_mask;
                    u1_t vp[17];
                    u1_t jam_ind;
                    u2_t reserved3;
                    x4_t pin_irq;
                    x4_t pull_h;
                    x4_t pull_l;
                } hw;
            } hw;
            union {
                struct {
                    u4_t rx_bytes;
                    u4_t tx_bytes;
                    u2_t parity_errs;
                    u2_t framing_errs;
                    u2_t overrun_errs;
                    u2_t break_cond;
                    u1_t rx_busy;
                    u1_t tx_busy;
                    u2_t reserved1;
                } io;
            } io;
            union {
                struct {
                    u2_t msg1[8];
                    u2_t msg2[8];
                    u2_t msg3[8];
                    u2_t msg4[8];
                    u2_t msg5[8];
                    u2_t msg6[8];
                    u4_t skipped[6];
                } msgpp;
            } msgpp;
            union {
                struct {
                    u2_t pending[6];
                    u1_t usage[6];
                    u1_t peak_usage[6];
                } rxbuff;
            } rxbuff;
            union {
                struct {
                    x1_t flags;
                } rxr;
            } rxr;
            union {
                struct {
                    u2_t pending[6];
                    u1_t usage[6];
                    u1_t peak_usage[6];
                    u1_t t_usage;
                    u1_t t_peak_usage;
                    x1_t errors;
                    u1_t reserved1;
                } txbuff;
            } txbuff;
            union {
                struct{
                    /* Empty */
                }poll;
                struct {
                    ch_t sw_version[30];
                    ch_t hw_version[10];
                    ch_t extention[30 * 4];
                } receive;
            } version;
        }ublox_ubx_msg_mon;
    }ublox_ubx_msgs;
}ublox_ubx_msg_t;

typedef struct {
    ublox_uart_send_cb_t    ublox_uart_send;
    ublox_uart_update_params_cb_t   ublox_uart_update_params;
    ublox_malloc_cb_t           ublox_malloc;
    ublox_free_cb_t     ublox_free;
    ublox_msg_received_cb  ublox_msg_received;
}ublox_platform_fns_t;


typedef struct {
    ublox_packet_t * ublox_rx_packet;
    ublox_packet_fsm_states_t ublox_current_state;
    u2_t ublox_rx_packet_payload_cnt;
    u2_t ublox_remained_bytes_number;
    ublox_platform_fns_t ublox_platform_fns;
}ublox_ins_t;


/***********************************************************************/
/*                     FUNCTION PROTOTYPES       .                     */
/***********************************************************************/
void ublox_set_periodic_message_status(ublox_instance ublox, ublox_ubx_class_id_t class_id,ublox_msg_id_t msg_id,ublox_periodic_msg_status_t status);
void ublox_send(ublox_instance ublox, ublox_ubx_class_id_t class_id,ublox_msg_id_t msg_id,ublox_ubx_msg_t* msg,u2_t msg_len);
void ublox_destroy(ublox_instance ublox);
ublox_instance ublox_create(ublox_platform_fns_t *ublox_platform_fns);
void ublox_set_spi_params(ublox_instance ublox, ublox_spi_mode_t spi_mode,ublox_spi_flow_control_t flow_control,u1_t ff_cnt);
void ublox_set_uart_params(ublox_instance ublox, ublox_uart_baudrate_t baudrate,ublox_uart_char_len_t char_len,ublox_uart_parity_t parity,ublox_uart_n_stop_bits_t n_stop_bits);
void ublox_set_periodic_interval(ublox_instance ublox, u2_t measurement_rate_ms,ublox_periodic_rate_time_ref_t time_ref);
void ublox_poll_req(ublox_instance ublox, ublox_ubx_class_id_t class_id,ublox_msg_id_t msg_id);
void ublox_data_on_recv(ublox_instance ublox, const uint8_t * data,
                        size_t length);

#ifdef __cplusplus
}
#endif
#endif /* UBLOX_INC_UBLOX_H_ */
