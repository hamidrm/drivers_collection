/**
 * @file ublox.c
 * @author Hamid Reza Mehrabian
 * @brief UBLOX library
 *
 */

/* UBLOX header */
#include "inc/ublox.h"

/***********************************************************************/
/*              LOCAL VARIABLES.                                       */
/***********************************************************************/

/***********************************************************************/
/*              API FUNCTION DEFINITIONS.                              */
/***********************************************************************/

static ublox_calc_checksum(uint8_t * data, uint8_t data_len, uint8_t * ck_a,
                           uint8_t * ck_b)
{
    uint32_t i;
    *ck_a = 0;
    *ck_b = 0;
    for (i = 0; i < data_len; i++)
    {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

static ublox_calc_rx_checksum(ublox_instance ublox, uint8_t * ck_a, uint8_t * ck_b)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    u2_t i;
    u1_t * data = &ublox_ins->ublox_rx_packet->class_id;
    *ck_a = 0;
    *ck_b = 0;

    for (i = 0; i < 4; i++)
    {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }

    data = ublox_ins->ublox_rx_packet->msg_payload;
    for (i = 0; i < ublox_ins->ublox_rx_packet->len; i++)
    {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

static ublox_ubx_packet_t ublox_build_packet_ubx(ublox_instance ublox,
                                                 ublox_ubx_class_id_t class_id,
                                                 ublox_msg_id_t msg_id,
                                                 uint16_t len,
                                                 uint8_t * payload,
                                                 uint16_t * packet_len)
{
    ublox_ubx_packet_t res;
    ublox_ubx_packet_t temp;
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    uint32_t total_len = UBLOX_UBX_HEADER_LEN + UBLOX_UBX_CLASS_ID_LEN +
    UBLOX_UBX_MSG_ID_LEN + UBLOX_UBX_LEN_LEN + UBLOX_UBX_CHKSUM_LEN
            + ((payload == NULL) ? 0 : len);
    if (packet_len == NULL)
        return NULL;
    len = (payload == NULL) ? 0 : len;
    res = (ublox_ubx_packet_t) ublox_ins->ublox_platform_fns.ublox_malloc(
            total_len);
    if (res == NULL)
        return NULL;

    temp = res + UBLOX_UBX_SYNC_CHAR_A_OFFSET;
    *temp = UBLOX_UBX_SYNC_CHAR_A;

    temp = res + UBLOX_UBX_SYNC_CHAR_B_OFFSET;
    *temp = UBLOX_UBX_SYNC_CHAR_B;

    temp = res + UBLOX_UBX_CLASS_ID_OFFSET;
    *temp = class_id;

    temp = res + UBLOX_UBX_MSG_ID_OFFSET;
    *temp = msg_id;

    temp = res + UBLOX_UBX_LEN_OFFSET;
    *(uint16_t *) temp = len;

    temp = res + UBLOX_UBX_PAYLOAD_OFFSET;

    if (len)
        memcpy(temp, payload, len);

    temp += len;
    ublox_calc_checksum(res + UBLOX_UBX_CLASS_ID_OFFSET,
                        total_len - UBLOX_UBX_CHKSUM_LEN - UBLOX_UBX_HEADER_LEN,
                        temp, temp + 1);

    *packet_len = total_len;
    return res;
}

static void ublox_destroy_packet_ubx(ublox_instance ublox,
                                     ublox_ubx_packet_t packet)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    ublox_ins->ublox_platform_fns.ublox_free(packet);
}

static void ublox_add_rx_data(ublox_instance ublox, u1_t * data, u2_t len)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    while (len--)
    {
        if ((ublox_ins->ublox_current_state == UBLOX_PACKET_FSM_SYNC_CHAR_A)
                && (*data == UBLOX_UBX_SYNC_CHAR_A))
        {
            ublox_ins->ublox_current_state = UBLOX_PACKET_FSM_SYNC_CHAR_B;
        }
        else if ((ublox_ins->ublox_current_state == UBLOX_PACKET_FSM_SYNC_CHAR_B)
                && (*data == UBLOX_UBX_SYNC_CHAR_B))
            ublox_ins->ublox_current_state = UBLOX_PACKET_FSM_CLASS_ID;
        else
        {
            switch (ublox_ins->ublox_current_state)
            {
            case UBLOX_PACKET_FSM_CLASS_ID:
            {
                ublox_ins->ublox_rx_packet->class_id = *data;
                ublox_ins->ublox_current_state++;
            }
                break;
            case UBLOX_PACKET_FSM_MSG_ID:
            {
                ublox_ins->ublox_rx_packet->msg_id = *data;
                ublox_ins->ublox_current_state++;
            }
                break;
            case UBLOX_PACKET_FSM_LEN_L:
            {
                ublox_ins->ublox_rx_packet->len = *data;
                ublox_ins->ublox_current_state++;
            }
                break;
            case UBLOX_PACKET_FSM_LEN_H:
            {
                ublox_ins->ublox_rx_packet->len |= (*data) << 8;
                if (ublox_ins->ublox_rx_packet->len)
                {
                    ublox_ins->ublox_rx_packet->msg_payload =
                            (u1_t *) ublox_ins->ublox_platform_fns.ublox_malloc(
                                    ublox_ins->ublox_rx_packet->len);
                    ublox_ins->ublox_current_state++;
                }
                else
                {
                    ublox_ins->ublox_rx_packet->msg_payload = NULL;
                    ublox_ins->ublox_current_state = UBLOX_PACKET_FSM_CK_A;
                }
                ublox_ins->ublox_rx_packet_payload_cnt = 0;
                ublox_ins->ublox_remained_bytes_number =
                        ublox_ins->ublox_rx_packet->len + 2;
            }
                break;
            case UBLOX_PACKET_FSM_PAYLOAD:
            {
                ublox_ins->ublox_rx_packet->msg_payload[ublox_ins->ublox_rx_packet_payload_cnt++] =
                        *data;
                ublox_ins->ublox_remained_bytes_number--;
                if (ublox_ins->ublox_rx_packet_payload_cnt
                        == ublox_ins->ublox_rx_packet->len)
                    ublox_ins->ublox_current_state++;
            }
                break;
            case UBLOX_PACKET_FSM_CK_A:
            {
                ublox_ins->ublox_remained_bytes_number--;
                ublox_ins->ublox_rx_packet->ck_a = *data;
                ublox_ins->ublox_current_state++;
            }
                break;
            case UBLOX_PACKET_FSM_CK_B:
            {
                u1_t ck_a, ck_b;
                ublox_ins->ublox_remained_bytes_number--;
                ublox_ins->ublox_rx_packet->ck_b = *data;
                ublox_calc_rx_checksum(ublox, &ck_a, &ck_b);
                if ((ck_a == ublox_ins->ublox_rx_packet->ck_a)
                        && (ck_b == ublox_ins->ublox_rx_packet->ck_b))
                {
                    if (ublox_ins->ublox_platform_fns.ublox_msg_received != NULL)
                        ublox_ins->ublox_platform_fns.ublox_msg_received(
                                (ublox_ubx_class_id_t) ublox_ins->ublox_rx_packet->class_id,
                                ublox_ins->ublox_rx_packet->msg_id,
                                (ublox_ubx_msg_t *) (ublox_ins->ublox_rx_packet->msg_payload));
                }
                if (ublox_ins->ublox_rx_packet->msg_payload)
                    ublox_ins->ublox_platform_fns.ublox_free(
                            ublox_ins->ublox_rx_packet->msg_payload);
                ublox_ins->ublox_current_state = UBLOX_PACKET_FSM_SYNC_CHAR_A;
            }
                break;
            default:
                ublox_ins->ublox_current_state = UBLOX_PACKET_FSM_SYNC_CHAR_A;
            }
        }
        data++;
    }
}

static void ublox_config(ublox_instance ublox)
{

    /* Disable all NMEA messages */
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_DTM,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GBS,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GGA,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GLL,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GLQ,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GNQ,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GNS,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GPQ,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GRS,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GSA,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GST,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_GSV,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_RMC,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_TXT,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_VTG,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_STD,
                                      UBLOX_UBX_MSG_ID_NMEA_STD_ZDA,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);

    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_PUBX,
                                      UBLOX_UBX_MSG_ID_NMEA_PUBX_CONFIG,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_PUBX,
                                      UBLOX_UBX_MSG_ID_NMEA_PUBX_POSITION,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_PUBX,
                                      UBLOX_UBX_MSG_ID_NMEA_PUBX_RATE,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_PUBX,
                                      UBLOX_UBX_MSG_ID_NMEA_PUBX_SVSTATUS,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
    ublox_set_periodic_message_status(ublox, UBLOX_UBX_CLASS_ID_NMEA_PUBX,
                                      UBLOX_UBX_MSG_ID_NMEA_PUBX_TIME,
                                      UBLOX_PERIODIC_MSG_STATUS_DISABLE);
}

/**
 * \brief    This function create a UBLOX instance.
 *
 * \param    ublox_platform_fns    UBLOX platform function.
 *
 * \return   Instance of U-BLOX manager.
 *
 */
ublox_instance ublox_create(ublox_platform_fns_t *ublox_platform_fns)
{
    ublox_ins_t * ublox_ins;

    ublox_ins = (ublox_ins_t *) ublox_platform_fns->ublox_malloc(
            sizeof(ublox_ins_t));
    ublox_ins->ublox_rx_packet = (ublox_packet_t *)ublox_platform_fns->ublox_malloc(sizeof(ublox_packet_t));

    memset(ublox_ins->ublox_rx_packet, 0, sizeof(ublox_packet_t));
    memcpy(&ublox_ins->ublox_platform_fns, ublox_platform_fns,
           sizeof(ublox_platform_fns_t));

    ublox_config(ublox_ins);

    return ublox_ins;
}

/**
 * \brief    This function will terminates processing loop.
 *
 * \param    ublox     Instance of ublox
 *
 * \return   None.
 *
 */
void ublox_destroy(ublox_instance ublox)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    if (ublox_ins->ublox_rx_packet->msg_payload)
        ublox_ins->ublox_platform_fns.ublox_free(
                ublox_ins->ublox_rx_packet->msg_payload);
    ublox_ins->ublox_platform_fns.ublox_free(ublox_ins->ublox_rx_packet);
}

/**
 * \brief    Push new arrived data to buffer and analysis its messages.
 *
 * \param    ublox     Instance of ublox
 * \param    data    arrived data pointer.
 * \param    length   arrived data length.
 *
 * \return   None.
 *
 */
void ublox_data_on_recv(ublox_instance ublox, const uint8_t * data,
                        size_t length)
{
    ublox_add_rx_data(ublox, (u1_t *)data, length);
}

/**
 * \brief    It sends a direct message to UBLOX.
 *
 * \param    ublox     Instance of ublox
 * \param    class_id    Message class ID \ref ublox_ubx_class_id_t.
 * \param    msg_id   Message ID \ref ublox_msg_id_t.
 * \param    msg   Message structure /ref ublox_ubx_msg_t.
 * \param    u2_t   Message length in bytes.
 *
 * \return   None.
 *
 */
void ublox_send(ublox_instance ublox, ublox_ubx_class_id_t class_id,
                ublox_msg_id_t msg_id, ublox_ubx_msg_t* msg, u2_t msg_len)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    u2_t packet_len;
    ublox_ubx_packet_t packet_data;
    packet_data = ublox_build_packet_ubx(ublox, class_id, msg_id, msg_len,
                                         (u1_t *) msg, &packet_len);

    if(ublox_ins->ublox_platform_fns.ublox_uart_send)
        ublox_ins->ublox_platform_fns.ublox_uart_send(packet_data, packet_len);
    ublox_destroy_packet_ubx(ublox, packet_data);
}

/**
 * \brief    It sets periodic messages status.
 *
 * \param    ublox     Instance of ublox
 * \param    class_id    Message class ID to set status.
 * \param    msg_id   Message ID to set status.
 * \param    status   New status of intended message /ref ublox_periodic_msg_status_t.
 *
 *
 * \return   None.
 *
 */
void ublox_set_periodic_message_status(ublox_instance ublox,
                                       ublox_ubx_class_id_t class_id,
                                       ublox_msg_id_t msg_id,
                                       ublox_periodic_msg_status_t status)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    ublox_ubx_msg_t * msg;
    u2_t msg_len = 8;
    msg = (ublox_ubx_msg_t *) ublox_ins->ublox_platform_fns.ublox_malloc(
            msg_len);
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.msg_class = class_id;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.msg_id = msg_id;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.rates[0] = 0;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.rates[1] = status;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.rates[2] = 0;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.rates[3] = 0;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.rates[4] = 0;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.msg.set_rates.rates[5] = 0;
    ublox_send(ublox, UBLOX_UBX_CLASS_ID_CFG, UBLOX_UBX_MSG_ID_CFG_MSG, msg,
               msg_len);
    ublox_ins->ublox_platform_fns.ublox_free(msg);
}

/**
 * \brief    It sends a register request to UBLOX. the response will accessible by processing messages function.
 *
 * \param    ublox     Instance of ublox
 * \param    class_id    Message class ID.
 * \param    msg_id   Message ID.
 *
 *
 * \return   None.
 *
 */
void ublox_poll_req(ublox_instance ublox, ublox_ubx_class_id_t class_id,
                    ublox_msg_id_t msg_id)
{
    ublox_send(ublox, class_id, msg_id, NULL, 0);
}

/**
 * \brief    It sets periodic messages interval.
 *
 * \param    ublox     Instance of ublox
 * \param    measurement_rate_ms    Messages interval in milliseconds.
 * \param    time_ref   Messages interval time reference \ref ublox_periodic_rate_time_ref_t.
 *
 *
 * \return   None.
 *
 */
void ublox_set_periodic_interval(ublox_instance ublox, u2_t measurement_rate_ms,
                                 ublox_periodic_rate_time_ref_t time_ref)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    ublox_ubx_msg_t * msg;
    u2_t msg_len = 6;
    msg = (ublox_ubx_msg_t *) ublox_ins->ublox_platform_fns.ublox_malloc(
            msg_len);
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.rate.setting.meas_rate =
            measurement_rate_ms;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.rate.setting.nav_rate = 1;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.rate.setting.time_ref = time_ref;
    ublox_send(ublox, UBLOX_UBX_CLASS_ID_CFG, UBLOX_UBX_MSG_ID_CFG_MSG, msg,
               msg_len);
    ublox_ins->ublox_platform_fns.ublox_free(msg);
}

/**
 * \brief    This function configures UART protocol parameters.
 *
 * \param    ublox     Instance of ublox
 * \param    baudrate    UART baudrate \ref ublox_uart_baudrate_t.
 * \param    char_len   UART Character Length \ref ublox_uart_char_len_t.
 * \param    parity   UART parity type \ref ublox_uart_parity_t.
 * \param    n_stop_bits   UART number of stop bits \ref ublox_uart_n_stop_bits_t
 *
 *
 * \return   None.
 *
 */
void ublox_set_uart_params(ublox_instance ublox, ublox_uart_baudrate_t baudrate,
                           ublox_uart_char_len_t char_len,
                           ublox_uart_parity_t parity,
                           ublox_uart_n_stop_bits_t n_stop_bits)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    ublox_ubx_msg_t * msg;
    u2_t msg_len = 20;
    msg = (ublox_ubx_msg_t *) ublox_ins->ublox_platform_fns.ublox_malloc(
            msg_len);
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.baud_rate = baudrate;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.flags = 0;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.in_proto_mask =
            UBLOX_UBX_CFG_PRT_IN_MASK_UBX |
            UBLOX_UBX_CFG_PRT_IN_MASK_NMEA | UBLOX_UBX_CFG_PRT_IN_MASK_RTCM;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.out_proto_mask =
            UBLOX_UBX_CFG_PRT_OUT_MASK_UBX |
            UBLOX_UBX_CFG_PRT_OUT_MASK_NMEA;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.mode =
            UBLOX_UBX_CFG_PRT_UART_MODE_CHAR_LEN(
                    char_len) |
                    UBLOX_UBX_CFG_PRT_UART_MODE_EVEN_PARITY(parity) | UBLOX_UBX_CFG_PRT_UART_MODE_N_STOP_BITS(n_stop_bits);
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.port_id =
            UBLOX_PORT_UART1;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.tx_ready = 0;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.reserved0 = 0;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.prt.config_uart.reserved5 = 0;
    ublox_send(ublox, UBLOX_UBX_CLASS_ID_CFG, UBLOX_UBX_MSG_ID_CFG_MSG, msg,
               msg_len);
    if (ublox_ins->ublox_platform_fns.ublox_uart_update_params)
        ublox_ins->ublox_platform_fns.ublox_uart_update_params(baudrate,
                                                               char_len, parity,
                                                               n_stop_bits);
    ublox_ins->ublox_platform_fns.ublox_free(msg);
}

/**
 * \brief    This function resets receiver/clears backup data structures.
 *
 * \param    ublox     Instance of ublox
 * \param    flags  BBR Sections to clear. The following special sets apply:\n
 *      UBLOX_UBX_CFG_RST_HOT_START to Hotstart\n
 *      UBLOX_UBX_CFG_RST_WARM_START to Warmstart\n
 *      UBLOX_UBX_CFG_RST_COLD_START to Coldstart\n
 *      Or user-defined by \ref UBLOX_UBX_CFG_RST_NAV_BBR_MASK_* flags.
 * \param    mode   Reset Type by \ref ublox_reset_mode_t
 *
 *
 * \return   None.
 *
 */
void ublox_reset(ublox_instance ublox, u4_t flags, ublox_reset_mode_t mode)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    ublox_ubx_msg_t * msg;
    u2_t msg_len = 4;
    msg = (ublox_ubx_msg_t *) ublox_ins->ublox_platform_fns.ublox_malloc(
            msg_len);
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.rst.reset.nav_bbr_mask = flags
            & 0xFFFF;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.rst.reset.reset_mode = mode & 0xFF;
    ublox_send(ublox, UBLOX_UBX_CLASS_ID_CFG, UBLOX_UBX_MSG_ID_CFG_MSG, msg,
               msg_len);
    ublox_ins->ublox_platform_fns.ublox_free(msg);
}

/**
 * \brief    This function is usable to configure time pulse parameters
 *
 * \param    ublox     Instance of ublox
 * \param    time_pulse     Time pulse selection (UBLOX_TIME_PULSE, UBLOX_TIME_PULSE2)
 * \param    ant_cable_delay   Antenna cable delay.
 * \param    rf_group_delay   RF group delay.
 * \param    frequency_period   Frequency or period time, depending on setting of bit \ref UBLOX_UBX_CFG_TP5_FLAGS_IS_FREQ.
 * \param    pulse_len_duty   Pulse length or duty cycle, depending on \ref UBLOX_UBX_CFG_TP5_FLAGS_IS_LENGTH
 * \param    frequency_period_lock   Frequency or period time when locked to GPS time, only used if \ref UBLOX_UBX_CFG_TP5_FLAGS_LOCKED_OTHER_SET is set.
 * \param    pulse_len_duty_lock   Pulse length or duty cycle when locked to GPS time, only used if \ref UBLOX_UBX_CFG_TP5_FLAGS_LOCKED_OTHER_SET is set.
 * \param    usr_time_pulse_delay   User configurable time pulse delay.
 * \param    flags   Configuration flags.
 *
 *
 * \return   None.
 *
 */
void ublox_config_time_pulse(ublox_instance ublox,
                             ublox_time_pulse_id_t time_pulse,
                             i2_t ant_cable_delay, i2_t rf_group_delay,
                             u4_t frequency_period, u4_t pulse_len_duty,
                             u4_t frequency_period_lock,
                             u4_t pulse_len_duty_lock,
                             i4_t usr_time_pulse_delay, x4_t flags)
{
    ublox_ins_t *ublox_ins = (ublox_ins_t *) ublox;
    ublox_ubx_msg_t * msg;
    u2_t msg_len = 32;
    msg = (ublox_ubx_msg_t *) ublox_ins->ublox_platform_fns.ublox_malloc(
            msg_len);
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.ant_cable_delay =
            ant_cable_delay;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.flags =
            flags;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.rf_group_delay =
            rf_group_delay;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.freq_period =
            frequency_period;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.freq_period_lock =
            frequency_period_lock;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.pulse_len_rato =
            pulse_len_duty;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.pulse_len_rato_lock =
            pulse_len_duty_lock;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.tp_idx =
            time_pulse;
    msg->ublox_ubx_msgs.ublox_ubx_msg_cfg.tp5.time_pulse_parameters.user_config_delay =
            usr_time_pulse_delay;
    ublox_send(ublox, UBLOX_UBX_CLASS_ID_CFG, UBLOX_UBX_MSG_ID_CFG_MSG, msg,
               msg_len);
    ublox_ins->ublox_platform_fns.ublox_free(msg);
}
