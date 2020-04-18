/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lsi_extended_server.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "access_reliable.h"
#include "nrf_mesh_assert.h"
#include "log.h"
#include "res.h"
#include "app_timer.h"
#include "app_uart.h"
#include "timing.h"

static uint8_t m_tid;
static bool is_query = false;

access_message_rx_t m_message;
//extern uint8_t m_motion_level;
//extern uint8_t m_dim_level;
//extern time_delay_t m_delay1;
//extern time_delay_t m_delay2;
extern uint8_t m_motion_previous_dim_level;

bool check_is_query(void)
{
    return is_query;
}
void clear_query(void)
{
    is_query = false;
}

#define LSI_EXTENDED_INDICATION_ACKED_TRANSACTION_TIMEOUT  (SEC_TO_US(30))

static void dlsi_params_set_timeout_handler(void * p_context)
{
    uint8_t * p_data = p_context;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "repeat.\n");
    app_uart_string_put((uint8_t *)p_data, 6);
}

static void cmd_handler(uint8_t * p_data)
{
    uint32_t err_code;
    if(p_data[0] == 0x55)
    {
        is_query = true;
    }

}

static void lsi_cmd_data_send(uint8_t * p_data,  uint16_t size)
{
    uint32_t err_code;
    uint16_t lenth = size;
    bool send_to_uart = true;
//    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "uart tx:", (uint8_t*)p_data, lenth);
//    while (NRF_SUCCESS != app_uart_put(0x3E));
//    app_uart_string_put((uint8_t *)p_data, lenth);
//    while (NRF_SUCCESS != app_uart_put(0x3C));

    sensor_params_t *p_sensor_params;
    sensor_params_get(&p_sensor_params);

    switch(p_data[0])
    {
        case 0x44:
            m_motion_previous_dim_level = p_data[1];
        break;
        case 0x4c:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Ambient light %d \n:", p_data[1]);
            send_to_uart = false;
            p_sensor_params->ambient_light_threshold1 = p_data[1];
            //p_sensor_params->ambient_light_threshold2 = p_data[1] * 95/100;
            p_sensor_params->ambient_light_threshold2 = p_data[1] - (255 * 5)/100;
            sensor_params_store(NULL, 0);
        break;
        case 0x50:
            //m_dim_level = p_data[1]; 
            p_sensor_params->dim_level = p_data[1];
            send_to_uart = false;
            sensor_params_store(NULL, 0);
        break;
        case 0x51:
            //m_motion_level = p_data[1];
            p_sensor_params->motion_level = p_data[1];
            send_to_uart = false;
            sensor_params_store(NULL, 0);
        break;
        case 0x58:
            //m_delay1.seconds = p_data[1];
            p_sensor_params->delay1.seconds = p_data[1];
            send_to_uart = false;
            sensor_params_store(NULL, 0);
        break;
        case 0x59:
            //m_delay1.minutes = p_data[1];
            p_sensor_params->delay1.minutes = p_data[1];
            send_to_uart = false;
        break;
        case 0x5a:
            //m_delay1.hours = p_data[1];
            p_sensor_params->delay1.hours = p_data[1];
            send_to_uart = false;
        break;
        case 0x48:
            //m_delay2.seconds = p_data[1];
            p_sensor_params->delay2.seconds = p_data[1];
            send_to_uart = false;
            sensor_params_store(NULL, 0);
        break;
        case 0x49:
            //m_delay2.minutes = p_data[1];
            p_sensor_params->delay2.minutes = p_data[1];
            send_to_uart = false;
        break;
        case 0x4a:
            //m_delay2.hours = p_data[1];
            p_sensor_params->delay2.hours = p_data[1];
            send_to_uart = false;
        break;
        case 0x53:
            //m_delay2.hours = p_data[1];
            p_sensor_params->sensitivity = p_data[1];
            sensor_params_store(NULL, 0);
        break;
        default:
        break;
    }

    if(send_to_uart) 
    {
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "uart tx:", (uint8_t*)p_data, lenth);
        while (NRF_SUCCESS != app_uart_put(0x3E));
        app_uart_string_put((uint8_t *)p_data, lenth);
        while (NRF_SUCCESS != app_uart_put(0x3C));        
    }
}

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reply_status(const lsi_extended_server_t * p_server,
                         const access_message_rx_t * p_message,
                         uint8_t * msg, uint8_t length)
{
    access_message_tx_t reply;
    reply.opcode.opcode = LSI_MESSAGE_ATTR_STATUS;
    reply.opcode.company_id = COMPANY_ID;
    reply.p_buffer = (const uint8_t *) msg;
    reply.length = length;
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();
    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}

static uint32_t publish_status(const lsi_extended_server_t * p_server,
                         uint8_t * msg, uint8_t length)
{
    access_message_tx_t publish;
    publish.opcode.opcode = LSI_MESSAGE_ATTR_STATUS;
    publish.opcode.company_id = COMPANY_ID;
    publish.p_buffer = (const uint8_t *) msg;
    publish.length = length;
    publish.force_segmented = false;
    publish.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    publish.access_token = nrf_mesh_unique_token_get();
    return access_model_publish(p_server->model_handle, &publish);
}

static void reliable_status_cb(access_model_handle_t model_handle,
                               void * p_args,
                               access_reliable_status_t status)
{
    lsi_extended_server_t * p_server = p_args;
    p_server->reliable_transfer_active = false;
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "ACCESS_RELIABLE_TRANSFER_SUCCESS\n");
            /* Ignore */
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "ACCESS_RELIABLE_TRANSFER_TIMEOUT\n");
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "ACCESS_RELIABLE_TRANSFER_CANCELLED\n");
            break;
        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }
}

static uint32_t send_reliable_message(const lsi_extended_server_t * p_server,                              
                                      const uint8_t * p_data,
                                      uint16_t length)
{
    access_reliable_t reliable;
    reliable.model_handle = p_server->model_handle;
    reliable.message.p_buffer = p_data;
    reliable.message.length = length;
    reliable.message.opcode.opcode = LSI_MESSAGE_ATTR_INDICATION;
    reliable.message.opcode.company_id = COMPANY_ID;
    reliable.message.force_segmented = false;
    reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reliable.message.access_token = nrf_mesh_unique_token_get();
    reliable.reply_opcode.opcode = LSI_MESSAGE_ATTR_CONFIRMATION;
    reliable.reply_opcode.company_id = COMPANY_ID;
    reliable.timeout = LSI_EXTENDED_INDICATION_ACKED_TRANSACTION_TIMEOUT;
    reliable.status_cb = reliable_status_cb;

    return access_model_reliable_publish(&reliable);
}


/*****************************************************************************
 * Opcode handler callbacks
 *****************************************************************************/

static void handle_set_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    lsi_extended_server_t * p_server = p_args;
    uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN];
    uint8_t ret_msg_index = 1;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "%s\n", __func__);

    if(p_message->length > SET_MSG_LENTH)
        return;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Set\n ");
    message_pool[POS_TYPE] = 0;
    memcpy(&message_pool[POS_VALID_DATA_START], p_message->p_data + POS_VALID_DATA_START, p_message->length - POS_VALID_DATA_START);
    ret_msg_index += (p_message->length - POS_VALID_DATA_START);

    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DBG1, "Prepare to send: ", message_pool, ret_msg_index);
    publish_status(p_server, message_pool, ret_msg_index);
    reply_status(p_server, p_message, message_pool, ret_msg_index);

    //to do:
    lsi_cmd_data_send((uint8_t *)&message_pool[POS_VALID_DATA_START], 2U);
    //cmd_handler(&message_pool[POS_VALID_DATA_START]);
}

static void handle_set_unreliable_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    lsi_extended_server_t * p_server = p_args;
    uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN];
    uint8_t ret_msg_index = 1;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "%s\n", __func__);

    if(p_message->length > SET_MSG_LENTH)
        return;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Unack Set\n ");
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    if(p_message->meta_data.src.value == node_address.address_start) return;

    message_pool[POS_TYPE] = 0;
    memcpy(&message_pool[POS_VALID_DATA_START], p_message->p_data + POS_VALID_DATA_START, p_message->length - POS_VALID_DATA_START);
    ret_msg_index += (p_message->length - POS_VALID_DATA_START);

    //__LOG_XB(LOG_SRC_APP, LOG_LEVEL_DBG1, "Prepare to send: ", message_pool, ret_msg_index);
    publish_status(p_server, message_pool, ret_msg_index);

    //to do:
    lsi_cmd_data_send((uint8_t *)&message_pool[POS_VALID_DATA_START], 2);
    //cmd_handler(&message_pool[POS_VALID_DATA_START]);
}

static void handle_event_set_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    lsi_extended_server_t * p_server = p_args;
    uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN];
    uint8_t ret_msg_index = 1;
//    event_trigger_t event;
//    uint8_t level;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "%s\n", __func__);

    if(p_message->length > SET_MSG_LENTH)
        return;

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    if(p_message->meta_data.src.value == node_address.address_start) return;

    message_pool[POS_TYPE] = 0;
    memcpy(&message_pool[POS_VALID_DATA_START], p_message->p_data + POS_VALID_DATA_START, p_message->length - POS_VALID_DATA_START);
    ret_msg_index += (p_message->length - POS_VALID_DATA_START);

    //__LOG_XB(LOG_SRC_APP, LOG_LEVEL_DBG1, "Prepare to send: ", message_pool, ret_msg_index);
    //publish_status(p_server, message_pool, ret_msg_index);

//    //to do:
//    event = message_pool[POS_VALID_DATA_START];
//    level = message_pool[POS_VALID_DATA_START+1];
//
////    message_pool[POS_VALID_DATA_START] = 0x44;
////    lsi_cmd_data_send((uint8_t *)&message_pool[POS_VALID_DATA_START], 2);
//
//    switch(event)
//    {
//        case MOTION_EVENT:
//            set_motion_status(MOTION);
//            motion_trigger_set_dim_level(level, false);
//            new_task_cover(timestamp_get(), m_dim_level);
//        break;
//        case DAWN:
//        break;
//        case DUST:
//        break;
//        case SCHEDULE_EVENT:
//        break;
//        default:
//        break;
//    }
//    
//    //cmd_handler(&message_pool[POS_VALID_DATA_START]);
}


static void handle_get_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    lsi_extended_server_t * p_server = p_args;
    uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN] = {0};
    uint8_t index = 0;
    uint8_t ret_msg_index = 1;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "%s\n", __func__);

    if(p_message->length > SET_MSG_LENTH || 0x55 != p_message->p_data[POS_VALID_DATA_START])
        return;

    memset(&m_message, 0 , sizeof(m_message));
    memcpy(&m_message, p_message, sizeof(m_message));

//    message_pool[POS_TYPE] = 0;
//    memcpy(&message_pool[POS_VALID_DATA_START], p_message->p_data + POS_VALID_DATA_START, p_message->length - POS_VALID_DATA_START);
//    ret_msg_index += (p_message->length - POS_VALID_DATA_START);

    sensor_params_t *p;
    sensor_params_get(&p);
    message_pool[index++] = 0x01;

    message_pool[index++] = 0x78;
    message_pool[index++] = p->delay1.seconds;
    message_pool[index++] = 0x79;
    message_pool[index++] = p->delay1.minutes;
    message_pool[index++] = 0x7a;
    message_pool[index++] = p->delay1.hours;

    message_pool[index++] = 0x68;
    message_pool[index++] = p->delay2.seconds;
    message_pool[index++] = 0x69;
    message_pool[index++] = p->delay2.minutes;
    message_pool[index++] = 0x6a;
    message_pool[index++] = p->delay2.hours;

    message_pool[index++] = 0x73;
    message_pool[index++] = p->sensitivity;
    message_pool[index++] = 0x70;
    message_pool[index++] = p->dim_level;
    message_pool[index++] = 0x71;
    message_pool[index++] = p->motion_level;
    message_pool[index++] = 0x6c;
    message_pool[index++] = p->ambient_light_threshold1;

    reply_status(p_server, p_message, message_pool, index);
    //to do:
//    lsi_cmd_data_send((uint8_t *)&message_pool[POS_VALID_DATA_START], 2);
//    cmd_handler(&message_pool[POS_VALID_DATA_START]);
}

static void handle_rx_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "%s\n", __func__);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(LSI_MESSAGE_ATTR_SET,            COMPANY_ID), handle_set_cb},
    {ACCESS_OPCODE_VENDOR(LSI_MESSAGE_ATTR_GET,            COMPANY_ID), handle_get_cb},
    {ACCESS_OPCODE_VENDOR(LSI_MESSAGE_ATTR_SET_UNACK,      COMPANY_ID), handle_set_unreliable_cb},
    {ACCESS_OPCODE_VENDOR(LSI_MESSAGE_ATTR_CONFIRMATION,   COMPANY_ID), handle_rx_cb},
    {ACCESS_OPCODE_VENDOR(LSI_MESSAGE_ATTR_EVENT_SET_UNACK,COMPANY_ID), handle_event_set_cb}
};
static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    lsi_extended_server_t * p_server = (lsi_extended_server_t *)p_args;

    uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN] = {0};
    uint8_t len = 0;
    lsi_status_t *p_lsi_status;

    get_lsi_status(&p_lsi_status);
    if(p_lsi_status->motion_status.updated)
    {
        p_lsi_status->motion_status.updated = 0;
        message_pool[len++] = 0x4d;
        message_pool[len++] = p_lsi_status->motion_status.motion_event;

    }
    if(p_lsi_status->light_reading_status.updated)
    {
        p_lsi_status->light_reading_status.updated = 0;
        message_pool[len++] = 0x42;
        message_pool[len++] = p_lsi_status->light_reading_status.light_reading;        
    }
        if(p_lsi_status->light_level_status.updated)
    {
        p_lsi_status->light_level_status.updated = 0;
        message_pool[len++] = 0x4c;
        message_pool[len++] = p_lsi_status->light_level_status.light_level;        
    }
    (void) lsi_extended_server_message_publish(p_server, message_pool, len);
}



/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t lsi_extended_server_init(lsi_extended_server_t * p_server, uint16_t element_index,uint8_t sup_num, lsi_supported_data_t sup_data[sup_num])
{

    uint32_t err_code;
    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }
    p_server->sup_config = sup_data;
    p_server->sup_num = (const int)sup_num;
    access_model_add_params_t init_params;
    init_params.element_index =  element_index;
    init_params.model_id.model_id = LSI_EXTEND_SERVER_MODEL_ID;
    init_params.model_id.company_id = COMPANY_ID;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;

    init_params.publish_timeout_cb = periodic_publish_cb;


    return access_model_add(&init_params, &p_server->model_handle);
}

uint32_t lsi_extended_server_message_publish(lsi_extended_server_t * p_server, uint8_t * message, uint8_t length)
{
    uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN];
    message_pool[POS_TYPE] = 2;
    memcpy(&message_pool[POS_VALID_DATA_START], message, length);
    access_message_tx_t msg;
    msg.opcode.opcode = LSI_MESSAGE_ATTR_STATUS;
    msg.opcode.company_id = COMPANY_ID;
    msg.p_buffer = message_pool;
    msg.length = length + POS_VALID_DATA_START;
    msg.force_segmented = false;
    msg.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    msg.access_token = nrf_mesh_unique_token_get();
    return access_model_publish(p_server->model_handle, &msg);
}

uint32_t lsi_extended_server_message_reply(lsi_extended_server_t * p_server, uint8_t * message, uint8_t length)
{
    uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN];
    message_pool[POS_TYPE] = 1;
    memcpy(&message_pool[POS_VALID_DATA_START], message, length);
    access_message_tx_t msg;
    msg.opcode.opcode = LSI_MESSAGE_ATTR_STATUS;
    msg.opcode.company_id = COMPANY_ID;
    msg.p_buffer = message_pool;
    msg.length = length + POS_VALID_DATA_START;
    msg.force_segmented = false;
    msg.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    msg.access_token = nrf_mesh_unique_token_get();

    static uint16_t cnt = 0;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "REPLY %d\n", cnt++);
    return access_model_reply(p_server->model_handle, &m_message, &msg);
}


uint32_t lsi_extended_attr_inidication(lsi_extended_server_t * p_server, uint8_t * message, uint8_t length)
{
    uint8_t i, j;
    void * data;
    static uint8_t message_pool[LSI_SUPPORT_MAX_MSG_LEN];
    uint8_t ret_msg_index = POS_VALID_DATA_START;
    uint8_t ret_data_len;
    
    if (p_server->reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    message_pool[POS_TYPE] = 2;
    memcpy(&message_pool[POS_VALID_DATA_START], message, length);

    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Prepare to send: ", message_pool, ret_msg_index);
    uint32_t status = send_reliable_message(p_server, message_pool, ret_msg_index);
    if (status == NRF_SUCCESS)
    {
        p_server->reliable_transfer_active = true;
    }
    return status;
}
