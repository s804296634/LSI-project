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

#include "ali_extended_server.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "access_reliable.h"
#include "nrf_mesh_assert.h"
#include "log.h"

static uint8_t m_tid;
#define ALI_EXTENDED_INDICATION_ACKED_TRANSACTION_TIMEOUT  (SEC_TO_US(30))

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reply_status(const ali_extended_server_t * p_server,
                         const access_message_rx_t * p_message,
                         uint8_t * msg, uint8_t length)
{
    access_message_tx_t reply;
    reply.opcode.opcode = ALI_MESSAGE_ATTR_STATUS;
    reply.opcode.company_id = COMPANY_ID;
    reply.p_buffer = (const uint8_t *) msg;
    reply.length = length;
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();
    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}

static uint32_t publish_status(const ali_extended_server_t * p_server,
                         uint8_t * msg, uint8_t length)
{
    access_message_tx_t publish;
    publish.opcode.opcode = ALI_MESSAGE_ATTR_STATUS;
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
    ali_extended_server_t * p_server = p_args;
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

static uint32_t send_reliable_message(const ali_extended_server_t * p_server,                              
                                      const uint8_t * p_data,
                                      uint16_t length)
{
    access_reliable_t reliable;
    reliable.model_handle = p_server->model_handle;
    reliable.message.p_buffer = p_data;
    reliable.message.length = length;
    reliable.message.opcode.opcode = ALI_MESSAGE_ATTR_INDICATION;
    reliable.message.opcode.company_id = COMPANY_ID;
    reliable.message.force_segmented = false;
    reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reliable.message.access_token = nrf_mesh_unique_token_get();
    reliable.reply_opcode.opcode = ALI_MESSAGE_ATTR_CONFIRMATION;
    reliable.reply_opcode.company_id = COMPANY_ID;
    reliable.timeout = ALI_EXTENDED_INDICATION_ACKED_TRANSACTION_TIMEOUT;
    reliable.status_cb = reliable_status_cb;

    return access_model_reliable_publish(&reliable);
}

static void msg_parsing_set(const ali_extended_server_t * p_server, const access_message_rx_t * p_message)
{
    uint16_t cur_type;
    uint8_t err_status;
    uint8_t set_data_len;
    uint8_t message_pool[ALI_SUPPORT_MAX_MSG_LEN];
    uint8_t ret_msg_index = 1;
    for(uint8_t i = ALI_POS_VALID_DATA_START; i < p_message->length;)
    {
        cur_type = (p_message->p_data[i]) | (p_message->p_data[i+1]) << 8;
        bool type_supported = false;
        for(uint8_t j = 0; j < p_server->sup_num; j++)
        {
            if(cur_type == p_server->sup_config[j].data_type)
            {
                type_supported = true;
                if(cur_type == 0xf003)
                {
                    //set_data_len = p_message->p_data[i+3];
                    set_data_len = p_message->p_data[i+2] + 1;
                }
                else if(cur_type == 0xf008)
                {
                    set_data_len = p_message->p_data[i+2+7] + 8;
                }
                else if(cur_type == 0xf010)
                {
                    set_data_len = p_message->p_data[i+2+8] + 9;
                }
                else
                {
                    set_data_len = p_server->sup_config[j].data_length;
                }
                err_status = p_server->sup_config[j].set_cb((void *)&(p_message->p_data[i+2]));
                
                if(p_message->opcode.opcode == ALI_MESSAGE_ATTR_SET)
                {
                    if(err_status == NRF_SUCCESS)
                    {
                        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "SET success with type %X, len = %d\n", cur_type, set_data_len);
                        memcpy(&message_pool[ret_msg_index], &cur_type, ALI_SIZE_VALUE_TYPE);
                        ret_msg_index += ALI_SIZE_VALUE_TYPE;
                        memcpy(&message_pool[ret_msg_index], (void *)&(p_message->p_data[i+ALI_SIZE_VALUE_TYPE]), set_data_len);
                        ret_msg_index += set_data_len;
                    }
                    else
                    {
                        uint16_t type_error = ALI_VALUE_TYPE_ERROR; 
                        memcpy(&message_pool[ret_msg_index], &type_error, ALI_SIZE_VALUE_TYPE);
                        ret_msg_index += ALI_SIZE_VALUE_TYPE;
                        memcpy(&message_pool[ret_msg_index], &cur_type, ALI_SIZE_VALUE_TYPE);
                        ret_msg_index += ALI_SIZE_VALUE_TYPE;
                        memcpy(&message_pool[ret_msg_index], &err_status, ALI_SIZE_ERROR_CODE);
                        ret_msg_index += ALI_SIZE_ERROR_CODE;
                    }
                }
                i += set_data_len + ALI_SIZE_VALUE_TYPE;
                
            }
            /* TODO: The specification need to put the unkown type data length into consideration
            if(type_supported == false && p_message->opcode.opcode == ALI_MESSAGE_ATTR_SET)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "type not support\n");
                uint16_t type_error = 0x0000; 
                err_status = 0x82;// not support
                memcpy(&message_pool[ret_msg_index], &type_error, SIZE_VALUE_TYPE);
                ret_msg_index += SIZE_VALUE_TYPE;
                memcpy(&message_pool[ret_msg_index], &cur_type, SIZE_VALUE_TYPE);
                ret_msg_index += SIZE_VALUE_TYPE;
                memcpy(&message_pool[ret_msg_index], &err_status, 1);
                ret_msg_index += 1;
            }
            */
        }
    }
    message_pool[ALI_POS_TID] = m_tid++;
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Prepare to send: ", message_pool, ret_msg_index);
    publish_status(p_server, message_pool, ret_msg_index);
    if(p_message->opcode.opcode == ALI_MESSAGE_ATTR_SET)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "set\n ");
        reply_status(p_server, p_message, message_pool, ret_msg_index);
    }

}

static void msg_parsing_get(const ali_extended_server_t * p_server, const access_message_rx_t * p_message)
{
    uint8_t i;
    uint16_t cur_type;
    uint8_t rsp_data[100];
    uint8_t rsp_data_len;
    uint8_t err_status;
    uint8_t ret_msg_index = ALI_POS_VALID_DATA_START;
    uint8_t message_pool[ALI_SUPPORT_MAX_MSG_LEN];
    for(i = 1; i < p_message->length; i+=2)
    {
        bool type_supported = false;
        cur_type = (p_message->p_data[i]) | (p_message->p_data[i+1]) << 8;
        for(uint8_t j =0; j < p_server->sup_num; j++)
        {
            if(cur_type == p_server->sup_config[j].data_type)
            {
                type_supported = true;
                rsp_data_len = p_server->sup_config[j].data_length;
                err_status = p_server->sup_config[j].get_cb((void *)&(p_message->p_data[i+2]), rsp_data);
                if(err_status == NRF_SUCCESS)
                {
                    memcpy(&message_pool[ret_msg_index], &cur_type, ALI_SIZE_VALUE_TYPE);
                    ret_msg_index += ALI_SIZE_VALUE_TYPE;
                    memcpy(&message_pool[ret_msg_index], &rsp_data, rsp_data_len);
                    ret_msg_index += rsp_data_len;                     
                }
                else
                {
                    uint16_t type_error = ALI_VALUE_TYPE_ERROR; 
                    memcpy(&message_pool[ret_msg_index], &type_error, ALI_SIZE_VALUE_TYPE);
                    ret_msg_index += ALI_SIZE_VALUE_TYPE;
                    memcpy(&message_pool[ret_msg_index], &cur_type, ALI_SIZE_VALUE_TYPE);
                    ret_msg_index += ALI_SIZE_VALUE_TYPE;
                    memcpy(&message_pool[ret_msg_index], &err_status, ALI_SIZE_ERROR_CODE);
                    ret_msg_index += ALI_SIZE_ERROR_CODE;
                }
            }

        }
        //the type is not supported
        if(type_supported == false)
        {
            uint16_t type_error = ALI_VALUE_TYPE_ERROR; 
            err_status = ALI_ERROR_ATTR_NOT_SUPPORT;  // operation not support
            memcpy(&message_pool[ret_msg_index], &type_error, ALI_SIZE_VALUE_TYPE);
            ret_msg_index += ALI_SIZE_VALUE_TYPE;
            memcpy(&message_pool[ret_msg_index], &cur_type, ALI_SIZE_VALUE_TYPE);
            ret_msg_index += ALI_SIZE_VALUE_TYPE;
            memcpy(&message_pool[ret_msg_index], &err_status, ALI_SIZE_ERROR_CODE);
            ret_msg_index += ALI_SIZE_ERROR_CODE;
        }
        
    }
    message_pool[ALI_POS_TID] = m_tid++;
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Prepare to send: ", message_pool, ret_msg_index);
    reply_status(p_server, p_message, message_pool, ret_msg_index);
}

/*****************************************************************************
 * Opcode handler callbacks
 *****************************************************************************/

static void handle_set_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    ali_extended_server_t * p_server = p_args;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "%s\n", __func__);
    msg_parsing_set(p_server, p_message);
}

static void handle_get_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    ali_extended_server_t * p_server = p_args;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "%s\n", __func__);
    msg_parsing_get(p_server, p_message);
}

static void handle_rx_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "%s\n", __func__);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(ALI_MESSAGE_ATTR_SET,            COMPANY_ID), handle_set_cb},
    {ACCESS_OPCODE_VENDOR(ALI_MESSAGE_ATTR_GET,            COMPANY_ID), handle_get_cb},
    {ACCESS_OPCODE_VENDOR(ALI_MESSAGE_ATTR_SET_UNACK,      COMPANY_ID), handle_set_cb},
    {ACCESS_OPCODE_VENDOR(ALI_MESSAGE_ATTR_CONFIRMATION,   COMPANY_ID), handle_rx_cb},
};

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t ali_extended_server_init(ali_extended_server_t * p_server, uint16_t element_index,uint8_t sup_num, ali_supported_data_t sup_data[sup_num])
{

    if (p_server == NULL || sup_data == NULL)
    {
        return NRF_ERROR_NULL;
    }
    p_server->sup_config = sup_data;
    p_server->sup_num = (const int)sup_num;
    access_model_add_params_t init_params;
    init_params.element_index =  element_index;
    init_params.model_id.model_id = ALI_EXTEND_SERVER_MODEL_ID;
    init_params.model_id.company_id = COMPANY_ID;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;
    if(p_server->timeout_cb != NULL)
    {
        init_params.publish_timeout_cb = p_server->timeout_cb;
    }
    return access_model_add(&init_params, &p_server->model_handle);
}

uint32_t ali_extended_server_message_publish(ali_extended_server_t * p_server, uint8_t * message, uint8_t length)
{
    uint8_t message_pool[ALI_SUPPORT_MAX_MSG_LEN];
    message_pool[ALI_POS_TID] = m_tid++;
    memcpy(&message_pool[ALI_POS_VALID_DATA_START], message, length);
    access_message_tx_t msg;
    msg.opcode.opcode = ALI_MESSAGE_ATTR_STATUS;
    msg.opcode.company_id = COMPANY_ID;
    msg.p_buffer = message_pool;
    msg.length = length + ALI_SIZE_TID;
    msg.force_segmented = false;
    msg.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    msg.access_token = nrf_mesh_unique_token_get();
    return access_model_publish(p_server->model_handle, &msg);
}

uint32_t ali_extended_server_status_publish(ali_extended_server_t * p_server, uint16_t * request_type, uint8_t type_length)
{
    uint8_t i, j;
    //void * data;
    uint8_t rsp_data[100];
    uint8_t message_pool[ALI_SUPPORT_MAX_MSG_LEN];
    uint8_t ret_msg_index = ALI_POS_VALID_DATA_START;
    uint8_t ret_data_len;
    
    for(i = 0; i < type_length; i++)
    {
        for(j = 0; j < p_server->sup_num; j++)
        {
            if(request_type[i] == p_server->sup_config[j].data_type)
            {
                ret_data_len = p_server->sup_config[j].data_length;
                p_server->sup_config[j].get_cb(NULL, rsp_data);
                memcpy(&message_pool[ret_msg_index], &request_type[i], ALI_SIZE_VALUE_TYPE);
                ret_msg_index += ALI_SIZE_VALUE_TYPE;
                memcpy(&message_pool[ret_msg_index], rsp_data, ret_data_len);
                ret_msg_index += ret_data_len;                                                
            }
        }
    }
    message_pool[ALI_POS_TID] = m_tid++;
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Prepare to send", message_pool, ret_msg_index);
    return publish_status(p_server, message_pool, ret_msg_index);
}

uint32_t ali_extended_attr_inidication(ali_extended_server_t * p_server, uint16_t * request_type, uint8_t type_length)
{
    uint8_t i, j;
    //void * data;
    uint8_t rsp_data[100];
    static uint8_t message_pool[ALI_SUPPORT_MAX_MSG_LEN];
    uint8_t ret_msg_index = ALI_POS_VALID_DATA_START;
    uint8_t ret_data_len;
    
    if (p_server->reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    for(i = 0; i < type_length; i++)
    {
        for(j = 0; j < p_server->sup_num; j++)
        {
            if(request_type[i] == p_server->sup_config[j].data_type)
            {
                ret_data_len = p_server->sup_config[j].data_length;
                p_server->sup_config[j].get_cb(NULL, rsp_data);
                memcpy(&message_pool[ret_msg_index], &request_type[i], ALI_SIZE_VALUE_TYPE);
                ret_msg_index += ALI_SIZE_VALUE_TYPE;
                memcpy(&message_pool[ret_msg_index], rsp_data, ret_data_len);
                ret_msg_index += ret_data_len;                                                
            }
        }
    }
    message_pool[ALI_POS_TID] = m_tid++;
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Prepare to send: ", message_pool, ret_msg_index);
    uint32_t status = send_reliable_message(p_server, message_pool, ret_msg_index);
    if (status == NRF_SUCCESS)
    {
        p_server->reliable_transfer_active = true;
    }
    return status;
}
