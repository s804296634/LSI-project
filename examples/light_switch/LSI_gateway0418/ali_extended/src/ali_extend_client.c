
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

#include "ali_extend_client.h"
#include "model_common.h"

#include "access.h"
#include "access_config.h"
#include "nordic_common.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#if UART_INTERFACE
#include "gateway_repository.h"
#endif

static uint32_t confirm_send(ali_extend_client_t * p_client,
                            const access_message_rx_t * p_message,
                            const ali_extend_status_params_t * p_params)
{
    ali_extend_confirm_msg_pkt_t msg_pkt;

    if (p_params->lenth > ALI_EXTEND_STATUS_MAXLEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    msg_pkt.tid = p_params->tid;

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_VENDOR(ALI_MESSAGE_ATTR_CONFIRMATION, ALI_EXTEND_COMPANY_ID),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = sizeof(ali_extend_confirm_msg_pkt_t),
        .force_segmented = p_client->settings.force_segmented,
        .transmic_size = p_client->settings.transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(p_client->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_client->model_handle, p_message, &reply);
    }
}

/** Opcode Handlers */

static void status_handle(
    access_model_handle_t handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    ali_extend_client_t * p_client = (ali_extend_client_t *)p_args;
    ali_extend_status_params_t in_data = {0};

    if ((p_rx_msg->length >= ALI_EXTEND_STATUS_MINLEN) &&
        (p_rx_msg->length <= ALI_EXTEND_STATUS_MAXLEN))
    {
        ali_extend_status_msg_pkt_t * p_msg_params_packed =
            (ali_extend_status_msg_pkt_t *)p_rx_msg->p_data;

        in_data.tid = p_msg_params_packed->tid;
        in_data.lenth = p_rx_msg->length - 1;
        memcpy(in_data.data, p_msg_params_packed->data, in_data.lenth);

        p_client->settings.p_callbacks->ali_extend_status_cb(
            p_client, &p_rx_msg->meta_data, &in_data);
#if UART_INTERFACE
        SERIAL_PRAMS_T serial_param;
        serial_param.type = TYPE_MESSAGE;

        serial_param.src = p_rx_msg->meta_data.src.value;
        serial_param.dest = p_rx_msg->meta_data.dst.value;
        serial_param.len = p_rx_msg->length + 2;
        serial_param.opcode = ALI_MESSAGE_ATTR_STATUS;
        memcpy(serial_param.data, p_msg_params_packed, p_rx_msg->length);
        app_uart_string_put((uint8_t *)&serial_param, serial_param.len + 7);
#endif
    }
}
static void indication_handle(
    access_model_handle_t handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    ali_extend_client_t * p_client = (ali_extend_client_t *)p_args;
    ali_extend_status_params_t in_data = {0};

    if ((p_rx_msg->length >= ALI_EXTEND_STATUS_MINLEN) &&
        (p_rx_msg->length <= ALI_EXTEND_STATUS_MAXLEN))
    {
        ali_extend_indication_msg_pkt_t * p_msg_params_packed =
            (ali_extend_indication_msg_pkt_t *)p_rx_msg->p_data;

        in_data.tid = p_msg_params_packed->tid;
        in_data.lenth = p_rx_msg->length - 1;
        memcpy(in_data.data, p_msg_params_packed->data, in_data.lenth);

        p_client->settings.p_callbacks->ali_extend_status_cb(
            p_client, &p_rx_msg->meta_data, &in_data);

        confirm_send(p_client, p_rx_msg, &in_data);
#if UART_INTERFACE
        SERIAL_PRAMS_T serial_param;
        serial_param.type = TYPE_MESSAGE;

        serial_param.src = p_rx_msg->meta_data.src.value;
        serial_param.dest = p_rx_msg->meta_data.dst.value;
        serial_param.len = p_rx_msg->length + 2;
        serial_param.opcode = ALI_MESSAGE_ATTR_INDICATION;
        memcpy(serial_param.data, p_msg_params_packed, p_rx_msg->length);
        app_uart_string_put((uint8_t *)&serial_param, serial_param.len + 7);
#endif
    }
}

static const access_opcode_handler_t m_opcode_handlers[] = {
    {ACCESS_OPCODE_VENDOR(ALI_MESSAGE_ATTR_STATUS, ACCESS_COMPANY_ID_NORDIC), status_handle},
    {ACCESS_OPCODE_VENDOR(ALI_MESSAGE_ATTR_INDICATION, ACCESS_COMPANY_ID_NORDIC), indication_handle},
};

static uint8_t message_set_packet_create(
    ali_extend_set_msg_pkt_t * p_set, const ali_extend_set_params_t * p_params)
{
    p_set->tid = p_params->tid;
    memcpy(p_set->data, p_params->data, p_params->lenth);
    return p_params->lenth + 1;
}

static void message_create(ali_extend_client_t * p_client, uint16_t tx_opcode,
    const uint8_t * p_buffer, uint16_t length, access_message_tx_t * p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_client->settings.force_segmented;
    p_message->transmic_size = p_client->settings.transmic_size;
    p_message->access_token = nrf_mesh_unique_token_get();
}

static void reliable_context_create(
    ali_extend_client_t * p_client, uint16_t reply_opcode, access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_client->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    p_reliable->timeout = p_client->settings.timeout;
    p_reliable->status_cb = p_client->settings.p_callbacks->ack_transaction_status_cb;
}

/** Interface functions */

uint32_t ali_extend_client_init(ali_extend_client_t * p_client, uint8_t element_index)
{
    if (p_client == NULL || p_client->settings.p_callbacks == NULL ||
        p_client->settings.p_callbacks->ali_extend_status_cb == NULL ||
        p_client->settings.p_callbacks->periodic_publish_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->settings.timeout == 0)
    {
        p_client->settings.timeout = MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params = {
        .model_id = ACCESS_MODEL_VENDOR(ALI_EXTEND_CLIENT_MODEL_ID, ACCESS_COMPANY_ID_NORDIC),
        .element_index = element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_client,
        .publish_timeout_cb = p_client->settings.p_callbacks->periodic_publish_cb};

    uint32_t status = access_model_add(&add_params, &p_client->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_client->model_handle);
    }

    return status;
}

uint32_t ali_extend_client_set(
    ali_extend_client_t * p_client, const ali_extend_set_params_t * p_params)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params);

        message_create(p_client, ALI_MESSAGE_ATTR_SET, (const uint8_t *)&p_client->msg_pkt.set,
            server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, ALI_MESSAGE_ATTR_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t ali_extend_client_set_unack(
    ali_extend_client_t * p_client, const ali_extend_set_params_t * p_params, uint8_t repeats)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params);

    message_create(p_client, ALI_MESSAGE_ATTR_SET_UNACK, (const uint8_t *)&p_client->msg_pkt.set,
        server_msg_length, &p_client->access_message.message);

    for (uint32_t i = 0; i <= repeats; ++i)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }
    return status;
}

uint32_t ali_extend_client_get(ali_extend_client_t * p_client, ali_extend_get_msg_pkt_t * p_msg_pkt)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, ALI_MESSAGE_ATTR_GET, (const uint8_t *)p_msg_pkt,
            sizeof(ali_extend_get_msg_pkt_t), &p_client->access_message.message);
        reliable_context_create(p_client, ALI_MESSAGE_ATTR_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}