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

#ifndef LSI_EXTENDED_SERVER_H__
#define LSI_EXTENDED_SERVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "access.h"

typedef uint8_t (*lsi_data_get_cb_t)(void * data);
typedef uint8_t (*lsi_data_set_cb_t)(void * data);

typedef struct{  
    uint16_t data_type;
    uint8_t data_length;
    lsi_data_get_cb_t get_cb;
    lsi_data_set_cb_t set_cb;
}lsi_supported_data_t;

/** Vendor specific company ID for Ali Extended model */
#define COMPANY_ID          (0x0059)
#define LSI_SUPPORT_MAX_MSG_LEN (50)
#define ATTR_DATA_MAX_LEN  (10)

#define FORWARD_MESSAGE_ADDR_INDEX (3)
#define FORWARD_MESSAGE_DATA_INDEX (4)
#define FORWARD_MESSAGE_SUM_INDEX (5)

#define SELECT_BIT_INDEX (0)

#define SET_MSG_LENTH 3
#define GET_MSG_LENTH 3
#define STATUS_MSG_LENTH 3

typedef struct __attribute((packed))
{
    uint16_t device_type;
    uint16_t fw_veriosn;
    uint8_t  vendor_model_version;
}lsi_data_version_t;

typedef struct __attribute((packed))
{
    uint16_t year;
    uint16_t month;
    uint8_t  day;
}lsi_data_date_t;

typedef struct __attribute((packed))
{
    uint8_t hour;
    uint8_t minute;
    uint8_t  second;
}lsi_data_time_t;

typedef struct __attribute((packed))
{
    uint8_t index;
    uint32_t time_2_attr_set;
    uint8_t action;
    uint16_t attr_type_or_scene_number;
    uint8_t len;
    uint8_t attr_para[ATTR_DATA_MAX_LEN];
}lsi_data_timing_set_t;

typedef struct __attribute((packed))
{
    uint8_t index;
    uint8_t weekday;
    uint8_t hour;
    uint8_t minute;
    uint8_t action;
    uint16_t attr_type_or_scene_number;
    uint8_t len;
    uint8_t attr_para[ATTR_DATA_MAX_LEN];
}lsi_data_timing_set_periodic_t;

/** Ali Extended opcodes. */
typedef enum
{
    LSI_MESSAGE_ATTR_GET          = 0xd6,
    LSI_MESSAGE_ATTR_SET          = 0xd7,
    LSI_MESSAGE_ATTR_SET_UNACK    = 0xd8,
    LSI_MESSAGE_ATTR_STATUS       = 0xd9,
    LSI_MESSAGE_ATTR_INDICATION   = 0xda,
    LSI_MESSAGE_ATTR_CONFIRMATION = 0xdb,
    LSI_MESSAGE_ATTR_EVENT_SET_UNACK = 0xdc,
    LSI_MESSAGE_ATTR_EVENT_STATUS = 0xdd
}lsi_extended_opcode_t;

/** Ali Extended message description */
typedef enum{
    SIZE_TID             = 1,
    SIZE_VALUE_TYPE      = 2,
    SIZE_ERROR_CODE      = 1,
    POS_TYPE              = 0,
    POS_VALID_DATA_START = 1,
}lsi_msg_description;

//typedef enum{
//    LSI_VALUE_TYPE_VERSION   = 0xFF01,
//    LSI_VALUE_TYPE_STEPS     = 0x021A,
//    LSI_VALUE_TYPE_ERROR     = 0x0000,
//    LSI_VALUE_TYPE_EVENTS    = 0xF009,
//}LSI_EXTENDED_WRISTBAND_DATA_TYPE;

typedef enum{
    LSI_VALUE_TYPE_ERROR               = 0x0000,

    LSI_VALUE_TYPE_ONOFF               = 0x0100,
    LSI_VALUE_TYPE_LIGHTNESS           = 0x0121,
    LSI_VALUE_TYPE_CTL                 = 0x0122,
    LSI_VALUE_TYPE_HSL                 = 0x0123,

    LSI_VALUE_TYPE_DATE                = 0x0101,
    LSI_VALUE_TYPE_TIME                = 0x0102,
    LSI_VALUE_TYPE_TIMESTAMP_S         = 0x0103,

    LSI_VALUE_TYPE_GATT_OTA_UPDATE     = 0x0500,

    LSI_VALUE_TYPE_TIMEIMNG_PERIOD     = 0xF008,
    LSI_VALUE_TYPE_EVENTS              = 0xF009,
    LSI_VALUE_TYPE_TIMEIMNG_SET        = 0xF010,
    LSI_VALUE_TYPE_TIMEIMNG_REPORT     = 0xF011,
    LSI_VALUE_TYPE_TIMEIMNG_CANCEL     = 0xF012,
    LSI_VALUE_TYPE_TIMEIMNG_REQ_UPDATE = 0xF013,

    LSI_VALUE_TYPE_VERSION             = 0xFF01,

    LSI_VALUE_TYPE_SUPPORT_AMOUNTS     = 15,
}LSI_EXTENDED_DATA_TYPE;

typedef enum{
    LSI_ERROR_DEV_NOT_READY    = 0x80,
    LSI_ERROR_ATTR_NOT_SUPPORT = 0x81,
    LSI_ERROR_OP_NOT_SUPPORT   = 0x82,
    LSI_ERROR_PARAM_INCORRECT  = 0x83,
    LSI_ERROR_STATE_INCORRECT  = 0x84,
}lsi_error_code_t;

typedef enum{
    LSI_EVENT_CLICK                       = 0x01,
    LSI_EVENT_DBLCLICK                    = 0x02,
    LSI_EVENT_LONGPRESS                   = 0x03,
    LSI_EVENT_DOWNTIME                    = 0x04,
    LSI_EVENT_POWERON                     = 0x05,
    LSI_EVENT_LOW_POWER                   = 0x06,
    LSI_EVENT_GATEWAY_CONFIG_RESULT       = 0x81,
}lsi_event_t;

/**
 * @defgroup LSI_EXTENED_SERVER Ali Extended Server
 * @ingroup LSI_EXTENED_MODEL
 * This module implements a vendor specific Ali Extended Server.
 * @{
 */

/** Ali Extended Server model ID. */
#define LSI_EXTEND_SERVER_MODEL_ID (0x1002)

/** Forward declaration. */
typedef struct __lsi_extended_server lsi_extended_server_t;


typedef void (*lsi_extended_timeout_cb_t)(access_model_handle_t handle, void * p_self);

/** Ali Extended Server state structure. */
struct __lsi_extended_server
{
    /** Model handle assigned to the server. */
    access_model_handle_t model_handle;
    /** Supported data configuration */
    lsi_supported_data_t * sup_config;
    /** The number of support data type */
    uint8_t sup_num;
    /** Timeout callback called after acknowledged message sending times out */
    lsi_extended_timeout_cb_t timeout_cb;
    /** Variable used to determine if a transfer is currently active. */
    bool reliable_transfer_active;
};

/**
 * Initilsizes the Ali Extended server.
 *
 * @note This function should only be called _once_.
 * @note The server handles the model allocation and adding.
 *
 * @param[in] p_server      Ali Extended Server structure pointer.
 * @param[in] element_index Element index to add the server model.
 * @param[in] sump_num      The number of supporting data types
 * @param[in] support_type  The support data types configuration
 *
 * @retval NRF_SUCCESS         Successfully added server.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invlsid element index.
 */
uint32_t lsi_extended_server_init(lsi_extended_server_t * p_server, uint16_t element_index, uint8_t sup_num, lsi_supported_data_t support_config[sup_num]);


/**
 * Publishes unsolicited customized status message.
 *
 * This API can be used to send unsolicited customized status messages
 *
 * @param[in]  p_server         Ali Extended Server structure pointer
 * @param[in]  message          The customized message which will be sent as response status
 * @param[in]  length           The length of the customeized message
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invlsid model handle or model not bound to element.
 * @retval NRF_ERROR_INVLSID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVLSID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVLSID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 *
 */
uint32_t lsi_extended_server_message_publish(lsi_extended_server_t * p_server, uint8_t * message, uint8_t length);
uint32_t lsi_extended_server_message_reply(lsi_extended_server_t * p_server, uint8_t * message, uint8_t length);

/**
 * Publishes the unsolicited message through reliable transmission
 *
 * @param[in]  p_server         Ali Extended Server structure pointer
 * @param[in]  request_type     The data types array which need to be report
 * @param[in]  length           The number of the elements in the report data types array
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invlsid model handle or model not bound to element.
 * @retval NRF_ERROR_INVLSID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVLSID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVLSID_PARAM  Model not bound to appkey, publish address not set or wrong
 */
//uint32_t lsi_extended_attr_inidication(lsi_extended_server_t * p_server, uint16_t * request_type, uint8_t type_length);
uint32_t lsi_extended_attr_inidication(lsi_extended_server_t * p_server, uint8_t * message, uint8_t length);

bool check_is_query(void);
void clear_query(void);

/** @} end of LSI_EXTENED_SERVER */

#endif /* LSI_EXTENED_SERVER_H__ */
