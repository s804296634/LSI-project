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

#ifndef ALI_EXTENED_SERVER_H__
#define ALI_EXTENED_SERVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "access.h"
#include "time.h"
#include "ali_extend_messages.h"

typedef uint8_t (*ali_data_get_cb_t)(void * p_in, void * p_out);
typedef uint8_t (*ali_data_set_cb_t)(void * data);

typedef struct{  
    uint16_t data_type;
    uint8_t data_length;
    ali_data_get_cb_t get_cb;
    ali_data_set_cb_t set_cb;
}ali_supported_data_t;

/** Vendor specific company ID for Ali Extended model */
#define COMPANY_ID          (0x0059)
#define ALI_SUPPORT_MAX_MSG_LEN (256)
#define ATTR_DATA_MAX_LEN  (10)
#define MAX_SCHEDULE_ITEM 6
#define MAGIC_NO  0x41

typedef struct __attribute((packed))
{
    uint16_t device_type;
    uint16_t fw_veriosn;
    uint8_t  vendor_model_version;
}ali_data_version_t;

typedef struct __attribute((packed))
{
    uint16_t year;
    uint16_t month;
    uint8_t  mday;
    uint8_t  wday;
}ali_data_date_t;

typedef struct __attribute((packed))
{
    uint8_t hour;
    uint8_t minute;
    uint8_t  second;
}ali_data_time_t;

typedef struct __attribute((packed))
{
    uint8_t on_hour;
    uint8_t on_minute;
    uint8_t on_dim_level;
    uint8_t off_hour;
    uint8_t off_minute;
    uint8_t off_dim_level;
}schedule_t;

typedef struct __attribute((packed))
{
    uint8_t weekday_bitmask;
    uint8_t sch_cnt;
    schedule_t schedule1;
    schedule_t schedule2;          
}weekday_schedule_set_t;

typedef struct __attribute((packed))
{
    uint8_t weekday_bitmask;          
}weekday_schedule_get_t;

typedef struct __attribute((packed))
{
    uint8_t weekday_bitmask;  
    uint8_t onoff;
}weekday_schedule_onoff_set_t;

typedef struct __attribute((packed))
{
    uint8_t weekday_bitmask;          
}weekday_schedule_onoff_get_t;

typedef struct __attribute((packed))
{
    int16_t longitude;
    int16_t latitude;          
}geographic_location_set_t;

typedef struct __attribute((packed))
{
    int16_t dawn_pom_time;         
}dawn_pom_time_set_t;
typedef struct __attribute((packed))
{
    int16_t dust_pom_time;         
}dust_pom_time_set_t;

typedef struct __attribute((packed))
{
    time_t dawn_time;
    time_t dust_time;          
}dawn_dust_time_get_t;

typedef struct __attribute((packed))
{
    uint8_t upper_threshold;
    uint8_t lower_threshold;         
}ambient_light_threshold_set_t;

typedef struct
{
    time_t timestamp;
    uint8_t dim_level;      
}schedule_list_item_t;

typedef struct
{
    uint8_t on_hour;
    uint8_t on_minute;
    uint8_t on_dim_level; 

    uint8_t off_hour;
    uint8_t off_minute;
    uint8_t off_dim_level;
}schedule_adaptor_t;

typedef struct
{
    uint8_t weekday;
    uint8_t onoff;
    uint8_t sch_cnt;
    schedule_adaptor_t schedule1;
    schedule_adaptor_t schedule2;
    schedule_list_item_t item[MAX_SCHEDULE_ITEM];        
}weekday_schedule_adaptor_t;

typedef struct
{
    uint8_t do_rise;
    uint8_t do_set;         
}rise_set_t;

typedef struct
{
    uint8_t value;
    double longitude;
    double latitude;          
}geographic_location_t;

typedef struct
{
    uint8_t value;         
}time_ut_t;

typedef struct
{
    uint8_t value;
    int16_t dawn_pom_time;
    int16_t dust_pom_time;          
}pom_time_t;

typedef struct
{
    uint8_t value;
    time_t dawn_time;
    time_t dust_time;          
}dawn_dust_time_t;

typedef struct
{
    uint8_t motion_detect;
    uint8_t astro_clock;
    uint8_t ambient_light;
    uint8_t motion_zone_ctr;
}control_t;

typedef struct
{
    uint8_t upper_threshold;
    uint8_t lower_threshold;         
}ambient_light_threshold_t;

typedef enum{
    INIT                     = 0x00,
    TD1,
    TD2,
    MOTION
}motion_ack_status_t;

/** Ali Extended opcodes. */
//typedef enum
//{
//    ALI_MESSAGE_ATTR_GET          = 0xD0,
//    ALI_MESSAGE_ATTR_SET          = 0xD1,
//    ALI_MESSAGE_ATTR_SET_UNACK    = 0xD2,
//    ALI_MESSAGE_ATTR_STATUS       = 0xD3,
//    ALI_MESSAGE_ATTR_INDICATION   = 0xD4,
//    ALI_MESSAGE_ATTR_CONFIRMATION = 0xD5,
//}ali_extended_opcode_t;

/** Ali Extended message description */
//typedef enum{
//    ALI_SIZE_TID             = 1,
//    ALI_SIZE_VALUE_TYPE      = 2,
//    ALI_SIZE_ERROR_CODE      = 1,
//    ALI_POS_TID              = 0,
//    ALI_POS_VALID_DATA_START = 1,
//}ali_msg_description;

//typedef enum{
//    ALI_VALUE_TYPE_VERSION   = 0xFF01,
//    ALI_VALUE_TYPE_STEPS     = 0x021A,
//    ALI_VALUE_TYPE_ERROR     = 0x0000,
//    ALI_VALUE_TYPE_EVENTS    = 0xF009,
//}ALI_EXTENDED_WRISTBAND_DATA_TYPE;

//typedef enum{
//    ALI_VALUE_TYPE_ERROR               = 0x0000,
//
//    ALI_VALUE_TYPE_TIMESTAMP_S         = 0x0103,
//    ALI_VALUE_TYPE_BATTERY_PERCENTAGE  = 0x0104,
//
//    ALI_VALUE_TYPE_GATT_OTA            = 0x0500, 
//    ALI_VALUE_TYPE_EVENTS              = 0xF009,
//
//    SCHEDULE                           = 0xF080,
//    SCHEDULE_ONOFF                     = 0xF081,
//    LONGLATITUDE                       = 0xF082,
//    DAWN_PLUS_OR_MINUS_TIME            = 0xF083,
//    DUST_PLUS_OR_MINUS_TIME            = 0xF084,
//    DUST_DAWM_TIME                     = 0xF085,
//    SCHEDULE_GET                       = 0xF086,
//    ASTRO_CLOCK_ENABLE                 = 0xF087,
//    MOTION_ENABLE                      = 0xF088,
//    AMBIENT_LIGHT_THRESHOLD            = 0xF089,
//    AMBIENT_LIGHT_THRESHOLD_EN         = 0xF08A,
//    MOTION_ZONE_CTR_EN                 = 0xF08C,
//    TX_POWER                           = 0xF08D,
//    ALI_VALUE_TYPE_VERSION             = 0xFF01,
//    //ALI_VALUE_TYPE_VERSION_EX          = 0xFF02,

//    ALI_VALUE_TYPE_SUPPORT_AMOUNTS     = 19,
//}ALI_EXTENDED_DATA_TYPE;
//
//typedef enum{
//    ALI_ERROR_DEV_NOT_READY    = 0x80,
//    ALI_ERROR_ATTR_NOT_SUPPORT = 0x81,
//    ALI_ERROR_OP_NOT_SUPPORT   = 0x82,
//    ALI_ERROR_PARAM_INCORRECT  = 0x83,
//    ALI_ERROR_STATE_INCORRECT  = 0x84,
//}ali_error_code_t;
//
//typedef enum{
//    ALI_EVENT_CLICK                       = 0x01,
//    ALI_EVENT_DBLCLICK                    = 0x02,
//    ALI_EVENT_LONGPRESS                   = 0x03,
//    ALI_EVENT_DOWNTIME                    = 0x04,
//    ALI_EVENT_POWERON                     = 0x05,
//    ALI_EVENT_LOW_POWER                   = 0x06,
//    ALI_EVENT_GATEWAY_CONFIG_RESULT       = 0x81,
//}ali_event_t;


/**
 * @defgroup ALI_EXTENED_SERVER Ali Extended Server
 * @ingroup ALI_EXTENED_MODEL
 * This module implements a vendor specific Ali Extended Server.
 * @{
 */

/** Ali Extended Server model ID. */
#define ALI_EXTEND_SERVER_MODEL_ID (0x0000)

/** Forward declaration. */
typedef struct __ali_extended_server ali_extended_server_t;


typedef void (*ali_extended_timeout_cb_t)(access_model_handle_t handle, void * p_self);

/** Ali Extended Server state structure. */
struct __ali_extended_server
{
    /** Model handle assigned to the server. */
    access_model_handle_t model_handle;
    /** Supported data configuration */
    ali_supported_data_t * sup_config;
    /** The number of support data type */
    uint8_t sup_num;
    /** Timeout callback called after acknowledged message sending times out */
    ali_extended_timeout_cb_t timeout_cb;
    /** Variable used to determine if a transfer is currently active. */
    bool reliable_transfer_active;
};

/**
 * Initializes the Ali Extended server.
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
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t ali_extended_server_init(ali_extended_server_t * p_server, uint16_t element_index, uint8_t sup_num, ali_supported_data_t support_config[sup_num]);

/**
 * Publishes unsolicited status message.
 *
 * This API can be used to send unsolicited status messages to report updated state value as a result
 * of local action.
 *
 * @param[in]  p_server         Ali Extended Server structure pointer
 * @param[in]  request_type     The data types array which need to be report
 * @param[in]  length           The number of the elements in the report data types array
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 *
 */
uint32_t ali_extended_server_status_publish(ali_extended_server_t * p_server, uint16_t * request_type, uint8_t type_length);


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
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 *
 */
uint32_t ali_extended_server_message_publish(ali_extended_server_t * p_server, uint8_t * message, uint8_t length);

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
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 */
uint32_t ali_extended_attr_inidication(ali_extended_server_t * p_server, uint16_t * request_type, uint8_t type_length);

/** @} end of ALI_EXTENED_SERVER */

#endif /* ALI_EXTENED_SERVER_H__ */
