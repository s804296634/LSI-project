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

#ifndef ALI_EXTEND_MESSAGES_H__
#define ALI_EXTEND_MESSAGES_H__

#include <stdint.h>

/** Ali Extended Server model ID. */
#define ALI_EXTEND_CLIENT_MODEL_ID (0x0001)

/**
 * @internal
 * @defgroup ali_extend_MESSAGES Internal header
 * @ingroup ali_extend_MODEL
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Shortest allowed length for the Set message. */
#define ALI_EXTEND_SET_MINLEN 6
/** Longest allowed length for the Set message. */
#define ALI_EXTEND_SET_MAXLEN 29

/** Shortest allowed length for the Status message. */
#define ALI_EXTEND_STATUS_MINLEN 4
/** Longest allowed length for the Status message. */
#define ALI_EXTEND_STATUS_MAXLEN 255

#define SSID_MAXLEN     20
#define PASSWORD_MAXLEN 20
/** Ali Extended opcodes. */
typedef enum
{
    ALI_MESSAGE_ATTR_GET          = 0xD0,
    ALI_MESSAGE_ATTR_SET          = 0xD1,
    ALI_MESSAGE_ATTR_SET_UNACK    = 0xD2,
    ALI_MESSAGE_ATTR_STATUS       = 0xD3,
    ALI_MESSAGE_ATTR_INDICATION   = 0xD4,
    ALI_MESSAGE_ATTR_CONFIRMATION = 0xD5,
}ali_extended_opcode_t;

/** Ali Extended message description */
typedef enum{
    ALI_SIZE_TID             = 1,
    ALI_SIZE_VALUE_TYPE      = 2,
    ALI_SIZE_ERROR_CODE      = 1,
    ALI_POS_TID              = 0,
    ALI_POS_VALID_DATA_START = 1,
}ali_msg_description;

//typedef enum{
//    ALI_VALUE_TYPE_HEARTRATE = 0x0218,
//    ALI_VALUE_TYPE_STEPS     = 0x021A,
//    ALI_VALUE_TYPE_ERROR     = 0x0000,
//    ALI_VALUE_TYPE_EVENTS    = 0xF009,
//}ALI_EXTENDED_WRISTBAND_DATA_TYPE;

typedef enum{
    ALI_VALUE_TYPE_ERROR               = 0x0000,

    ALI_VALUE_TYPE_TIMESTAMP_S         = 0x0103,
    ALI_VALUE_TYPE_BATTERY_PERCENTAGE  = 0x0104,

    ALI_VALUE_TYPE_GATT_OTA            = 0x0500, 
    ALI_VALUE_TYPE_EVENTS              = 0xF009,

    SCHEDULE                           = 0xF080,
    SCHEDULE_ONOFF                     = 0xF081,
    LONGLATITUDE                       = 0xF082,
    DAWN_PLUS_OR_MINUS_TIME            = 0xF083,
    DUST_PLUS_OR_MINUS_TIME            = 0xF084,
    DUST_DAWM_TIME                     = 0xF085,
    SCHEDULE_GET                       = 0xF086,
    ASTRO_CLOCK_ENABLE                 = 0xF087,
    MOTION_ENABLE                      = 0xF088,
    AMBIENT_LIGHT_THRESHOLD            = 0xF089,
    AMBIENT_LIGHT_THRESHOLD_EN         = 0xF08A,
    WORK_MODE                          = 0xF08B,
    MOTION_ZONE_CTR_EN                 = 0xF08C,
    TX_POWER                           = 0xF08D,
    ALI_VALUE_TYPE_VERSION             = 0xFF01,
    ALI_VALUE_TYPE_VERSION_EX          = 0xFF02,

    ALI_VALUE_TYPE_SUPPORT_AMOUNTS     = 21,
}ALI_EXTENDED_DATA_TYPE;

typedef enum{
    ALI_ERROR_DEV_NOT_READY    = 0x80,
    ALI_ERROR_ATTR_NOT_SUPPORT = 0x81,
    ALI_ERROR_OP_NOT_SUPPORT   = 0x82,
    ALI_ERROR_PARAM_INCORRECT  = 0x83,
    ALI_ERROR_STATE_INCORRECT  = 0x84,
}ali_error_code_t;

typedef enum{
    ALI_EVENT_CLICK                       = 0x01,
    ALI_EVENT_DBLCLICK                    = 0x02,
    ALI_EVENT_LONGPRESS                   = 0x03,
    ALI_EVENT_DOWNTIME                    = 0x04,
    ALI_EVENT_POWERON                     = 0x05,
    ALI_EVENT_LOW_POWER                   = 0x06,
    ALI_EVENT_GATEWAY_CONFIG_RESULT       = 0x81,
}ali_event_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the ali_extend Set message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
    uint8_t  data[255];
} ali_extend_set_msg_pkt_t;

/** Message format for the ali_extend Set message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
    uint16_t message_type;            /**< message type. */
} ali_extend_get_msg_pkt_t;

/** Message format for the ali_extend Status message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
    uint8_t  data[255];
} ali_extend_status_msg_pkt_t;

/** Message format for the ali_extend Status message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
    uint8_t  data[255];
} ali_extend_indication_msg_pkt_t;

/** Message format for the ali_extend Status message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
} ali_extend_confirm_msg_pkt_t;

/**@} end of ali_extend_MODEL_INTENRAL */
#endif /* ali_extend_MESSAGES_H__ */
