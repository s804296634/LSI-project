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

#ifndef THROUGHPUT_MESSAGES_H__
#define THROUGHPUT_MESSAGES_H__

#include <stdint.h>

/** Ali Extended Server model ID. */
#define THROUGHPUT_CLIENT_MODEL_ID (0x1003)

/**
 * @internal
 * @defgroup throughput_MESSAGES Internal header
 * @ingroup throughput_MODEL
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Shortest allowed length for the Set message. */
#define THROUGHPUT_SET_MINLEN 3
/** Longest allowed length for the Set message. */
#define THROUGHPUT_SET_MAXLEN 3

/** Shortest allowed length for the Status message. */
#define THROUGHPUT_STATUS_MINLEN 3
/** Longest allowed length for the Status message. */
#define THROUGHPUT_STATUS_MAXLEN 3

/** Shortest allowed length for the Status message. */
#define THROUGHPUT_EVENT_STATUS_MINLEN 1
/** Longest allowed length for the Status message. */
#define THROUGHPUT_EVENT_STATUS_MAXLEN 3


#define SSID_MAXLEN     20
#define PASSWORD_MAXLEN 20
/** Ali Extended opcodes. */
typedef enum
{
    THROUGHPUT_GET          = 0xd6,
    THROUGHPUT_SET          = 0xd7,
    THROUGHPUT_SET_UNACK    = 0xd8,
    THROUGHPUT_STATUS       = 0xd9,
    THROUGHPUT_EVENT_SET_UNACK       = 0xdc,
    THROUGHPUT_EVENT_STATUS          = 0xdd
}throughput_opcode_t;

/** Ali Extended message description */
typedef enum{
    TH_SIZE_TID             = 1,
    TH_SIZE_VALUE           = 3,
    TH_SIZE_ERROR_CODE      = 1,
    TH_POS_TID              = 0,
    TH_POS_VALID_DATA_START = 1,
}throughput_msg_description;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the throughput Set message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
    uint8_t cmd;
    uint8_t  params;
} throughput_set_msg_pkt_t;

/** Message format for the throughput Set message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
    uint8_t cmd;
    uint8_t  params;
} throughput_get_msg_pkt_t;

/** Message format for the throughput Status message. */
typedef struct __attribute((packed))
{
    uint8_t tid;                      /**< Transaction number. */
    uint8_t cmd;
    uint8_t  params;
} throughput_status_msg_pkt_t;

/** Message format for the throughput Status message. */
typedef struct __attribute((packed))
{
    uint8_t event;
} throughput_event_msg_pkt_t;

/**@} end of throughput_MODEL_INTENRAL */
#endif /* throughput_MESSAGES_H__ */
