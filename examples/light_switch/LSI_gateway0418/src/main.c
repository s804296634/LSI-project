/* Copyright (c) 2010 - 2019, Nordic Semiconductor ASA
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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"


/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "ble_softdevice_support.h"

#include "res.h"
#include "ali_extend_client.h"
#include "ali_extended_server.h"
#include "throughput_client.h"

static bool m_device_provisioned;


ali_extended_server_t m_ali_server;
throughput_client_t m_throughput_client;
ali_extend_client_t m_ali_extend_client;

static bool m_config_process_allow = false;
static bool m_do_syn_time = false;
static bool m_do_syn_position = false;
/*************************************************************************************************/



/*************************************************************************************************/

static void app_throughput_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_throughput_client_status_cb(const throughput_client_t * p_self,
    const access_message_rx_meta_t * p_meta, const throughput_status_params_t * p_in);
static void app_throughput_client_event_status_cb(const throughput_client_t * p_self,
    const access_message_rx_meta_t * p_meta, const throughput_event_status_params_t * p_in);
static void app_throughput_client_transaction_status_cb(
    access_model_handle_t model_handle, void * p_args, access_reliable_status_t status);


    static void app_ali_extend_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_ali_extend_client_status_cb(const ali_extend_client_t * p_self,
    const access_message_rx_meta_t * p_meta, const ali_extend_status_params_t * p_in);
static void app_ali_extend_client_transaction_status_cb(
    access_model_handle_t model_handle, void * p_args, access_reliable_status_t status);

const throughput_client_callbacks_t throughput_client_cbs =
{
    .throughput_status_cb = app_throughput_client_status_cb,
    .throughput_event_status_cb = app_throughput_client_event_status_cb,
    .ack_transaction_status_cb = app_throughput_client_transaction_status_cb,
    .periodic_publish_cb = app_throughput_client_publish_interval_cb
};

const ali_extend_client_callbacks_t ali_extend_client_cbs =
{
    .ali_extend_status_cb = app_ali_extend_client_status_cb,
    .ack_transaction_status_cb = app_ali_extend_client_transaction_status_cb,
    .periodic_publish_cb = app_ali_extend_client_publish_interval_cb
};
/* Throughput */
/* This callback is called periodically if model is configured for periodic publishing */
static void app_throughput_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "throughput client: Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_throughput_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{  
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "throughput client: Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "throughput client: Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "throughput client: Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void throughputed_timeout(access_model_handle_t handle, void * p_self)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}


/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_throughput_client_status_cb(const throughput_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const throughput_status_params_t * p_in)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "throughput client: 0x%04x\n", p_meta->src.value);
    
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"replay statu: cmd: 0x%02x  param: 0x%02x\n",  p_in->cmd,p_in->params);

    uint8_t uart_data[255] = {0};
    uint16_t dest_addr = p_meta->dst.value;
    uint16_t source_addr = p_meta->src.value;
    uint16_t lenth =  2 + 2; // 2 = the length of opcode
    uint16_t opcode = 0x00d9;  // THROUGHPUT_STATUS
    uart_data[0] = TYPE_MESSAGE;

     memcpy(uart_data +DEST_ADDR_INDEX,(uint8_t*)&dest_addr,sizeof(dest_addr) );
           memcpy(uart_data +SOURCE_ADDR_INDEX,(uint8_t*)&source_addr,sizeof(source_addr) );
           memcpy(uart_data +DATA_LENGTH_INDEX,(uint8_t*)&lenth,sizeof(lenth) );
           memcpy(uart_data +DATA_OPCODE_INDEX,(uint8_t*)&opcode,sizeof(opcode) );

           memcpy(uart_data+DATA_PARAMS_INDEX,(uint8_t*)&p_in->cmd,2);

          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "uart data lenth: 0x%04x\n", lenth);
          __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "uart data",uart_data, lenth+7); // 7 = type + dst +src +len
   
          app_uart_string_put((uint8_t *)&uart_data, lenth+7);

}

static void app_throughput_client_event_status_cb(const throughput_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const throughput_event_status_params_t * p_in)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "throughput client: 0x%04x\n", p_meta->src.value);

    if(p_in->event == MOTION_EVENT)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Motion event from slave\n", p_meta->src.value);
        motion_detect_handler();   
    }

}

/*  client model interface: Process the received status message in this callback */
static void app_ali_extend_client_status_cb(const ali_extend_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const ali_extend_status_params_t * p_in)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
        "ali extend client: 0x%04x\n", p_meta->src.value);

     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
        "statu lenth: 0x%04x\n",  p_in->lenth);
     __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "reply status", p_in->data, p_in->lenth);

    uint16_t cur_type;
    uint8_t uart_data[255] = {0};


    
    
    uint16_t dest_addr = p_meta->dst.value;
    uint16_t source_addr = p_meta->src.value;
    uint16_t lenth =  p_in->lenth + 2; // 2 = the length of opcode
    uint16_t opcode = 0x00d3;

    uart_data[0] = TYPE_MESSAGE;
   
    cur_type = (p_in->data[0]) | (p_in->data[1]) << 8;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Attr type: 0x%04x\n", cur_type);
    switch(cur_type)
    {
        case ALI_VALUE_TYPE_TIMESTAMP_S:
        {
           
        }
        break;
        case ALI_VALUE_TYPE_EVENTS:
        {
       
        }
        break;
        default:
        {
           memcpy(uart_data +DEST_ADDR_INDEX,(uint8_t*)&dest_addr,sizeof(dest_addr) );
           memcpy(uart_data +SOURCE_ADDR_INDEX,(uint8_t*)&source_addr,sizeof(source_addr) );
           memcpy(uart_data +DATA_LENGTH_INDEX,(uint8_t*)&lenth,sizeof(lenth) );
           memcpy(uart_data +DATA_OPCODE_INDEX,(uint8_t*)&opcode,sizeof(opcode) );

           memcpy(uart_data+DATA_PARAMS_INDEX,p_in->data,p_in->lenth);

          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "uart data lenth: 0x%04x\n", lenth);
          __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "uart data",uart_data, lenth+7); // 7 = type + dst +src +len
   
          app_uart_string_put((uint8_t *)&uart_data, lenth+7);
        
        }
        break;
    }
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_ali_extend_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "ali extend client: Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_ali_extend_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{ 
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ali extend client: Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ali extend client: Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ali extend client: Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
        /* throughput */
    m_throughput_client.settings.p_callbacks = &throughput_client_cbs;
    m_throughput_client.settings.timeout = 0;
    m_throughput_client.settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
    m_throughput_client.settings.transmic_size = APP_CONFIG_MIC_SIZE;
    ERROR_CHECK(throughput_client_init(&m_throughput_client, 0));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Throughput Handle: %d\n", m_throughput_client.model_handle);

    /* ali extend */
    m_ali_extend_client.settings.p_callbacks = &ali_extend_client_cbs;
    m_ali_extend_client.settings.timeout = 0;
    m_ali_extend_client.settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
    m_ali_extend_client.settings.transmic_size = APP_CONFIG_MIC_SIZE;
    ERROR_CHECK(ali_extend_client_init(&m_ali_extend_client, 0));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ali client Model Handle: %d\n", m_ali_extend_client.model_handle);

    
}

/*************************************************************************************************/



static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
      
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

      static uint8_t tid = 0;
      uint32_t status;
           throughput_set_params_t set_params;
            set_params.tid = tid++;
//            set_params.cmd = p[DATA_PARAMS_INDEX];
//            set_params.params = p[DATA_PARAMS_INDEX + 1];
          
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " set_params.cmd : %x\n",   set_params.cmd );
             __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " set_params.params : %x\n",   set_params.params );


      
    switch (button_number)
    {
        /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 0:
        {
             set_params.cmd =0x44;
            set_params.params =0x55;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "button 0: ON \n");
           status = throughput_client_set_unack(&m_throughput_client, &set_params, 2);
            break;
        }
        case 1:
        {
        
           set_params.cmd =0x44;
            set_params.params =0x00;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "button 1: OFF \n");
           status = throughput_client_set_unack(&m_throughput_client, &set_params, 2);
        
        }

        /* Initiate node reset */
        case 3:
        {
//            /* Clear all the states to reset the node. */
//            if (mesh_stack_is_device_provisioned())
//            {
//#if MESH_FEATURE_GATT_PROXY_ENABLED
//                (void) proxy_stop();
//#endif
//                mesh_stack_config_clear();
//               
//            }
//            else
//            {
//                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
//            }
//            break;
        }

        default:
            break;
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '4')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_LS_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    hd_init();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
