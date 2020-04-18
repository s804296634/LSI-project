#include "res.h"

#include <stdint.h>
#include <string.h>

#include "ble_gap.h"
#include "access.h"
#include "access_config.h"
#include "access_reliable.h"

#include "config_messages.h"
#include "proxy.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "log.h"
#include "mesh_stack.h"
#include "nrf_drv_wdt.h"
#include "boards.h"
#include "app_uart.h"


#include "ali_extended_server.h"
#include "throughput_client.h"

#include "nrf_calendar.h"
#include "drv_rtc.h"
#include "macros_common.h"
#include "sunriset.h"
#include "app_scheduler.h"
#include "timing.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "ali_extend_client.h"
#include "ali_extended_server.h"
#include "throughput_client.h"
#include "lsi_extended_server.h"

extern throughput_client_t m_throughput_client;
extern ali_extend_client_t m_ali_extend_client;

static ack_status_params_t m_ack_status_params = {0};

APP_TIMER_DEF(m_dog_warnming_timer_id);
APP_TIMER_DEF(m_recovery_timer_id);   
APP_TIMER_DEF(m_params_set_timer_id); 
APP_TIMER_DEF(m_provision_check_timer_id);
APP_TIMER_DEF(m_publish_status_timer_id);
APP_TIMER_DEF(m_ota_timer_id);
APP_TIMER_DEF(m_factory_timer_id);
APP_TIMER_DEF(m_reset_timer_id);
APP_TIMER_DEF(m_manual_mode_timer_id);

static void config_wdt(void);
static void uart_init(void);
static void hw_timer_init(void);
static void on_serial_cmd(void * data, uint16_t len);
static void default_params_check(void);
static void hd_application_timers_init(void);
static void hd_application_timers_start(void);
static uint32_t rtc_sensor_init(const nrf_drv_twi_t * p_twi_instance);
static void schedule_update(void);
static void previous_schedule_recover(void);

static void schedule_trigger_set_dim_percentage(uint8_t weekday, uint8_t percentage, bool relay);
static void ambient_trigger_set_dim_level(uint8_t level, event_trigger_t event, bool relay);
static void astro_trigger_set_dim_level(uint8_t level, event_trigger_t event, bool relay);
static uint32_t led_on_allow_event_publish(uint8_t allow);

static bool timing_task_handler(void * p_event_data, uint16_t event_size);

volatile static bool m_hungry = false;

static uint8_t m_data_cmd[MAX_SERIALCMA_LENGTH];
volatile static uint16_t m_index = 0;
static uint8_t m_factory_cnt = 0;

static weekday_schedule_adaptor_t m_weekday_schedule_adaptor[DAYOFWEEK];
static geographic_location_t m_geographic_location;
static pom_time_t m_pom_time;
static time_ut_t m_ut_time;
static ali_data_date_t m_date = {.wday = 7};
static struct tm m_tm;
static dawn_dust_time_t m_dawn_dust_time;
static control_t m_control = {.motion_detect = 0, .astro_clock = 0, .ambient_light = 0, .motion_zone_ctr = 0};
static motion_ack_status_t m_motion_ack_status = INIT;
//static rise_set_t m_rise_set = {.do_rise = 0, .do_set = 0};
static day_condition_t m_day_condition = UNKNOWN;
static ambient_light_threshold_t m_ambient_light_th = {.upper_threshold = 0x80, .lower_threshold = 0x60};
static ambient_light_status_t m_ambient_status = NORMAL;
//uint8_t m_motion_level = 0;
//uint8_t m_dim_level = 0;
//time_delay_t m_delay1;
//time_delay_t m_delay2;
//uint16_t  m_ambient_light_threshold; 
sensor_params_t m_sensor_params = {.motion_level = 255, .dim_level = 77, 
                                   .delay1 = {.seconds = 2}, .delay2 = {.seconds = 10},
                                   .ambient_light_threshold1 = 255, .ambient_light_threshold2 = 242,
                                   .sensitivity = 1
                                   };

static uint8_t m_tid = 0;
static time_t m_current_time;
uint8_t m_motion_previous_dim_level = 255;

//static uint8_t m_z8_version = 0;

#define TWI0_SENSOR_INSTANCE 0
static const nrf_drv_twi_t m_twi0_sensors = NRF_DRV_TWI_INSTANCE(TWI0_SENSOR_INSTANCE);
static uint8_t m_schedule_action = 0;

extern throughput_client_t m_throughput_client;
extern ali_extended_server_t m_ali_server;
extern bool m_device_provisioned;

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                2048

void tm_get(struct tm **pp)
{
    *pp = &m_tm;
}
time_t timestamp_get(void)
{
    return m_current_time;
}
void set_motion_status(motion_ack_status_t status)
{
    m_motion_ack_status = status;
}
void clear_schedule_action(void)
{
    m_schedule_action = 0;
}

void weekday_schedule_adaptor_get(weekday_schedule_adaptor_t **pp)
{
    *pp = m_weekday_schedule_adaptor;
}

void geographic_location_get(geographic_location_t **pp)
{
    *pp = &m_geographic_location;
}

void pom_time_get(pom_time_t **pp)
{
    *pp = &m_pom_time;
}

void ut_time_get(time_ut_t **pp)
{
    *pp = &m_ut_time;
}

void dawndust_time_get(dawn_dust_time_t **pp)
{
    *pp = &m_dawn_dust_time;
}

void control_get(control_t **pp)
{
    *pp = &m_control;
}

void ambient_light_th_get(ambient_light_threshold_t **pp)
{
    *pp = &m_ambient_light_th;
}

void weekday_schedule_adaptor_init(void)
{
    for(uint8_t i = 0; i < 7; i++)
    {
        m_weekday_schedule_adaptor[i].sch_cnt = 0xff;
        m_weekday_schedule_adaptor[i].schedule1.on_hour = 0xff;
        m_weekday_schedule_adaptor[i].schedule1.off_hour = 0xff;
        m_weekday_schedule_adaptor[i].schedule2.on_hour = 0xff;
        m_weekday_schedule_adaptor[i].schedule2.off_hour = 0xff;
    }
}

void sensor_params_get(sensor_params_t **pp)
{
    *pp = &m_sensor_params;
}


//uint8_t get_z8_version(void)
//{
//    return m_z8_version;
//}

void hd_init(void)
{
 //  config_wdt();
   uart_init();
   hw_timer_init();
   //buttons_init();
  // timing_task_mem_init();
 //  rtc_sensor_init(&m_twi0_sensors);
  // hd_application_timers_init();
  // user_add_flash_manager();
  // hd_application_timers_start();
 //  weekday_schedule_adaptor_init();

}


void hd_uuid_set(uint8_t *p_uuid)
{
    ble_gap_addr_t mac_addr;
    uint16_big_encode(ACCESS_COMPANY_ID_NORDIC, p_uuid);
    p_uuid[UUID_PID_INDEX] = (ADV_VER << PID_BIT_ADV_VER) | (KEY_TYPE << PID_BIT_KEY) |
                             (IS_OTA << PID_BIT_OTA) | (BLE_VER << PID_BIT_BLE_VER);
    memset(p_uuid + UUID_PRODUCTID_INDEX, 0, NRF_MESH_UUID_SIZE - 3); 

    uint16_big_encode(DEVICE_PRODUCT_ID, &p_uuid[UUID_PRODUCTID_INDEX]);
    uint16_big_encode(DEVICE_PROJECT_ID, &p_uuid[UUID_PROJECTID_INDEX]);
    sd_ble_gap_addr_get(&mac_addr);
    for(uint8_t i = 0; i < 6; i++)
    {
      p_uuid[UUID_MAC_INDEX+i] = mac_addr.addr[5-i];
    }
    uint16_big_encode(DEVICE_VERSION_ID, &p_uuid[UUID_VID_INDEX]);
}


uint32_t config_process(provisioned_call_back_func_t func)
{
    static uint32_t status = ACCESS_STATUS_SUCCESS;
    access_model_id_t model_id;
    access_model_handle_t model_handle;

    uint16_t element_index = 0;
    uint16_t appkey_index = 0;

    dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;
    access_publish_period_t publish_period;
    access_publish_retransmit_t publish_retransmit;

    dsm_handle_t subscription_address_handle;
    dsm_handle_t appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index);  


/** lsi extend model */




/** ali extend model */
    model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
    model_id.model_id = ALI_EXTEND_SERVER_MODEL_ID;
    status = access_handle_get(element_index, model_id, &model_handle);
    status = access_model_application_bind(model_handle, appkey_handle);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "bind appkey to ali extend server model, status = %d.\n", status);
    if (NRF_SUCCESS == status)
        access_flash_config_store();
    status = dsm_address_publish_add(ADDR1, &publish_address_handle);
    /* Disable publishing for the model while updating the publication parameters: */
    status = access_model_publish_period_set(model_handle, ACCESS_PUBLISH_RESOLUTION_100MS, 0);
    publish_period.step_num = 0;
    publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_100MS;
    if (status == NRF_SUCCESS && publish_period.step_num != 0)
    {
        /* Set publishing parameters for the model: */
        NRF_MESH_ASSERT(access_model_publish_period_set(model_handle,
                            (access_publish_resolution_t)publish_period.step_res,
                            publish_period.step_num) == NRF_SUCCESS);
    }
    publish_retransmit.count = 0;
    publish_retransmit.interval_steps = 9;
    NRF_MESH_ASSERT(access_model_publish_retransmit_set(model_handle, publish_retransmit) == NRF_SUCCESS);
    NRF_MESH_ASSERT(access_model_publish_application_set(model_handle, appkey_handle) == NRF_SUCCESS);
    NRF_MESH_ASSERT(access_model_publish_address_set(model_handle, publish_address_handle) == NRF_SUCCESS);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
        "set ali extend server model pub addr, model_handle = %d, status = %d.\n", model_handle, status);
    if (NRF_SUCCESS == status)
        access_flash_config_store();

    /* Add the address to the DSM as a subscription address: */
    status = dsm_address_subscription_add(ADDR0, &subscription_address_handle);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "DSM ALI ADD1, status = %d.\n", status);
    /* Add the subscription to the model: */
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "set ali extend server model sub addr1, status = %d.\n", status);
    if (NRF_SUCCESS == status)
        access_flash_config_store();

    nrf_delay_ms(50);

/** throughput client model */
    model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
    model_id.model_id = THROUGHPUT_CLIENT_MODEL_ID;
    status = access_handle_get(element_index, model_id, &model_handle);
    status = access_model_application_bind(model_handle, appkey_handle);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "bind appkey to throughput client model, status = %d.\n", status);
    if (NRF_SUCCESS == status)
        access_flash_config_store();


/** health model */
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    model_id.model_id = HEALTH_SERVER_MODEL_ID;
    status = access_handle_get(element_index, model_id, &model_handle);
    status = access_model_application_bind(model_handle, appkey_handle);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "bind appkey to health server model, status = %d.\n", status);
    if (NRF_SUCCESS == status)
        access_flash_config_store();


    (void)app_timer_stop(m_provision_check_timer_id);

    rtc_reseal(false);

    if(func != NULL)
    {
        func();
    }
}


void reset_publish(uint16_t id)
{
    static uint32_t status = ACCESS_STATUS_SUCCESS;
    access_model_id_t model_id;
    access_model_handle_t model_handle;
    uint16_t element_index = 0;
    uint16_t appkey_index = 0;
    dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;
    access_publish_period_t publish_period;
    access_publish_retransmit_t publish_retransmit;
    nrf_mesh_address_t publish_address_stored;

    model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
    model_id.model_id = id;
    status = access_handle_get(element_index, model_id, &model_handle);
    dsm_handle_t appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index); 
     
    status = access_model_publish_address_get(model_handle, &publish_address_handle);
    if (dsm_address_get(publish_address_handle, &publish_address_stored) == NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "public addr 0x%x \n", publish_address_stored.value);
        if(publish_address_stored.value == 0xDF00) return;

        NRF_MESH_ASSERT(dsm_address_publish_remove(publish_address_handle) == NRF_SUCCESS);
        status = dsm_address_publish_add(0xDF00, &publish_address_handle);
        NRF_MESH_ASSERT(access_model_publish_address_set(model_handle, publish_address_handle) == NRF_SUCCESS);
        if(NRF_SUCCESS == status)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication reset\n");
            access_flash_config_store();
        }
        
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get public addr failed\n");
    }             
}

static uint32_t m_touched = 0;
static uint32_t m_untouched_cnt = 0;
void recovery_check()
{
    uint32_t lenth = 4;

    usrdata_load((uint8_t *)&m_touched, FLASH_HANDLE_USRDATA | FLASH_HANDLE_ISTOUCHED_ID, &lenth);
    if (1 == m_touched)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "LAST TIME TOUCHED!\n");
        m_touched = 0;
        m_untouched_cnt = 0;
        usrdata_store(
            (uint8_t *)&m_touched, FLASH_HANDLE_USRDATA | FLASH_HANDLE_ISTOUCHED_ID, 4);
        usrdata_store(
            (uint8_t *)&m_untouched_cnt, FLASH_HANDLE_USRDATA | FLASH_HANDLE_UNTOUCHED_CNT_ID, 4);
    }
    else
    {   
        usrdata_load((uint8_t *)&m_untouched_cnt, FLASH_HANDLE_USRDATA | FLASH_HANDLE_UNTOUCHED_CNT_ID, &lenth);
        //m_untouched_cnt = m_usr_data.data[0];
        m_untouched_cnt++;

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "NO TOUCHED %d!\n",m_untouched_cnt);
        usrdata_store((uint8_t *)&m_untouched_cnt,
            FLASH_HANDLE_USRDATA | FLASH_HANDLE_UNTOUCHED_CNT_ID, 4);
        if (m_untouched_cnt >= 3)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "time to reset!\n");
            /* Clear all the states to reset the node. */
            (void)proxy_stop();
            mesh_stack_config_clear();
            userdata_clear();
            nrf_delay_ms(50);
            node_reset();
        }
    }    
}


static void provision_check_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "provision unfinished!\n");

    (void)proxy_stop();
    mesh_stack_config_clear();
            userdata_clear();
            nrf_delay_ms(50);
    node_reset();
}

void provison_check_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_provision_check_timer_id, APP_TIMER_TICKS(25000), NULL);
    APP_ERROR_CHECK(err_code); 
}

static void hungry_timeout_handler(void * p_context)
{
    m_hungry = true;
}

static void params_set_timeout_handler(void * p_context)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "params load and set.\n");
#if POWER_ON_OFF_RECOVERY
    recovery_check();
#endif
    default_params_check();
}

#define RESET_REASON_MASK   (0xFFFFFFFF)
static void ota_timeout_handler(void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    UNUSED_PARAMETER(p_context);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ota reset!\n");
    static uint32_t temp = 0;

    (void) sd_power_reset_reason_clr(RESET_REASON_MASK); /* avoid wrongful state-readout on reboot */

    err_code = sd_power_gpregret_clr(0, RESET_REASON_MASK);
    APP_ERROR_CHECK(err_code);
    err_code = sd_power_gpregret_set(0, 0xB1);
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_gpregret_get(0, &temp);
    APP_ERROR_CHECK(err_code);

     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "temp = 0x%x\n",temp);

    (void) sd_nvic_SystemReset();
}

static void deadline_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "EN HA TOUCHED!\n");
    m_touched = 1;
    usrdata_store((uint8_t *)&m_touched, FLASH_HANDLE_USRDATA | FLASH_HANDLE_ISTOUCHED_ID, 1);    
}

static void factory_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "timeout to fac\n");
    m_factory_cnt = 0;
}

static void reset_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "timeout to reset\n");
    NVIC_SystemReset(); 
}

static void manual_mode_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "set manual mode.\n");
    set_manual_mode();
  
}

/**@brief Function for initilsizing the timer module.
 */
static void hd_application_timers_init(void)
{
    ret_code_t err_code;

    // Create timers.

    err_code = app_timer_create(&m_dog_warnming_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                hungry_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_recovery_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                deadline_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_params_set_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                params_set_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_provision_check_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                provision_check_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ota_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                ota_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_factory_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                factory_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_reset_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                reset_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_manual_mode_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                manual_mode_timeout_handler);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_create(&m_publish_status_timer_id,
//                                APP_TIMER_MODE_SINGLE_SHOT,
//                                publish_status_timeout_handler);
//    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void hd_application_timers_start(void)
{
    ret_code_t err_code;
#if POWER_ON_OFF_RECOVERY 
    // Start application timers.
    err_code = app_timer_start(m_recovery_timer_id, APP_TIMER_TICKS(3000), NULL);
    APP_ERROR_CHECK(err_code);
#endif
    err_code = app_timer_start(m_dog_warnming_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_params_set_timer_id, APP_TIMER_TICKS(300), NULL);
    APP_ERROR_CHECK(err_code);    
}

void ota_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_ota_timer_id, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);   
}

void reset_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_reset_timer_id, APP_TIMER_TICKS(100), NULL);
    APP_ERROR_CHECK(err_code);   
}


static void default_params_check(void)
{
    uint32_t err_code;
    uint16_t element_index = 0;
    radio_tx_power_t tx_power;

    mesh_opt_core_adv_t relay;
    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &relay));
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
        "relay state: enabled=%d, tx_count=%d, tx_interval_ms=%d ms\n", relay.enabled,
        relay.tx_count, relay.tx_interval_ms);
//    relay.tx_count = 1;
//    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &relay));

    mesh_opt_core_adv_t net;
    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &net));
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
        "net state: enabled=%d, tx_count=%d, tx_interval_ms=%d ms\n", net.enabled,
        net.tx_count, net.tx_interval_ms);
//    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_set(CORE_TX_ROLE_ORIGINATOR, &net));

     mesh_opt_core_tx_power_get(CORE_TX_ROLE_RELAY, &tx_power);
     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Relay role tx power %d dbm.\n", (int8_t)tx_power);
     mesh_opt_core_tx_power_get(CORE_TX_ROLE_ORIGINATOR, &tx_power);
     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "originator role tx power %d dbm.\n", (int8_t)tx_power);

    (void)sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 0, (int8_t)tx_power);  
    net_beacon_adv_tx_power_set((int8_t)tx_power);
//
    uint16_t request_type = ALI_VALUE_TYPE_TIMESTAMP_S;
    ali_extended_server_status_publish(&m_ali_server, &request_type, 2);


//    const config_msg_proxy_status_t status_message = {
//        .proxy_state = (proxy_is_enabled() ? CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED
//                        : CONFIG_GATT_PROXY_STATE_RUNNING_DISABLED)};
//    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "proxy state: proxy_state=%d\n", status_message.proxy_state);

}

bool m_force_relay = false;
void connect_force_relay(void)
{
    mesh_opt_core_adv_t relay;
    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &relay));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
        "relay state: enabled=%d, tx_count=%d, tx_interval_ms=%d ms\n", relay.enabled,
        relay.tx_count, relay.tx_interval_ms);  
         
    if(!relay.enabled)
    {
        m_force_relay = true;
        relay.enabled = true;
        NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &relay));          
    }
 
}

void disconnect_relay_restore(void)
{
    if(m_force_relay)  
    {
        mesh_opt_core_adv_t relay;
        NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &relay));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "relay state: enabled=%d, tx_count=%d, tx_interval_ms=%d ms\n", relay.enabled,
            relay.tx_count, relay.tx_interval_ms); 

        m_force_relay = false;              
        relay.enabled = false;
        NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &relay));         
    }
}

void reset_force_relay()
{
    m_force_relay = false;
}


nrf_drv_wdt_channel_id m_channel_id;
static void config_wdt(void)
{
    uint32_t err_code;
    // Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

void feed_dog(void)
{
    if (m_hungry)
    {
        m_hungry = false;
        nrf_drv_wdt_channel_feed(m_channel_id);
    }
}
/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
static uint16_t const crc16_table[256] = {0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280,
    0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80,
    0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880,
    0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81,
    0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680,
    0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180,
    0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480,
    0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80,
    0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80,
    0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580,
    0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080,
    0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781,
    0x6740, 0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80,
    0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981,
    0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81,
    0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381,
    0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280,
    0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80,
    0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880,
    0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81,
    0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680,
    0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

static inline uint16_t crc16_byte(uint16_t crc, const uint8_t data)
{
    return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

/** * crc16 - compute the CRC-16 for the data buffer 
    * @crc: previous CRC value 
    * @buffer:data pointer 
    * @len: number of bytes in the buffer 
    * * Returns the updated CRC value. */
uint16_t comm_crc16(uint16_t crc, uint8_t const * buffer, uint16_t len)
{
    while (len--)
        crc = crc16_byte(crc, *buffer++);
    return crc;
}


uart_received_status_t m_uart_received_status = NOTREADY;
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    //static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
        {
            UNUSED_VARIABLE(app_uart_get(&m_data_cmd[m_index++]));
            if (m_index >= MAX_SERIALCMA_LENGTH)
            {
                m_index = 0;
                memset(m_data_cmd, 0, MAX_SERIALCMA_LENGTH);
            }

            if ((0 != m_index))
            {
                NRF_TIMER1->TASKS_START = 1;
            }
            else
            {
                NRF_TIMER1->TASKS_STOP = 1;
            }
        }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [UART Initialization] */


/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       NRF_MESH_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

void app_uart_string_put(void * p_string, uint16_t size)
{
    const uint8_t * p = p_string;
#if UART_CRC16
    uint16_t crc = 0;
    uint8_t p_buf[2] = {0};
    crc = comm_crc16(0, p, size);
    uint16_encode(crc, p_buf);
#endif
    for (uint8_t i = 0; i < size; i++)
    {
        while (NRF_SUCCESS != app_uart_put(*(p + i)))
            ;
    }
#if UART_CRC16
    for(uint8_t i = 0; i < 2; i++)
    {
        while (NRF_SUCCESS != app_uart_put(*(p_buf + i)))
            ;      
    }
#endif

}

void TIMER1_IRQHandler(void)
{
    if (0 != NRF_TIMER1->EVENTS_COMPARE[0])
    {
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
        NRF_TIMER1->TASKS_STOP = 1;
        NRF_TIMER1->TASKS_CLEAR = 1;

        if (m_index < MAX_SERIALCMA_LENGTH)
           on_serial_cmd(m_data_cmd, m_index);

        memset(m_data_cmd, 0, MAX_SERIALCMA_LENGTH);
        m_index = 0;
    }

    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_CLEAR = 1;
}

static void hw_timer_init(void)
{
      uint16_t timing = 0;
    timing = 2 * 1000000 / (115200 / 10) + 6000;
    /* Timer1 is used to detect uart input time or detect reset time.*/
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->PRESCALER = 4;  // Prescaler 4 results in 1 tick equals 1 microsecond.
    NRF_TIMER1->CC[0] = timing; // 2000; // 2ms
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER1->INTENSET =
        (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos); // Interrupt setup.

    NVIC_SetPriority(TIMER1_IRQn, NRF_MESH_IRQ_PRIORITY_LOWEST);
    NVIC_EnableIRQ(TIMER1_IRQn);
}

lsi_status_t m_lsi_status;
void get_lsi_status(lsi_status_t ** pp_status)
{
    *pp_status = &m_lsi_status;
}

static void on_serial_cmd(void * data, uint16_t len)
{ 
     SERIAL_ACK_STATUS_T ack_status;
    uint32_t status = NRF_SUCCESS;
    uint8_t module_ack[100] = {0};
  

    if (len > MAX_SERIALCMA_LENGTH)
        return;
    const uint8_t * p = data;
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "uart received", p, len);



    uint8_t type = p[TYPE_INDEX] & TYPE_TYPE_BITMASK;

    if (TYPE_CMD == type)
    {
        memset(module_ack, 0, sizeof(module_ack));
        memcpy(module_ack, p, VALUE_INDEX);
        switch (p[CMD_INDEX])
        {
            case CMD_READ_MODULE_STATUS:
            {
                module_ack[VALUE_INDEX] = 0x01;
                app_uart_string_put(module_ack, 3);
            }
            break;
            case CMD_SET_UUID:
            {
                //nrf_mesh_configure_device_uuid_set(&p[VALUE_INDEX]);
                // to be continued
            }
            break;
            case CMD_START_NETWORKING:
            {
               
            }
            break;
            case CMD_GET_NODE_ADDR:
            {
       //         send_node_addr();
            }
            break;
            case CMD_GET_NETWORK_KEY:
            {
      
       //         send_netkey();
            }
            break;

            default:
                break;
        }
    }
    else if (TYPE_MESSAGE == type)
    {
        uint16_t dest_addr = uint16_decode(p + DEST_ADDR_INDEX);
        uint16_t source_addr = uint16_decode(p + SOURCE_ADDR_INDEX);
        uint16_t data_len = uint16_decode(p + DATA_LENGTH_INDEX);
        uint16_t opcode = uint16_decode(p + DATA_OPCODE_INDEX);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "dest_addr = 0x%x\n",dest_addr);
          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "source_addr = 0x%x\n",source_addr);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data_len = 0x%x\n",data_len);
              __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "opcode = 0x%x\n",opcode);

        ack_status.type = TYPE_STATUS;
        ack_status.opcode = opcode;
        ack_status.status_code = STATUS_RECEIVED;
        app_uart_string_put((uint8_t *)&ack_status, sizeof(SERIAL_ACK_STATUS_T));

       

        dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;
        uint16_t publish_address = dest_addr;
        nrf_mesh_address_t publish_address_stored;
        
        uint32_t status = NRF_SUCCESS;
        access_model_handle_t model_handle;
        volatile bool add = false;
        bool server_status_ack = false;
        switch(ack_status.opcode)
        {
            case ALI_MESSAGE_ATTR_SET:
            case ALI_MESSAGE_ATTR_GET:
            case ALI_MESSAGE_ATTR_SET_UNACK:
                model_handle = m_ali_extend_client.model_handle;
                status = access_model_publish_address_get(model_handle, &publish_address_handle);
                break;

            case LSI_MESSAGE_ATTR_SET:
            case LSI_MESSAGE_ATTR_GET:
            case LSI_MESSAGE_ATTR_SET_UNACK:
                model_handle = m_ali_extend_client.model_handle;
                status = access_model_publish_address_get(model_handle, &publish_address_handle);
                break;
          

            
//            case ALI_MESSAGE_ATTR_STATUS:
//                server_status_ack = true;
            break;
            default:
            {
                ack_status.status_code = STATUS_UNKNOWN;
                app_uart_string_put((uint8_t *)&ack_status, sizeof(SERIAL_ACK_STATUS_T));
            }
                return;
        }

        if (!server_status_ack)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "handle = %d\n", publish_address_handle);

            /* Check if given publish address is different than the currently assigned address */
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "no publish address\n");
                add = true;
                status = dsm_address_publish_add(publish_address, &publish_address_handle);
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "status = %d\n", status);
                // return;
            }
            else
            {
                if (dsm_address_get(publish_address_handle, &publish_address_stored) == NRF_SUCCESS)
                {

                    if ((publish_address_stored.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL) ||
                        (publish_address_stored.type != NRF_MESH_ADDRESS_TYPE_VIRTUAL &&
                            publish_address_stored.value != publish_address))
                    {
                        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication reset\n");
                        /* This should never assert */
                        NRF_MESH_ASSERT(
                            dsm_address_publish_remove(publish_address_handle) == NRF_SUCCESS);
                        status = dsm_address_publish_add(publish_address, &publish_address_handle);
                        add = true;
                    }
                    else
                    {
                        /* Use the retrieved publish_address_handle */
                    }
                }
                else
                {

                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get public addr failed\n");
                    // NRF_MESH_ASSERT(dsm_address_publish_remove(publish_address_handle) ==
                    // NRF_SUCCESS);
                    status = dsm_address_publish_add(publish_address, &publish_address_handle);
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "status = %d\n", status);
                    add = true;
                    // return;
                }
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "type = %x, value = %x \n",
                    publish_address_stored.type, publish_address_stored.value);
            }
            if (add)
            {
                NRF_MESH_ASSERT(access_model_publish_address_set(
                                    model_handle, publish_address_handle) == NRF_SUCCESS);
                access_flash_config_store();
            }
        }

        static uint8_t tid = 0;
        model_transition_t transition_params;
        transition_params.delay_ms = 0;
        transition_params.transition_time_ms = 0;
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.on_off);
        m_ack_status_params.opcode = ack_status.opcode;
        
        switch(ack_status.opcode)
        {

         case ALI_MESSAGE_ATTR_SET:
         {
              ali_extend_set_params_t  set_params;
              memcpy(set_params.data,p+DATA_PARAMS_INDEX,p[DATA_LENGTH_INDEX]-2); //minus the length of opcode
              set_params.tid = tid++;
              set_params.lenth = p[DATA_LENGTH_INDEX]-2;

               (void)access_model_reliable_cancel(m_ali_extend_client.model_handle);
              status = ali_extend_client_set(&m_ali_extend_client, &set_params);

              __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "set_params.data : ", set_params.data, set_params.lenth);
         }
         break;
         case ALI_MESSAGE_ATTR_GET:
         {
              ali_extend_get_msg_pkt_t  get_msg_pkt;
              get_msg_pkt.tid = tid++;
              get_msg_pkt.message_type = uint16_decode(p+DATA_PARAMS_INDEX);//p[DATA_PARAMS_INDEX] || (p[DATA_PARAMS_INDEX +1]<<8);

              __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get_msg_pkt.message_type : %x\n",  get_msg_pkt.message_type);
              (void)access_model_reliable_cancel(m_ali_extend_client.model_handle);

              status = ali_extend_client_get(&m_ali_extend_client,  &get_msg_pkt);
         }
         break;
         case ALI_MESSAGE_ATTR_SET_UNACK:
         {
                ali_extend_set_params_t set_params;
                set_params.tid = tid++;
                 set_params.lenth = p[DATA_LENGTH_INDEX]-2;
                 memcpy(set_params.data,p+DATA_PARAMS_INDEX,p[DATA_LENGTH_INDEX]-2); //minus the length of opcode
              status = ali_extend_client_set_unack(&m_ali_extend_client,&set_params, 2);
         }
         break;
        
        case LSI_MESSAGE_ATTR_SET:
        {
              throughput_set_params_t set_params;
              set_params.tid = tid++;
              set_params.cmd = p[DATA_PARAMS_INDEX];
              set_params.params = p[DATA_PARAMS_INDEX + 1];


               status = throughput_client_set(&m_throughput_client, &set_params);
        }
        break;
        case LSI_MESSAGE_ATTR_GET:
        {
              throughput_get_params_t get_params;
              (void)access_model_reliable_cancel(m_throughput_client.model_handle);
             status = throughput_client_get(&m_throughput_client, &get_params);
        }
        break;
        case LSI_MESSAGE_ATTR_SET_UNACK:
        {
            throughput_set_params_t set_params;
            set_params.tid = tid++;
            set_params.cmd = p[DATA_PARAMS_INDEX];
            set_params.params = p[DATA_PARAMS_INDEX + 1];
//           set_params.cmd =0x44;
//            set_params.params =0x00;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " set_params.cmd : %x\n",   set_params.cmd );
             __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " set_params.params : %x\n",   set_params.params );


            status = throughput_client_set_unack(&m_throughput_client, &set_params, 2);
        }
         break;
            default:
                return;
        }
        switch (status)
        {
            case NRF_SUCCESS:
                break;

            case NRF_ERROR_NO_MEM:
            case NRF_ERROR_BUSY:
            case NRF_ERROR_INVALID_STATE:
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot send - client is busy\n");
                ack_status.status_code = STATUS_BUSY;
                app_uart_string_put((uint8_t *)&ack_status, sizeof(SERIAL_ACK_STATUS_T));
               // hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
                break;

            case NRF_ERROR_INVALID_PARAM:
                /* Publication not enabled for this client. One (or more) of the following
                 * is wrong:
                 * - An application key is missing, or there is no application key bound
                 * to the model
                 * - The client does not have its publication state set
                 *
                 * It is the provisioner that adds an application key, binds it to the
                 * model and sets
                 * the model's publication state.
                 */
                ack_status.status_code = STATUS_UNKNOWN;
                app_uart_string_put((uint8_t *)&ack_status, sizeof(SERIAL_ACK_STATUS_T));
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client\n");
                break;

            default:
                // ERROR_CHECK(status);
                break;
        } // switch
    }
    else
    {
    }

}

static bool get_action_item_no(uint8_t n, uint8_t day, time_t time, uint8_t * p_index)
{
    uint8_t tmp = 0;
    uint8_t i = n;

    for(i = n; i < MAX_SCHEDULE_ITEM; i++)
    {
        if(time < m_weekday_schedule_adaptor[day].item[i].timestamp ||
           0 == m_weekday_schedule_adaptor[day].item[i].timestamp)
        {
            break;
        }
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "item = %d \n", i);

    if(i == n) return false;

    *p_index = i - 1;
    return true;

}

void schedule_action_handler(void * p_event_data, uint16_t event_size)
{
    time_t current_time = *(time_t *)p_event_data;
    uint8_t previous_weekday = PREVIOUS_DAY(m_tm.tm_wday);

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "CT: %ld \n", current_time);
    uint8_t index = 0;

    /* get from today scedule */
    if(current_time >= m_weekday_schedule_adaptor[m_tm.tm_wday].item[0].timestamp)
    {
        if(m_control.astro_clock)
        {
            if(current_time < m_weekday_schedule_adaptor[m_tm.tm_wday].item[1].timestamp)
            {
                m_day_condition = SUN_RISE;
                astro_trigger_set_dim_level(0, DAWN, true);
            }        
            else
            {
               m_day_condition = SUN_SET;
               if(m_weekday_schedule_adaptor[m_tm.tm_wday].onoff)
               {
                   if(get_action_item_no(2, m_tm.tm_wday, current_time, &index) && 
                      (m_weekday_schedule_adaptor[m_tm.tm_wday].item[index].timestamp >= 
                       m_weekday_schedule_adaptor[m_tm.tm_wday].item[1].timestamp))
                   {
                       //to do:action
                        schedule_trigger_set_dim_percentage(
                        m_tm.tm_wday, m_weekday_schedule_adaptor[m_tm.tm_wday].item[index].dim_level,true);            
                   }
                   else
                   {
                        astro_trigger_set_dim_level(255, DUST, true); 
                   }
               }
               else
               {
                  astro_trigger_set_dim_level(255, DUST, true);   
               }
            }
        }
        else
        {
           if(get_action_item_no(2, m_tm.tm_wday, current_time, &index))
           {
               //to do:action
                schedule_trigger_set_dim_percentage(
                m_tm.tm_wday, m_weekday_schedule_adaptor[m_tm.tm_wday].item[index].dim_level,true);            
           }
        }

    }
    /* get from previous day schedule */
    else
    {
        if(m_control.astro_clock)
        {
            if(current_time < m_weekday_schedule_adaptor[previous_weekday].item[1].timestamp)
            {
                m_day_condition = SUN_RISE;
                astro_trigger_set_dim_level(0, DAWN, true);
            }        
            else
            {
                
               m_day_condition = SUN_SET;
               if(m_weekday_schedule_adaptor[previous_weekday].onoff)
               {
                   if(get_action_item_no(2, previous_weekday, current_time, &index) &&
                      (m_weekday_schedule_adaptor[previous_weekday].item[index].timestamp >= 
                       m_weekday_schedule_adaptor[previous_weekday].item[1].timestamp))
                   {
                       //to do:action
                        schedule_trigger_set_dim_percentage(
                        previous_weekday, m_weekday_schedule_adaptor[previous_weekday].item[index].dim_level,true);            
                   }
                   else
                   {
                        astro_trigger_set_dim_level(255, DUST, true);  
                   }
               }
               else
               {
                  astro_trigger_set_dim_level(255, DUST, true);   
               }
            }
        }
        else
        {
           if(get_action_item_no(2, previous_weekday, current_time, &index))
           {
               //to do:action
                schedule_trigger_set_dim_percentage(
                previous_weekday, m_weekday_schedule_adaptor[previous_weekday].item[index].dim_level,true);            
                //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "deal with %d weekday's %d schedule\n", previous_weekday, index);                          
           }
        }            
    }


#if 0
        if(get_action_item_no(0, m_tm.tm_wday, current_time, &index))
        {
            if(0 == index)
            {             
                if(!m_control.astro_clock)
                {
                    return;
                }
                m_day_condition = SUN_RISE;
                astro_trigger_set_dim_level(0, DAWN, true);
                  
            }
            else if(1 == index)
            {
                if(!m_control.astro_clock)
                {
                    return;
                }
                m_day_condition = SUN_SET;
                astro_trigger_set_dim_level(255, DUST, true);                
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "deal with %d weekday's %d schedule\n", m_tm.tm_wday, index); 
                schedule_trigger_set_dim_percentage(m_tm.tm_wday, 
                                                    m_weekday_schedule_adaptor[m_tm.tm_wday].item[index].dim_level,true);
            }

//            if(index < 2)
//            {
//                if(!m_control.astro_clock)
//                    return;
//            }
//            else
//            {
//                if(!m_weekday_schedule_adaptor[m_tm.tm_wday].onoff)
//                    return;              
//            }
//            //to do:action
//            if(m_motion_ack_status > INIT) return;
//            set_dim_level_percentage(m_weekday_schedule_adaptor[m_tm.tm_wday].item[index].dim_level);           
        }
    }
    else if(current_time >= m_weekday_schedule_adaptor[previous_weekday].item[2].timestamp)
    {
        if(get_action_item_no(2, previous_weekday, current_time, &index))
        {
            //to do:action
            schedule_trigger_set_dim_percentage(previous_weekday, 
                                                m_weekday_schedule_adaptor[previous_weekday].item[index].dim_level,true);            
            //set_dim_level_percentage(m_weekday_schedule_adaptor[m_tm.tm_wday].item[index].dim_level);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "deal with %d weekday's %d schedule\n", m_tm.tm_wday, index);            
        }       
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "schedlue wrong.\n");
    }
#endif

}

bool utc_hm_to_local_hm(uint8_t h, uint8_t m, uint8_t * p_h, uint8_t * p_m)
{
    if(h > 23) return false;

    struct tm * p_tm;
    static struct tm l_tm;
    time_t local_time = {0};

    l_tm.tm_year = 2019 - 1900;
    l_tm.tm_mon = 10;
    l_tm.tm_mday = 5;
    l_tm.tm_hour = h;
    l_tm.tm_min = m;
    local_time = mktime(&l_tm) + 240 * m_geographic_location.longitude;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "lt %ld, h:%d,m:%d \n", local_time, h, m);

    p_tm = localtime(&local_time);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "h:%d,m:%d \n", p_tm->tm_hour, p_tm->tm_min);

    *p_h = p_tm->tm_hour;
    *p_m = p_tm->tm_min;

     return true;
}

void local_hm_to_utc_hm(uint8_t h, uint8_t m, uint8_t * p_h, uint8_t * p_m)
{
    struct tm l_tm;
    time_t local_time;

    l_tm.tm_year = 2019 - 1900;
    l_tm.tm_mon = 10;
    l_tm.tm_mday = 5;
    l_tm.tm_hour = h;
    l_tm.tm_min = m;

    local_time = mktime(&l_tm) - 240 * m_geographic_location.longitude;
    l_tm = *localtime(&local_time);

    *p_h = l_tm.tm_hour;
    *p_m = l_tm.tm_min;
}


/**@brief rtc tick event handler.
 */
static void drv_rtc_evt_handler(drv_rtc_evt_t event)
{
    uint32_t err_code;

    if (event == DRV_RTC_EVT_DATA)
     {
        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "rtc alert signal\n");
        static uint8_t index;

        static uint8_t previous_wday = 7;

        err_code = drv_rtc_datetime_get(&m_tm);
        APP_ERROR_CHECK(err_code);
        m_current_time = mktime(&m_tm);

        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "Time:%lx \n", m_current_time);
        m_date.year = m_tm.tm_year + 1900;
        m_date.month = m_tm.tm_mon + 1;
        m_date.mday = m_tm.tm_mday;
        m_date.wday = m_tm.tm_wday;
    
        //if(timing_task_handler((void *)&m_current_time, sizeof(time_t))) return;
                 
        if(0 == previous_wday && previous_wday != m_date.wday)
        {
            // when sunday to monday,update schedule
            //schedule_update();
        }
        if(previous_wday != 7 && previous_wday != m_date.wday)
        {
            /* when a new come, 
               update sun rise and set time,
               load schedule
            */
//            err_code = app_sched_event_put(&m_current_time, sizeof(m_current_time), calculate_currentday_rise_and_set_time);
//            APP_ERROR_CHECK(err_code); 
            calculate_currentday_rise_and_set_time(&m_current_time, sizeof(m_current_time));  
            previous_wday = m_date.wday;
            schedule_recover();
            return;
        }
        previous_wday = m_date.wday;

        schedule_action_handler((void *)&m_current_time, sizeof(time_t));

        timing_task_handler((void *)&m_current_time, sizeof(time_t));

        //app_sched_event_put((void *)&current_time, sizeof(time_t), schedule_action_handler);
#if 0
        if(current_time >= m_weekday_schedule_adaptor[m_date.wday].item[0].timestamp &&
           current_time  < m_weekday_schedule_adaptor[m_date.wday].item[0].timestamp + 5)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "sun rises!!!\n");
            set_dim_level(0);
            //to do:action
            return;
        }
        if(current_time >= m_weekday_schedule_adaptor[m_date.wday].item[1].timestamp && 
           current_time  < m_weekday_schedule_adaptor[m_date.wday].item[1].timestamp + 5)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "sun set!!!\n");
            //to do:action
            index = m_date.wday;
            schedule_recover();
            set_dim_level(100);
            return;
        }

        if(!m_weekday_schedule_adaptor[index].onoff)
            return;

        for(uint8_t i = 2; i < MAX_SCHEDULE_ITEM; i++)
        {
            if(current_time > m_weekday_schedule_adaptor[index].item[i].timestamp)
            {
                //to do:action
                set_dim_level(m_weekday_schedule_adaptor[index].item[i].dim_level);
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "deal with %d weekday's timer %d schedule\n", index, i - 1);
                return;                
            }
        }
#endif
    }
    else
    {
        APP_ERROR_CHECK_BOOL(false);
    }

}

/**@brief Function for initializing the humidity/temperature sensor
 */
static uint32_t rtc_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    ret_code_t err_code = NRF_SUCCESS;

    static const nrf_drv_twi_config_t twi_config = {.scl = SCL_RTC,
        .sda = SDA_RTC,
        .frequency = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST};

    drv_rtc_init_t init_params = {.twi_addr = RTC_ADDR,
        .pin_int = INT_RTC,
        .p_twi_instance = p_twi_instance,
        .p_twi_cfg = &twi_config,
        .evt_handler = drv_rtc_evt_handler};

    err_code = drv_rtc_init(&init_params);

    return err_code;
}

static struct tm m_tm;
/**@brief Function for starting humidity sampling.
 */
uint32_t rtc_start(struct tm * p_tm)
{
    uint32_t err_code;
    time_t current_time;

    err_code = drv_rtc_enable();
    RETURN_IF_ERROR(err_code);         

// to get the weekday
    current_time = mktime(p_tm);
    m_tm = *localtime(&current_time);
          
    rtc_set(&m_tm);
//    rtc_get(&m_tm);
}

/**@brief Function for starting humidity sampling.
 */
uint32_t rtc_get(struct tm * p_tm)
{
    uint32_t err_code;

    err_code = drv_rtc_datetime_get(p_tm);
    RETURN_IF_ERROR(err_code);  

    m_date.year = m_tm.tm_year + 1900;
    m_date.month = m_tm.tm_mon + 1;
    m_date.mday = m_tm.tm_mday;
    m_date.wday = m_tm.tm_wday;  
     
    return err_code;
}

uint32_t rtc_set(struct tm * p_tm)
{
    uint32_t err_code;

    err_code = drv_rtc_datetime_set(p_tm);
    RETURN_IF_ERROR(err_code); 
       
    return err_code;
}
uint32_t rtc_reseal(bool enable)
{
    uint32_t err_code;
    err_code =  drv_rtc_reseal(enable);
    RETURN_IF_ERROR(err_code);

    return err_code;
}

uint32_t rtc_bat_lvl_manual_trip(void)
{
    uint32_t err_code;
    err_code =  drv_rtc_bat_lvl_manual_trip();
    RETURN_IF_ERROR(err_code);

    return err_code;
}

uint8_t rtc_bat_percentage_get(void)
{
    return drv_rtc_bat_percentage_get();
}

void calculate_currentday_rise_and_set_time(void * p_event_data, uint16_t event_size)
{
    double daylen;
    double rise, set;
    int    rs;
    time_t local_rise_time, local_set_time;
    time_t day_start_time;
    struct tm l_tm_rise, l_tm_set;
    
    time_t timestamp = *(time_t *)p_event_data;

    /* get time */
    struct tm l_tm = *localtime(&timestamp);

   // memcpy((uint8_t *)&l_tm, (uint8_t *)&m_tm, sizeof(m_tm));
    /* also need the location */
    daylen  = day_length(l_tm.tm_year + 1900, l_tm.tm_mon + 1,l_tm.tm_mday, m_geographic_location.longitude, m_geographic_location.latitude);
    rs      = sun_rise_set(l_tm.tm_year + 1900, l_tm.tm_mon + 1, l_tm.tm_mday, m_geographic_location.longitude, m_geographic_location.latitude, &rise, &set);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Day length:                 " NRF_LOG_FLOAT_MARKER " hours\n", NRF_LOG_FLOAT(daylen) );
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sun rises " NRF_LOG_FLOAT_MARKER "h UT, sets " NRF_LOG_FLOAT_MARKER "h UT\n",
          NRF_LOG_FLOAT(rise), NRF_LOG_FLOAT(set) );

    l_tm.tm_hour = 0;
    l_tm.tm_min = 0;
    l_tm.tm_sec = 0;
    day_start_time = mktime(&l_tm);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "day start time  %ld \n", day_start_time);

    local_rise_time = (double)day_start_time + rise * 3600.0 + 240.0 * m_geographic_location.longitude;
    local_set_time = (double)day_start_time + set * 3600.0 + 240.0 * m_geographic_location.longitude;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "local time sun rises  %ld, sets %ld \n", local_rise_time, local_set_time);

    m_dawn_dust_time.dawn_time = local_rise_time;
    m_dawn_dust_time.dust_time = local_set_time;
    m_dawn_dust_time.value = MAGIC_NO;

//    if(MAGIC_NO == m_pom_time.value)
//    {
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[0].timestamp = local_rise_time + m_pom_time.dawn_pom_time * 60u;
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[1].timestamp = local_set_time + m_pom_time.dust_pom_time * 60u; 
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[1].dim_level = 100;
//    }

    l_tm_rise = *localtime(&local_rise_time);
    l_tm_set = *localtime(&local_set_time);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "rises at %d:%d, sets at %d:%d \n", l_tm_rise.tm_hour,  l_tm_rise.tm_min,
           l_tm_set.tm_hour,  l_tm_set.tm_min);
 
}


static void schedule_update(void)
{
    for(uint8_t w = 0; w < DAYOFWEEK; w++)
    {
        for(uint8_t i = 0; i < MAX_SCHEDULE_ITEM; i++)
        {
            m_weekday_schedule_adaptor[w].item[i].timestamp = 
            m_weekday_schedule_adaptor[w].item[i].timestamp + 86400ul;
        }         
    }
   
}

static uint8_t hour_convert(uint8_t hour)
{
    uint8_t temp = hour < 12 ? (hour+24) : hour;
    return temp;
}

void schedule_recover(void)
{   
    struct tm l_tm;
    memcpy((uint8_t *)&l_tm, (uint8_t *)&m_tm, sizeof(m_tm));
    time_t day_start_time, timestamp;
    uint8_t h, m;

    uint8_t i = 2;
    l_tm.tm_hour = 0;
    l_tm.tm_min = 0;
    l_tm.tm_sec = 0;

    day_start_time = mktime(&l_tm);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "day start time  %ld \n", day_start_time);

//    if(MAGIC_NO == m_pom_time.value)
//    {
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[0].timestamp = m_dawn_dust_time.dawn_time + m_pom_time.dawn_pom_time * 60u;
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[1].timestamp = m_dawn_dust_time.dust_time + m_pom_time.dust_pom_time * 60u; 
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[1].dim_level = 100;
//    }

    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[l_tm.tm_wday].schedule1.on_hour,
                       m_weekday_schedule_adaptor[l_tm.tm_wday].schedule1.on_minute,
                       &h, &m)) return;
    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[l_tm.tm_wday].item[0].timestamp)
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp + 3600 * 24;
    m_weekday_schedule_adaptor[l_tm.tm_wday].item[i++].dim_level = 
    m_weekday_schedule_adaptor[l_tm.tm_wday].schedule1.on_dim_level;
//
    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[l_tm.tm_wday].schedule1.off_hour,
                       m_weekday_schedule_adaptor[l_tm.tm_wday].schedule1.off_minute,
                       &h, &m)) return;
    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[l_tm.tm_wday].item[0].timestamp)
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp + 3600 * 24;

    m_weekday_schedule_adaptor[l_tm.tm_wday].item[i++].dim_level = 
    m_weekday_schedule_adaptor[l_tm.tm_wday].schedule1.off_dim_level;

//
    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[l_tm.tm_wday].schedule2.on_hour,
                       m_weekday_schedule_adaptor[l_tm.tm_wday].schedule2.on_minute,
                       &h, &m)) return;

    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[l_tm.tm_wday].item[0].timestamp)
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp + 3600 * 24;

    m_weekday_schedule_adaptor[l_tm.tm_wday].item[i++].dim_level = 
    m_weekday_schedule_adaptor[l_tm.tm_wday].schedule2.on_dim_level;

//
    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[l_tm.tm_wday].schedule2.off_hour,
                       m_weekday_schedule_adaptor[l_tm.tm_wday].schedule2.off_minute,
                       &h, &m)) return;
    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[l_tm.tm_wday].item[0].timestamp)
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[l_tm.tm_wday].item[i].timestamp = timestamp + 3600 * 24;

    m_weekday_schedule_adaptor[l_tm.tm_wday].item[i++].dim_level = 
    m_weekday_schedule_adaptor[l_tm.tm_wday].schedule2.off_dim_level;

}

static void previous_schedule_recover(void)
{   
    struct tm l_tm;
    memcpy((uint8_t *)&l_tm, (uint8_t *)&m_tm, sizeof(m_tm));
    time_t day_start_time, timestamp;
    uint8_t h, m;

    uint8_t i = 2;
    l_tm.tm_hour = 0;
    l_tm.tm_min = 0;
    l_tm.tm_sec = 0;
    day_start_time = mktime(&l_tm);
    day_start_time -= 86400ul;
    uint8_t index = PREVIOUS_DAY(l_tm.tm_wday);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "day start time  %ld \n", day_start_time);

//    if(MAGIC_NO == m_pom_time.value)
//    {
        m_weekday_schedule_adaptor[index].item[0].timestamp = m_dawn_dust_time.dawn_time + m_pom_time.dawn_pom_time * 60u - 86400ul;
        m_weekday_schedule_adaptor[index].item[1].timestamp = m_dawn_dust_time.dust_time + m_pom_time.dust_pom_time * 60u - 86400ul; 
        m_weekday_schedule_adaptor[index].item[1].dim_level = 100;
//    }        
//
    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[index].schedule1.on_hour,
                       m_weekday_schedule_adaptor[index].schedule1.on_minute,
                       &h, &m)) return;
    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[index].item[0].timestamp)
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp + 3600 * 24;

    m_weekday_schedule_adaptor[index].item[i++].dim_level = 
    m_weekday_schedule_adaptor[index].schedule1.on_dim_level;

//
    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[index].schedule1.off_hour,
                       m_weekday_schedule_adaptor[index].schedule1.off_minute,
                       &h, &m)) return;
    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[index].item[0].timestamp)
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp + 3600 * 24;

    m_weekday_schedule_adaptor[index].item[i++].dim_level = 
    m_weekday_schedule_adaptor[index].schedule1.off_dim_level;
//
    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[index].schedule2.on_hour,
                       m_weekday_schedule_adaptor[index].schedule2.on_minute,
                       &h, &m)) return;
    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[index].item[0].timestamp)
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp + 3600 * 24;

    m_weekday_schedule_adaptor[index].item[i++].dim_level = 
    m_weekday_schedule_adaptor[index].schedule2.on_dim_level;
//
    if(!utc_hm_to_local_hm(m_weekday_schedule_adaptor[index].schedule2.off_hour,
                       m_weekday_schedule_adaptor[index].schedule2.off_minute,
                       &h, &m)) return;
    //timestamp = day_start_time + 3600 * hour_convert(h) + 60 * m;
    timestamp = day_start_time + 3600 * h + 60 * m;
    if(timestamp > m_weekday_schedule_adaptor[index].item[0].timestamp)
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp;
    else 
        m_weekday_schedule_adaptor[index].item[i].timestamp = timestamp + 3600 * 24;

    m_weekday_schedule_adaptor[index].item[i++].dim_level = 
    m_weekday_schedule_adaptor[index].schedule2.off_dim_level;
}


void schedule_store(void * p_event_data, uint16_t event_size)
{
    usrdata_store((uint8_t *)m_weekday_schedule_adaptor, FLASH_HANDLE_USRDATA | FLASH_HANDLE_SCHEDULE_ID, 
                  sizeof(m_weekday_schedule_adaptor));    
}

void location_store(void * p_event_data, uint16_t event_size)
{
    usrdata_store((uint8_t *)&m_geographic_location, FLASH_HANDLE_USRDATA | FLASH_HANDLE_POSITION_ID, 
                  sizeof(m_geographic_location));    
}

void ut_time_store(void * p_event_data, uint16_t event_size)
{
    usrdata_store((uint8_t *)&m_ut_time, FLASH_HANDLE_USRDATA | FLASH_HANDLE_TIME_ID, 
                  sizeof(m_ut_time));    
}


void pom_time_store(void * p_event_data, uint16_t event_size)
{
    usrdata_store((uint8_t *)&m_pom_time, FLASH_HANDLE_USRDATA | FLASH_HANDLE_POM_ID, 
                  sizeof(m_pom_time));    
}

void control_store(void * p_event_data, uint16_t event_size)
{
    usrdata_store((uint8_t *)&m_control, FLASH_HANDLE_USRDATA | FLASH_HANDLE_CTR_ID, 
                  sizeof(m_control));    
}

void ambient_light_store(void * p_event_data, uint16_t event_size)
{
    usrdata_store((uint8_t *)&m_ambient_light_th, FLASH_HANDLE_USRDATA | FLASH_HANDLE_AMBIENT_LIGHT_ID, 
                  sizeof(m_ambient_light_th));    
}

void sensor_params_store(void * p_event_data, uint16_t event_size)
{
    usrdata_store((uint8_t *)&m_sensor_params, FLASH_HANDLE_USRDATA | FLASH_HANDLE_SENSOR_PARAMS_ID, 
                  sizeof(sensor_params_t));    
}


uint32_t reset_operate(void)
{
    uint32_t err_code;
    uint32_t lenth = sizeof(m_ut_time);

    if(usrdata_load((uint8_t *)&m_ut_time, FLASH_HANDLE_USRDATA | FLASH_HANDLE_TIME_ID, &lenth))
    {
        err_code = drv_rtc_enable(); 
        RETURN_IF_ERROR(err_code); 
        err_code = rtc_get(&m_tm);
        RETURN_IF_ERROR(err_code);         
    }

    lenth = sizeof(m_pom_time);
    if(usrdata_load((uint8_t *)&m_pom_time, FLASH_HANDLE_USRDATA | FLASH_HANDLE_POM_ID, &lenth))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "load plus or minute success\n");
    }
          
    lenth = sizeof(m_geographic_location);
    if(usrdata_load((uint8_t *)&m_geographic_location, FLASH_HANDLE_USRDATA | FLASH_HANDLE_POSITION_ID, &lenth))
    {
        time_t timestamp = mktime(&m_tm);
        calculate_currentday_rise_and_set_time(&timestamp, sizeof(timestamp));
    }            
    lenth = sizeof(m_weekday_schedule_adaptor);
    if(usrdata_load((uint8_t *)m_weekday_schedule_adaptor, FLASH_HANDLE_USRDATA | FLASH_HANDLE_SCHEDULE_ID, &lenth))
    {
        schedule_recover();
        previous_schedule_recover();
    }
    else
    {
        
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "load schedule fail\n");
    }


    lenth = sizeof(m_control);
    if(usrdata_load((uint8_t *)&m_control, FLASH_HANDLE_USRDATA | FLASH_HANDLE_CTR_ID, &lenth))
    {
        
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "load control fail\n");
    }

    lenth = sizeof(m_sensor_params);
    if(usrdata_load((uint8_t *)&m_sensor_params, FLASH_HANDLE_USRDATA | FLASH_HANDLE_SENSOR_PARAMS_ID, &lenth))
    {
        
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "load sensor params fail\n");
    }
    return NRF_SUCCESS;
}

void set_dim_level_percentage(uint8_t percentage)
{
    if(percentage > 100) return;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "tx set level percentage\n");

    uint8_t cmd[4] = {0x3e,0x44, 0x00, 0x3c};
    uint16_t dim_level = 255 * percentage / 100;
    cmd[2] = dim_level;

    m_motion_previous_dim_level = dim_level;
    app_uart_string_put(cmd, 4);
}

void set_dim_level(uint8_t level)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "tx set level %d\n", level);
    uint8_t cmd[4] = {0x3e,0x44, 0x00, 0x3c};
    cmd[2] = level;
    app_uart_string_put(cmd, 4);
}

void set_manual_mode(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tx set manual mode\n");
    uint8_t cmd[4] = {0x3e,0x4d, 0x00, 0x3c};

    app_uart_string_put(cmd, 4);
}

void set_automatic_mode(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tx set automatic mode\n");
    uint8_t cmd[4] = {0x3e,0x41, 0x00, 0x3c};

    app_uart_string_put(cmd, 4);    
}

void settings_req(void)
{
    uint8_t cmd[4] = {0x3e,0x55, 0x00, 0x3c};
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tx set settings req\n");
    app_uart_string_put(cmd, 4);
}

void settings_set(void)
{
    uint8_t cmd[4] = {0x3e,0x00, 0x00, 0x3c};
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tx set settings\n");

//    cmd[1] = 0x5a;
//    cmd[2] = m_sensor_params.delay1.hours;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x59;
//    cmd[2] = m_sensor_params.delay1.minutes;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x58;
//    cmd[2] = m_sensor_params.delay1.seconds;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x4a;
//    cmd[2] = m_sensor_params.delay2.hours;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x49;
//    cmd[2] = m_sensor_params.delay2.minutes;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x48;
//    cmd[2] = m_sensor_params.delay2.seconds;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x4c;
//    cmd[2] = m_sensor_params.ambient_light_threshold1;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x50;
//    cmd[2] = m_sensor_params.dim_level;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);
//
//    cmd[1] = 0x51;
//    cmd[2] = m_sensor_params.motion_level;
//    app_uart_string_put(cmd, 4);
//    nrf_delay_ms(10);

    cmd[1] = 0x53;
    cmd[2] = m_sensor_params.sensitivity;
    app_uart_string_put(cmd, 4);
    nrf_delay_ms(10);
}


static void motion_event_to_status(event_trigger_t event)
{
    if(MOTION_EVENT == event)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Motion action! \n");
        m_motion_ack_status = MOTION;
//        if(MAGIC_NO == m_ut_time.value)
//            new_task_cover(m_current_time, m_sensor_params.dim_level);               
    }
        
    else if(TIME_DELAY1 == event)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Td1 action! \n");
        m_motion_ack_status = TD1;
    }
         
    else if(TIME_DELAY2 == event)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Td2 action! \n");
        m_motion_ack_status = INIT;        
    }
     
}

void motion_trigger_set_dim_level(event_trigger_t event, uint8_t level)
{
    bool action = true;

    if(!m_control.motion_zone_ctr) 
    {
        return;
    }

//    if(!m_control.motion) 
//    {
//        return;
//    }

    
    if(m_control.astro_clock && SUN_RISE == m_day_condition)
    {
       return;                  
    }

    if(m_control.ambient_light && SUN_RISE == m_day_condition)
    {
       return;
    }

    if(action)
    {
        throughput_set_params_t params;
        params.tid = 0;
        params.cmd = event;
        motion_event_to_status(event);
        m_schedule_action = 0; 
        set_dim_level(level);
        params.params = level;
        throughput_client_event_set_unack(&m_throughput_client, &params, 1);  
 
    }
}


static void schedule_trigger_set_dim_percentage(uint8_t weekday, uint8_t percentage, bool relay)
{
    throughput_set_params_t params;
    params.tid = 0;
    params.cmd = SCHEDULE_EVENT;
    bool action = true;
    uint8_t dim_level;

    if(!m_weekday_schedule_adaptor[weekday].onoff)
        return;

    if(m_control.motion_zone_ctr && m_motion_ack_status > INIT) 
    {
        return;       
    }

    if(m_control.ambient_light && SUN_RISE == m_day_condition)
    {
        return;
    }

    if(action)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "schedule %d action percentage %d!\n", weekday, percentage); 
        set_dim_level_percentage(percentage);
        m_schedule_action = 1; 
        if(relay)
        {
            params.params = 255 * percentage / 100;
            throughput_client_event_set_unack(&m_throughput_client, &params, 1);  
        }
    }
}

static void ambient_trigger_set_dim_level(uint8_t level, event_trigger_t event, bool relay)
{
    throughput_set_params_t params;
    params.tid = 0;
    params.cmd = event;
    bool action = true;
    uint8_t dim_level;

    if(!m_control.ambient_light) 
    {
        return;
    }

    if(DAWN == event)
    {
        if(m_control.motion_zone_ctr && m_motion_ack_status > INIT)
        {
            // to do: recover td1 td2 task
            delete_all_tasks();
            m_motion_ack_status = INIT;
        }        
    }
    else if(DUST == event)
    {
        if(m_control.motion_zone_ctr && m_motion_ack_status > INIT)
        {
            action = false;
        }
        if(m_schedule_action)
        {
            action = false;
        }
    }


    if(action)
    { 
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ambient action %d \n", level);
        m_schedule_action = 0; 
        set_dim_level(level);
        m_motion_previous_dim_level = level;
        if(relay)
        {
            params.params = level;
            throughput_client_event_set_unack(&m_throughput_client, &params, 1);  
        }
    }
}

static void astro_trigger_set_dim_level(uint8_t level, event_trigger_t event, bool relay)
{
    throughput_set_params_t params;
    params.tid = 0;
    params.cmd = event;
    bool action = true;
    uint8_t dim_level;

    if(!m_control.astro_clock) 
    {
        return;
    }

    if(DAWN == event)
    {
        if(m_control.motion_zone_ctr && m_motion_ack_status > INIT)
        {
            // to do: recover td1 td2 task
            delete_all_tasks();
            m_motion_ack_status = INIT;
        }        
    }
    else if(DUST == event)
    {
        if(m_control.motion_zone_ctr && m_motion_ack_status > INIT)
        {
            action = false;
        } 
    }


    if(action)
    { 
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "astro action %d\n", level);
        m_schedule_action = 0; 
        set_dim_level(level);
        m_motion_previous_dim_level = level;
        if(relay)
        {
            params.params = level;
            throughput_client_event_set_unack(&m_throughput_client, &params, 1);  
        }
    }

}


static uint32_t led_on_allow_event_publish(uint8_t allow)
{
    uint32_t err_code;
    uint8_t index = ALI_POS_VALID_DATA_START;
    uint16_t req_type = ALI_VALUE_TYPE_EVENTS;
    uint8_t msg[10] = {0};
    msg[0] = m_tid++;
    memcpy(&msg[index], &req_type, ALI_SIZE_VALUE_TYPE);
    index += ALI_SIZE_VALUE_TYPE;
    msg[index++] = 0x83;
    msg[index++] = allow;

    err_code = ali_extended_server_message_publish(&m_ali_server, msg, index);
    return err_code;
}

static bool timing_task_handler(void * p_event_data, uint16_t event_size)
{
    const nrf_sortlist_item_t * p_item = sortlist_peek();
    if (NULL == p_item)
        return false;
    item_t * p_timing_item = CONTAINER_OF(p_item, item_t, item);
    time_t timestamp = *(time_t *)p_event_data; 
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "tick,%ld \n", p_timing_item->timing_task.future_timestamp);
    control_t *p_ctr;
    control_get(&p_ctr);
    if (p_timing_item->timing_task.future_timestamp < timestamp)
    {
        if(1 == p_timing_item->timing_task.index)
        {           
            motion_trigger_set_dim_level(TIME_DELAY1, p_timing_item->timing_task.level);
        }
        else if(2 == p_timing_item->timing_task.index)
        {
            motion_trigger_set_dim_level(TIME_DELAY2, p_timing_item->timing_task.level);
        }
        
        p_item = sortlist_pop(); 
        timing_task_mem_free(p_timing_item);
        return true;
 
    }
    return true;
}

void motion_detect_handler(void)
{
    control_t *p_ctr;
    control_get(&p_ctr);    
    //m_motion_ack_status = MOTION;
    motion_trigger_set_dim_level(MOTION_EVENT, m_sensor_params.motion_level);
}


APP_TIMER_DEF(m_blink_timer);
static uint32_t m_blink_count;
static uint32_t m_blink_mask;
static bool m_prev_state;
/** Lowest possible blinking period in milliseconds. */
#define LED_BLINK_PERIOD_MIN_MS (50)

static void led_timeout_handler(void * p_context)
{
    APP_ERROR_CHECK_BOOL(m_blink_count > 0);

    if(m_prev_state)
    {
        set_dim_level(0);
        m_motion_previous_dim_level = 0;
        m_prev_state = false;
    }
    else
    {
        set_dim_level(255);
        m_motion_previous_dim_level = 255;
        m_prev_state = true;
    }
    m_blink_count--;
    if (m_blink_count == 0)
    {
        (void) app_timer_stop(m_blink_timer);
    }
}


void led_blink_ms_uart_cmd(uint32_t delay_ms, uint32_t blink_count)
{
    if (blink_count == 0 || delay_ms < LED_BLINK_PERIOD_MIN_MS)
    {
        return;
    }

    m_blink_count = blink_count * 2 - 1;

    if (app_timer_start(m_blink_timer, APP_TIMER_TICKS(delay_ms), NULL) == NRF_SUCCESS)
    {
        /* Start by "clearing" the mask, i.e., turn the LEDs on -- in case a user calls the
         * function twice. */
        set_manual_mode();
        set_dim_level(0);
        m_motion_previous_dim_level = 0;
        m_prev_state = false;
  
    }
}

void leds_init_uart_cmd(void)
{
    APP_ERROR_CHECK(app_timer_create(&m_blink_timer, APP_TIMER_MODE_REPEATED, led_timeout_handler));
}

