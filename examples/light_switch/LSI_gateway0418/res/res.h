#ifndef __RESPOSITORY_H__
#define __RESPOSITORY_H__

#include "user_app_data_storage.h"
#include "ali_extended_server.h"
#include "time.h"
#include "radio_config.h"
#include "mesh_opt_core.h"
#include <stdint.h>

/* uuid related macro */
#define PID_BIT_ADV_VER           0x00
#define PID_BIT_KEY               0x04
#define PID_BIT_OTA               0x05
#define PID_BIT_BLE_VER           0x06

#define ADV_VER                   0x01
#define KEY_TYPE                  0x00
#define IS_OTA                    0x01
#define BLE_VER                   0x01

#define UUID_CID_INDEX            0x00
#define UUID_PID_INDEX            0x02
#define UUID_PRODUCTID_INDEX      0x03
#define UUID_PROJECTID_INDEX      0x05
#define UUID_MAC_INDEX            0x07
#define UUID_VID_INDEX            0x0D

#define MAX_SERIALCMA_LENGTH  (256)

#define DEVICE_PROJECT_ID        (0x1923)
#define ADDR0          0xF000
#define ADDR1          0xF001

#define DAYOFWEEK  7
#define PREVIOUS_DAY(n)      ((n+6)%7)

#define SCL_RTC           0x07
#define SDA_RTC           0x06
#define INT_RTC           0x08
#define RTC_ADDR          0x6F


#define TYPE_INDEX                0x00
#define CMD_INDEX                 0x01 
#define VALUE_INDEX               0x02
#define DEST_ADDR_INDEX           0x01
#define SOURCE_ADDR_INDEX         0x03
     
#define DATA_LENGTH_INDEX         0x05
#define DATA_OPCODE_INDEX         0x07
#define DATA_PARAMS_INDEX         0x09

#define DATA_PARAMS_INDEX_VENDOR  0x0E
#define DATA_PARAMS_INDEX_SIG_TWO 0x0D
#define DATA_PARAMS_INDEX_SIG_ONE 0x0C

#define TYPE_CMD                  0x00
#define TYPE_MESSAGE              0x01
#define TYPE_STATUS               0x02

#define TYPE_TYPE_BITMASK         0x0F
#define TYPE_VERSION_BITMASK      0xF0

#define CMD_READ_MODULE_STATUS    0x00
#define CMD_SET_UUID              0x01
#define CMD_START_NETWORKING      0x02
#define CMD_GET_NODE_ADDR         0x03
#define CMD_GET_NETWORK_KEY       0x04

#define VALUE_SUCCESS             0x00
#define VALUE_FAIL                0x01

#define MSG_SIZE 255
typedef struct __attribute((packed))
{
  uint8_t type;
  uint16_t dest;
  uint16_t src;
  uint16_t len;
  uint16_t opcode;
  uint8_t data[MSG_SIZE];
}SERIAL_PRAMS_T;

typedef struct __attribute((packed))
{
  uint8_t type;
  uint16_t opcode;
  uint8_t status_code;  
}SERIAL_ACK_STATUS_T;

typedef enum
{
  STATUS_RECEIVED = 0,
  STATUS_TIMEOUT = 0x80,
  STATUS_ATTR_NOT_SUPPORT = 0x81,
  STATUS_OPERATION_NOT_SUPPORT = 0x82,
  STATUS_PARAMS_ERROR = 0x83 ,
  STATUS_BUSY = 0x84,
  STATUS_UNKNOWN,
}ACK_STATUS_T;

typedef struct
{
  uint16_t opcode;
}ack_status_params_t;

/**
 * @brief Macro to be used in a formatted string to a pass float number to the log.
 *
 * Macro should be used in formatted string instead of the %f specifier together with
 * @ref NRF_LOG_FLOAT macro.
 * Example: NRF_LOG_INFO("My float number" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(f)))
 */
#define NRF_LOG_FLOAT_MARKER "%s%d.%02d"

/**
 * @brief Macro for dissecting a float number into two numbers (integer and residuum).
 */
#define NRF_LOG_FLOAT(val) (uint32_t)(((val) < 0 && (val) > -1.0) ? "-" : ""),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*100)


typedef enum{
    LIGHT_SUBSCRIBE_ADDR     = 0xC000,
    SWITCH_SUBSCRIBE_ADDR    = 0xC001,
    SOCKET_SUBSCRIBE_ADDR    = 0xC002,
    IOT_SUBSCRIBE_ADDR       = 0xCFFF,
}ALL_IOT_SUBSCRIBE_T;

typedef struct motion_status
{
    uint8_t motion_event;
    uint8_t updated;
}motion_status_t;

typedef struct light_level_status
{
    uint8_t light_level;
    uint8_t updated;
}light_level_status_t;
typedef struct light_reading_status
{
    uint8_t light_reading;
    uint8_t updated;
}light_reading_status_t;

typedef struct lsi_status
{
    motion_status_t motion_status;
    light_level_status_t light_level_status;
    light_reading_status_t light_reading_status;
}lsi_status_t;

typedef enum{
    START,
    CMD,
    PARAM,
    STOP,
    NOTREADY
}uart_received_status_t;

typedef struct time_delay
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
}time_delay_t;

typedef enum{
    LOWER                                 = 0x00,
    NORMAL,                    
    UPPER                   
}ambient_light_status_t;

typedef enum{
    MOTION_EVENT                          = 0x01,
    DAWN,   
    DUST,
    AMBIENT, 
    SCHEDULE_EVENT,
    TIME_DELAY1,
    TIME_DELAY2
}event_trigger_t;

typedef enum{
    UNKNOWN,
    SUN_RISE,
    SUN_SET
}day_condition_t;

typedef struct sensor_params
{
    uint8_t motion_level;
    uint8_t dim_level;
    time_delay_t delay1;
    time_delay_t delay2;
    uint16_t  ambient_light_threshold1; 
    uint16_t  ambient_light_threshold2; 
    uint8_t sensitivity;
}sensor_params_t;

typedef void (*provisioned_call_back_func_t)(void);

extern void net_beacon_adv_tx_power_set(int8_t power);

void set_motion_status(motion_ack_status_t status);
void clear_schedule_action(void);

void node_reset(void);
void hd_init(void);
void hd_uuid_set(uint8_t *p_uuid);
uint32_t config_process(provisioned_call_back_func_t func);
void reset_publish(uint16_t id);

void provison_check_timer_start(void);
void feed_dog(void);
void app_uart_string_put(void * p_string, uint16_t size);
void ota_timer_start(void);
void reset_timer_start(void);

void get_lsi_status(lsi_status_t ** pp_status);

void weekday_schedule_adaptor_get(weekday_schedule_adaptor_t **pp);
void geographic_location_get(geographic_location_t **pp);
void pom_time_get(pom_time_t **pp);
void ut_time_get(time_ut_t **pp);
void dawndust_time_get(dawn_dust_time_t **pp);
void tm_get(struct tm **pp);
void control_get(control_t **pp);
void ambient_light_th_get(ambient_light_threshold_t **pp);
void sensor_params_get(sensor_params_t **pp);

void cal_init(void);
uint32_t rtc_start(struct tm * p_tm);
uint32_t rtc_get(struct tm * p_tm);
uint32_t rtc_set(struct tm * p_tm);
uint32_t rtc_reseal(bool enable);

void calculate_currentday_rise_and_set_time(void * p_event_data, uint16_t event_size);
uint32_t reset_operate(void);
void schedule_store(void * p_event_data, uint16_t event_size);
void location_store(void * p_event_data, uint16_t event_size);
void ut_time_store(void * p_event_data, uint16_t event_size);
void pom_time_store(void * p_event_data, uint16_t event_size);
void control_store(void * p_event_data, uint16_t event_size);
void ambient_light_store(void * p_event_data, uint16_t event_size);
void sensor_params_store(void * p_event_data, uint16_t event_size);

void schedule_recover(void);
bool utc_hm_to_local_hm(uint8_t h, uint8_t m, uint8_t * p_h, uint8_t * p_m);
void local_hm_to_utc_hm(uint8_t h, uint8_t m, uint8_t * p_h, uint8_t * p_m);

void set_dim_level(uint8_t level);
void set_dim_level_percentage(uint8_t percentage);

void set_manual_mode(void);
void set_automatic_mode(void);
void settings_req(void);
void settings_set(void);

void motion_trigger_set_dim_level(event_trigger_t event, uint8_t level);
time_t timestamp_get(void);

void motion_detect_handler(void);

void leds_init_uart_cmd(void);
void led_blink_ms_uart_cmd(uint32_t delay_ms, uint32_t blink_count);

uint32_t rtc_bat_lvl_manual_trip(void);
uint8_t rtc_bat_percentage_get(void);

uint8_t get_z8_version(void);

void connect_force_relay(void);
void disconnect_relay_restore(void);
void reset_force_relay();

#endif


