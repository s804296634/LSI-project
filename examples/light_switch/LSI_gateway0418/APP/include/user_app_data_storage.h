#ifndef USER_APP_DATA_STORAGE_H__
#define USER_APP_DATA_STORAGE_H__

#include "flash_manager.h"
#include "config_server_events.h"

#define SIZE 512
#define FLASH_HANDLE_USRDATA          0x4000
#define FLASH_HANDLE_ISTOUCHED_ID     0x01
#define FLASH_HANDLE_UNTOUCHED_CNT_ID 0x02
#define FLASH_HANDLE_MAJOR_ID         0x03
#define FLASH_HANDLE_MINOR_ID         0x04
#define FLASH_HANDLE_UUID_ID          0x05
#define FLASH_HANDLE_RSSI_ID          0x06
#define FLASH_HANDLE_TX_POWER_ID      0x07
#define FLASH_HANDLE_ADV_INT_ID       0x08
#define FLASH_HANDLE_BEACON_ALL_ID    0x09
#define FLASH_HANDLE_PROXY_ID         0x0a
#define FLASH_HANDLE_RELAY_ID         0x0b
#define FLASH_HANDLE_TTL_ID           0x0c
#define FLASH_HANDLE_NETWORK_TRANSMIT_ID 0x0D
#define FLASH_HANDLE_FILTER_RSSI_ID      0x0F

#define FLASH_HANDLE_SCHEDULE_ID           0x10
#define FLASH_HANDLE_POSITION_ID           0x11
#define FLASH_HANDLE_TIME_ID               0x12
#define FLASH_HANDLE_POM_ID                0x13
#define FLASH_HANDLE_CTR_ID                0x14
#define FLASH_HANDLE_AMBIENT_LIGHT_ID      0x15
#define FLASH_HANDLE_SENSOR_PARAMS_ID      0x16

typedef  struct{
    uint8_t data[SIZE];
} usr_storage_t;

void user_add_flash_manager(void);
uint32_t usrdata_store(uint8_t * p_data, fm_handle_t handle, uint16_t length);
bool usrdata_load(uint8_t * p_data, fm_handle_t handle, uint32_t * p_lenth);
void userdata_clear(void);


#endif
