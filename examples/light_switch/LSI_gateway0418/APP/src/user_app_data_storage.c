#include "user_app_data_storage.h"
#include "nrf_mesh_assert.h"
#include "device_state_manager.h"
#include "log.h"


extern const void * access_flash_area_get(void);

/** Global flag to keep track of if the flash_manager is not ready for use (being erased or not added) .*/
static bool m_flash_not_ready;

/* The flash manager instance used by this module. */
static flash_manager_t m_flash_manager;

static usr_storage_t m_usr_data;

static void flash_write_complete(const flash_manager_t * p_manager, const fm_entry_t * p_entry, fm_result_t result)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "result %d\n", result);
    /* If we get an AREA_FULL then our calculations for flash space required are buggy. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_AREA_FULL);
    /* We do not invalidate in this module, so a NOT_FOUND should not be received. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_NOT_FOUND);

   
    if (result == FM_RESULT_ERROR_FLASH_MALFUNCTION)
    {
        /* Let the user know that the flash is dying. */
//        nrf_mesh_evt_t evt = {
//            .type = NRF_MESH_EVT_FLASH_FAILED,
//            .params.flash_failed.user = NRF_MESH_FLASH_USER_ACCESS,
//            .params.flash_failed.p_flash_entry = p_entry,
//            .params.flash_failed.p_flash_page = NULL,
//            .params.flash_failed.p_area = p_manager->config.p_area,
//            .params.flash_failed.page_count = p_manager->config.page_count
//        };
//        event_handle(&evt);
    }
}

static void flash_invalidate_complete(const flash_manager_t * p_manager, fm_handle_t handle, fm_result_t result)
{
    /* Expect no invalidate complete calls. */
    NRF_MESH_ASSERT(false);
}

static void add_flash_manager(void);
static void flash_remove_complete(const flash_manager_t * p_manager)
{
//    mark_all_as_outdated();
    add_flash_manager();
}


/** Flash operation function to call when the memory returns. */
typedef void (*flash_op_func_t)(void);

static void flash_mem_listener_callback(void * p_args)
{
    NRF_MESH_ASSERT(p_args != NULL);
    flash_op_func_t func = (flash_op_func_t) p_args; /*lint !e611 Suspicious cast */
    func();
}


/**
 * Erase all entries, and re-add up to date metainfo once removal is complete.
 */
static void reset_flash_area(void)
{
    m_flash_not_ready = false;
    if (flash_manager_remove(&m_flash_manager) != NRF_SUCCESS)
    {
         __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "user remmove err\n");
        /* Register the listener and wait for some memory to be freed up before we retry. */
        static fm_mem_listener_t mem_listener = {.callback = flash_mem_listener_callback,
                                                 .p_args = reset_flash_area};
        flash_manager_mem_listener_register(&mem_listener);
    }
}

void userdata_clear(void)
{
  reset_flash_area();
}

typedef void (*flash_op_func_t) (void);
static void flash_manager_mem_available(void * p_args)
{
    ((flash_op_func_t) p_args)(); /*lint !e611 Suspicious cast */
}

#define  USER_DATA_FLASH_PAGE_COUNT  2
static void add_flash_manager(void)
{

    static fm_mem_listener_t flash_add_mem_available_struct = {
        .callback = flash_manager_mem_available,
        .p_args = add_flash_manager
    };
    flash_manager_config_t manager_config;
    manager_config.write_complete_cb = flash_write_complete;
    manager_config.invalidate_complete_cb = flash_invalidate_complete;
    manager_config.remove_complete_cb = flash_remove_complete;
    manager_config.min_available_space = WORD_SIZE;
#ifdef ACCESS_FLASH_AREA_LOCATION
    manager_config.p_area = (const flash_manager_page_t *) ACCESS_FLASH_AREA_LOCATION;
#else
    manager_config.p_area = (const flash_manager_page_t *) (((const uint8_t *) access_flash_area_get()) - ((ACCESS_FLASH_PAGE_COUNT + USER_DATA_FLASH_PAGE_COUNT) * PAGE_SIZE));
#endif
    manager_config.page_count = USER_DATA_FLASH_PAGE_COUNT;
    m_flash_not_ready = true;
    uint32_t status = flash_manager_add(&m_flash_manager, &manager_config);
    if (NRF_SUCCESS != status)
    {
        flash_manager_mem_listener_register(&flash_add_mem_available_struct);
    }
    else
    {
        m_flash_not_ready = false;
    }
}

void user_add_flash_manager(void)
{
    add_flash_manager();
}

uint32_t usrdata_store(uint8_t * p_data, fm_handle_t handle, uint16_t length)
{
    if(length > SIZE)
        return NRF_ERROR_DATA_SIZE;

    fm_entry_t * p_entry = flash_manager_entry_alloc(&m_flash_manager, handle, length);

    if (p_entry == NULL)
    {
        return NRF_ERROR_BUSY;
    }
    else
    {
        uint8_t * p_usr_data = (uint8_t *) p_entry->data;
        memcpy(p_usr_data, p_data, length);
//        p_metadata->element_count = ACCESS_ELEMENT_COUNT;
//        p_metadata->model_count = ACCESS_MODEL_COUNT;
//        p_metadata->subscription_list_count = ACCESS_SUBSCRIPTION_LIST_COUNT;
        flash_manager_entry_commit(p_entry);
//        m_metadata_stored = true;
        return NRF_SUCCESS;
    }
}

bool usrdata_load(uint8_t * p_data, fm_handle_t handle, uint32_t * p_lenth)
{
    uint32_t err_code;
    flash_manager_wait();
    err_code = flash_manager_entry_read(&m_flash_manager, handle, p_data, p_lenth);
    if(err_code == NRF_SUCCESS)
       return true; 
    else
    {
       __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "err:%x\n", err_code);
        return false; 
    }          
}

