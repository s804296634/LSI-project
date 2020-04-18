#include "timing.h"
#include "res.h"
#include "log.h"

#include "app_util.h"
#include "mem_manager.h"
#include "app_uart.h"

#include <stdint.h>
#include <string.h>

NRF_SORTLIST_DEF(timing_task_list, compare_function);
static item_t m_item0 = {0};

//extern uint8_t m_motion_level;
//extern uint8_t m_dim_level;
//extern time_delay_t m_delay1;
//extern time_delay_t m_delay2;
extern uint8_t m_motion_previous_dim_level;

bool compare_function(nrf_sortlist_item_t * p_item0, nrf_sortlist_item_t * p_item1)
{
    item_t * p_timing_item0 = CONTAINER_OF(p_item0, item_t, item);
    item_t * p_timing_item1 = CONTAINER_OF(p_item1, item_t, item);
    return (p_timing_item0->timing_task.future_timestamp <
               p_timing_item1->timing_task.future_timestamp)
               ? true
               : false;
}


void timing_task_mem_init(void)
{
    uint32_t err_code;
    err_code = nrf_mem_init();
}

void timing_task_mem_free(void * p_buffer) { nrf_free(p_buffer); }

item_t * timing_task_mem_molloc(void) { return nrf_malloc(sizeof(item_t)); }

nrf_sortlist_item_t const * sortlist_peek(void) { return nrf_sortlist_peek(&timing_task_list); }

nrf_sortlist_item_t const * sortlist_pop(void) { return nrf_sortlist_pop(&timing_task_list); }

void sortlist_add(nrf_sortlist_item_t * p_item) { nrf_sortlist_add(&timing_task_list, p_item); }

uint32_t timing_task_remove_by_index(uint8_t index)
{
    nrf_sortlist_item_t const * p_item;
    item_t * p_timing_item;
    p_item = nrf_sortlist_peek(&timing_task_list);

    if (NULL == p_item || 0 == index)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        p_timing_item = CONTAINER_OF(p_item, item_t, item);
        if (p_timing_item->timing_task.index == index)
        {
            p_item = nrf_sortlist_pop(&timing_task_list);
            nrf_free(p_timing_item);
            return NRF_SUCCESS;
        }
    }
    while (true)
    {
        p_item = nrf_sortlist_next(p_item);
        if (p_item == NULL)
            return NRF_ERROR_NOT_FOUND;
        else
        {
            p_timing_item = CONTAINER_OF(p_item, item_t, item);
            if (p_timing_item->timing_task.index == index)
            {
                if (nrf_sortlist_remove(&timing_task_list, (nrf_sortlist_item_t *)p_item))
                {
                    nrf_free(p_timing_item);
                    return NRF_SUCCESS;
                }

                else
                    return NRF_ERROR_INTERNAL;
            }
        }
    }
}

bool item_find_by_index(uint8_t index, item_t ** pp_timing_item)
{
    nrf_sortlist_item_t const * p_item;
    item_t * p_timing_item;
    p_item = nrf_sortlist_peek(&timing_task_list);

    if (NULL == pp_timing_item || 0 == index)
    {
        return false;
    }
    else
    {
        p_timing_item = CONTAINER_OF(p_item, item_t, item);
        if (p_timing_item->timing_task.index == index)
        {
            *pp_timing_item = p_timing_item;
            return true;
        }
    }

    while (true)
    {
        p_item = nrf_sortlist_next(p_item);
        if (p_item == NULL)
            return false;
        else
        {
            p_timing_item = CONTAINER_OF(p_item, item_t, item);
            if (p_timing_item->timing_task.index == index)
            {
                *pp_timing_item = p_timing_item;
            }
        }
    }
}


uint32_t new_task_cover(time_t timestamp, uint8_t dim_level)
{
    nrf_sortlist_item_t const * p_item;
    item_t * p_timing_item;
    item_t * p_dealy1_item = timing_task_mem_molloc();
    item_t * p_dealy2_item = timing_task_mem_molloc();

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "current time:%ld, dim_level: %d \n", timestamp, dim_level);

    sensor_params_t *p_sensor_params;
    sensor_params_get(&p_sensor_params);
    
    for(uint8_t i = 0; i < 2; i++)
    {
        p_item = nrf_sortlist_peek(&timing_task_list);
        if (NULL == p_item)
        {
            break;
        }
        p_timing_item = CONTAINER_OF(p_item, item_t, item);
        p_item = nrf_sortlist_pop(&timing_task_list); 
        nrf_free(p_timing_item);
    }

    p_dealy1_item->timing_task.index = 1;
    p_dealy1_item->timing_task.level = p_sensor_params->dim_level;
    p_dealy1_item->timing_task.future_timestamp =
        timestamp + p_sensor_params->delay1.seconds + p_sensor_params->delay1.minutes * 60 + p_sensor_params->delay1.hours * 3600;
    if(p_dealy1_item->timing_task.future_timestamp > timestamp)
    {
        sortlist_add(&p_dealy1_item->item);  
        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "TD1 timestamp %ld \n", p_dealy1_item->timing_task.future_timestamp);
    }
    else
    {
        nrf_free(p_dealy1_item);
    }

    p_dealy2_item->timing_task.index = 2;
    p_dealy2_item->timing_task.level = m_motion_previous_dim_level;
    p_dealy2_item->timing_task.future_timestamp =
        p_dealy1_item->timing_task.future_timestamp + p_sensor_params->delay2.seconds + p_sensor_params->delay2.minutes * 60 + p_sensor_params->delay2.hours * 3600;

    if(p_dealy2_item->timing_task.future_timestamp > timestamp)
    {
        sortlist_add(&p_dealy2_item->item);
        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "TD2 timestamp %ld \n", p_dealy2_item->timing_task.future_timestamp);
    }
    else
    {
        nrf_free(p_dealy2_item);
    }    

    return NRF_SUCCESS;
}

void delete_all_tasks(void)
{
   nrf_sortlist_item_t const * p_item;
   item_t * p_timing_item;

   for(uint8_t i = 0; i < 2; i++)
    {
        p_item = nrf_sortlist_peek(&timing_task_list);
        if (NULL == p_item)
        {
            break;
        }
        p_timing_item = CONTAINER_OF(p_item, item_t, item);
        p_item = nrf_sortlist_pop(&timing_task_list); 
        nrf_free(p_timing_item);
    }    
}