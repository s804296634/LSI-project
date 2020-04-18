#ifndef TIMING_H__
#define TIMING_H__

#include <stdint.h>
#include "nrf_sortlist.h"
#include "ali_extended_server.h"

typedef struct
{
  uint8_t index;
  uint8_t level;
  uint32_t future_timestamp;
} timing_task_item_t;

typedef struct
{
  timing_task_item_t timing_task;
  nrf_sortlist_item_t item;
}item_t;

bool compare_function(nrf_sortlist_item_t * p_item0, nrf_sortlist_item_t * p_item1);
void timing_task_list_init(uint32_t timestamp);
void timing_task_mem_init(void);
void timing_task_mem_free(void * p_buffer);
item_t * timing_task_mem_molloc(void);

nrf_sortlist_item_t const * sortlist_peek(void);
nrf_sortlist_item_t const * sortlist_pop(void);
void sortlist_add(nrf_sortlist_item_t * p_item);
uint32_t timing_task_remove_by_index(uint8_t index);
bool item_find_by_index(uint8_t index, item_t ** pp_timing_item);

uint32_t new_task_cover(time_t timestamp, uint8_t dim_level);
void delete_all_tasks(void);

#endif