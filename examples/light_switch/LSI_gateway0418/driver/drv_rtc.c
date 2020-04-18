/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include "drv_rtc.h"
#include "twi_manager.h"
#include "drv_isl12022.h"
#include "nrf_drv_gpiote.h"
#include "app_scheduler.h"
//#define  NRF_LOG_MODULE_NAME "drv_humidity  "
#include "log.h"
#include "macros_common.h"
#include "app_timer.h"

APP_TIMER_DEF(rtc_timer_id);

/**@brief Configuration struct.
 */
typedef struct
{
    drv_isl12022_twi_cfg_t         cfg;           ///< sht30 configuraion.
    drv_rtc_evt_handler_t   evt_handler;   ///< Event handler, called when data is ready.
    bool                         enabled;       ///< Driver enabled flag.
} drv_rtc_t;

/**@brief Stored configuration.
 */
static drv_rtc_t m_drv_rtc;

/**@brief GPIOTE sceduled handler, executed in main-context.
 */
static void gpiote_evt_sceduled(void * p_event_data, uint16_t event_size)
{
    // Data ready
    //m_drv_rtc.evt_handler(DRV_RTC_EVT_DATA);

}


/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;

    if ((pin == m_drv_rtc.cfg.pin_int) && (nrf_gpio_pin_read(m_drv_rtc.cfg.pin_int) == 0))
    {
//        err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
//        APP_ERROR_CHECK(err_code);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "666 \n");
    }
}

static void rtc_timeout_handler(void * p_context)
{
    uint32_t err_code;
    m_drv_rtc.evt_handler(DRV_RTC_EVT_DATA);

//    err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
//    APP_ERROR_CHECK(err_code);
//    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "current_time = %ld \n", current_time);
     
}
/**@brief Intitialize GPIOTE to catch pin interrupts.
 */
static uint32_t gpiote_init(uint32_t pin)
{
    uint32_t err_code;

    /* Configure gpiote for the sensors data ready interrupt */
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        RETURN_IF_ERROR(err_code);
    }

    nrf_drv_gpiote_in_config_t gpiote_in_config;
    gpiote_in_config.is_watcher  = false;
    gpiote_in_config.hi_accuracy = false;
    gpiote_in_config.pull        = NRF_GPIO_PIN_NOPULL;
    gpiote_in_config.sense       = NRF_GPIOTE_POLARITY_TOGGLE;
    err_code = nrf_drv_gpiote_in_init(pin, &gpiote_in_config, gpiote_evt_handler);
    RETURN_IF_ERROR(err_code);

    nrf_drv_gpiote_in_event_enable(pin, true);

    return NRF_SUCCESS;
}


/**@brief Unintitialize GPIOTE pin interrupt.
 */
static void gpiote_uninit(uint32_t pin)
{
    nrf_drv_gpiote_in_event_disable(pin);
    nrf_drv_gpiote_in_uninit(pin);
}


uint32_t drv_rtc_init(drv_rtc_init_t * p_params)
{
    uint32_t err_code;
    uint8_t status;

    NULL_PARAM_CHECK(p_params);
    NULL_PARAM_CHECK(p_params->p_twi_instance);
    NULL_PARAM_CHECK(p_params->p_twi_cfg);
    NULL_PARAM_CHECK(p_params->evt_handler);

    m_drv_rtc.evt_handler        = p_params->evt_handler;
    m_drv_rtc.cfg.twi_addr       = p_params->twi_addr;
    m_drv_rtc.cfg.pin_int        = p_params->pin_int;
    m_drv_rtc.cfg.p_twi_instance = p_params->p_twi_instance;
    m_drv_rtc.cfg.p_twi_config   = p_params->p_twi_cfg;

    m_drv_rtc.enabled            = false;

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);


    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);

    /**@brief Init application timers */
    err_code = app_timer_create(
        &rtc_timer_id, APP_TIMER_MODE_REPEATED, rtc_timeout_handler);
     RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


uint32_t drv_rtc_enable(void)
{
    uint32_t err_code;

    if (m_drv_rtc.enabled)
    {
        return NRF_SUCCESS;
    }

    m_drv_rtc.enabled = true;

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);

//    err_code = drv_sht30_cfg_set(&cfg);
//    RETURN_IF_ERROR(err_code);

    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);

    err_code = gpiote_init(m_drv_rtc.cfg.pin_int);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_start(
        rtc_timer_id, APP_TIMER_TICKS(RTC_INTERVAL_MS), NULL);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


uint32_t drv_rtc_disable(void)
{
    uint32_t err_code;

    if (m_drv_rtc.enabled == false)
    {
        return NRF_SUCCESS;
    }

    m_drv_rtc.enabled = false;

    gpiote_uninit(m_drv_rtc.cfg.pin_int);

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);

//    err_code = drv_sht30_cfg_set(&cfg);
//    RETURN_IF_ERROR(err_code);

    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_stop(rtc_timer_id);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


uint32_t drv_rtc_reset(void)
{
    uint32_t err_code;

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);

//    err_code = drv_isl12022_reboot();
//    RETURN_IF_ERROR(err_code);

    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);
    return NRF_SUCCESS;
}


uint32_t drv_rtc_datetime_get(struct tm * tm)
{
    uint32_t err_code;

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);

    err_code = isl12022_rtc_read_time(tm);
    RETURN_IF_ERROR(err_code);

    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);
    
    return NRF_SUCCESS;
}

uint32_t drv_rtc_datetime_set(struct tm * tm)
{
    uint32_t err_code;

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);

    err_code = isl12022_rtc_set_time(tm);
    RETURN_IF_ERROR(err_code);

    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);
    
    return NRF_SUCCESS;
}

uint32_t drv_rtc_reseal(bool enable)
{
    uint8_t value;
    uint32_t err_code;

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);

    if(enable)
    {
        err_code = isl12022_rtc_bat_level_monitor_trip_set(0x52);
        RETURN_IF_ERROR(err_code);         
    }
    else
    {
        err_code = isl12022_rtc_bat_level_monitor_trip_set(0x12);
        RETURN_IF_ERROR(err_code); 

        err_code = isl12022_rtc_bat_reseal_read(&value);
        RETURN_IF_ERROR(err_code); 
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "rtc bat value = 0x%02x.\n", value);


    }

    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);
    
    return NRF_SUCCESS;

}

uint32_t drv_rtc_bat_lvl_manual_trip(void)
{
    uint32_t err_code;

    err_code = drv_isl12022_open(&m_drv_rtc.cfg);
    RETURN_IF_ERROR(err_code);

    err_code = isl12022_rtc_bat_level_monitor_manual_trip();
    RETURN_IF_ERROR(err_code);

    err_code = drv_isl12022_close();
    RETURN_IF_ERROR(err_code);
      
    return NRF_SUCCESS;
}

uint8_t drv_rtc_bat_percentage_get(void)
{
    return isl12022_rtc_bat_percentage_get();
}