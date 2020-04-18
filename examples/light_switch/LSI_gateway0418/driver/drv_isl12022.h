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

/** @file HTS221 humidity sensor
 *
 * @defgroup hts221_humidity_driver HTS221 humidity sensor
 * @{
 * @ingroup humidity_driver
 * @brief HTS221 humidity sensor API.
 *
 */

#ifndef __DRV_ISL12022_H__
#define __DRV_ISL12022_H__

#include "nrf_drv_twi.h"
#include "time.h"
#include <stdint.h>

#define DRV_VERSION "0.1"

#define ISL12022_REG_SC		0x00
#define ISL12022_REG_MN		0x01
#define ISL12022_REG_HR		0x02
#define ISL12022_REG_DT		0x03
#define ISL12022_REG_MO		0x04
#define ISL12022_REG_YR		0x05
#define ISL12022_REG_DW		0x06

#define ISL12022_REG_SR		0x07
#define ISL12022_REG_INT	0x08
#define ISL12022_REG_PWR_VDD	0x09
#define ISL12022_REG_PWR_VBAT	0x0A
#define ISL12022_REG_BETA	0x0D

#define ISL12022_REG_SCA        0x10
#define ISL12022_REG_MNA0	0x11
#define ISL12022_REG_HRA0       0x12
#define ISL12022_REG_DTA0	0x13
#define ISL12022_REG_M0A0       0x14
#define ISL12022_REG_DWA0	0x15

#define ISL12022_HR_MIL		(1 << 7)	

#define ISL12022_SR_LBAT85	(1 << 2)
#define ISL12022_SR_LBAT75	(1 << 1)

#define ISL12022_INT_WRTC	(1 << 6)
#define ISL12022_INT_IM	        (1 << 5)

#define ISL12022_SCA0_EN	(1 << 7)

#define ISL12022_PWR_VBAT_RESEAL            (1 << 6)
#define ISL12022_PWR_VBAT_VB85Tp2           (1 << 5)
#define ISL12022_PWR_VBAT_VB85Tp1	    (1 << 4)
#define ISL12022_PWR_VBAT_VB85Tp0	    (1 << 3)
#define ISL12022_PWR_VBAT_VB75Tp2           (1 << 2)
#define ISL12022_PWR_VBAT_VB75Tp1	    (1 << 1)
#define ISL12022_PWR_VBAT_VB75Tp0	    (1 << 0)

#define ISL12022_PWR_BETA_TSE	            (1 << 7)

/**@brief Initialization struct for the humidity driver.
 */
typedef struct
{
    uint8_t                      twi_addr;          ///< TWI address on bus.
    uint32_t                     pin_int;           ///< Interrupt pin.
    nrf_drv_twi_t        const * p_twi_instance;    ///< TWI instance.
    nrf_drv_twi_config_t const * p_twi_config;      ///< TWI configuraion.
}drv_isl12022_twi_cfg_t;


uint32_t drv_isl12022_init(void);

uint32_t drv_isl12022_open(drv_isl12022_twi_cfg_t const * const p_cfg);

uint32_t drv_isl12022_close(void);

int isl12022_rtc_read_time(struct tm * tm);

int isl12022_rtc_set_time(struct tm * tm);

int isl12022_rtc_bat_reseal_read(uint8_t *p_value);

int isl12022_rtc_bat_level_monitor_trip_set(uint8_t value);

int isl12022_rtc_bat_level_monitor_manual_trip(void);

uint8_t isl12022_rtc_bat_percentage_get(void);

#endif

/** @} */
