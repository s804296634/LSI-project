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

#include "drv_isl12022.h"
#include "nrf_error.h"
#include "twi_manager.h"
//#define  NRF_LOG_MODULE_NAME "drv_isl12022    "
#include "macros_common.h"
#include "log.h"

static uint8_t m_bat_percentage = 100;

/**@brief Check if the driver is open, if not return NRF_ERROR_INVALID_STATE.
 */
#define DRV_CFG_CHECK(PARAM)                                                                       \
    if ((PARAM) == NULL)                                                                           \
    {                                                                                              \
        return NRF_ERROR_INVALID_STATE;                                                            \
    }

static struct
{
    drv_isl12022_twi_cfg_t const * p_cfg;
    bool write_enabled;
} m_isl12022;

// BCD???¡§¡éa?a?t????
unsigned bcd2bin(unsigned char val) { return (val & 0x0f) + (val >> 4) * 10; }

unsigned char bin2bcd(unsigned val) { return ((val / 10) << 4) + val % 10; }

/*?
?*?Does?the?rtc_time?represent?a?valid?date/time???*/

uint32_t rtc_valid_tm(struct tm * tm)
{
    if (tm->tm_year < 70 || ((unsigned)tm->tm_mon) >= 12 || tm->tm_mday < 1 ||
        //        tm->tm_mday > rtc_month_days(tm->tm_mon, tm->tm_year + 1900) ||
        ((unsigned)tm->tm_hour) >= 24 || ((unsigned)tm->tm_min) >= 60 ||
        ((unsigned)tm->tm_sec) >= 60)

        return NRF_ERROR_INVALID_DATA;
    return NRF_SUCCESS;
}

/**@brief Function to init the TWI module when this driver needs to communicate on the TWI bus.
 */
static __inline uint32_t twi_open(void)
{
    uint32_t err_code;

    err_code = twi_manager_request(
        m_isl12022.p_cfg->p_twi_instance, m_isl12022.p_cfg->p_twi_config, NULL, NULL);
    RETURN_IF_ERROR(err_code);

    nrf_drv_twi_enable(m_isl12022.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}


/**@brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module.
 */
static __inline uint32_t twi_close(void)
{
    nrf_drv_twi_disable(m_isl12022.p_cfg->p_twi_instance);

    nrf_drv_twi_uninit(m_isl12022.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}


/**@brief Function for reading a sensor register.
 *
 * @param[in]  reg_addr            Address of the register to read.
 * @param[out] p_reg_val           Pointer to a buffer to receive the read value.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_read(uint8_t reg_addr, uint8_t * p_reg_val)
{
    uint32_t err_code;

    err_code = nrf_drv_twi_tx(
        m_isl12022.p_cfg->p_twi_instance, m_isl12022.p_cfg->twi_addr, &reg_addr, 1, true);
    RETURN_IF_ERROR(err_code);

    err_code =
        nrf_drv_twi_rx(m_isl12022.p_cfg->p_twi_instance, m_isl12022.p_cfg->twi_addr, p_reg_val, 1);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for writing to a sensor register.
 *
 * @param[in]  reg_addr            Address of the register to write to.
 * @param[in]  reg_val             Value to write to the register.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_write(uint8_t reg_addr, uint8_t reg_val)
{
    uint32_t err_code;
    uint8_t buffer[2] = {reg_addr, reg_val};

    err_code = nrf_drv_twi_tx(
        m_isl12022.p_cfg->p_twi_instance, m_isl12022.p_cfg->twi_addr, buffer, 2, false);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for reading multiple sensor registers.
 *
 * @param[in]  reg_addr        Address of the register to read.
 * @param[out] p_buf           Pointer to a buffer to receive the read value.
 * @param[in]  size            Number of bytes to read.
 *
 * @retval NRF_SUCCESS         If operation was successful.
 * @retval NRF_ERROR_BUSY      If the TWI drivers are busy.
 */
static uint32_t buf_read(uint8_t reg_addr, uint8_t * p_buf, uint32_t size)
{
    uint32_t err_code;

    for (uint32_t i = 0; i < size; i++)
    {
        err_code = reg_read((reg_addr + i), &p_buf[i]);
        RETURN_IF_ERROR(err_code);
    }

    return NRF_SUCCESS;
}


uint32_t drv_isl12022_open(drv_isl12022_twi_cfg_t const * const p_cfg)
{
    m_isl12022.p_cfg = p_cfg;

    return twi_open();
}


uint32_t drv_isl12022_close(void)
{
    uint32_t err_code = twi_close();

    m_isl12022.p_cfg = NULL;

    return err_code;
}

uint32_t drv_isl12022_init(void)
{
    m_isl12022.p_cfg = NULL;

    return NRF_SUCCESS;
}

static int isl12022_get_datetime(struct tm * tm)
{
    uint8_t buf[ISL12022_REG_INT + 1];
    int ret;

    ret = buf_read(ISL12022_REG_SC, buf, sizeof(buf));
    RETURN_IF_ERROR(ret);

    if (buf[ISL12022_REG_SR] & (ISL12022_SR_LBAT85 | ISL12022_SR_LBAT75))
    {
        m_bat_percentage = 85;
        if(buf[ISL12022_REG_SR] & ISL12022_SR_LBAT75)
            m_bat_percentage = 75; 
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "voltage dropped below %u%%, "
            "date and time is not reliable. \n",
            buf[ISL12022_REG_SR] & ISL12022_SR_LBAT85 ? 85 : 75);
    }
    else
        m_bat_percentage = 100;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3,
        "raw data is sec=%02x, min=%02x, hr=%02x, "
        "mday=%02x, mon=%02x, year=%02x, wday=%02x, "
        "sr=%02x, int=%02x \n",
        buf[ISL12022_REG_SC], buf[ISL12022_REG_MN], buf[ISL12022_REG_HR], buf[ISL12022_REG_DT],
        buf[ISL12022_REG_MO], buf[ISL12022_REG_YR], buf[ISL12022_REG_DW], buf[ISL12022_REG_SR],
        buf[ISL12022_REG_INT]);

    tm->tm_sec = bcd2bin(buf[ISL12022_REG_SC] & 0x7F);
    tm->tm_min = bcd2bin(buf[ISL12022_REG_MN] & 0x7F);
    tm->tm_hour = bcd2bin(buf[ISL12022_REG_HR] & 0x3F);
    tm->tm_mday = bcd2bin(buf[ISL12022_REG_DT] & 0x3F);
    tm->tm_wday = buf[ISL12022_REG_DW] & 0x07;
    tm->tm_mon = bcd2bin(buf[ISL12022_REG_MO] & 0x1F) - 1;
    tm->tm_year = bcd2bin(buf[ISL12022_REG_YR]) + 100;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3,
        "secs=%02d, mins=%02d, hours=%02d, "
        "mday=%02d, mon=%02d, year=%03d, wday=%02d \n",
        tm->tm_sec, tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

    if (rtc_valid_tm(tm) < 0)
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "retrieved date and time is invalid.\n");

    return 0;
}

static int isl12022_set_datetime(struct tm * tm)
{
    size_t i;
    int ret;
    uint8_t buf[ISL12022_REG_DW + 1];

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
        "secs=%d, mins=%d, hours=%d, "
        "mday=%d, mon=%d, year=%d, wday=%d\n",
        tm->tm_sec, tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

    if (!m_isl12022.write_enabled)
    {

        ret = buf_read(ISL12022_REG_INT, buf, 1);
        if (ret)
            return ret;

        if (!(buf[0] & ISL12022_INT_WRTC))
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "init write enable£¬24 hour format\n");

//            ret = reg_write(ISL12022_REG_INT, buf[0] | ISL12022_INT_WRTC | ISL12022_INT_IM);
//            RETURN_IF_ERROR(ret);
//
//            ret = reg_write(ISL12022_REG_SCA, 0 | ISL12022_SCA0_EN | bin2bcd(5));
//            RETURN_IF_ERROR(ret);

            ret = reg_write(ISL12022_REG_INT, buf[0] | ISL12022_INT_WRTC);
            RETURN_IF_ERROR(ret);

            ret = buf_read(ISL12022_REG_HR, buf, 1);
            RETURN_IF_ERROR(ret);

            ret = reg_write(ISL12022_REG_HR, buf[0] | ISL12022_HR_MIL);
            RETURN_IF_ERROR(ret);
        }

        m_isl12022.write_enabled = true;
    }


    buf[ISL12022_REG_SC] = bin2bcd(tm->tm_sec);
    buf[ISL12022_REG_MN] = bin2bcd(tm->tm_min);
    buf[ISL12022_REG_HR] = bin2bcd(tm->tm_hour) | ISL12022_HR_MIL;

    buf[ISL12022_REG_DT] = bin2bcd(tm->tm_mday);


    buf[ISL12022_REG_MO] = bin2bcd(tm->tm_mon + 1);


    buf[ISL12022_REG_YR] = bin2bcd(tm->tm_year % 100);

    buf[ISL12022_REG_DW] = tm->tm_wday & 0x07;


    for (i = 0; i < ARRAY_SIZE(buf); i++)
    {
        ret = reg_write(ISL12022_REG_SC + i, buf[ISL12022_REG_SC + i]);
        RETURN_IF_ERROR(ret);
    };

    return 0;
}

static int isl12022_bat_reseal_read(uint8_t *p_value)
{
    int ret;
    uint8_t buf[1];
    ret = buf_read(ISL12022_REG_PWR_VBAT, buf, 1);
    if (ret)
        return ret; 

    *p_value = buf[0];

//    if(enable)
//    {
//        ret = reg_write(ISL12022_REG_PWR_VBAT, buf[0] | ISL12022_PWR_VBAT_RESEAL);
//        RETURN_IF_ERROR(ret);           
//    }
//    else
//    {
//        ret = reg_write(ISL12022_REG_PWR_VBAT, buf[0] & (~ISL12022_PWR_VBAT_RESEAL));
//        RETURN_IF_ERROR(ret);         
//    }
        
    return 0;
}

static int isl12022_bat_level_monitor_trip_set(uint8_t value)
{
    int ret;
//    uint8_t buf[1];
//    ret = buf_read(ISL12022_REG_PWR_VBAT, buf, 1);
//    if (ret)
//        return ret;
//             
//    ret = reg_write(ISL12022_REG_PWR_VBAT, buf[0] & (~ISL12022_PWR_VBAT_VB85Tp2)
//                                                  | ISL12022_PWR_VBAT_VB85Tp1
//                                                  & (~ISL12022_PWR_VBAT_VB85Tp0)
//                                                  & (~ISL12022_PWR_VBAT_VB75Tp2)
//                                                  | ISL12022_PWR_VBAT_VB75Tp1
//                                                  & (~ISL12022_PWR_VBAT_VB75Tp0)
//                                                  );
    ret = reg_write(ISL12022_REG_PWR_VBAT, value);
    RETURN_IF_ERROR(ret);  

    return 0;
}

static int isl12022_bat_level_monitor_manual_trip(void)
{
    int ret;
    uint8_t buf[1];
    ret = buf_read(ISL12022_REG_BETA, buf, 1);
    if (ret)
        return ret;

    ret = reg_write(ISL12022_REG_BETA, buf[0] | ISL12022_PWR_BETA_TSE);
    RETURN_IF_ERROR(ret);  

    return 0;
}

int isl12022_rtc_read_time(struct tm * tm) { return isl12022_get_datetime(tm); }

int isl12022_rtc_set_time(struct tm * tm) { return isl12022_set_datetime(tm); }

int isl12022_rtc_bat_reseal_read(uint8_t *p_value) { return isl12022_bat_reseal_read(p_value); }

int isl12022_rtc_bat_level_monitor_trip_set(uint8_t value) { return isl12022_bat_level_monitor_trip_set(value); }

int isl12022_rtc_bat_level_monitor_manual_trip(void) { return isl12022_bat_level_monitor_manual_trip(); }

uint8_t isl12022_rtc_bat_percentage_get(void)
{
    return m_bat_percentage;
}