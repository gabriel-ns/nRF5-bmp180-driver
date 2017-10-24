/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 * 
 */

/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include <stdint.h>
#include <stdio.h>
#include "nrf_rtc.h"
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_twi.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf5-htu21d-drv.h"
#include "nrf_delay.h"

#define MAX_PENDING_TRANSACTIONS    5

#define APP_TIMER_PRESCALER         0
#define APP_TIMER_OP_QUEUE_SIZE     2

static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);

static htu21d_t sensor;

#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif

void twi_init (void)
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t config = {
			.scl                = 1,
			.sda                = 0,
			.frequency          = NRF_TWI_FREQ_100K,
			.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
			.clear_bus_init     = false
	};

	err_code = nrf_drv_twi_init(&m_twi, &config, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}


// RTC tick events generation.
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        // On each RTC tick (their frequency is set in "nrf_drv_config.h")
        // we read data from our sensors.
        htu21d_drv_convert_data(&sensor);
    }
}
static void rtc_config(void)
{
    uint32_t err_code;

    // Initialize RTC instance with default configuration.
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = RTC_FREQ_TO_PRESCALER(1); //Set RTC frequency to 32Hz
    err_code = nrf_drv_rtc_init(&m_rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    // Enable tick event and interrupt.
    nrf_drv_rtc_tick_enable(&m_rtc, true);

    // Power on RTC instance.
    nrf_drv_rtc_enable(&m_rtc);
}


static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

static void sensor_callback(htu21d_evt_data_t * event_data)
{
    if(event_data->evt_type == HTU21D_EVT_DATA_READY)
    {
        NRF_LOG_INFO("T = %d | H = %d\n", event_data->htu21d->temperature, event_data->htu21d->humidity);
    }
}


int main(void)
{
    volatile ret_code_t err_code;

    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).
    lfclk_config();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    APP_TIMER_INIT(0, 10, NULL);

    NRF_LOG_INFO("TWI master example\r\n");
    NRF_LOG_FLUSH();
    twi_init();

    htu21d_drv_begin(sensor_callback);

    htu21d_drv_start_sensor(&sensor, &m_twi, HTU21D_RES_RH_12_TEMP_14);

    htu21d_drv_convert_data(&sensor);

    rtc_config();


    while (true)
    {
        __WFI();
        NRF_LOG_FLUSH();
    }
}


/** @} */
