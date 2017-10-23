#include <string.h>
#include <stdint.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log_ctrl.h"
#include "app_timer.h"
#include "nrf5-htu21d-drv.h"

#define HTU21D_BYTES_REVERSE_32BIT(x) ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24))
#define HTU21D_BYTES_REVERSE_16BIT(x) (((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF))

#define HTU21D_NULL_PARAM_CHECK(PARAM)  \
    do{                                 \
        if(PARAM == NULL)               \
        {                               \
            return NRF_ERROR_NULL;      \
        }                               \
    }while(0)

#define HTU21D_RETURN_IF_ERROR(ERR)     \
    do{                                 \
        if(ERR != NRF_SUCCESS)          \
        {                               \
            return ERR;                 \
        }                               \
    }while(0)

#define HTU21D_DEVICE_ADDR                  0x40

#define HTU21D_TEMP_14BIT_CONVERSION_TIME   55
#define HTU21D_TEMP_13BIT_CONVERSION_TIME   30
#define HTU21D_TEMP_12BIT_CONVERSION_TIME   15
#define HTU21D_TEMP_11BIT_CONVERSION_TIME   10

#define HTU21D_RH_12BIT_CONVERSION_TIME   20
#define HTU21D_RH_11BIT_CONVERSION_TIME   10
#define HTU21D_RH_10BIT_CONVERSION_TIME   6
#define HTU21D_RH_8BIT_CONVERSION_TIME    5

APP_TIMER_DEF(htu21d_internal_timer);

typedef enum htu21d_reg_addr
{
    HTU21D_REG_CONV_TEMP_HOLD = 0xE3,
    HTU21D_REG_CONV_HUM_HOLD = 0xE5,
    HTU21D_REG_CONV_TEMP_NO_HOLD = 0xF3,
    HTU21D_REG_CONV_HUM_NO_HOLD = 0xF5,
    HTU21D_REG_WRITE_USER = 0xE6,
    HTU21D_REG_READ_USER = 0xE7,
    HTU21D_REG_SOFT_RESET = 0xFE
} htu21d_reg_addr_t;

static void htu21d_timeout_cb(void * p_ctx);
static uint16_t bmp180_drv_get_temp_conv_time(htu21d_resolution_t resolution);
static uint16_t bmp180_drv_get_hum_conv_time(htu21d_resolution_t resolution);
static ret_code_t htu21d_drv_check_res_integrity(htu21d_resolution_t * resolution);

static htu21d_event_cb_t(* p_htu21d_event_cb)(htu21d_evt_data_t * event_data) = NULL;

ret_code_t htu21d_drv_begin(htu21d_event_cb_t (* htu21d_event_cb)(htu21d_evt_data_t * event_data))
{
    if(htu21d_event_cb != NULL)
    {
        p_htu21d_event_cb = htu21d_event_cb;
    }

    ret_code_t err_code;
    err_code = app_timer_create(&htu21d_internal_timer, APP_TIMER_MODE_SINGLE_SHOT, htu21d_timeout_cb);
    return err_code;
}

ret_code_t htu21d_drv_start_sensor(htu21d_t * htu, nrf_drv_twi_t * p_twi, htu21d_resolution_t resolution)
{
    HTU21D_NULL_PARAM_CHECK(htu);
    HTU21D_NULL_PARAM_CHECK(p_twi);

    htu->p_twi = p_twi;
    htu->state = HTU21D_STATE_IDLE;
    htu->last_evt.evt_type = HTU21D_NO_EVT;
    htu->last_evt.htu21d = htu;

    ret_code_t err_code;
    err_code = htu21d_drv_set_resolution(htu, resolution);

    return err_code;
}

ret_code_t htu21d_drv_convert_data(htu21d_t * htu)
{
    HTU21D_NULL_PARAM_CHECK(htu);
    HTU21D_NULL_PARAM_CHECK(htu->p_twi);

    if(htu->state == HTU21D_STATE_ERROR)
    {
        return NRF_ERROR_INTERNAL;
    }

    uint8_t cmd = HTU21D_REG_CONV_TEMP_NO_HOLD;
    ret_code_t err_code;
    htu->state = HTU21D_STATE_TEMP_CONV;

    err_code = nrf_drv_twi_tx(htu->p_twi, HTU21D_DEVICE_ADDR, &cmd, sizeof(cmd), false);
    HTU21D_RETURN_IF_ERROR(err_code);

    uint16_t conversion_time = htu21d_drv_get_temp_conversion_time(htu->resolution);

    //TODO app timer prescaler
    err_code = app_timer_start(htu21d_internal_timer, APP_TIMER_TICKS(conversion_time, 0) , (void *) htu );
    HTU21D_RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_set_resolution(htu21d_t * htu, htu21d_resolution_t resolution)
{
    ret_code_t err_code;
    err_code = htu21d_drv_check_res_integrity(&resolution);
    HTU21D_RETURN_IF_ERROR(err_code);

    uint8_t cmd[2] = {HTU21D_REG_WRITE_USER, resolution};
    err_code = nrf_drv_twi_tx(htu->p_twi, HTU21D_DEVICE_ADDR, cmd, sizeof(cmd), false);
    HTU21D_RETURN_IF_ERROR(err_code);

    htu->resolution = resolution;
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_reset_sensor(htu21d_t * htu)
{
    HTU21D_NULL_PARAM_CHECK(htu);
    HTU21D_NULL_PARAM_CHECK(htu->p_twi);

    uint8_t cmd = HTU21D_REG_SOFT_RESET;

    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(htu->p_twi, HTU21D_DEVICE_ADDR, &cmd, sizeof(cmd), false);

    return err_code;
}

static uint16_t htu21d_drv_get_temp_conversion_time(htu21d_resolution_t resolution)
{
    switch(resolution)
    {
        case HTU21D_RES_RH_12_TEMP_14:
            return HTU21D_TEMP_14BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_8_TEMP_12:
            return HTU21D_TEMP_12BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_10_TEMP_13:
            return HTU21D_TEMP_13BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_11_TEMP_11:
            return HTU21D_TEMP_11BIT_CONVERSION_TIME;
            break;
    }
    return HTU21D_TEMP_14BIT_CONVERSION_TIME;
}

static uint16_t htu21d_drv_get_hum_conversion_time(htu21d_resolution_t resolution)
{
    switch(resolution)
    {
        case HTU21D_RES_RH_12_TEMP_14:
            return HTU21D_RH_12BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_8_TEMP_12:
            return HTU21D_RH_8BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_10_TEMP_13:
            return HTU21D_RH_10BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_11_TEMP_11:
            return HTU21D_RH_11BIT_CONVERSION_TIME;
            break;
    }
    return HTU21D_RH_12BIT_CONVERSION_TIME;
}

static ret_code_t htu21d_drv_check_res_integrity(htu21d_resolution_t * resolution)
{

}


static void htu21d_timeout_cb(void * p_ctx)
{
    if(p_ctx == NULL)
       {
           return;
       }

       htu21d_t * htu = (htu21d_t * ) p_ctx;

       ret_code_t err_code;
       static uint16_t buffer = 0;
       static uint8_t data_reg_addr = HTU21D_REG_READ_USER;
       static uint8_t convert_cmd = HTU21D_REG_CONV_HUM_NO_HOLD;

       if(htu->state == HTU21D_STATE_TEMP_CONV)
       {

           err_code = nrf_drv_twi_tx(htu->p_twi, HTU21D_DEVICE_ADDR, &data_reg_addr, sizeof(data_reg_addr), false);
           if(err_code != NRF_SUCCESS) htu21d_error_call(htu, HTU21D_EVT_ERROR, err_code);

           /* Store the raw data in the temperature buffer */
           err_code = nrf_drv_twi_rx(htu->p_twi, HTU21D_DEVICE_ADDR,  (uint8_t *) &buffer,2);
           if(err_code != NRF_SUCCESS) htu21d_error_call(htu, HTU21D_EVT_ERROR, err_code);

           buffer = HTU21D_BYTES_REVERSE_16BIT(buffer);

           htu->temperature = htu21d_calculate_temperature(buffer);

           htu->state = HTU21D_STATE_HUM_CONV;

           err_code = nrf_drv_twi_tx(htu->p_twi, HTU21D_DEVICE_ADDR, &convert_cmd, sizeof(convert_cmd), false);
           if(err_code != NRF_SUCCESS) htu21d_error_call(htu, HTU21D_EVT_ERROR, err_code);

           uint16_t conversion_time = htu21d_drv_get_hum_conversion_time(htu->resolution);

           //TODO app timer prescaler
           err_code = app_timer_start(htu21d_internal_timer, APP_TIMER_TICKS(conversion_time, 0) , (void *) bmp );
           if(err_code != NRF_SUCCESS) htu21d_error_call(htu, HTU21D_EVT_CRIT_ERROR, err_code);

       }
       else if(htu->state == HTU21D_STATE_HUM_CONV)
       {
           err_code = nrf_drv_twi_tx(htu->p_twi, HTU21D_DEVICE_ADDR, &data_reg_addr, sizeof(data_reg_addr), false);
           if(err_code != NRF_SUCCESS) htu21d_error_call(htu, HTU21D_EVT_ERROR, err_code);
           err_code = nrf_drv_twi_rx(htu->p_twi, HTU21D_DEVICE_ADDR,  (uint8_t *) &buffer, 2);
           if(err_code != NRF_SUCCESS) htu21d_error_call(htu, HTU21D_EVT_ERROR, err_code);

           buffer = HTU21D_BYTES_REVERSE_16BIT(buffer);
           err_code = htu21d_calculate_rh(buffer);
           if(err_code != NRF_SUCCESS) htu21d_error_call(htu, HTU21D_EVT_ERROR, err_code);

           htu->state = HTU21D_STATE_IDLE;
           htu->last_evt.evt_type = HTU21D_EVT_DATA_READY;
           htu->last_evt.err_code = NRF_SUCCESS;

           if(p_htu21d_event_cb != NULL)
           {
               p_htu21d_event_cb(&htu->last_evt);
           }
       }
}

int16_t htu21d_calculate_temperature(uint16_t buffer)
{
    int32_t temp;
    temp = (17572*buffer)/65535;
    temp = temp - 4685;
    return (int16_t) temp;
}

uint16_t htu21d_calculate_rh(uint16_t buffer)
{
    uint32_t rh;
    rh = (12500*buffer)/65535;
    rh = rh - 600;
    return (uint16_t) rh;
}

static ret_code_t htu21d_drv_check_res_integrity(htu21d_resolution_t * res)
{
    if(*res == HTU21D_RES_RH_12_TEMP_14 ||
            *res == HTU21D_RES_RH_8_TEMP_12 ||
            *res == HTU21D_RES_RH_10_TEMP_13 ||
            *res == HTU21D_RES_RH_11_TEMP_11)
    {
        return NRF_SUCCESS;
    }

    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }

}

#if 0

ret_code_t htu21d_drv_convert_hum_hold()
{
    uint8_t cmd = HTU21D_REG_CONV_HUM_HOLD;

    return nrf_drv_twi_tx(p_twi,
                HTU21D_DEVICE_ADDR,
                &cmd,
                sizeof(cmd),
                true);
}

ret_code_t htu21d_drv_convert_hum_no_hold()
{
    uint8_t cmd = HTU21D_REG_CONV_TEMP_NO_HOLD;

    return nrf_drv_twi_tx(p_twi,
                HTU21D_DEVICE_ADDR,
                &cmd,
                sizeof(cmd),
                false);
}








#endif
