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

static htu21d_event_cb_t(* p_htu21d_event_cb)(htu21d_evt_data_t * event_data) = NULL;

ret_code_t htu21d_drv_begin(htu21d_event_cb_t (* htu21d_event_cb)(htu21d_evt_data_t * event_data))
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_start_sensor(htu21d_t * htu, nrf_drv_twi_t * p_twi, htu21d_resolution_t resolution)
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_convert_data(htu21d_t * htu)
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_set_resolution(htu21d_t * htu, htu21d_resolution_t resolution)
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_reset_sensor(htu21d_t * htu)
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_get_sensor_id(htu21d_t * htu, uint8_t * id_buff)
{
    return NRF_SUCCESS;
}

#if 0
htu21d_resolution_t resolution = HTU21D_RES_RH_8_TEMP_12;
nrf_drv_twi_t * p_twi = NULL;

static ret_code_t htu21d_drv_check_res_integrity(htu21d_resolution_t * res);

ret_code_t htu21d_drv_begin(nrf_drv_twi_t * p_ext_twi, htu21d_resolution_t res)
{
    ret_code_t err_code;

    HTU21D_NULL_PARAM_CHECK(p_ext_twi);
    p_twi = p_ext_twi;

    err_code = htu21d_drv_set_resolution(res);
    HTU21D_RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_convert_temp_hold()
{
    uint8_t cmd = HTU21D_REG_CONV_TEMP_HOLD;

    return nrf_drv_twi_tx(p_twi,
                HTU21D_DEVICE_ADDR,
                &cmd,
                sizeof(cmd),
                true);
}

ret_code_t htu21d_drv_convert_temp_no_hold()
{
    uint8_t cmd = HTU21D_REG_CONV_TEMP_NO_HOLD;

    return nrf_drv_twi_tx(p_twi,
                HTU21D_DEVICE_ADDR,
                &cmd,
                sizeof(cmd),
                false);
}

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

ret_code_t htu21d_drv_set_resolution(htu21d_resolution_t res)
{
    ret_code_t err_code;
    err_code = htu21d_drv_check_res_integrity(&res);
    HTU21D_RETURN_IF_ERROR(err_code);

    resolution = res;
    return NRF_SUCCESS;
}

htu21d_resolution_t htu21d_drv_get_resolution()
{
    return resolution;
}

ret_code_t htu21d_drv_soft_reset()
{
    uint8_t cmd = HTU21D_REG_SOFT_RESET;

    return nrf_drv_twi_tx(p_twi,
                HTU21D_DEVICE_ADDR,
                &cmd,
                sizeof(cmd),
                false);
    return NRF_SUCCESS;
}

uint16_t htu21d_drv_get_temp_conversion_time()
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

uint16_t htu21d_drv_get_hum_conversion_time()
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

ret_code_t htu21d_get_last_conversion(uint16_t * buffer)
{
    ret_code_t err_code;
    err_code = nrf_drv_twi_rx(p_twi,
                    HTU21D_DEVICE_ADDR,
                   (uint8_t *) buffer,
                    sizeof(uint16_t));
    HTU21D_RETURN_IF_ERROR(err_code);

    *buffer = HTU21D_BYTES_REVERSE_16BIT(*buffer);
    return NRF_SUCCESS;

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
    return (uint32_t) rh;
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

#endif
