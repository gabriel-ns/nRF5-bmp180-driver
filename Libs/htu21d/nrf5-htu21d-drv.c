#include <string.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_log_ctrl.h"
#include "nrf5-htu21d-drv.h"

#define BMP180_NULL_PARAM_CHECK(PARAM)  \
    do{                                 \
        if(PARAM == NULL)               \
        {                               \
            return NRF_ERROR_NULL;      \
        }                               \
    }while(0)

#define BMP180_RETURN_IF_ERROR(ERR)     \
    do{                                 \
        if(ERR != NRF_SUCCESS)          \
        {                               \
            return ERR;                 \
        }                               \
    }while(0)


#define HTU21D_TEMP_14BIT_CONVERSION_TIME   55
#define HTU21D_TEMP_13BIT_CONVERSION_TIME   30
#define HTU21D_TEMP_12BIT_CONVERSION_TIME   15
#define HTU21D_TEMP_11BIT_CONVERSION_TIME   10

#define HTU21D_RH_12BIT_CONVERSION_TIME   20
#define HTU21D_RH_11BIT_CONVERSION_TIME   10
#define HTU21D_RH_10BIT_CONVERSION_TIME   6
#define HTU21D_RH_8BIT_CONVERSION_TIME    5

htu21d_resolution_t resolution = HTU21D_RES_RH_8_TEMP_12;

ret_code_t htu21d_drv_begin(nrf_drv_twi_t * p_ext_twi)
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_convert_temp_hold()
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_convert_temp_no_hold()
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_convert_hum_hold()
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_convert_hum_no_hold()
{
    return NRF_SUCCESS;
}

ret_code_t htu21d_drv_set_resolution(htu21d_resolution_t res)
{
    ret_code_t err_code;
    err_code = htu21d_drv_check_res_integrity(res);
    RETURN_IF_ERROR(err_code);

    resolution = res;
    return NRF_SUCCESS;
}

htu21d_resolution_t htu21d_drv_get_resolution()
{
    return resolution;
}

ret_code_t htu21d_drv_soft_reset()
{
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

