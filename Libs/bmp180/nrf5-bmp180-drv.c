#include "nrf_drv_twi.h"
#include <string.h>
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf5-bmp180-drv.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_log_ctrl.h"

#define BMP180_BYTES_REVERSE_32BIT(x) ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24))
#define BMP180_BYTES_REVERSE_16BIT(x) (((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF))

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

#define BMP180_DEVICE_ADDR    0x77

APP_TIMER_DEF(bmp180_internal_timer);

typedef enum bmp180_reg_addr
{
    BMP180_REG_OUT_XLSB = 0xF8,
    BMP180_REG_OUT_LSB = 0xF7,
    BMP180_REG_OUT_MSB = 0xF6,
    BMP180_REG_CTRL_MEAS = 0xF4,
    BMP180_REG_SOFT_RESET = 0xE0,
    BMP180_REG_ID = 0xD0,
    BMP180_REG_CALIB_START = 0xAA,
} bmp180_reg_addr_t;

typedef enum cmd
{
    CONVERT_TEMPERATURE         = 0x2E,
    CONVERT_PRES_OSS_SINGLE     = 0x34,
    CONVERT_PRES_OSS_2_TIMES    = 0x74,
    CONVERT_PRES_OSS_4_TIMES    = 0xB4,
    CONVERT_PRES_OSS_8_TIMES    = 0xF4,
    SOFTWARE_RESET              = 0xB6,
    GET_CHIP_ID                 = 0x55
}bmp180_cmd_t;


/* Private functions prototypes */
static void bmp180_timeout_cb(void * p_ctx);
static ret_code_t read_callibration_data(bmp180_t * bmp);
static uint16_t bmp180_drv_get_conv_time(bmp180_pwr_mode_t pwr_mode);
static ret_code_t bmp180_calculate_values(bmp180_t * bmp, int16_t raw_temp, uint32_t raw_pres);
static void bmp180_error_call(bmp180_t * bmp, bmp180_evt_type_t evt_type, ret_code_t err_code);

static bmp180_event_cb_t (* p_bmp180_event_cb)(bmp180_evt_data_t * event_data) = NULL;
/* Public functions definition */
ret_code_t bmp180_drv_begin(bmp180_event_cb_t (* bmp180_event_cb)(bmp180_evt_data_t * event_data))
{
    ret_code_t err_code;

    if(bmp180_event_cb != NULL)
    {
        p_bmp180_event_cb = bmp180_event_cb;
    }

    err_code = app_timer_create(&bmp180_internal_timer, APP_TIMER_MODE_SINGLE_SHOT, bmp180_timeout_cb);
    return (ret_code_t) err_code;
}

ret_code_t bmp180_drv_start_sensor(bmp180_t * bmp, nrf_drv_twi_t * p_twi, bmp180_pwr_mode_t pwr_mode)
{
    BMP180_NULL_PARAM_CHECK(bmp);
    BMP180_NULL_PARAM_CHECK(p_twi);

    ret_code_t err_code;
    bmp->p_twi = p_twi;
    bmp->state = BMP180_STATE_IDLE;
    bmp->last_evt.evt_type = BMP180_NO_EVT;
    bmp->last_evt.bmp180 = bmp;

    err_code = read_callibration_data(bmp);
    BMP180_RETURN_IF_ERROR(err_code);

    err_code = bmp180_drv_set_pwr_mode(bmp, pwr_mode);
    BMP180_RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

ret_code_t bmp180_drv_convert_data(bmp180_t * bmp)
{
    BMP180_NULL_PARAM_CHECK(bmp);
    BMP180_NULL_PARAM_CHECK(bmp->p_twi);

    static uint8_t tx_data[2] = {BMP180_REG_CTRL_MEAS, CONVERT_TEMPERATURE};
    ret_code_t err_code;
    bmp->state = BMP180_STATE_TEMP_CONV;

    err_code = nrf_drv_twi_tx(bmp->p_twi, BMP180_DEVICE_ADDR, tx_data, sizeof(tx_data), false);
    BMP180_RETURN_IF_ERROR(err_code);

    uint16_t conversion_time = bmp180_drv_get_conv_time(bmp->pwr_mode);

    //TODO app timer prescaler
        err_code = app_timer_start(bmp180_internal_timer, APP_TIMER_TICKS(conversion_time, 0) , (void *) bmp );
        BMP180_RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

ret_code_t bmp180_drv_set_pwr_mode(bmp180_t * bmp, bmp180_pwr_mode_t pwr_mode)
{
    BMP180_NULL_PARAM_CHECK(bmp);

    bmp->pwr_mode = pwr_mode;
    return NRF_SUCCESS;
}

ret_code_t bmp180_drv_reset_sensor(bmp180_t * bmp)
{
    BMP180_NULL_PARAM_CHECK(bmp);
    BMP180_NULL_PARAM_CHECK(bmp->p_twi);

    static uint8_t tx_data[2] = {BMP180_REG_SOFT_RESET, SOFTWARE_RESET};

    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(bmp->p_twi, BMP180_DEVICE_ADDR, tx_data, sizeof(tx_data), false);
    BMP180_RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

ret_code_t bmp180_drv_get_sensor_id(bmp180_t * bmp, uint8_t * id_buff)
{
    BMP180_NULL_PARAM_CHECK(bmp);
    BMP180_NULL_PARAM_CHECK(id_buff);
    BMP180_NULL_PARAM_CHECK(bmp->p_twi);

    /** Set the target register address */
    static uint8_t reg_addr = BMP180_REG_ID;

    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(bmp->p_twi, BMP180_DEVICE_ADDR, &reg_addr, sizeof(reg_addr), false);
    BMP180_RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_rx(bmp->p_twi, BMP180_DEVICE_ADDR, id_buff, 1);
    BMP180_RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

/* Private functions definition */
static ret_code_t read_callibration_data(bmp180_t * bmp)
{
    BMP180_NULL_PARAM_CHECK(bmp);
    BMP180_NULL_PARAM_CHECK(bmp->p_twi);
    ret_code_t err_code;

    /** Clear the current value for all the calibration variables */
    memset(&bmp->calib_data, 0, sizeof(bmp180_callibration_t));

    /** Set the target register address */
    static uint8_t reg_addr = BMP180_REG_CALIB_START;

    err_code = nrf_drv_twi_tx(bmp->p_twi, BMP180_DEVICE_ADDR, &reg_addr, sizeof(reg_addr), true);
    BMP180_RETURN_IF_ERROR(err_code);

    /** Store the received callibration data in the g_bmp180_callibration_data structure */
    err_code = nrf_drv_twi_rx(bmp->p_twi, BMP180_DEVICE_ADDR, (uint8_t *) &bmp->calib_data, sizeof(bmp180_callibration_t));

    bmp180_callibration_t data_buff = bmp->calib_data;
    uint16_t * buff;

    for(uint8_t n = 0; n < (sizeof(data_buff)/2);n++)
    {
        buff = ((uint16_t *) (&data_buff)) + n;
        *buff = BMP180_BYTES_REVERSE_16BIT(*buff);
    }
    bmp->calib_data = data_buff;

    return NRF_SUCCESS;
}

static void bmp180_timeout_cb(void * p_ctx)
{
    if(p_ctx == NULL)
    {
        return;
    }

    bmp180_t * bmp = (bmp180_t * ) p_ctx;
    static uint8_t reg_addr = BMP180_REG_OUT_MSB;
    ret_code_t err_code;

    if(bmp->state == BMP180_STATE_TEMP_CONV)
    {
        err_code = nrf_drv_twi_tx(bmp->p_twi, BMP180_DEVICE_ADDR, &reg_addr, sizeof(reg_addr), false);
        if(err_code != NRF_SUCCESS) bmp180_error_call(bmp, BMP180_EVT_ERROR, err_code);

        /** Store the raw data in the temperature buffer */
        err_code = nrf_drv_twi_rx(bmp->p_twi, BMP180_DEVICE_ADDR,  (uint8_t *) &bmp->raw_buffer,2);
        if(err_code != NRF_SUCCESS) bmp180_error_call(bmp, BMP180_EVT_ERROR, err_code);

        bmp->raw_buffer = BMP180_BYTES_REVERSE_16BIT(bmp->raw_buffer);

        bmp->state = BMP180_STATE_PRES_CONV;

        static uint8_t tx_data[2] = {BMP180_REG_CTRL_MEAS, CONVERT_PRES_OSS_SINGLE};
        /** Set the conversion command according to the power mode */        switch(bmp->pwr_mode)        {            case BMP180_ULTRA_LOW_PWR:                tx_data[1] = CONVERT_PRES_OSS_SINGLE;                break;
            case BMP180_STANDARD:                tx_data[1] = CONVERT_PRES_OSS_2_TIMES;                break;
            case BMP180_HIGH_RES:                tx_data[1] = CONVERT_PRES_OSS_4_TIMES;                break;
            case BMP180_ULTRA_HIGH_RES:                tx_data[1] = CONVERT_PRES_OSS_8_TIMES;                break;
            default:                return;                break;
        }
        err_code = nrf_drv_twi_tx(bmp->p_twi, BMP180_DEVICE_ADDR, tx_data, sizeof(tx_data), false);
        if(err_code != NRF_SUCCESS) bmp180_error_call(bmp, BMP180_EVT_ERROR, err_code);

        uint16_t conversion_time = bmp180_drv_get_conv_time(bmp->pwr_mode);

        //TODO app timer prescaler
        err_code = app_timer_start(bmp180_internal_timer, APP_TIMER_TICKS(conversion_time, 0) , (void *) bmp );
        if(err_code != NRF_SUCCESS) bmp180_error_call(bmp, BMP180_EVT_CRIT_ERROR, err_code);

    }
    else if(bmp->state == BMP180_STATE_PRES_CONV)
    {
        uint32_t pressure_data;
        err_code = nrf_drv_twi_tx(bmp->p_twi, BMP180_DEVICE_ADDR, &reg_addr, sizeof(reg_addr), false);
        if(err_code != NRF_SUCCESS) bmp180_error_call(bmp, BMP180_EVT_ERROR, err_code);
        err_code = nrf_drv_twi_rx(bmp->p_twi, BMP180_DEVICE_ADDR,  (uint8_t *) &pressure_data, 3);
        if(err_code != NRF_SUCCESS) bmp180_error_call(bmp, BMP180_EVT_ERROR, err_code);

        pressure_data = ((BMP180_BYTES_REVERSE_32BIT(pressure_data)) >> 8) & 0x00FFFFFF;

        /* Resolution correction */
        pressure_data = (pressure_data) >> (8 - bmp->pwr_mode);
        err_code = bmp180_calculate_values(bmp, bmp->raw_buffer, pressure_data);
        if(err_code != NRF_SUCCESS) bmp180_error_call(bmp, BMP180_EVT_ERROR, err_code);

        bmp->state = BMP180_STATE_IDLE;
        bmp->last_evt.evt_type = BMP180_EVT_DATA_READY;
        bmp->last_evt.err_code = NRF_SUCCESS;

        if(p_bmp180_event_cb != NULL)
        {
            p_bmp180_event_cb(&bmp->last_evt);
        }
    }
}

static ret_code_t bmp180_calculate_values(bmp180_t * bmp, int16_t raw_temp, uint32_t raw_pres)
{
    BMP180_NULL_PARAM_CHECK(bmp);
    int32_t X1;
    int32_t X2;
    int32_t X3;

    int32_t B3;
    int32_t B5;
    int32_t B6;
    uint32_t B4;
    uint32_t B7;

    uint32_t local_buff = 0;;

    X1 = ((raw_temp - bmp->calib_data.AC6) * bmp->calib_data.AC5) >> 15;
    X2 = (bmp->calib_data.MC * 2048) / (X1 + bmp->calib_data.MD);
    B5 = X1 + X2;
    bmp->temperature = ((B5 + 8) >> 4) * 10;

    B6 = B5 - 4000;
    X1 = (bmp->calib_data.B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (bmp->calib_data.AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((bmp->calib_data.AC1 * 4) + X3) << bmp->pwr_mode) + 2) >> 2;
    X1 = (bmp->calib_data.AC3 * B6) >> 13;
    X2 = (bmp->calib_data.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = bmp->calib_data.AC4 * (uint32_t)(X3 + 32768) >> 15;
    B7 = ((uint32_t) raw_pres - B3) * (50000 >> bmp->pwr_mode);

    if(B7 > 0x80000000)
    {
        local_buff = (B7 * 2) / B4;
    }
    else
    {
        local_buff = (B7 / B4) * 2;
    }

    X1 = (local_buff >> 8) * (local_buff >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * local_buff) >> 16;
    local_buff = local_buff + ((X1 + X2 + 3791) / 16);

    bmp->pressure = local_buff;

    return NRF_SUCCESS;
}


static uint16_t bmp180_drv_get_conv_time(bmp180_pwr_mode_t pwr_mode)
{
    /** Return the conversion time in ms according to the power mode */
    switch(pwr_mode)
    {
        case BMP180_ULTRA_LOW_PWR:
            return 5;
            break;

        case BMP180_STANDARD:
            return 8;
            break;

        case BMP180_HIGH_RES:
            return 14;
            break;

        case BMP180_ULTRA_HIGH_RES:
            return 26;
            break;

        default:
            break;
    }

    return 0;
}

static void bmp180_error_call(bmp180_t * bmp, bmp180_evt_type_t evt_type, ret_code_t err_code)
{
    if(bmp == NULL) return;

    bmp->last_evt.err_code = err_code;
    bmp->last_evt.evt_type = evt_type;

    if(p_bmp180_event_cb != NULL)
    {
        p_bmp180_event_cb(&bmp->last_evt);
    }
}


