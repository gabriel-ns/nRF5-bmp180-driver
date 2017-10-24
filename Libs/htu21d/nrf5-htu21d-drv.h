#ifndef NRF5_HTU21D_LIB_H
#define NRF5_HTU21D_LIB_H

typedef enum htu21d_state
{
    HTU21D_STATE_IDLE,
    HTU21D_STATE_TEMP_CONV,
    HTU21D_STATE_HUM_CONV,
    HTU21D_STATE_ERROR
}htu21d_state_t;

typedef enum htu21d_resolution
{
    HTU21D_RES_RH_12_TEMP_14 = 0x02,
    HTU21D_RES_RH_8_TEMP_12 = 0x03,
    HTU21D_RES_RH_10_TEMP_13 = 0x82,
    HTU21D_RES_RH_11_TEMP_11 = 0x83,
}htu21d_resolution_t;

typedef enum htu21d_evt_type
{
    HTU21D_NO_EVT,
    HTU21D_EVT_DATA_READY,
    HTU21D_EVT_ERROR,
    HTU21D_EVT_CRIT_ERROR
}htu21d_evt_type_t;


typedef struct htu21d htu21d_t;

typedef struct htu21d_evt_data
{
        htu21d_evt_type_t evt_type;
        htu21d_t * htu21d;
        ret_code_t err_code;
}htu21d_evt_data_t;

typedef void (*htu21d_event_cb_t)(htu21d_evt_data_t * event_data);

struct htu21d
{
        nrf_drv_twi_t * p_twi;
        htu21d_resolution_t resolution;
        htu21d_state_t state;
        uint16_t humidity;
        int16_t temperature;
        htu21d_evt_data_t last_evt;
};

ret_code_t htu21d_drv_begin(htu21d_event_cb_t (* htu21d_event_cb)(htu21d_evt_data_t * event_data));
ret_code_t htu21d_drv_start_sensor(htu21d_t * htu, nrf_drv_twi_t * p_twi, htu21d_resolution_t resolution);
ret_code_t htu21d_drv_convert_data(htu21d_t * htu);
ret_code_t htu21d_drv_set_resolution(htu21d_t * htu, htu21d_resolution_t resolution);
ret_code_t htu21d_drv_reset_sensor(htu21d_t * htu);

#if 0
ret_code_t htu21d_drv_begin(nrf_drv_twi_t * p_ext_twi, htu21d_resolution_t res);
ret_code_t htu21d_drv_convert_temp_hold();
ret_code_t htu21d_drv_convert_temp_no_hold();
ret_code_t htu21d_drv_convert_hum_hold();
ret_code_t htu21d_drv_convert_hum_no_hold();
ret_code_t htu21d_drv_set_resolution(htu21d_resolution_t res);
htu21d_resolution_t htu21d_drv_get_resolution();
ret_code_t htu21d_drv_soft_reset();
uint16_t htu21d_drv_get_temp_conversion_time();
uint16_t htu21d_drv_get_hum_conversion_time();
uint16_t htu21d_calculate_rh(uint16_t buffer);
int16_t htu21d_calculate_temperature(uint16_t buffer);
ret_code_t htu21d_get_last_conversion(uint16_t * buffer);
#endif

#endif // NRF5_HTU21D_LIB_H
