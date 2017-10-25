#ifndef NRF5_TSL2561_LIB_H
#define NRF5_TSL2561_LIB_H

typedef enum tsl2561_state
{
    TSL2561_STATE_IDLE,
    TSL2561_STATE_CONV,
    TSL2561_STATE_ERROR
}tsl2561_state_t;

typedef enum tsl2561_evt_type
{
    TSL2561_NO_EVT,
    TSL2561_EVT_DATA_READY,
    TSL2561_EVT_ERROR,
    TSL2561_EVT_CRIT_ERROR
}tsl2561_evt_type_t;

typedef struct tsl2561 tsl2561_t;

typedef struct tsl2561_evt_data
{
        tsl2561_evt_type_t evt_type;
        tsl2561_t * tsl;
        ret_code_t err_code;
}tsl2561_evt_data_t;

typedef enum tsl2561_integration_time
{
    TSL2561_INTEGRATION_TIME_13_7MS = 0x00,
    TSL2561_INTEGRATION_TIME_101MS = 0x01,
    TSL2561_INTEGRATION_TIME_402MS = 0x02,
}tsl2561_integration_time_t;

typedef enum tsl2561_gain
{
    TSL2561_GAIN_0 = 0,
    TSL2561_GAIN_16X = 1
}tsl2561_gain_t;

typedef enum tsl2561_power
{
    TSL2561_POWER_UP = 0x03,
    TSL2561_POWER_DOWN = 0x00
}tsl2561_power_t;

struct tsl2561
{
        nrf_drv_twi_t * p_twi;
        tsl2561_gain_t gain;
        tsl2561_integration_time_t int_time;
        tsl2561_power_t pwr;
        tsl2561_state_t state;
        uint32_t visible_lux;
        uint32_t infrared_lux;
        tsl2561_evt_data_t last_evt;
};

typedef void (*tsl2561_event_cb_t)(tsl2561_evt_data_t * event_data);

ret_code_t tsl2561_drv_begin(tsl2561_event_cb_t (* tsl2561_event_cb)(tsl2561_evt_data_t * event_data));
ret_code_t tsl2561_drv_start_sensor(tsl2561_t * tsl, nrf_drv_twi_t * p_twi,tsl2561_integration_time_t int_time, tsl2561_gain_t gain);
ret_code_t tsl2561_drv_convert_data(tsl2561_t * tsl);
ret_code_t tsl2561_drv_set_integration_time(tsl2561_t * tsl, tsl2561_integration_time_t int_time);
ret_code_t tsl2561_drv_set_gain(tsl2561_t * tsl, tsl2561_gain_t gain);
ret_code_t tsl2561_drv_get_device_id(tsl2561_t * tsl, uint8_t * id);

#endif // NRF5_TSL2561_LIB_H
