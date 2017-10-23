#ifndef NRF5_BMP180_LIB_H
#define NRF5_BMP180_LIB_H

typedef enum bmp180_state
{
    BMP180_STATE_IDLE,
    BMP180_STATE_TEMP_CONV,
    BMP180_STATE_PRES_CONV,
    BMP180_STATE_ERROR
}bmp180_state_t;

typedef enum pwr_mode
{
    BMP180_ULTRA_LOW_PWR = 0x00,
    BMP180_STANDARD,
    BMP180_HIGH_RES,
    BMP180_ULTRA_HIGH_RES,
    BMP180_SHUTDOWN
}bmp180_pwr_mode_t;

typedef PACKED_STRUCT bmp180_callibration
{
    int16_t     AC1;
    int16_t     AC2;
    int16_t     AC3;
    uint16_t    AC4;
    uint16_t    AC5;
    uint16_t    AC6;
    int16_t     B1;
    int16_t     B2;
    int16_t     MB;
    int16_t     MC;
    int16_t     MD;
}bmp180_callibration_t;

typedef enum bmp180_evt_type
{
    BMP180_NO_EVT,
    BMP180_EVT_DATA_READY,
    BMP180_EVT_ERROR,
    BMP180_EVT_CRIT_ERROR
}bmp180_evt_type_t;

typedef struct bmp180 bmp180_t;

typedef struct bmp180_evt_data
{
        bmp180_evt_type_t evt_type;
        bmp180_t * bmp180;
        ret_code_t err_code;
}bmp180_evt_data_t;

typedef void (*bmp180_event_cb_t)(bmp180_evt_data_t * event_data);

struct bmp180
{
        nrf_drv_twi_t * p_twi;
        bmp180_pwr_mode_t pwr_mode;
        bmp180_callibration_t calib_data;
        bmp180_state_t state;
        uint32_t pressure;
        int16_t temperature;
        int16_t raw_buffer;
        bmp180_evt_data_t last_evt;
};

ret_code_t bmp180_drv_begin(bmp180_event_cb_t (* bmp180_event_cb)(bmp180_evt_data_t * event_data));
ret_code_t bmp180_drv_start_sensor(bmp180_t * bmp, nrf_drv_twi_t * p_twi, bmp180_pwr_mode_t pwr_mode);
ret_code_t bmp180_drv_convert_data(bmp180_t * bmp);
ret_code_t bmp180_drv_set_pwr_mode(bmp180_t * bmp, bmp180_pwr_mode_t pwr_mode);
ret_code_t bmp180_drv_reset_sensor(bmp180_t * bmp);
ret_code_t bmp180_drv_get_sensor_id(bmp180_t * bmp, uint8_t * id_buff);
#endif /** NRF5_BMP180_LIB_H */
