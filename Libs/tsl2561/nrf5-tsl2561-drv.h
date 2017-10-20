#ifndef NRF5_TSL2561_LIB_H
#define NRF5_TSL2561_LIB_H

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

typedef struct tsl2561_config
{
        tsl2561_power_t power;
        tsl2561_integration_time_t integration_time;
        tsl2561_gain_t gain;
}tsl2561_config_t;

typedef struct tsl2561_adc_data
{
        uint16_t data0;
        uint16_t data1;
}tsl2561_adc_data_t;

ret_code_t tsl2561_drv_begin(nrf_drv_twi_t * p_ext_twi);
ret_code_t tsl2561_drv_set_power(tsl2561_power_t pwr);
ret_code_t tsl2561_drv_set_gain(tsl2561_gain_t gain);
ret_code_t tsl2561_drv_set_integration_time(tsl2561_integration_time_t time);
ret_code_t tsl2561_drv_get_device_id(uint8_t * id);
ret_code_t tsl2561_drv_read_data(tsl2561_adc_data_t * data);
uint32_t tsl2561_drv_calculate_lux(tsl2561_adc_data_t * data);
uint16_t tsl2561_get_read_time();


#endif // NRF5_TSL2561_LIB_H
