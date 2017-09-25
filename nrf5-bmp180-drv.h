#ifndef NRF5_BMP180_LIB_H
#define NRF5_BMP180_LIB_H

typedef enum pwr_mode
{
    BMP180_ULTRA_LOW_PWR,
    BMP180_STANDARD,
    BMP180_HIGH_RES,
    BMP180_ULTRA_HIGH_RES,
    BMP180_SHUTDOWN
}bmp180_pwr_mode_t;

ret_code_t  bmp180_drv_begin(nrf_drv_twi_t * p_ext_twi, bmp180_pwr_mode_t pwr_mode);

ret_code_t bmp180_drv_convert_temp();

ret_code_t bmp180_drv_convert_pres();

ret_code_t bmp180_drv_get_pres(int32_t * app_buffer);

ret_code_t bmp180_drv_get_temp(int16_t * app_buffer);

ret_code_t bmp180_drv_set_pwr_mode(bmp180_pwr_mode_t pwr_mode);

bmp180_pwr_mode_t bmp180_drv_get_pwr_mode();

ret_code_t bmp180_drv_reset();

ret_code_t bmp180_drv_get_id(uint8_t * id_buff);

uint16_t bmp180_drv_get_conv_time();

#endif /** NRF5_BMP180_LIB_H */
