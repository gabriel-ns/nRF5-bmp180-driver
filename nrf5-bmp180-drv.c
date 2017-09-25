#include "nrf_drv_twi.h"
#include <string.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf5-bmp180-drv.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_log_ctrl.h"

#define BMP180_BYTES_REVERSE_32BIT(x) ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24))
#define BMP180_BYTES_REVERSE_16BIT(x) (((x & 0x00FF) << 8) | ((x & 0xFF00) >> 8))

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

bmp180_callibration_t    g_bmp180_callibration_data = {
        .AC1 =  408,
        .AC2 =  -72,
        .AC3 =  -14383,
        .AC4 =  32741,
        .AC5 =  32757,
        .AC6 =  23153,
        .B1 =  6190,
        .B2 =  4,
        .MB =  -32768,
        .MC =  -8711,
        .MD =  2868
};

typedef struct data_buffer
{
        int32_t raw_pressure;
        int32_t pressure;
        int16_t raw_temperature;
        int16_t temperature;

}bmp180_data_buffer_t;

bmp180_data_buffer_t g_data_buffer = {0,0,0,0};
nrf_drv_twi_t * p_twi = NULL;
bmp180_pwr_mode_t g_pwr_mode = BMP180_SHUTDOWN;

static ret_code_t bmp180_drv_read_calibration_data();

static ret_code_t bmp180_drv_calculate_temp(bmp180_data_buffer_t * p_buff);

static ret_code_t bmp180_drv_calculate_pres(bmp180_data_buffer_t * p_buff);

ret_code_t bmp180_drv_begin(nrf_drv_twi_t * p_ext_twi, bmp180_pwr_mode_t pwr_mode)
{
    /** This module will not work if the pointer for the twi driver is not set. */
    BMP180_NULL_PARAM_CHECK(p_ext_twi);
    p_twi = p_ext_twi;

    /** Set the sensor power mode */
    g_pwr_mode = pwr_mode;

//    g_data_buffer.raw_pressure = 23843;
//    g_data_buffer.raw_temperature = 27898;
//
//    bmp180_drv_calculate_pres(&g_data_buffer);

    /** Read the calibration data and store it in the global structure */
    return bmp180_drv_read_calibration_data();
}

ret_code_t bmp180_drv_convert_temp()
{
    /** Set the data to transfer.
     *  The first byte set the target register address,
     *  The second byte is the command. */
    static uint8_t tx_data[2] = {BMP180_REG_CTRL_MEAS, CONVERT_TEMPERATURE};

    /** If the p_twi is null, this transaction will bug the app, so it's important to check for safety */
    BMP180_NULL_PARAM_CHECK(p_twi);

    return nrf_drv_twi_tx(p_twi,
            BMP180_DEVICE_ADDR,
            tx_data,
            sizeof(tx_data),
            false);
}

ret_code_t bmp180_drv_convert_pres()
{
    /** Set the data to transfer.
     *  The first byte set the target register address,
     *  The second byte is the command. */
    static uint8_t tx_data[2] = {BMP180_REG_CTRL_MEAS, CONVERT_PRES_OSS_SINGLE};

    /** If the p_twi is null, this transaction will bug the app, so it's important to check for safety */
    BMP180_NULL_PARAM_CHECK(p_twi);

    /** Set the conversion command according to the power mode */
    switch(g_pwr_mode)
    {
        case BMP180_ULTRA_LOW_PWR:
            tx_data[1] = CONVERT_PRES_OSS_SINGLE;
            break;

        case BMP180_STANDARD:
            tx_data[1] = CONVERT_PRES_OSS_2_TIMES;
            break;

        case BMP180_HIGH_RES:
            tx_data[1] = CONVERT_PRES_OSS_4_TIMES;
            break;

        case BMP180_ULTRA_HIGH_RES:
            tx_data[1] = CONVERT_PRES_OSS_8_TIMES;
            break;

        default:
            return NRF_ERROR_FORBIDDEN;
            break;
    }

    return nrf_drv_twi_tx(p_twi,
                BMP180_DEVICE_ADDR,
                tx_data,
                sizeof(tx_data),
                false);
}

ret_code_t bmp180_drv_get_pres(int32_t * app_buffer)
{
    /** If the p_twi is null, this transaction will bug the app, so it's important to check for safety */
    BMP180_NULL_PARAM_CHECK(p_twi);

    /** Set the target register address */
    static uint8_t reg_addr = BMP180_REG_OUT_MSB;

    static uint32_t err_code;

    /** Clears the pressure buffer. As it's a static variable, we don't clear it in the declaration */
    g_data_buffer.raw_pressure = 0;

    err_code = nrf_drv_twi_tx(p_twi,
            BMP180_DEVICE_ADDR,
            &reg_addr,
            sizeof(reg_addr),
            false);
    BMP180_RETURN_IF_ERROR(err_code);

    /** Store the raw data in the pressure buffer */
    err_code = nrf_drv_twi_rx(p_twi,
            BMP180_DEVICE_ADDR,
            (uint8_t *) &g_data_buffer.raw_pressure,
            3);
    BMP180_RETURN_IF_ERROR(err_code);

    /**
     * This transfer model will give us the value in big endian, so we need to convert it to little endian.
     * Besides that, our input value is a 24bit value, so we must shift one byte to the right and
     * use a logic and with 0x00ffffff because the shifted bits are filled with 1's.
     */
    g_data_buffer.raw_pressure = ((BMP180_BYTES_REVERSE_32BIT(g_data_buffer.raw_pressure)) >> 8) & 0x00FFFFFF;

    /** Resolution correction */
    g_data_buffer.raw_pressure = (g_data_buffer.raw_pressure) >> (8 - g_pwr_mode);

    /** Calculate the pressure with the buffer raw value and store it in the pressure buffer */
    bmp180_drv_calculate_pres(&g_data_buffer);

    if(app_buffer != NULL)*app_buffer = g_data_buffer.pressure;

    return NRF_SUCCESS;
}

ret_code_t bmp180_drv_get_temp(int16_t * app_buffer)
{
    /** If the p_twi is null, this transaction will bug the app, so it's important to check for safety */
    BMP180_NULL_PARAM_CHECK(p_twi);

    /** Set the target register address */
    static uint8_t reg_addr = BMP180_REG_OUT_MSB;

    static uint32_t err_code;

    /** Clears the temperature buffer. */
    g_data_buffer.raw_temperature = 0;

    err_code = nrf_drv_twi_tx(p_twi,
            BMP180_DEVICE_ADDR,
            &reg_addr,
            sizeof(reg_addr),
            false);
    BMP180_RETURN_IF_ERROR(err_code);

    /** Store the raw data in the temperature buffer */
    err_code = nrf_drv_twi_rx(p_twi,
            BMP180_DEVICE_ADDR,
            (uint8_t *) &g_data_buffer.raw_temperature,
            2);
    BMP180_RETURN_IF_ERROR(err_code);

    /** This transfer model will give us the value in big endian, so we need to convert it to little endian */
    g_data_buffer.raw_temperature = BMP180_BYTES_REVERSE_16BIT(g_data_buffer.raw_temperature);

    /** Calculate the temperature with the buffer raw value and store it in the temperature buffer */
    bmp180_drv_calculate_temp(&g_data_buffer);

    if(app_buffer != NULL) *app_buffer = g_data_buffer.temperature;

    return NRF_SUCCESS;
}

ret_code_t bmp180_drv_set_pwr_mode(bmp180_pwr_mode_t pwr_mode)
{
    g_pwr_mode = pwr_mode;
    return NRF_SUCCESS;
}

bmp180_pwr_mode_t bmp180_drv_get_pwr_mode()
{
    return g_pwr_mode;
}

ret_code_t bmp180_drv_reset()
{
    /** Set the data to transfer.
     *  The first byte set the target register address,
     *  The second byte is the command. */
    static uint8_t tx_data[2] = {BMP180_REG_SOFT_RESET, SOFTWARE_RESET};

    /** If the p_twi is null, this transaction will bug the app, so it's important to check for safety */
    BMP180_NULL_PARAM_CHECK(p_twi);

    return nrf_drv_twi_tx(p_twi,
            BMP180_DEVICE_ADDR,
            tx_data,
            sizeof(tx_data),
            false);
}

ret_code_t bmp180_drv_get_id(uint8_t * id_buff)
{
    static ret_code_t err_code;

    /** Set the target register address */
    static uint8_t reg_addr = BMP180_REG_ID;

    /** If the p_twi is null, this transaction will bug the app, so it's important to check for safety */
    BMP180_NULL_PARAM_CHECK(p_twi);

    BMP180_NULL_PARAM_CHECK(id_buff);

    err_code = nrf_drv_twi_tx(p_twi,
            BMP180_DEVICE_ADDR,
            &reg_addr,
            sizeof(reg_addr),
            false);
    BMP180_RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_rx(p_twi,
               BMP180_DEVICE_ADDR,
               id_buff,
               1);
    BMP180_RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;

}

uint16_t bmp180_drv_get_conv_time()
{
    /** Return the conversion time in ms according to the power mode */
    switch(g_pwr_mode)
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

static ret_code_t bmp180_drv_read_calibration_data()
{
    ret_code_t err_code;

    /** If the p_twi is null, this transaction will bug the app, so it's important to check for safety */
    BMP180_NULL_PARAM_CHECK(p_twi);

    /** Clear the current value for all the calibration variables */
    memset(&g_bmp180_callibration_data, 0, sizeof(g_bmp180_callibration_data));

    /** Set the target register address */
    static uint8_t reg_addr = BMP180_REG_CALIB_START;

    err_code = nrf_drv_twi_tx(p_twi,
            BMP180_DEVICE_ADDR,
            &reg_addr,
            sizeof(reg_addr),
            true);
    if(err_code != NRF_SUCCESS) return err_code;

    /** Store the received callibration data in the g_bmp180_callibration_data structure */
    err_code = nrf_drv_twi_rx(p_twi,
            BMP180_DEVICE_ADDR,
            (uint8_t *) &g_bmp180_callibration_data,
            sizeof(g_bmp180_callibration_data));

    uint16_t * buff;
    for(uint8_t n = 0; n < (sizeof(g_bmp180_callibration_data)/2);n++)
    {
        buff = ((uint16_t *) (&g_bmp180_callibration_data)) + n;
        *buff = BMP180_BYTES_REVERSE_16BIT(*buff);
    }

    return err_code;
}

static ret_code_t bmp180_drv_calculate_temp(bmp180_data_buffer_t * p_buff)
{
    int32_t X1;
    int32_t X2;
    int32_t B5;

    X1 = ((p_buff->raw_temperature - g_bmp180_callibration_data.AC6) * g_bmp180_callibration_data.AC5) >> 15;
    X2 = (g_bmp180_callibration_data.MC * 2048) / (X1 + g_bmp180_callibration_data.MD);
    B5 = X1 + X2;

    p_buff->temperature = (B5 + 8) >> 4;
    return NRF_SUCCESS;
}

static ret_code_t bmp180_drv_calculate_pres(bmp180_data_buffer_t * p_buff)
{
    int32_t X1;
    int32_t X2;
    int32_t X3;

    int32_t B3;
    int32_t B5;
    int32_t B6;
    uint32_t B4;
    uint32_t B7;

    int32_t local_buff = 0;


    X1 = ((p_buff->raw_temperature - g_bmp180_callibration_data.AC6) * g_bmp180_callibration_data.AC5) >> 15;
    X2 = (g_bmp180_callibration_data.MC * 2048) / (X1 + g_bmp180_callibration_data.MD);
    B5 = X1 + X2;
    p_buff->temperature = (B5 + 8) >> 4;

    B6 = B5 - 4000;
    X1 = (g_bmp180_callibration_data.B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (g_bmp180_callibration_data.AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((g_bmp180_callibration_data.AC1 * 4) + X3) << g_pwr_mode) + 2) >> 2;
    X1 = (g_bmp180_callibration_data.AC3 * B6) >> 13;
    X2 = (g_bmp180_callibration_data.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = g_bmp180_callibration_data.AC4 * (uint32_t)(X3 + 32768) >> 15;
    B7 = ((uint32_t) p_buff->raw_pressure - B3) * (50000 >> g_pwr_mode);

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

    p_buff->pressure = local_buff;

    return NRF_SUCCESS;
}

