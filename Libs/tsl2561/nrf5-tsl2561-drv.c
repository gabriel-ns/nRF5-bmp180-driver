#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "nrf_error.h"
#include "nrf_drv_twi.h"
#include "nrf5-tsl2561-drv.h"


#define TSL2561_BYTES_REVERSE_32BIT(x) ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24))
#define TSL2561_BYTES_REVERSE_16BIT(x) (((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF))

#define TSL2561_NULL_PARAM_CHECK(PARAM)  \
    do{                                 \
        if(PARAM == NULL)               \
        {                               \
            return NRF_ERROR_NULL;      \
        }                               \
    }while(0)

#define TSL2561_RETURN_IF_ERROR(ERR)     \
    do{                                 \
        if(ERR != NRF_SUCCESS)          \
        {                               \
            return ERR;                 \
        }                               \
    }while(0)

#define TSL2561_DEVICE_ADDR         0x39

#define TSL2561_CMD_TEMPLATE        0xC0

#define TSL2561_VISIBLE 2                   // channel 0 - channel 1
#define TSL2561_INFRARED 1                  // channel 1
#define TSL2561_FULLSPECTRUM 0              // channel 0


#define TSL2561_LUX_SCALE         (14)      // Scale by 2^14
#define TSL2561_RATIO_SCALE   (9)       // Scale ratio by 2^9

//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// Integration time scaling factors
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
#define TSL2561_CH_SCALE       (10)      // Scale channel values by 2^10
#define TSL2561_CH_SCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_CH_SCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE

//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// T, FN, and CL Package coefficients
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// For Ch1/Ch0=0.00 to 0.50
//      Lux/Ch0=0.0304−0.062*((Ch1/Ch0)^1.4)
//      piecewise approximation
//          For Ch1/Ch0=0.00 to 0.125:
//              Lux/Ch0=0.0304−0.0272*(Ch1/Ch0)
//
//          For Ch1/Ch0=0.125 to 0.250:
//              Lux/Ch0=0.0325−0.0440*(Ch1/Ch0)
//
//          For Ch1/Ch0=0.250 to 0.375:
//              Lux/Ch0=0.0351−0.0544*(Ch1/Ch0)
//
//          For Ch1/Ch0=0.375 to 0.50:
//              Lux/Ch0=0.0381−0.0624*(Ch1/Ch0)
//
// For Ch1/Ch0=0.50 to 0.61:
//      Lux/Ch0=0.0224−0.031*(Ch1/Ch0)
//
// For Ch1/Ch0=0.61 to 0.80:
//      Lux/Ch0=0.0128−0.0153*(Ch1/Ch0)
//
// For Ch1/Ch0=0.80 to 1.30:
//      Lux/Ch0=0.00146−0.00112*(Ch1/Ch0)
//
// For Ch1/Ch0>1.3:
//      Lux/Ch0=0
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
#define TSL2561_K1T           (0x0040)  // 0.125 * 2^RATIO_SCALE
#define TSL2561_B1T           (0x01f2)  // 0.0304 * 2^LUX_SCALE
#define TSL2561_M1T           (0x01be)  // 0.0272 * 2^LUX_SCALE

#define TSL2561_K2T           (0x0080)  // 0.250 * 2^RATIO_SCALE
#define TSL2561_B2T           (0x0214)  // 0.0325 * 2^LUX_SCALE
#define TSL2561_M2T           (0x02d1)  // 0.0440 * 2^LUX_SCALE

#define TSL2561_K3T           (0x00c0)  // 0.375 * 2^RATIO_SCALE
#define TSL2561_B3T           (0x023f)  // 0.0351 * 2^LUX_SCALE
#define TSL2561_M3T           (0x037b)  // 0.0544 * 2^LUX_SCALE

#define TSL2561_K4T           (0x0100)  // 0.50 * 2^RATIO_SCALE
#define TSL2561_B4T           (0x0270)  // 0.0381 * 2^LUX_SCALE
#define TSL2561_M4T           (0x03fe)  // 0.0624 * 2^LUX_SCALE

#define TSL2561_K5T           (0x0138)  // 0.61 * 2^RATIO_SCALE
#define TSL2561_B5T           (0x016f)  // 0.0224 * 2^LUX_SCALE
#define TSL2561_M5T           (0x01fc)  // 0.0310 * 2^LUX_SCALE

#define TSL2561_K6T           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_B6T           (0x00d2)  // 0.0128 * 2^LUX_SCALE
#define TSL2561_M6T           (0x00fb)  // 0.0153 * 2^LUX_SCALE

#define TSL2561_K7T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B7T           (0x0018)  // 0.00146 * 2^LUX_SCALE
#define TSL2561_M7T           (0x0012)  // 0.00112 * 2^LUX_SCALE

#define TSL2561_K8T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B8T           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_M8T           (0x0000)  // 0.000 * 2^LUX_SCALE

//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// CS package coefficients
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// For 0 <= Ch1/Ch0 <= 0.52
//     Lux/Ch0 = 0.0315−0.0593*((Ch1/Ch0)^1.4)
//        piecewise approximation
//            For 0 <= Ch1/Ch0 <= 0.13
//                Lux/Ch0 = 0.0315−0.0262*(Ch1/Ch0)
//            For 0.13 <= Ch1/Ch0 <= 0.26
//                Lux/Ch0 = 0.0337−0.0430*(Ch1/Ch0)
//            For 0.26 <= Ch1/Ch0 <= 0.39
//                Lux/Ch0 = 0.0363−0.0529*(Ch1/Ch0)
//            For 0.39 <= Ch1/Ch0 <= 0.52
//                Lux/Ch0 = 0.0392−0.0605*(Ch1/Ch0)
//
//     For 0.52 < Ch1/Ch0 <= 0.65
//         Lux/Ch0 = 0.0229−0.0291*(Ch1/Ch0)
//     For 0.65 < Ch1/Ch0 <= 0.80
//         Lux/Ch0 = 0.00157−0.00180*(Ch1/Ch0)
//     For 0.80 < Ch1/Ch0 <= 1.30
//         Lux/Ch0 = 0.00338−0.00260*(Ch1/Ch0)
//     For Ch1/Ch0 > 1.30
//         Lux = 0
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
#define TSL2561_K1C           (0x0043)  // 0.130 * 2^RATIO_SCALE
#define TSL2561_B1C           (0x0204)  // 0.0315 * 2^LUX_SCALE
#define TSL2561_M1C           (0x01ad)  // 0.0262 * 2^LUX_SCALE

#define TSL2561_K2C           (0x0085)  // 0.260 * 2^RATIO_SCALE
#define TSL2561_B2C           (0x0228)  // 0.0337 * 2^LUX_SCALE
#define TSL2561_M2C           (0x02c1)  // 0.0430 * 2^LUX_SCALE

#define TSL2561_K3C           (0x00c8)  // 0.390 * 2^RATIO_SCALE
#define TSL2561_B3C           (0x0253)  // 0.0363 * 2^LUX_SCALE
#define TSL2561_M3C           (0x0363)  // 0.0529 * 2^LUX_SCALE

#define TSL2561_K4C           (0x010a)  // 0.520 * 2^RATIO_SCALE
#define TSL2561_B4C           (0x0282)  // 0.0392 * 2^LUX_SCALE
#define TSL2561_M4C           (0x03df)  // 0.0605 * 2^LUX_SCALE

#define TSL2561_K5C           (0x014d)  // 0.65 * 2^RATIO_SCALE
#define TSL2561_B5C           (0x0177)  // 0.0229 * 2^LUX_SCALE
#define TSL2561_M5C           (0x01dd)  // 0.0291 * 2^LUX_SCALE

#define TSL2561_K6C           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_B6C           (0x0101)  // 0.0157 * 2^LUX_SCALE
#define TSL2561_M6C           (0x0127)  // 0.0180 * 2^LUX_SCALE

#define TSL2561_K7C           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B7C           (0x0037)  // 0.00338 * 2^LUX_SCALE
#define TSL2561_M7C           (0x002b)  // 0.00260 * 2^LUX_SCALE

#define TSL2561_K8C           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B8C           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_M8C           (0x0000)  // 0.000 * 2^LUX_SCALE

typedef enum tsl2561_reg_addr
{
    TSL2561_REG_ADDR_CONTROL    = 0x00,
    TSL2561_REG_ADDR_TIMING     = 0x01,
    TSL2561_REG_ADDR_TSH_LL     = 0x02,
    TSL2561_REG_ADDR_TSH_LH     = 0x03,
    TSL2561_REG_ADDR_TSH_HL     = 0x04,
    TSL2561_REG_ADDR_TSH_HH     = 0x05,
    TSL2561_REG_ADDR_INT        = 0x06,
    TSL2561_REG_ADDR_CRC        = 0x08,
    TSL2561_REG_ADDR_ID         = 0x0A,
    TSL2561_REG_ADDR_DATA0L     = 0x0C,
    TSL2561_REG_ADDR_DATA0H     = 0x0D,
    TSL2561_REG_ADDR_DATA1L     = 0x0E,
    TSL2561_REG_ADDR_DATA1H     = 0x0F
} tsl2561_reg_addr_t;

nrf_drv_twi_t * p_twi = NULL;

/** Default config */
tsl2561_config_t config = {
        .integration_time = TSL2561_INTEGRATION_TIME_13_7MS,
        .gain = TSL2561_GAIN_0,
        .power = TSL2561_POWER_DOWN
};

ret_code_t tsl2561_drv_begin(nrf_drv_twi_t * p_ext_twi)
{
    TSL2561_NULL_PARAM_CHECK(p_ext_twi);
    p_twi = p_ext_twi;

    tsl2561_drv_set_gain(config.gain);
    tsl2561_drv_set_integration_time(config.integration_time);

    return NRF_SUCCESS;
}

ret_code_t tsl2561_drv_set_power(tsl2561_power_t pwr)
{
    config.power = pwr;

    uint8_t cmd[2];
    cmd[0] = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_CONTROL;
    cmd[1] = config.power;
    return nrf_drv_twi_tx(p_twi, TSL2561_DEVICE_ADDR, cmd, 2, false);
}

ret_code_t tsl2561_drv_set_gain(tsl2561_gain_t gain)
{
    config.gain = gain;

    uint8_t cmd[2];
    cmd[0] = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_TIMING;
    cmd[1] = 0x00 | (config.gain << 4) | config.integration_time;
    return nrf_drv_twi_tx(p_twi, TSL2561_DEVICE_ADDR, cmd, 2, false);
}

ret_code_t tsl2561_drv_set_integration_time(tsl2561_integration_time_t time)
{
    config.integration_time = time;

    uint8_t cmd[2];
    cmd[0] = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_TIMING;
    cmd[1] = 0x00 | (config.gain << 4) | config.integration_time;
    return nrf_drv_twi_tx(p_twi, TSL2561_DEVICE_ADDR, cmd, 2, false);
}

ret_code_t tsl2561_drv_get_device_id(uint8_t * id)
{
    uint8_t cmd;
    ret_code_t err_code;

    cmd = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_ID;
    err_code = nrf_drv_twi_tx(p_twi, TSL2561_DEVICE_ADDR, &cmd, 1, true);
    TSL2561_RETURN_IF_ERROR(err_code);
    return nrf_drv_twi_rx(p_twi, TSL2561_DEVICE_ADDR, id, 1);
}

ret_code_t tsl2561_drv_read_data(tsl2561_adc_data_t * data)
{
    uint8_t cmd;
    ret_code_t err_code;

    cmd = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_DATA0L;
    err_code = nrf_drv_twi_tx(p_twi, TSL2561_DEVICE_ADDR, &cmd, 1, true);
    TSL2561_RETURN_IF_ERROR(err_code);
    return nrf_drv_twi_rx(p_twi, TSL2561_DEVICE_ADDR, (uint8_t *) data, 4);
}

uint32_t tsl2561_drv_calculate_lux(tsl2561_adc_data_t * data)
{
    uint32_t ch0buff = data->data0;
    uint32_t ch1buff = data->data1;

    uint32_t ch_scale;

    /** If the integration time is NOT 402ms it is scaled */
    switch(config.integration_time)
    {
        case TSL2561_INTEGRATION_TIME_13_7MS:
            ch_scale = TSL2561_CH_SCALE_TINT0;
            break;
        case TSL2561_INTEGRATION_TIME_101MS:
            ch_scale = TSL2561_CH_SCALE_TINT0;
            break;
        default: // Assume no scaling
            ch_scale = (1 << TSL2561_CH_SCALE);
            break;
    }


    /** Scale if the gain is not 16x */
    if(!config.gain)
    {
        ch_scale = ch_scale << 4; // Scale 1x to 16x
    }

    ch0buff = (data->data0 * ch_scale) >> TSL2561_CH_SCALE;
    ch1buff = (data->data1 * ch_scale) >> TSL2561_CH_SCALE;

    /** find the ratio between channels (ch1 / ch0) */
    uint32_t ratio = 0;
    if(ch0buff != 0)
    {
        ratio = (ch1buff << (TSL2561_RATIO_SCALE + 1)) / ch0buff;
    }

    /** Rounding the division */
    ratio = (ratio + 1) >> 1;

    uint16_t b;
    uint16_t m;

    /** For now, it's only supported the T, FN and CL package */

    if ((ratio >= 0) && (ratio <= TSL2561_K1T))
    {
        b = TSL2561_B1T;
        m = TSL2561_M1T;
    }
    else if (ratio <= TSL2561_K2T)
    {
        b = TSL2561_B2T;
        m = TSL2561_M2T;
    }
    else if (ratio <= TSL2561_K3T)
    {
        b = TSL2561_B3T;
        m = TSL2561_M3T;
    }
    else if (ratio <= TSL2561_K4T)
    {
        b = TSL2561_B4T;
        m = TSL2561_M4T;
    }
    else if (ratio <= TSL2561_K5T)
    {
        b = TSL2561_B5T;
        m = TSL2561_M5T;
    }
    else if (ratio <= TSL2561_K6T)
    {
        b = TSL2561_B6T;
        m = TSL2561_M6T;
    }
    else if (ratio <= TSL2561_K7T)
    {
        b = TSL2561_B7T;
        m = TSL2561_M7T;
    }
    else if (ratio > TSL2561_K8T)
    {
        b = TSL2561_B8T;
        m = TSL2561_M8T;
    }

    uint32_t lux;
    lux = ((ch0buff * b) - (ch1buff * m));

    lux += (1 << (TSL2561_LUX_SCALE - 1));

    lux = lux >> TSL2561_LUX_SCALE;

    return lux;
}

uint16_t tsl2561_get_read_time()
{
    switch(config.integration_time)
    {
        case TSL2561_INTEGRATION_TIME_13_7MS:
            return 15;
            break;
        case TSL2561_INTEGRATION_TIME_101MS:
            return 105;
            break;
        case TSL2561_INTEGRATION_TIME_402MS:
            return 405;
            break;
    }
    return 405;
}
