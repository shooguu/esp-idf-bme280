// SWC
#include "bme280.hpp"

// ESP-IDF SDK
#include "driver/spi_common.h"
#include "hal/spi_types.h"
#include "soc/soc.h"

BME280::BME280(
    const spi_device_interface_config_t &dev_cfg,
    const spi_bus_config_t &bus_cfg,
    spi_device_handle_t &spi_dev) :
        dev_cfg{dev_cfg}, bus_cfg{bus_cfg}, spi_dev{spi_dev}
{
    spi_bus_initialize(VSPI_HOST, &bus_cfg, 1u);
    spi_bus_add_device(VSPI_HOST, &dev_cfg, &spi_dev);

    get_calibration_data();
}

void BME280::get_calibration_data(
    void)
{
    register_read(ADDR_DIG_T1, (uint8_t*)&(calibration_data.dig_T1), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_T2, (uint8_t*)&(calibration_data.dig_T2), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_T3, (uint8_t*)&(calibration_data.dig_T3), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P1, (uint8_t*)&(calibration_data.dig_P1), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P2, (uint8_t*)&(calibration_data.dig_P2), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P3, (uint8_t*)&(calibration_data.dig_P3), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P4, (uint8_t*)&(calibration_data.dig_P4), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P5, (uint8_t*)&(calibration_data.dig_P5), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P6, (uint8_t*)&(calibration_data.dig_P6), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P7, (uint8_t*)&(calibration_data.dig_P7), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P8, (uint8_t*)&(calibration_data.dig_P8), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_P9, (uint8_t*)&(calibration_data.dig_P9), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_H1, (uint8_t*)&(calibration_data.dig_H1), CAL_SIZE_1_BYTE);
    register_read(ADDR_DIG_H2, (uint8_t*)&(calibration_data.dig_H2), CAL_SIZE_2_BYTE);
    register_read(ADDR_DIG_H3, (uint8_t*)&(calibration_data.dig_H3), CAL_SIZE_1_BYTE);

    register_read(ADDR_DIG_H4, (uint8_t*)&(calibration_data.dig_H4), CAL_SIZE_2_BYTE);
    calibration_data.dig_H4 = ((calibration_data.dig_H4 >> 8u) & 0xF) |
                              ((calibration_data.dig_H4 & 0xFF) << 4u);

    register_read(ADDR_DIG_H5, (uint8_t*)&(calibration_data.dig_H5), CAL_SIZE_2_BYTE);
    calibration_data.dig_H5 = (((calibration_data.dig_H5 & 0xF) << 4u) |
                                (calibration_data.dig_H5 & 0xFF00)) >> 4u;

    register_read(ADDR_DIG_H6, (uint8_t*)&(calibration_data.dig_H6), CAL_SIZE_1_BYTE);
}

void BME280::clear_all_registers(
    void)
{
    uint8_t tx_buffer[6u] =
    {
        CTRL_MEAS & REG_WRITE_ONLY,
        0x00u,
        CONFIG & REG_WRITE_ONLY,
        0x00u,
        CTRL_HUM & REG_WRITE_ONLY,
        0x00u,
    };

    spi_transaction_t trans =
    {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 8u * 6u,
        .rxlength = 0u,
        .user = NULL,
        .tx_buffer = &tx_buffer,
        .rx_buffer = NULL,
    };

    // Aquire bus to prevent others from writing
    spi_device_acquire_bus(spi_dev, portMAX_DELAY);

    // Transmit data
    spi_device_transmit(spi_dev, &trans);

    // Release bus to allow other threads to access bus
    spi_device_release_bus(spi_dev);
}

void BME280::register_write(
    const uint8_t address,
    const uint8_t data)
{
    uint8_t tx_buffer[2u] =
    {
        (uint8_t)(address & REG_WRITE_ONLY),
        data,
    };

    spi_transaction_t trans =
    {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 8u * 2u,
        .rxlength = 0u,
        .user = NULL,
        .tx_buffer = &tx_buffer,
        .rx_buffer = NULL,
    };

    // Aquire bus to prevent others from writing
    spi_device_acquire_bus(spi_dev, portMAX_DELAY);

    // Transmit data
    spi_device_transmit(spi_dev, &trans);

    // Release bus to allow other threads to access bus
    spi_device_release_bus(spi_dev);
}

void BME280::register_read(
    const uint8_t address,
    uint8_t *buffer,
    const uint8_t size)
{
    uint8_t tx_buffer[MAX_TX_BUFFER_SIZE] = { 0u };
    uint8_t rx_buffer[MAX_RX_BUFFER_SIZE] = { 0u };

    // Set the address
    tx_buffer[0u] = address | REG_READ_ONLY;

    spi_transaction_t trans =
    {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 8u * (size + 1u),
        .rxlength = 8u * (size + 1u),
        .user = NULL,
        .tx_buffer = &tx_buffer,
        .rx_buffer = &rx_buffer,
    };

    // Aquire bus to prevent others from writing
    spi_device_acquire_bus(spi_dev, portMAX_DELAY);

    // Transmit data
    spi_device_transmit(spi_dev, &trans);

    // Release bus to allow other threads to access bus
    spi_device_release_bus(spi_dev);

    // write rx data to return buffer
    (void) memcpy(buffer, &(rx_buffer[1u]), size);
}

void BME280::burst_read_data(
    void)
{
    uint8_t tx_buffer = PRESS_MSB | REG_READ_ONLY;
    uint8_t rx_buffer[9u] = { 0u };

    spi_transaction_t trans =
    {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 8u * 9u,
        .rxlength = 8u * 9u,
        .user = NULL,
        .tx_buffer = &tx_buffer,
        .rx_buffer = &rx_buffer,
    };

    // Aquire bus to prevent others from writing
    spi_device_acquire_bus(spi_dev, portMAX_DELAY);

    // Transmit data
    spi_device_transmit(spi_dev, &trans);

    // Release bus to allow other threads to access bus
    spi_device_release_bus(spi_dev);

    BME280_S32_t pressure_adc = ((rx_buffer[1u] << 16u) |
                                 (rx_buffer[2u] << 8u) |
                                  rx_buffer[3u]) >> 4u;

    BME280_S32_t temperature_adc = ((rx_buffer[4u] << 16u) |
                                    (rx_buffer[5u] << 8u) |
                                     rx_buffer[6u]) >> 4u;

    BME280_S32_t humidity_adc = (rx_buffer[7u] << 8u) |
                                 rx_buffer[8u];

    temperature = (float)compensate_T_int32(temperature_adc) / 100.0;
    pressure = (float)compensate_P_int64(pressure_adc) / 25600.0;
    humidity = (float)compensate_H_int32(humidity_adc) / 1024.0;
}

void BME280::sample_data(
    const uint8_t address,
    const uint8_t &data)
{
    uint8_t data_buffer = 0u;

    // read existing register data
    register_read(address, &data_buffer, 1u);

    // write new data
    data_buffer |= data;

    // write data over spi
    register_write(address, data_buffer);
}

void BME280::pressure_oversample(
    oversample_e os)
{
    // write new data
    uint8_t data = ((uint8_t)os << PRESS_OVERSAMPLE_SHIFT);

    sample_data(CTRL_MEAS, data);
}

void BME280::humidity_oversample(
    oversample_e os)
{
    // write new data
    uint8_t data = ((uint8_t)os << HUM_OVERSAMPLE_SHIFT);

    // write data over spi
    register_write(CTRL_HUM, data);
}

void BME280::temperature_oversample(
    oversample_e os)
{
    // write new data
    uint8_t data = ((uint8_t)os << TEMP_OVERSAMPLE_SHIFT);

    // write data over spi
    register_write(CTRL_MEAS, data);
}

void BME280::set_force_mode(
    void)
{
    // write new data
    uint8_t data = FORCED_MODE;

    // write data over spi
    register_write(CTRL_MEAS, data);
}

void BME280::set_normal_mode(
    void)
{
    // write new data
    uint8_t data = NORMAL_MODE;

    // write data over spi
    register_write(CTRL_MEAS, data);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
BME280_S32_t BME280::compensate_T_int32(
    BME280_S32_t adc_T)
{
    BME280_S32_t var1, var2, temp;

    var1 = ((((adc_T >> 3) - ((BME280_S32_t)calibration_data.dig_T1 << 1))) * ((BME280_S32_t)calibration_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((BME280_S32_t)calibration_data.dig_T1)) * ((adc_T >> 4) -
                ((BME280_S32_t)calibration_data.dig_T1))) >> 12) * ((BME280_S32_t)calibration_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    temp = (t_fine * 5 + 128) >> 8;
    return temp;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BME280_U32_t BME280::compensate_P_int64(
    BME280_S32_t adc_P)
{
    BME280_S64_t var1, var2, p;

    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)calibration_data.dig_P6;
    var2 = var2 + ((var1 * (BME280_S64_t)calibration_data.dig_P5) << 17);
    var2 = var2 + (((BME280_S64_t)calibration_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (BME280_S64_t)calibration_data.dig_P3) >> 8) + ((var1 * (BME280_S64_t)calibration_data.dig_P2) << 12);
    var1 = (((((BME280_S64_t)1) << 47) + var1))*((BME280_S64_t)calibration_data.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((BME280_S64_t)calibration_data.dig_P9) * (p>> 13) * ( p>> 13)) >> 25;
    var2 = (((BME280_S64_t)calibration_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)calibration_data.dig_P7) << 4);

    return (BME280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t BME280::compensate_H_int32(
    BME280_S32_t adc_H)
{
    BME280_S32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)calibration_data.dig_H4) << 20) - (((BME280_S32_t)calibration_data.dig_H5) * v_x1_u32r)) +
        ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)calibration_data.dig_H6)) >> 10) * (((v_x1_u32r *
        ((BME280_S32_t)calibration_data.dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
        ((BME280_S32_t)calibration_data.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)calibration_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (BME280_U32_t)(v_x1_u32r >> 12);
}
