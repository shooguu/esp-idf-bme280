// Standard libraries
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

// ESP-IDF SDK
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/clk_tree_defs.h"

// SWC
#include "bme280.hpp"

extern "C" { void app_main(); }

static spi_device_interface_config_t dev_cfg =
{
    .command_bits = 0u,
    .address_bits = 0u,
    .dummy_bits = 0u,
    .mode = 0,
    .clock_source = SPI_CLK_SRC_DEFAULT,
    .duty_cycle_pos = 0,
    .cs_ena_pretrans = 1u,
    .cs_ena_posttrans = 0u,
    .clock_speed_hz = 1000000u,
    .input_delay_ns = 0u,
    .spics_io_num = GPIO_NUM_15,
    .flags = 0u,
    .queue_size = 7u,
    .pre_cb = 0,
    .post_cb = 0,
};

static spi_bus_config_t bus_cfg =
{
    .mosi_io_num = 23u,
    .miso_io_num = 19u,
    .sclk_io_num = 18u,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .data4_io_num = -1,
    .data5_io_num = -1,
    .data6_io_num = -1,
    .data7_io_num = -1,
    .max_transfer_sz = 32u,
    .flags = 0,
    .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
    .intr_flags = 0,
};

void task_500ms(void *pvParameters)
{
    spi_device_handle_t spi_dev;
    BME280 bme(dev_cfg, bus_cfg, spi_dev);

    bme.clear_all_registers();

    bme.pressure_oversample(
        oversample_16x);

    bme.humidity_oversample(
        oversample_16x);

    bme.temperature_oversample(
        oversample_16x);

    for (;;)
    {
        bme.set_force_mode();

        bme.burst_read_data();

        printf("Temperature: %f\n", bme.temperature);
        printf("Pressure: %f\n", bme.pressure);
        printf("Humidity: %f\n", bme.humidity);

        vTaskDelay(250u / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Create task
    xTaskCreate(task_500ms,
                "task_500ms",
                4096u,
                NULL,
                1u,
                NULL);

}
