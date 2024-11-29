#include <stdio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <soc/spi_struct.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "icm20948.h"
// ICM20948 I2C Pins
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define AD0_PIN GPIO_NUM_13
// ICM20948 SPI Pins
#define CS_PIN   GPIO_NUM_10
#define MOSI_PIN GPIO_NUM_11
#define SCLK_PIN GPIO_NUM_12
#define MISO_PIN GPIO_NUM_13
static icm20948_handle_t icm20948 = NULL;
static icm20948_data_t icm20948_data;
static void icm20948_init()
{
    // gpio_config_t io_conf = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT|GPIO_MODE_INPUT,
    //     .pin_bit_mask = (1ULL << AD0_PIN)|(1ULL << CS_PIN),
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    // };
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT|GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << CS_PIN,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    // gpio_set_level(AD0_PIN, 1);
    // gpio_set_level(CS_PIN, 1);
    icm20948 = icm20948_create(&icm20948_data);
    // icm20948_i2c_bus_init(icm20948, I2C_NUM_0, ICM20948_I2C_ADDRESS, SCL_PIN, SDA_PIN, 400000);
    icm20948_spi_bus_init(icm20948, SPI2_HOST, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_PIN, SPI_MASTER_FREQ_10M);
    icm20948_configure(icm20948, ACCE_FS_8G, GYRO_FS_2000DPS);
}
static void icm20948_read()
{
    const int64_t timestamp = esp_timer_get_time();
    icm20948_get_acce(icm20948);
    icm20948_get_gyro(icm20948);
    icm20948_get_angle(icm20948);
    icm20948_get_temp(icm20948);
    int64_t dt = esp_timer_get_time() - timestamp;
    // firewater format
    printf(":%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lld\n",
                icm20948_data.ax, icm20948_data.ay, icm20948_data.az,
                icm20948_data.gx, icm20948_data.gy, icm20948_data.gz,
                icm20948_data.anglex, icm20948_data.angley, icm20948_data.anglez,icm20948_data.temp,dt);
    // printf(":%lld\n",dt);
}
void app_main(void)
{
    esp_task_wdt_deinit();
    icm20948_init();
    const esp_timer_create_args_t cal_timer_config = {
        .callback = icm20948_read,
        .arg = NULL,
        .name = "icm20948 timer",
        .skip_unhandled_events = true,
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_handle_t cal_timer = NULL;
    esp_timer_create(&cal_timer_config, &cal_timer);
    esp_timer_start_periodic(cal_timer, 10000); // 5ms

    while(1){
    }
}
