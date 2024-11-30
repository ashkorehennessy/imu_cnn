#include <stdio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "icm20948.h"
// icm20948_i2c I2C Pins
#define SDA_PIN GPIO_NUM_46
#define SCL_PIN GPIO_NUM_9
// icm20948_spi SPI Pins
#define CS_PIN   GPIO_NUM_10
#define MOSI_PIN GPIO_NUM_11
#define SCLK_PIN GPIO_NUM_12
#define MISO_PIN GPIO_NUM_13
static icm20948_handle_t icm20948_spi = NULL;
static icm20948_handle_t icm20948_i2c = NULL;
static icm20948_data_t icm20948_data_spi;
static icm20948_data_t icm20948_data_i2c;
SemaphoreHandle_t mutex;
static void icm20948_spi_init()
{
    icm20948_spi = icm20948_create(&icm20948_data_spi, "ICM20948 SPI");
    icm20948_spi_bus_init(icm20948_spi, SPI2_HOST, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_PIN, SPI_MASTER_FREQ_8M);
    icm20948_configure(icm20948_spi, ACCE_FS_8G, GYRO_FS_2000DPS);
}
static void icm20948_i2c_init()
{
    icm20948_i2c = icm20948_create(&icm20948_data_i2c, "ICM20948 I2C");
    icm20948_i2c_bus_init(icm20948_i2c, I2C_NUM_0, ICM20948_I2C_ADDRESS, SCL_PIN, SDA_PIN, 400000);
    icm20948_configure(icm20948_i2c, ACCE_FS_8G, GYRO_FS_2000DPS);
}
static void icm20948_spi_read(void *pvParameter)
{
    icm20948_spi_init();
    while (1) {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE){
            int64_t timestamp = esp_timer_get_time();
            icm20948_get_acce(icm20948_spi);
            icm20948_get_gyro(icm20948_spi);
            icm20948_get_angle(icm20948_spi);
            icm20948_get_temp(icm20948_spi);
            int64_t dt = esp_timer_get_time() - timestamp;
            ESP_LOGI("ICM20948 SPI", "angles x:%7.2f y:%7.2f z:%7.2f time used:%5lldus",
                icm20948_data_spi.anglex, icm20948_data_spi.angley, icm20948_data_spi.anglez, dt);
            xSemaphoreGive(mutex);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
static void icm20948_i2c_read(void *pvParameter)
{
    icm20948_i2c_init();
    while (1) {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE){
            int64_t timestamp = esp_timer_get_time();
            icm20948_get_acce(icm20948_i2c);
            icm20948_get_gyro(icm20948_i2c);
            icm20948_get_angle(icm20948_i2c);
            icm20948_get_temp(icm20948_i2c);
            int64_t dt = esp_timer_get_time() - timestamp;
            ESP_LOGI("ICM20948 I2C", "angles x:%7.2f y:%7.2f z:%7.2f time used:%5lldus",
                icm20948_data_i2c.anglex, icm20948_data_i2c.angley, icm20948_data_i2c.anglez, dt);
            xSemaphoreGive(mutex);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void app_main(void)
{
    mutex = xSemaphoreCreateMutex();
    xTaskCreate(icm20948_spi_read, "icm20948_spi_read", 1024 * 4, NULL, 5, NULL);
    xTaskCreate(icm20948_i2c_read, "icm20948_i2c_read", 1024 * 4, NULL, 5, NULL);
    while(1){
        vTaskDelay(1000);
    }
}
