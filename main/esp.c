#include <stdio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "icm20948.h"
#define LED1 1
#define LED2 2
#define I2C_NUM I2C_NUM_0
static icm20948_handle_t icm20948 = NULL;
static icm20948_acce_value_t icm20948_acce;
static icm20948_gyro_value_t icm20948_gyro;
void blink_task(void *pvParameter)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT|GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<LED1) | (1ULL<<LED2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    while(1){
        gpio_set_level(LED1, 0);
        gpio_set_level(LED2, 1);
        ESP_LOGI("blink_task", "LED1 ON, LED2 OFF, query status: %d, %d", gpio_get_level(LED1), gpio_get_level(LED2));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 1);
        gpio_set_level(LED2, 0);
        ESP_LOGI("blink_task", "LED1 OFF, LED2 ON, query status: %d, %d", gpio_get_level(LED1), gpio_get_level(LED2));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
static void icm20948_init()
{
    icm20948 = icm20948_create(I2C_NUM, ICM20948_I2C_ADDRESS);
    icm20948_bus_init(icm20948);
    icm20948_configure(icm20948, ACCE_FS_8G, GYRO_FS_2000DPS);
    icm20948_wake_up(icm20948);
}
static void icm20948_read()
{
    icm20948_get_acce(icm20948, &icm20948_acce);
    icm20948_get_gyro(icm20948, &icm20948_gyro);
    // log format
    // ESP_LOGI("ICM20948", "Ax:%f, Ay:%f, Az:%f, Gx:%f, Gy:%f, Gz:%f",
    //             icm20948_acce.acce_x, icm20948_acce.acce_y, icm20948_acce.acce_z,
    //             icm20948_gyro.gyro_x, icm20948_gyro.gyro_y, icm20948_gyro.gyro_z);
    // firewater format
    printf(":%f,%f,%f,%f,%f,%f\n",
                icm20948_acce.acce_x, icm20948_acce.acce_y, icm20948_acce.acce_z,
                icm20948_gyro.gyro_x, icm20948_gyro.gyro_y, icm20948_gyro.gyro_z);
}
void app_main(void)
{
    esp_task_wdt_deinit();
    icm20948_init();
//    xTaskCreate(&blink_task, "blink_task", 4096, NULL, 5, NULL);
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
