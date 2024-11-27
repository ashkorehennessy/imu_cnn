#include <stdio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "icm20948.h"
#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2
static icm20948_handle_t icm20948 = NULL;
static icm20948_data_t icm20948_data;
static void icm20948_init()
{
    icm20948 = icm20948_create(I2C_NUM_0, ICM20948_I2C_ADDRESS, &icm20948_data);
    icm20948_i2c_bus_init(icm20948, SCL_PIN, SDA_PIN, 400000);
    icm20948_configure(icm20948, ACCE_FS_8G, GYRO_FS_2000DPS);
}
static void icm20948_read()
{
    const int64_t timestamp = esp_timer_get_time();
    icm20948_get_all(icm20948);
    int64_t dt = esp_timer_get_time() - timestamp;
    // firewater format
    printf(":%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lld\n",
                icm20948_data.ax, icm20948_data.ay, icm20948_data.az,
                icm20948_data.gx, icm20948_data.gy, icm20948_data.gz,
                icm20948_data.anglex, icm20948_data.angley, icm20948_data.anglez,icm20948_data.temp,dt);
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
