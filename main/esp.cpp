#include <stdio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "icm20948.h"
#include "tgmath.h"
#include "imu_cnn.h"
// icm20948_spi SPI Pins
#define CS_PIN   GPIO_NUM_10
#define MOSI_PIN GPIO_NUM_11
#define SCLK_PIN GPIO_NUM_12
#define MISO_PIN GPIO_NUM_13
static icm20948_handle_t icm20948_spi = NULL;
static icm20948_data_t icm20948_data;
#define SAMPLE_SIZE 200
float sample_data[SAMPLE_SIZE][6];
static TaskHandle_t notify_task_handle = NULL;

static void icm20948_spi_init()
{
    icm20948_spi = icm20948_create(&icm20948_data, "ICM20948 SPI", -11);
    icm20948_spi_bus_init(icm20948_spi, SPI2_HOST, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_PIN, SPI_MASTER_FREQ_8M);
    icm20948_configure(icm20948_spi, ACCE_FS_4G, GYRO_FS_500DPS);
}

static void icm20948_task(void *pvParameter)
{
    int flag_write_sample = 0;
    int sample_index = 0;
    icm20948_spi_init();
    while (1) {
        icm20948_get_acce(icm20948_spi);
        icm20948_get_gyro(icm20948_spi);

        if(flag_write_sample == 0){
            if(fabsf(icm20948_data.ax) > 3.8f || fabsf(icm20948_data.ay) > 3.8f || fabsf(icm20948_data.az) > 3.8f){
                flag_write_sample = 1;
                ESP_LOGI("icm20948", "waiting jitter end");
                vTaskDelay(500 / portTICK_PERIOD_MS);
                ESP_LOGI("icm20948", "write sample");
                sample_index = 0;
            }
        }
        if(flag_write_sample == 1){
            sample_data[sample_index][0] = icm20948_data.ax;
            sample_data[sample_index][1] = icm20948_data.ay;
            sample_data[sample_index][2] = icm20948_data.az;
            sample_data[sample_index][3] = icm20948_data.gx;
            sample_data[sample_index][4] = icm20948_data.gy;
            sample_data[sample_index][5] = icm20948_data.gz;
            sample_index++;
            if(sample_index >= SAMPLE_SIZE){
                flag_write_sample = 0;
                ESP_LOGI("icm20948", "notify to process data");
                xTaskNotifyGive(notify_task_handle);
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void data_process(void *pvParameter)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("SAMPLE_START\n");
        printf("ax,ay,az,gx,gy,gz\n");
        for(int i = 0; i < SAMPLE_SIZE; i++){
            printf("%f,%f,%f,%f,%f,%f\n", sample_data[i][0], sample_data[i][1], sample_data[i][2], sample_data[i][3], sample_data[i][4], sample_data[i][5]);
        }
        printf("SAMPLE_END\n");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void imu_reference(void *pvParameter)
{
    int predict_result = 0;
    load_ncnn_model();
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t start_timer = esp_timer_get_time();
        predict_result = predict_imu_data();
        int64_t end_timer = esp_timer_get_time();
        ESP_LOGI("imu_cnn", "predict result: %d, time used %dms", predict_result, (int)((end_timer - start_timer) / 1000));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    xTaskCreate(icm20948_task, "icm20948_task", 1024 * 4, NULL, 5, NULL);
//    xTaskCreate(data_process, "data_process", 1024 * 4, NULL, 4, &notify_task_handle);
    xTaskCreate(imu_reference, "imu_reference", 1024 * 4, NULL, 4, &notify_task_handle);
    while(1){
        vTaskDelay(1000);
    }
}
