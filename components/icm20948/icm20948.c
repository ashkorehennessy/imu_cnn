#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <esp_log.h>
#include "esp_system.h"
#include "icm20948.h"
#include <esp_timer.h>
#include <string.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                                                           \
	(byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'), (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'),        \
	    (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

#define ALPHA      0.99f        /*!< Weight of gyroscope */
#define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */

/* ICM20948 register */
#define ICM20948_GYRO_CONFIG_1 0x01
#define ICM20948_ACCEL_CONFIG  0x14
#define ICM20948_INT_PIN_CFG   0x0F
#define ICM20948_INT_ENABLE    0x10
#define ICM20948_INT_ENABLE_1  0x11
#define ICM20948_INT_ENABLE_2  0x12
#define ICM20948_INT_ENABLE_3  0x13
#define ICM20948_INT_STATUS    0x19
#define ICM20948_INT_STATUS_1  0x1A
#define ICM20948_INT_STATUS_2  0x1B
#define ICM20948_INT_STATUS_3  0x1C
#define ICM20948_ACCEL_XOUT_H  0x2D
#define ICM20948_GYRO_XOUT_H   0x33
#define ICM20948_TEMP_XOUT_H   0x39
#define ICM20948_PWR_MGMT_1    0x06
#define ICM20948_WHO_AM_I      0x00
#define ICM20948_REG_BANK_SEL  0x7F

static esp_err_t icm20948_write_i2c(icm20948_handle_t sensor,
                                const uint8_t reg_start_addr,
                                const uint8_t *const data_buf) {
	esp_err_t ret;
	const icm20948_dev_t *sens = sensor;
	uint8_t write_buf[2];

	write_buf[0] = reg_start_addr; // 设置寄存器地址
	write_buf[1] = *data_buf;      // 设置数据

	ret = i2c_master_transmit(sens->i2c_dev_handle, write_buf, 2, -1);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "I2C write failed: %s", esp_err_to_name(ret));
	}
	return ret;
}

static esp_err_t icm20948_read_i2c(icm20948_handle_t sensor,
								const uint8_t reg_start_addr,
								uint8_t *const data_buf,
								const uint8_t data_len) {
	esp_err_t ret;
	const icm20948_dev_t *sens = sensor;
	ret = i2c_master_transmit_receive(sens->i2c_dev_handle, &reg_start_addr, 1, data_buf, data_len, -1);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "I2C read failed: %s", esp_err_to_name(ret));
	}
	return ret;
}

static esp_err_t icm20948_write_spi(icm20948_handle_t sensor,
								const uint8_t reg_start_addr,
								const uint8_t *const data_buf) {
	esp_err_t ret = ESP_OK;
	uint8_t write_buf[2];
	write_buf[0] = reg_start_addr;
	write_buf[1] = *data_buf;
	const icm20948_dev_t *sens = sensor;
	spi_device_acquire_bus(sens->spi_dev_handle, portMAX_DELAY);
	spi_transaction_t t = {
		.length = 16,
		.tx_buffer = write_buf,
	};
	ret = spi_device_transmit(sens->spi_dev_handle, &t);  // transmit the register address and data
	spi_device_release_bus(sens->spi_dev_handle);
	return ret;
}

static esp_err_t icm20948_read_spi(icm20948_handle_t sensor,
								const uint8_t reg_start_addr,
								uint8_t *const data_buf,
								const uint8_t data_len) {
	esp_err_t ret = ESP_OK;
	const uint8_t reg = reg_start_addr | 0x80;
	const icm20948_dev_t *sens = sensor;
	spi_device_acquire_bus(sens->spi_dev_handle, portMAX_DELAY);
	spi_transaction_t t_write = {
		.length = 8,
		.tx_buffer = &reg,
		.flags = SPI_TRANS_CS_KEEP_ACTIVE,
	};
	spi_device_polling_transmit(sens->spi_dev_handle, &t_write);  // transmit the register address
	spi_transaction_t t_read = {
		.length = 8 * data_len,
		.rx_buffer = data_buf,
	};
	ret = spi_device_polling_transmit(sens->spi_dev_handle, &t_read);  // receive the data
	spi_device_release_bus(sens->spi_dev_handle);
	return ret;
}

esp_err_t icm20948_i2c_bus_init(icm20948_handle_t sensor, i2c_port_t port, const uint16_t dev_addr, gpio_num_t SCL, gpio_num_t SDA, uint32_t scl_speed) {
	icm20948_dev_t *sens = sensor;
	// 配置 I2C 主机
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = port,
		.scl_io_num = SCL,
		.sda_io_num = SDA,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	// 创建 I2C 主机
	i2c_master_bus_handle_t bus_handle;
	esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
		return ret;
	}

	// 配置 I2C 设备
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = dev_addr,  // 7 位地址模式
		.scl_speed_hz = scl_speed,
	};

	// 添加设备
	ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &sens->i2c_dev_handle);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "Failed to add I2C device: %s", esp_err_to_name(ret));
		return ret;
	}
	sens->mode = ICM20948_MODE_I2C;
	sens->icm20948_read = icm20948_read_i2c;
	sens->icm20948_write = icm20948_write_i2c;
	ESP_LOGI(sens->tag, "I2C bus and device initialized successfully");
	return ESP_OK;
}
esp_err_t icm20948_spi_bus_init(icm20948_handle_t sensor, spi_host_device_t host, gpio_num_t MISO, gpio_num_t MOSI, gpio_num_t SCLK, gpio_num_t CS, int clk_speed) {
	icm20948_dev_t *sens = sensor;
	// 配置 SPI 总线
	esp_err_t ret;
	spi_bus_config_t spi_bus_cfg = {
		.miso_io_num = MISO,
		.mosi_io_num = MOSI,
		.sclk_io_num = SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
	};

	// 创建 SPI 总线
	ret = spi_bus_initialize(host, &spi_bus_cfg, SPI_DMA_DISABLED);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
		return ret;
	}

	// 配置 SPI 设备
	spi_device_interface_config_t dev_cfg = {
		.clock_speed_hz = clk_speed,
		.mode = 0,
		.spics_io_num = CS,
		.queue_size = 1,
	};

	ret = spi_bus_add_device(host, &dev_cfg, &sens->spi_dev_handle);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "Failed to add SPI device: %s", esp_err_to_name(ret));
	}
	sens->mode = ICM20948_MODE_SPI;
	sens->icm20948_read = icm20948_read_spi;
	sens->icm20948_write = icm20948_write_spi;
	ESP_LOGI(sens->tag, "SPI bus and device initialized successfully");
	return ESP_OK;
}
icm20948_handle_t icm20948_create(icm20948_data_t *data, char *tag) {
	icm20948_dev_t *sensor = (icm20948_dev_t *)calloc(1, sizeof(icm20948_dev_t));
	if (!sensor) {
		ESP_LOGE(tag, "Memory allocation failed");
		return NULL;
	}

	// 参数初始化
	sensor->tag	= tag;
	sensor->data = data;
	sensor->bank = -1;
	sensor->timer = 0;
	sensor->i2c_dev_handle = NULL;
	sensor->spi_dev_handle = NULL;
	sensor->KalmanX = (Kalman_t){.Q_angle = 0.001f,.Q_bias = 0.003f,.R_measure = 0.03f};
	sensor->KalmanY = (Kalman_t){.Q_angle = 0.001f,.Q_bias = 0.003f,.R_measure = 0.03f};
	sensor->KalmanZ = (Kalman_t){.Q_angle = 0.001f,.Q_bias = 0.003f,.R_measure = 0.03f};

	return (icm20948_handle_t)sensor;
}

void icm20948_delete(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = sensor;
	free(sens);
}

esp_err_t icm20948_configure(icm20948_handle_t icm20948, icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs)
{
    esp_err_t ret;
	icm20948_dev_t *sens = icm20948;

    ret = icm20948_reset(icm20948);
    if (ret != ESP_OK){
        ESP_LOGE(sens->tag, "Reset failed!");
        return ret;
    }

	vTaskDelay(20 / portTICK_PERIOD_MS);

    ret = icm20948_wake_up(icm20948);
    if (ret != ESP_OK) {
        ESP_LOGE(sens->tag, "Wake up failed!");
        return ret;
    }

    ret = icm20948_set_bank(icm20948, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(sens->tag, "Set bank failed!");
        return ret;
    }

    uint8_t device_id;
    ret = icm20948_get_deviceid(icm20948, &device_id);
    if (ret != ESP_OK){
        ESP_LOGE(sens->tag, "Get device id failed!");
        return ret;
    }
    ESP_LOGI(sens->tag, "Device ID:0x%02X", device_id);
    if (device_id != ICM20948_WHO_AM_I_VAL){
        ESP_LOGE(sens->tag, "Device id mismatch!");
        return ESP_FAIL;
    }
    ESP_LOGI(sens->tag, "Device id correct!");

    ret = icm20948_set_gyro_fs(icm20948, gyro_fs);
    if (ret != ESP_OK){
        return ret;
    }

    ret = icm20948_set_acce_fs(icm20948, acce_fs);
    if (ret != ESP_OK){
        return ret;
    }

	ret = icm20948_set_bank(icm20948, 0);
	if (ret != ESP_OK) {
		return ret;
	}
    return ret;
}

esp_err_t icm20948_get_deviceid(icm20948_handle_t sensor, uint8_t *const deviceid)
{
	icm20948_dev_t *sens = sensor;
	return sens->icm20948_read(sensor, ICM20948_WHO_AM_I, deviceid, 1);
}

esp_err_t icm20948_wake_up(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = sensor;
	esp_err_t ret;
	uint8_t tmp;
	ret = sens->icm20948_read(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	if (ESP_OK != ret) {
		return ret;
	}
	tmp &= (~BIT6);
	ret = sens->icm20948_write(sensor, ICM20948_PWR_MGMT_1, &tmp);
	return ret;
}

esp_err_t icm20948_sleep(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = sensor;
	esp_err_t ret;
	uint8_t tmp;
	ret = sens->icm20948_read(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	if (ESP_OK != ret) {
		return ret;
	}
	tmp |= BIT6;
	ret = sens->icm20948_write(sensor, ICM20948_PWR_MGMT_1, &tmp);
	return ret;
}

esp_err_t icm20948_reset(icm20948_handle_t sensor)
{
	esp_err_t ret;
	uint8_t tmp;
	icm20948_dev_t *sens = sensor;

	ret = sens->icm20948_read(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	if (ret != ESP_OK)
		return ret;
	tmp |= 0x80;
	ret = sens->icm20948_write(sensor, ICM20948_PWR_MGMT_1, &tmp);
	if (ret != ESP_OK)
		return ret;

	return ret;
}

esp_err_t icm20948_set_bank(icm20948_handle_t sensor, uint8_t bank)
{
	esp_err_t ret;
	icm20948_dev_t *sens = sensor;
	if (bank > 3)
		return ESP_FAIL;
	bank = (bank << 4) & 0x30;
	ret = sens->icm20948_write(sensor, ICM20948_REG_BANK_SEL, &bank);
	if (ret != ESP_OK)
		return ret;
	sens->bank = bank;
	return ret;
}

float icm20948_get_acce_sensitivity(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = sensor;
	icm20948_acce_fs_t acce_fs = sens->acce_fs;
	switch (acce_fs){
	case ACCE_FS_2G:
		return 16384;
	case ACCE_FS_4G:
		return 8192;
	case ACCE_FS_8G:
		return 4096;
	case ACCE_FS_16G:
		return 2048;
	}
	return 16384;
}

float icm20948_get_gyro_sensitivity(icm20948_handle_t sensor){
	icm20948_dev_t *sens = sensor;
	icm20948_gyro_fs_t gyro_fs = sens->gyro_fs;
	switch (gyro_fs){
	case GYRO_FS_250DPS:
		return 131;
	case GYRO_FS_500DPS:
		return 65.5f;
	case GYRO_FS_1000DPS:
		return 32.8f;
	case GYRO_FS_2000DPS:
		return 16.4f;
	}
	return 131;
}

esp_err_t icm20948_get_acce(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	uint8_t data_rd[6];
	float acce_sensitivity = icm20948_get_acce_sensitivity(sensor);
	if (sens->bank != 0){
		icm20948_set_bank(sensor, 0);
	}
	esp_err_t ret = sens->icm20948_read(sensor, ICM20948_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

	sens->data->ax_raw = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
	sens->data->ay_raw = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
	sens->data->az_raw = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

	sens->data->ax = (float)sens->data->ax_raw / acce_sensitivity;
	sens->data->ay = (float)sens->data->ay_raw / acce_sensitivity;
	sens->data->az = (float)sens->data->az_raw / acce_sensitivity;

	return ret;
}

esp_err_t icm20948_get_gyro(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	uint8_t data_rd[6];
	float gyro_sensitivity = icm20948_get_gyro_sensitivity(sensor);
	if (sens->bank != 0){
		icm20948_set_bank(sensor, 0);
	}
	esp_err_t ret = sens->icm20948_read(sensor, ICM20948_GYRO_XOUT_H, data_rd, sizeof(data_rd));

	sens->data->gx_raw = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
	sens->data->gy_raw = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
	sens->data->gz_raw = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

	sens->data->gx = (float)sens->data->gx_raw / gyro_sensitivity;
	sens->data->gy = (float)sens->data->gy_raw / gyro_sensitivity;
	sens->data->gz = (float)sens->data->gz_raw / gyro_sensitivity;

	return ret;
}

void icm20948_get_angle(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = sensor;
	float dt = (float)(esp_timer_get_time() - sens->timer) / 1000000;
	sens->timer = esp_timer_get_time();
	float roll;
	float roll_sqrt = sqrtf(powf(sens->data->ax_raw, 2) + powf(sens->data->az_raw, 2));
	if (roll_sqrt != 0.0)
	{
		roll = atanf((float)sens->data->ay_raw / roll_sqrt) * RAD_TO_DEG;
	}
	else
	{
		roll = 0.0f;
	}
	float pitch = atan2f((float)-sens->data->ax_raw, (float)sens->data->az_raw) * RAD_TO_DEG;
	if ((pitch < -90 && sens->data->angley > 90) || (pitch > 90 && sens->data->angley < -90))
	{
		sens->KalmanY.angle = pitch;
		sens->data->angley = pitch;
	}
	else
	{
		sens->data->angley = icm20948_kalman_get_angle(&sens->KalmanY, pitch, sens->data->gy, dt);
	}
	if (fabsf(sens->data->angley) > 90)
		sens->data->gx = -sens->data->gx;
	sens->data->anglex = icm20948_kalman_get_angle(&sens->KalmanX, roll, sens->data->gx, dt);

	float yaw_inc = sens->data->gz * dt;
	if(fabsf(yaw_inc) < 1000) {
		sens->data->anglez = icm20948_kalman_get_angle(&sens->KalmanZ, sens->data->anglez + yaw_inc, sens->data->gz, dt);
	}
}

float icm20948_kalman_get_angle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
	float rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	float S = Kalman->P[0][0] + Kalman->R_measure;
	float K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	float y = newAngle - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	float P00_temp = Kalman->P[0][0];
	float P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
};

esp_err_t icm20948_get_temp(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = sensor;
	if (sens->bank != 0){
		icm20948_set_bank(sensor, 0);
	}
	uint8_t data_rd[2];
	esp_err_t ret = sens->icm20948_read(sensor, ICM20948_TEMP_XOUT_H, data_rd, sizeof(data_rd));

	int16_t temp_raw = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
	float temp = (float)temp_raw / 333.87f + 21.0f;
	// lowpass filter
	sens->data->temp = 0.9f * sens->data->temp + (1 - 0.9f) * temp;

	return ret;
}

void icm20948_get_all(icm20948_handle_t sensor)
{
	icm20948_get_acce(sensor);
	icm20948_get_gyro(sensor);
	icm20948_get_angle(sensor);
	icm20948_get_temp(sensor);
}

esp_err_t icm20948_set_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t gyro_fs)
{
	esp_err_t ret;
	uint8_t tmp;
	icm20948_dev_t *sens = sensor;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ret;

	ret = sens->icm20948_read(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);

	if (ret != ESP_OK)
		return ret;
	tmp &= 0x09;
	tmp |= (gyro_fs << 1);

	ret = sens->icm20948_write(sensor, ICM20948_GYRO_CONFIG_1, &tmp);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "Set gyro fs failed!");
		return ret;
	}
	// if set gyro fs success, record to sensor
	sens->gyro_fs = gyro_fs;
	return ret;
}

esp_err_t icm20948_set_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t acce_fs)
{
	esp_err_t ret;
	uint8_t tmp;
	icm20948_dev_t *sens = sensor;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ret;

	ret = sens->icm20948_read(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	if (ret != ESP_OK)
		return ret;
	tmp &= 0x09;
	tmp |= (acce_fs << 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	ret = sens->icm20948_write(sensor, ICM20948_ACCEL_CONFIG, &tmp);
	if (ret != ESP_OK) {
		ESP_LOGE(sens->tag, "Set acce fs failed!");
		return ret;
	}
	// if set acce fs success, record to sensor
	sens->acce_fs = acce_fs;
	return ret;
}

esp_err_t icm20948_get_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t *acce_fs)
{
	esp_err_t ret;
	uint8_t tmp;
	icm20948_dev_t *sens = sensor;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ret;

	ret = sens->icm20948_read(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	tmp &= 0x06;
	tmp >>= 1;
	*acce_fs = tmp;
	return ret;
}


esp_err_t icm20948_set_acce_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_acce)
{
	esp_err_t ret;
	uint8_t tmp;
	icm20948_dev_t *sens = sensor;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = sens->icm20948_read(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);
	if (ret != ESP_OK)
		return ESP_FAIL;

	tmp &= 0xC7;
	tmp |= dlpf_acce << 3;

	ret = sens->icm20948_write(sensor, ICM20948_ACCEL_CONFIG, &tmp);
	if (ret != ESP_OK)
		return ESP_FAIL;

	return ret;
}

esp_err_t icm20948_set_gyro_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_gyro)
{
	esp_err_t ret;
	uint8_t tmp;
	icm20948_dev_t *sens = sensor;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = sens->icm20948_read(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);
	if (ret != ESP_OK)
		return ESP_FAIL;

	tmp &= 0xC7;
	tmp |= dlpf_gyro << 3;

	ret = sens->icm20948_write(sensor, ICM20948_GYRO_CONFIG_1, &tmp);
	if (ret != ESP_OK)
		return ESP_FAIL;

	return ret;
}

esp_err_t icm20948_enable_dlpf(icm20948_handle_t sensor, bool enable)
{
	esp_err_t ret;
	uint8_t tmp;
	icm20948_dev_t *sens = sensor;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = sens->icm20948_read(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);
	if (ret != ESP_OK)
		return ESP_FAIL;

	if (enable)
		tmp |= 0x01;
	else
		tmp &= 0xFE;

	ret = sens->icm20948_write(sensor, ICM20948_ACCEL_CONFIG, &tmp);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = sens->icm20948_read(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);
	if (ret != ESP_OK)
		return ESP_FAIL;

	if (enable)
		tmp |= 0x01;
	else
		tmp &= 0xFE;

	ret = sens->icm20948_write(sensor, ICM20948_GYRO_CONFIG_1, &tmp);
	if (ret != ESP_OK)
		return ESP_FAIL;

	return ret;
}
