#ifndef __ICM20948_H__
#define __ICM20948_H__

#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define ICM20948_I2C_ADDRESS   0x69
#define ICM20948_I2C_ADDRESS_1 0x68
#define ICM20948_WHO_AM_I_VAL  0xEA

typedef enum {
	ACCE_FS_2G = 0,  /*!< Accelerometer full scale range is +/- 2g */
	ACCE_FS_4G = 1,  /*!< Accelerometer full scale range is +/- 4g */
	ACCE_FS_8G = 2,  /*!< Accelerometer full scale range is +/- 8g */
	ACCE_FS_16G = 3, /*!< Accelerometer full scale range is +/- 16g */
} icm20948_acce_fs_t;

typedef enum {
	GYRO_FS_250DPS = 0,  /*!< Gyroscope full scale range is +/- 250 degree per sencond */
	GYRO_FS_500DPS = 1,  /*!< Gyroscope full scale range is +/- 500 degree per sencond */
	GYRO_FS_1000DPS = 2, /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
	GYRO_FS_2000DPS = 3, /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} icm20948_gyro_fs_t;

typedef enum {
	INTERRUPT_PIN_ACTIVE_HIGH = 0, /*!< The icm20948 sets its INT pin HIGH on interrupt */
	INTERRUPT_PIN_ACTIVE_LOW = 1   /*!< The icm20948 sets its INT pin LOW on interrupt */
} icm20948_int_pin_active_level_t;

typedef enum {
	INTERRUPT_PIN_PUSH_PULL = 0, /*!< The icm20948 configures its INT pin as push-pull */
	INTERRUPT_PIN_OPEN_DRAIN = 1 /*!< The icm20948 configures its INT pin as open drain*/
} icm20948_int_pin_mode_t;

typedef enum {
	INTERRUPT_LATCH_50US = 0,         /*!< The icm20948 produces a 50 microsecond pulse on interrupt */
	INTERRUPT_LATCH_UNTIL_CLEARED = 1 /*!< The icm20948 latches its INT pin to its active level, until
	                                     interrupt is cleared */
} icm20948_int_latch_t;

typedef enum {
	INTERRUPT_CLEAR_ON_ANY_READ = 0,   /*!< INT_STATUS register bits are cleared on any register read */
	INTERRUPT_CLEAR_ON_STATUS_READ = 1 /*!< INT_STATUS register bits are cleared
	                                      only by reading INT_STATUS value*/
} icm20948_int_clear_t;

typedef enum {
	ICM20948_DLPF_0,
	ICM20948_DLPF_1,
	ICM20948_DLPF_2,
	ICM20948_DLPF_3,
	ICM20948_DLPF_4,
	ICM20948_DLPF_5,
	ICM20948_DLPF_6,
	ICM20948_DLPF_7,
	ICM20948_DLPF_OFF
} icm20948_dlpf_t;

typedef enum {
	ICM20948_MODE_I2C,
	ICM20948_MODE_SPI
} icm20948_mode_t;


typedef struct {
	gpio_num_t interrupt_pin;                      /*!< GPIO connected to icm20948 INT pin       */
	icm20948_int_pin_active_level_t active_level;  /*!< Active level of icm20948 INT pin         */
	icm20948_int_pin_mode_t pin_mode;              /*!< Push-pull or open drain mode for INT pin*/
	icm20948_int_latch_t interrupt_latch;          /*!< The interrupt pulse behavior of INT pin */
	icm20948_int_clear_t interrupt_clear_behavior; /*!< Interrupt status clear behavior */
} icm20948_int_config_t;

extern const uint8_t icm20948_DATA_RDY_INT_BIT;      /*!< DATA READY interrupt bit */
extern const uint8_t icm20948_I2C_MASTER_INT_BIT;    /*!< I2C MASTER interrupt bit               */
extern const uint8_t icm20948_FIFO_OVERFLOW_INT_BIT; /*!< FIFO Overflow interrupt bit */
extern const uint8_t icm20948_MOT_DETECT_INT_BIT;    /*!< MOTION DETECTION interrupt bit         */
extern const uint8_t icm20948_ALL_INTERRUPTS;        /*!< All interrupts supported by icm20948    */

typedef struct {
	int16_t ax_raw;
	int16_t ay_raw;
	int16_t az_raw;
	int16_t gx_raw;
	int16_t gy_raw;
	int16_t gz_raw;
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float anglex;
	float angley;
	float anglez;
	float temp;

} icm20948_data_t;

typedef void *icm20948_handle_t;

typedef esp_err_t (*icm20948_write_function_t)(icm20948_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf);
typedef esp_err_t (*icm20948_read_function_t)(icm20948_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len);

typedef gpio_isr_t icm20948_isr_t;

// Kalman structure
typedef struct
{
	float Q_angle;
	float Q_bias;
	float R_measure;
	float angle;
	float bias;
	float P[2][2];
} Kalman_t;

typedef struct {
	char *tag;
	gpio_num_t int_pin;
	uint16_t dev_addr;
	int bank;
	int64_t timer;
	Kalman_t KalmanX;
	Kalman_t KalmanY;
	Kalman_t KalmanZ;
	icm20948_acce_fs_t acce_fs;
	icm20948_gyro_fs_t gyro_fs;
	icm20948_data_t *data;
	icm20948_mode_t mode;
	i2c_master_dev_handle_t i2c_dev_handle;
	spi_device_handle_t spi_dev_handle;
	icm20948_read_function_t icm20948_read;
	icm20948_write_function_t icm20948_write;

} icm20948_dev_t;


/**
 * @brief Initialize the I2C bus and device
*
 * @param port I2C port number
 * @param dev_addr I2C device address of sensor
 * @param sensor object handle of icm20948
 * @param SCL I2C SCL gpio number
 * @param SDA I2C SDA gpio number
 * @param scl_speed I2C SCL speed in Hz
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_i2c_bus_init(icm20948_handle_t sensor, i2c_port_t port, const uint16_t dev_addr, gpio_num_t SCL, gpio_num_t SDA, uint32_t scl_speed);

/**
 * @brief Initialize the SPI bus and device
 *
 * @param sensor object handle of icm20948
 * @param host SPI host
 * @param MISO SPI MISO gpio number
 * @param MOSI SPI MOSI gpio number
 * @param SCLK SPI SCLK gpio number
 * @param CS SPI CS gpio number
 * @param clk_speed SPI clock speed
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_spi_bus_init(icm20948_handle_t sensor, spi_host_device_t host, gpio_num_t MISO, gpio_num_t MOSI, gpio_num_t SCLK, gpio_num_t CS, int clk_speed);

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param data sensor data structure
 * @param tag sensor tag for logging
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
icm20948_handle_t icm20948_create(icm20948_data_t* data, char *tag);

/**
 * @brief Configure the sensor with the given full scale range
 *
 * @param icm20948 object handle of icm20948
 * @param acce_fs accelerometer full scale range
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_configure(icm20948_handle_t icm20948, icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of icm20948
 */
void icm20948_delete(icm20948_handle_t sensor);

/**
 * @brief Get device identification of icm20948
 *
 * @param sensor object handle of icm20948
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_get_deviceid(icm20948_handle_t sensor, uint8_t *const deviceid);

/**
 * @brief Wake up icm20948
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_wake_up(icm20948_handle_t sensor);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_sleep(icm20948_handle_t sensor);

/**
 * @brief Set gyroscope full scale range
 *
 * @param sensor object handle of icm20948
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_set_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t gyro_fs);

/**
 * @brief Get gyroscope full scale range
 *
 * @param sensor object handle of icm20948
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_get_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t *gyro_fs);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of icm20948
 *
 * @return gyroscope sensitivity
 */
float icm20948_get_gyro_sensitivity(icm20948_handle_t sensor);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_get_gyro(icm20948_handle_t sensor);

/**
 * @brief Set accelerometer full scale range
 *
 * @param sensor object handle of icm20948
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_set_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t acce_fs);

/**
 * @brief Get accelerometer full scale range
 *
 * @param sensor object handle of icm20948
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_get_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t *acce_fs);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
float icm20948_get_acce_sensitivity(icm20948_handle_t sensor);
/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_get_acce(icm20948_handle_t sensor);

/**
 * @brief get euler angle
 *
 * @param sensor object handle of icm20948
 *
 */
void icm20948_get_angle(icm20948_handle_t sensor);

/**
 * @brief filter the angle using kalman filter
 *
 * @param Kalman kalman filter object
 * @param newAngle euler angle
 * @param newRate real gyroscope value
 * @param dt time interval
 *
 */
float icm20948_kalman_get_angle(Kalman_t *Kalman, float newAngle, float newRate, float dt);

/**
 * @brief Read the temperature value
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_get_temp(icm20948_handle_t sensor);

/**
 * @brief Read all sensor values
 *
 * @param sensor object handle of icm20948
 *
 */
void icm20948_get_all(icm20948_handle_t sensor);

/**
 * @brief Reset the internal registers and restores the default settings
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_reset(icm20948_handle_t sensor);

/**
 * @brief Waking the chip from sleep mode.
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_wakeup(icm20948_handle_t sensor);

/**
 * @brief Select USER BANK.
 * 0: Select USER BANK 0.
 * 1: Select USER BANK 1.
 * 2: Select USER BANK 2.
 * 3: Select USER BANK 3.
 *
 * @param sensor object handle of icm20948
 * @param bank   user bank number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_set_bank(icm20948_handle_t sensor, uint8_t bank);

/**
 * @brief Enable low pass filter for gyroscope and accelerometer.
 * true:  enable low pass filter.
 * false: bypass low pass filter.
 *
 * @param sensor object handle of icm20948
 * @param enable boolean variable whether to turn on or bypass the filter
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_enable_dlpf(icm20948_handle_t sensor, bool enable);

/**
 * @brief Configure low pass filter for accelerometer
 *
 * @param sensor      object handle of icm20948
 * @param dlpf_acce   dlpf configuration for accelerometer
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_set_acce_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_acce);

/**
 * @brief Configure low pass filter for gyroscope
 *
 * @param sensor      object handle of icm20948
 * @param dlpf_gyro   dlpf configuration for gyroscope
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm20948_set_gyro_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_gyro);

#endif // !__ICM20948_H__
