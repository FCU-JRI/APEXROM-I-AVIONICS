#include "Sensors.hpp"
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include "esp_timer.h"

#ifdef ESP_PLATFORM
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "minmea.h"
#endif

// GPS UART Config
#define GPS_UART_NUM UART_NUM_2
#define GPS_TX_PIN GPIO_NUM_17
#define GPS_RX_PIN GPIO_NUM_16
#define GPS_PPS_PIN GPIO_NUM_34
#define BMP388_I2C_ADDRESS 0x76

#define MINMEA_MAX_LENGTH 82

SensorManager::SensorManager(InterCoreComm* comm, KalmanFilter* kalman) : 
    _icmActive(false), _bmpActive(false), _gpsActive(false),
    _comm(comm), _kalman(kalman),
    _icmTaskHandle(NULL), _bmpTaskHandle(NULL), _gpsTaskHandle(NULL) {
    
    permitIcm = xSemaphoreCreateBinary();
    permitBmp = xSemaphoreCreateBinary();
    permitGps = xSemaphoreCreateBinary();
    _i2cMutex = xSemaphoreCreateMutex();
}

#ifdef ESP_PLATFORM
esp_err_t SensorManager::i2cWriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t val) {
    uint8_t write_buf[2] = {regAddr, val};
    return i2c_master_write_to_device(I2C_NUM_0, devAddr, write_buf, 2, pdMS_TO_TICKS(50));
}

esp_err_t SensorManager::i2cReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t* readBuf, size_t len) {
    return i2c_master_write_read_device(I2C_NUM_0, devAddr, &regAddr, 1, readBuf, len, pdMS_TO_TICKS(50));
}
#endif

bool SensorManager::initIcm20948() {
#ifdef ESP_PLATFORM
    // Select User Bank 0
    if (i2cWriteReg(0x68, 0x7F, 0x00) != ESP_OK) return false;
    // Reset ICM-20948
    if (i2cWriteReg(0x68, 0x06, 0x80) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(50));
    // Wake up and select auto clock source
    if (i2cWriteReg(0x68, 0x06, 0x01) != ESP_OK) return false;
    // Enable Accel and Gyro axes
    if (i2cWriteReg(0x68, 0x07, 0x00) != ESP_OK) return false;

    // Select User Bank 2
    if (i2cWriteReg(0x68, 0x7F, 0x20) != ESP_OK) return false;
    // Configure Accel: ±16g range, filter enabled
    if (i2cWriteReg(0x68, 0x14, 0x07) != ESP_OK) return false;
    // Configure Gyro: ±2000dps range, filter enabled
    if (i2cWriteReg(0x68, 0x01, 0x07) != ESP_OK) return false;

    // Back to Bank 0
    if (i2cWriteReg(0x68, 0x7F, 0x00) != ESP_OK) return false;

    _icmInitialized = true;
    ESP_LOGI("Sensors", "ICM-20948 Initialized Successfully!");
    return true;
#else
    _icmInitialized = true;
    return true;
#endif
}

bool SensorManager::initBmp388() {
#ifdef ESP_PLATFORM
    uint8_t chip_id = 0;
    esp_err_t err = ESP_FAIL;
    for (int i=0; i<3; i++) {
        err = i2cReadRegs(BMP388_I2C_ADDRESS, 0x00, &chip_id, 1);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (err != ESP_OK || chip_id != 0x50) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Chip ID 0x%02X (err: %d)", chip_id, err);
        return false;
    }

    // Reset BMP388
    if (i2cWriteReg(BMP388_I2C_ADDRESS, 0x7E, 0xB6) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Reset command error");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t status;
    i2cReadRegs(BMP388_I2C_ADDRESS, 0x03, &status, 1);

    // Read 21 bytes calibration data starting at register 0x31
    uint8_t calib_buf[21] = {0};
    if (i2cReadRegs(BMP388_I2C_ADDRESS, 0x31, calib_buf, 21) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Read calibration error");
        return false;
    }

    #define CONCAT_BYTES(msb, lsb) ((uint16_t)(((uint16_t)(msb) << 8) | (uint16_t)(lsb)))
    
    _bmpCalib.par_t1 = CONCAT_BYTES(calib_buf[1], calib_buf[0]);
    _bmpCalib.par_t2 = CONCAT_BYTES(calib_buf[3], calib_buf[2]);
    _bmpCalib.par_t3 = (int8_t)calib_buf[4];
    _bmpCalib.par_p1 = (int16_t)CONCAT_BYTES(calib_buf[6], calib_buf[5]);
    _bmpCalib.par_p2 = (int16_t)CONCAT_BYTES(calib_buf[8], calib_buf[7]);
    _bmpCalib.par_p3 = (int8_t)calib_buf[9];
    _bmpCalib.par_p4 = (int8_t)calib_buf[10];
    _bmpCalib.par_p5 = CONCAT_BYTES(calib_buf[12], calib_buf[11]);
    _bmpCalib.par_p6 = CONCAT_BYTES(calib_buf[14], calib_buf[13]);
    _bmpCalib.par_p7 = (int8_t)calib_buf[15];
    _bmpCalib.par_p8 = (int8_t)calib_buf[16];
    _bmpCalib.par_p9 = (int16_t)CONCAT_BYTES(calib_buf[18], calib_buf[17]);
    _bmpCalib.par_p10 = (int8_t)calib_buf[19];
    _bmpCalib.par_p11 = (int8_t)calib_buf[20];

    #undef CONCAT_BYTES

    // Configure ODR: 25Hz (0x03) (內部硬體取樣頻率)
    if (i2cWriteReg(BMP388_I2C_ADDRESS, 0x1D, 0x03) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: ODR config error");
        return false;
    }

    // Configure OSR: OSR_p = 16 (0x04), OSR_t = 2 (0x01) -> 0x0C (Ultra high resolution)
    if (i2cWriteReg(BMP388_I2C_ADDRESS, 0x1C, 0x0C) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: OSR config error");
        return false;
    }

    // Configure Filter: IIR filter coeff 15 (100 << 1) -> 0x08
    if (i2cWriteReg(BMP388_I2C_ADDRESS, 0x1F, 0x08) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Filter config error");
        return false;
    }

    // Power Control: Enable Temp, Press, Normal Mode -> 0x33
    if (i2cWriteReg(BMP388_I2C_ADDRESS, 0x1B, 0x33) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Power control error");
        return false;
    }

    _bmpInitialized = true;
    ESP_LOGI("Sensors", "BMP388 Initialized Successfully!");
    return true;
#else
    _bmpInitialized = true;
    return true;
#endif
}

int64_t SensorManager::compensateTemp(uint32_t uncomp_temp) {
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t comp_temp;

    partial_data1 = ((int64_t)uncomp_temp - (256 * _bmpCalib.par_t1));
    partial_data2 = _bmpCalib.par_t2 * partial_data1;
    partial_data3 = (partial_data1 * partial_data1);
    partial_data4 = (int64_t)partial_data3 * _bmpCalib.par_t3;
    partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
    partial_data6 = partial_data5 / 4294967296;

    _bmpCalib.t_lin = partial_data6;
    comp_temp = (int64_t)((partial_data6 * 25) / 16384);

    return comp_temp; 
}

uint64_t SensorManager::compensatePress(uint32_t uncomp_press) {
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    partial_data1 = _bmpCalib.t_lin * _bmpCalib.t_lin;
    partial_data2 = partial_data1 / 64;
    partial_data3 = (partial_data2 * _bmpCalib.t_lin) / 256;
    partial_data4 = (_bmpCalib.par_p8 * partial_data3) / 32;
    partial_data5 = (_bmpCalib.par_p7 * partial_data1) * 16;
    partial_data6 = (_bmpCalib.par_p6 * _bmpCalib.t_lin) * 4194304;
    offset = (_bmpCalib.par_p5 * 140737488355328LL) + partial_data4 + partial_data5 + partial_data6;
    
    partial_data2 = (_bmpCalib.par_p4 * partial_data3) / 32;
    partial_data4 = (_bmpCalib.par_p3 * partial_data1) * 4;
    partial_data5 = (_bmpCalib.par_p2 - 16384) * _bmpCalib.t_lin * 2097152;
    sensitivity = ((_bmpCalib.par_p1 - 16384) * 70368744177664LL) + partial_data2 + partial_data4 + partial_data5;
    
    partial_data1 = (sensitivity / 16777216) * uncomp_press;
    partial_data2 = _bmpCalib.par_p10 * _bmpCalib.t_lin;
    partial_data3 = partial_data2 + (65536 * _bmpCalib.par_p9);
    partial_data4 = (partial_data3 * uncomp_press) / 8192;

    partial_data5 = (uncomp_press * (partial_data4 / 10)) / 512;
    partial_data5 = partial_data5 * 10;
    partial_data6 = (int64_t)((uint64_t)uncomp_press * (uint64_t)uncomp_press);
    partial_data2 = (_bmpCalib.par_p11 * partial_data6) / 65536;
    partial_data3 = (partial_data2 * uncomp_press) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776LL);

    return comp_press; 
}

void SensorManager::begin() {
    static SensorManager::TaskParams icmParams = {this, SENSOR_ICM20948};
    static SensorManager::TaskParams bmpParams = {this, SENSOR_BMP388};
    static SensorManager::TaskParams gpsParams = {this, SENSOR_MAX_M10S};

#ifdef ESP_PLATFORM
    // Initialize I2C Master
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100kHz for stability
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    i2c_set_timeout(I2C_NUM_0, 0xFFFFF); // Increase timeout for slow sensors
    
    // Give sensors time to power up
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize GPS UART
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk= UART_SCLK_APB,
    };
    uart_driver_install(GPS_UART_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Initialize GPS PPS pin
    gpio_config_t pps_conf = {
        .pin_bit_mask = (1ULL << GPS_PPS_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE, // Optional: configure interrupt if needed
    };
    gpio_config(&pps_conf);

    ESP_LOGI("Sensors", "Scanning I2C Bus...");
    for (uint8_t i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        if (res == ESP_OK) {
            ESP_LOGI("Sensors", "Found I2C device at 0x%02X", i);
        }
    }
#endif

    // Attempt to initialize physical sensors
    initIcm20948();
    initBmp388();

    xTaskCreatePinnedToCore(taskWrapper, "ICM_Task", 4096, &icmParams, 5, &_icmTaskHandle, 0);
    xTaskCreatePinnedToCore(taskWrapper, "BMP_Task", 4096, &bmpParams, 5, &_bmpTaskHandle, 0);
    xTaskCreatePinnedToCore(taskWrapper, "GPS_Task", 4096, &gpsParams, 5, &_gpsTaskHandle, 0);
}

void SensorManager::enableIcm() { _icmActive = true; xSemaphoreGive(permitIcm); }
void SensorManager::enableBmp() { _bmpActive = true; xSemaphoreGive(permitBmp); }
void SensorManager::enableGps() { _gpsActive = true; xSemaphoreGive(permitGps); }

void SensorManager::disableIcm() { _icmActive = false; }
void SensorManager::disableBmp() { _bmpActive = false; }
void SensorManager::disableGps() { _gpsActive = false; }

void SensorManager::taskWrapper(void* pvParameters) {
    SensorManager::TaskParams* params = (SensorManager::TaskParams*)pvParameters;
    SensorManager* instance = params->instance;
    switch (params->type) {
        case SENSOR_ICM20948: instance->icm20948Task(); break;
        case SENSOR_BMP388:   instance->bmp388Task();   break;
        case SENSOR_MAX_M10S: instance->maxM10sTask(); break;
    }
}



void SensorManager::icm20948Task() {
    while (1) {
        if (!_icmActive) xSemaphoreTake(permitIcm, portMAX_DELAY);
        if (xSemaphoreTake(_i2cMutex, portMAX_DELAY) == pdTRUE) {

            if (_icmInitialized) {
#ifdef ESP_PLATFORM
                uint8_t raw_data[12] = {0};
                if (i2cReadRegs(0x68, 0x2D, raw_data, 12) == ESP_OK) {
                    int16_t accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
                    int16_t accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
                    int16_t accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
                    int16_t gyro_x  = (int16_t)((raw_data[6] << 8) | raw_data[7]);
                    int16_t gyro_y  = (int16_t)((raw_data[8] << 8) | raw_data[9]);
                    int16_t gyro_z  = (int16_t)((raw_data[10] << 8) | raw_data[11]);

                    // Scale: accel (±16g => 2048 LSB/g)
                    float ax = ((float)accel_x / 2048.0f) * 9.80665f;
                    float ay = ((float)accel_y / 2048.0f) * 9.80665f;
                    float az = ((float)accel_z / 2048.0f) * 9.80665f;

                    // Scale: gyro (±2000dps => 16.4 LSB/dps, convert to rad/s)
                    float gx = (((float)gyro_x / 16.4f) * M_PI) / 180.0f;
                    float gy = (((float)gyro_y / 16.4f) * M_PI) / 180.0f;
                    float gz = (((float)gyro_z / 16.4f) * M_PI) / 180.0f;

                    // Magnetometer fallback for Kalman
                    float mx = 1.0f, my = 0.0f, mz = 0.0f;

                    if (_kalman) _kalman->updateImu(ax, ay, az, gx, gy, gz, mx, my, mz);
                }
#endif
            }



            xSemaphoreGive(_i2cMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void SensorManager::bmp388Task() {
    while (1) {
        if (!_bmpActive) xSemaphoreTake(permitBmp, portMAX_DELAY);
        // 等待 40ms 讓硬體自行在 Normal Mode 下測量 (25Hz 取樣率 = 40ms 週期)
        vTaskDelay(pdMS_TO_TICKS(40));

        if (xSemaphoreTake(_i2cMutex, portMAX_DELAY) == pdTRUE) {

            if (_bmpInitialized) {
#ifdef ESP_PLATFORM
                uint8_t raw_data[6] = {0};
                // 讀取測量完成的資料
                if (i2cReadRegs(BMP388_I2C_ADDRESS, 0x04, raw_data, 6) == ESP_OK) {
                    uint32_t raw_press = (uint32_t)raw_data[0] | ((uint32_t)raw_data[1] << 8) | ((uint32_t)raw_data[2] << 16);
                    uint32_t raw_temp  = (uint32_t)raw_data[3] | ((uint32_t)raw_data[4] << 8) | ((uint32_t)raw_data[5] << 16);

                    int64_t temp_val = compensateTemp(raw_temp);
                    uint64_t press_val = compensatePress(raw_press);

                    float t_lin = temp_val / 100.0f;
                    float pressure = press_val / 100.0f; // Pa

                    static int bmp_log_cnt = 0;
                    if (bmp_log_cnt++ % 10 == 0) {
                        ESP_LOGI("Sensors", "BMP388 Read OK - Temp: %.2f C, Press: %.2f hPa", t_lin, pressure / 100.0f);
                    }

                    if (_kalman) _kalman->updateBmp(pressure, t_lin);
                } else {
                    static int bmp_err_cnt = 0;
                    if (bmp_err_cnt++ % 10 == 0) {
                        ESP_LOGE("Sensors", "BMP388 Read Failed!");
                    }
                }
#endif
            }



            xSemaphoreGive(_i2cMutex);
        }
    }
}

void SensorManager::maxM10sTask() {
#ifdef ESP_PLATFORM
    uint8_t data[128];
    char line[MINMEA_MAX_LENGTH];
    int line_idx = 0;
#endif

    while (1) {
        if (!_gpsActive) xSemaphoreTake(permitGps, portMAX_DELAY);


#ifdef ESP_PLATFORM
        int length = uart_read_bytes(GPS_UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (length > 0) {
            for (int i = 0; i < length; i++) {
                char c = (char)data[i];
                if (c == '\r' || c == '\n') {
                    if (line_idx > 0) {
                        line[line_idx] = '\0';
                        switch (minmea_sentence_id(line, false)) {
                            case MINMEA_SENTENCE_GGA: {
                                struct minmea_sentence_gga frame;
                                if (minmea_parse_gga(&frame, line)) {
                                    float alt = minmea_tofloat(&frame.altitude);
                                    double lat = minmea_tocoord(&frame.latitude);
                                    double lon = minmea_tocoord(&frame.longitude);
                                    if (frame.fix_quality > 0) {
                                        if (_kalman) _kalman->updateGps(lat, lon, alt);
                                    }
                                }
                                break;
                            }
                            default:
                                break;
                        }
                        line_idx = 0;
                    }
                } else {
                    if (line_idx < MINMEA_MAX_LENGTH - 1) {
                        line[line_idx++] = c;
                    } else {
                        line_idx = 0; // line too long, drop
                    }
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
#else
        // Simulation fallback for native
        if (_kalman) _kalman->updateGps(25.033, 121.565, 0.0);
        vTaskDelay(pdMS_TO_TICKS(200));
#endif
    }
}
