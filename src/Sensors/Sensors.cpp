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
        err = i2cReadRegs(0x76, 0x00, &chip_id, 1);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (err != ESP_OK || chip_id != 0x50) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Chip ID 0x%02X (err: %d)", chip_id, err);
        return false;
    }

    // Reset BMP388
    if (i2cWriteReg(0x76, 0x7E, 0xB6) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Reset command error");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // Read 21 bytes calibration data starting at register 0x31
    uint8_t calib_buf[21] = {0};
    if (i2cReadRegs(0x76, 0x31, calib_buf, 21) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Read calibration error");
        return false;
    }

    #define CONCAT_BYTES(msb, lsb) ((uint16_t)(((uint16_t)(msb) << 8) | (uint16_t)(lsb)))

    // Temperature coefficients
    uint16_t raw_t1 = CONCAT_BYTES(calib_buf[1], calib_buf[0]);
    uint16_t raw_t2 = CONCAT_BYTES(calib_buf[3], calib_buf[2]);
    int8_t raw_t3 = (int8_t)calib_buf[4];

    _bmpCalib.t1 = (double)raw_t1 / 0.00390625;      // raw_t1 / 2^-8
    _bmpCalib.t2 = (double)raw_t2 / 1073741824.0;    // raw_t2 / 2^30
    _bmpCalib.t3 = (double)raw_t3 / 281474976710656.0; // raw_t3 / 2^48

    // Pressure coefficients
    int16_t raw_p1 = (int16_t)CONCAT_BYTES(calib_buf[6], calib_buf[5]);
    int16_t raw_p2 = (int16_t)CONCAT_BYTES(calib_buf[8], calib_buf[7]);
    int8_t raw_p3 = (int8_t)calib_buf[9];
    int8_t raw_p4 = (int8_t)calib_buf[10];
    uint16_t raw_p5 = CONCAT_BYTES(calib_buf[12], calib_buf[11]);
    uint16_t raw_p6 = CONCAT_BYTES(calib_buf[14], calib_buf[13]);
    int8_t raw_p7 = (int8_t)calib_buf[15];
    int8_t raw_p8 = (int8_t)calib_buf[16];
    int16_t raw_p9 = (int16_t)CONCAT_BYTES(calib_buf[18], calib_buf[17]);
    int8_t raw_p10 = (int8_t)calib_buf[19];
    int8_t raw_p11 = (int8_t)calib_buf[20];

    #undef CONCAT_BYTES

    _bmpCalib.p1 = ((double)raw_p1 - 16384.0) / 1048576.0;
    _bmpCalib.p2 = ((double)raw_p2 - 16384.0) / 536870912.0;
    _bmpCalib.p3 = (double)raw_p3 / 4294967296.0;
    _bmpCalib.p4 = (double)raw_p4 / 137438953472.0;
    _bmpCalib.p5 = (double)raw_p5 / 0.125;
    _bmpCalib.p6 = (double)raw_p6 / 64.0;
    _bmpCalib.p7 = (double)raw_p7 / 256.0;
    _bmpCalib.p8 = (double)raw_p8 / 32768.0;
    _bmpCalib.p9 = (double)raw_p9 / 281474976710656.0;
    _bmpCalib.p10 = (double)raw_p10 / 281474976710656.0;
    _bmpCalib.p11 = (double)raw_p11 / 36893488147419103232.0;

    // Configure ODR: 50Hz (0x02)
    if (i2cWriteReg(0x76, 0x1D, 0x02) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: ODR config error");
        return false;
    }

    // Configure OSR: Temp x8 (011), Press x16 (100) -> 0x1C
    if (i2cWriteReg(0x76, 0x1C, 0x1C) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: OSR config error");
        return false;
    }

    // Configure Filter: coef 3 (010 << 1) -> 0x04
    if (i2cWriteReg(0x76, 0x1F, 0x04) != ESP_OK) {
        ESP_LOGE("Sensors", "BMP388 Init Failed: Filter config error");
        return false;
    }

    // Power Control: Enable Temp, Press, Normal Mode
    if (i2cWriteReg(0x76, 0x1B, 0x33) != ESP_OK) {
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

double SensorManager::compensateTemp(double uncomp_temp) {
    double partial_data1 = uncomp_temp - _bmpCalib.t1;
    double partial_data2 = partial_data1 * _bmpCalib.t2;
    return partial_data2 + (partial_data1 * partial_data1) * _bmpCalib.t3;
}

double SensorManager::compensatePress(double uncomp_press, double t_lin) {
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    partial_data1 = _bmpCalib.p6 * t_lin;
    partial_data2 = _bmpCalib.p7 * (t_lin * t_lin);
    partial_data3 = _bmpCalib.p8 * (t_lin * t_lin * t_lin);
    partial_out1 = _bmpCalib.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = _bmpCalib.p2 * t_lin;
    partial_data2 = _bmpCalib.p3 * (t_lin * t_lin);
    partial_data3 = _bmpCalib.p4 * (t_lin * t_lin * t_lin);
    partial_out2 = uncomp_press * (_bmpCalib.p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = uncomp_press * uncomp_press;
    partial_data2 = _bmpCalib.p9 + _bmpCalib.p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (uncomp_press * uncomp_press * uncomp_press) * _bmpCalib.p11;

    return partial_out1 + partial_out2 + partial_data4;
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

// -----------------------------------------------------------------
// 物理模擬器：強化推力，確保高度 > 300m
// -----------------------------------------------------------------
static portMUX_TYPE simMux = portMUX_INITIALIZER_UNLOCKED;
static float sim_az = 9.8f;
static float sim_alt = 0.0f;
static float sim_vz = 0.0f;
static uint64_t last_sim_us = 0;

void stepPhysics() {
    portENTER_CRITICAL(&simMux);
    uint64_t now = esp_timer_get_time();
    if (last_sim_us == 0) { last_sim_us = now; portEXIT_CRITICAL(&simMux); return; }
    float dt = (now - last_sim_us) / 1000000.0f;
    last_sim_us = now;

    float t = now / 1000000.0f;
    
    if (t < 10.0f) {
        sim_az = 9.8f;   
    } else if (t < 12.0f) {
        sim_az = 60.0f;  // 強力啟動 (6G)
    } else if (t < 20.0f) {
        sim_az = 40.0f;  // 持續推力 (4G)
    } else if (t < 45.0f) {
        sim_az = 0.0f;   // 燃盡，自由拋物線
    } else {
        sim_az = 9.8f;   // 下降段
        if (sim_vz < -15.0f) sim_vz = -15.0f; 
    }

    sim_vz += (sim_az - 9.8f) * dt;
    sim_alt += sim_vz * dt;
    if (sim_alt < 0) { sim_alt = 0; sim_vz = 0; }
    portEXIT_CRITICAL(&simMux);
}

float getNoise() { return ((rand() % 100) - 50) / 2000.0f; } 
// -----------------------------------------------------------------

void SensorManager::icm20948Task() {
    while (1) {
        if (!_icmActive) xSemaphoreTake(permitIcm, portMAX_DELAY);
        if (xSemaphoreTake(_i2cMutex, portMAX_DELAY) == pdTRUE) {
            stepPhysics();
            bool read_success = false;

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
                    float mx = 1.0f + getNoise(), my = getNoise(), mz = getNoise();

                    if (_kalman) _kalman->updateImu(ax, ay, az, gx, gy, gz, mx, my, mz);
                    read_success = true;
                }
#endif
            }

            if (!read_success) {
                // Fallback simulation
                float ax = getNoise(), ay = getNoise(), az = sim_az + getNoise();
                float gx = getNoise(), gy = getNoise(), gz = getNoise();
                float mx = 1.0f + getNoise(), my = getNoise(), mz = getNoise();
                if (_kalman) _kalman->updateImu(ax, ay, az, gx, gy, gz, mx, my, mz);
            }

            xSemaphoreGive(_i2cMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void SensorManager::bmp388Task() {
    while (1) {
        if (!_bmpActive) xSemaphoreTake(permitBmp, portMAX_DELAY);
        if (xSemaphoreTake(_i2cMutex, portMAX_DELAY) == pdTRUE) {
            stepPhysics();
            bool read_success = false;

            if (_bmpInitialized) {
#ifdef ESP_PLATFORM
                uint8_t raw_data[6] = {0};
                if (i2cReadRegs(0x76, 0x04, raw_data, 6) == ESP_OK) {
                    uint32_t raw_press = (uint32_t)raw_data[0] | ((uint32_t)raw_data[1] << 8) | ((uint32_t)raw_data[2] << 16);
                    uint32_t raw_temp  = (uint32_t)raw_data[3] | ((uint32_t)raw_data[4] << 8) | ((uint32_t)raw_data[5] << 16);

                    double t_lin = compensateTemp((double)raw_temp);
                    double pressure = compensatePress((double)raw_press, t_lin);

                    static int bmp_log_cnt = 0;
                    if (bmp_log_cnt++ % 10 == 0) {
                        ESP_LOGI("Sensors", "BMP388 Read OK - Temp: %.2f C, Press: %.2f Pa", t_lin, pressure);
                    }

                    if (_kalman) _kalman->updateBmp((float)pressure, (float)t_lin);
                    read_success = true;
                } else {
                    static int bmp_err_cnt = 0;
                    if (bmp_err_cnt++ % 10 == 0) {
                        ESP_LOGE("Sensors", "BMP388 Read Failed!");
                    }
                }
#endif
            }

            if (!read_success) {
                // Fallback simulation
                float pressure = 101325.0f * powf(1.0f - sim_alt / 44330.0f, 5.255f);
                if (_kalman) _kalman->updateBmp(pressure, 25.0f);
            }

            xSemaphoreGive(_i2cMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
        if (xSemaphoreTake(_i2cMutex, portMAX_DELAY) == pdTRUE) {
            stepPhysics();
            xSemaphoreGive(_i2cMutex);
        }

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
        if (_kalman) _kalman->updateGps(25.033, 121.565, sim_alt);
        vTaskDelay(pdMS_TO_TICKS(200));
#endif
    }
}
