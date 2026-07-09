#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include "../Communication/InterCoreComm.hpp"
#include "../Kalman/KalmanFilter.hpp"

enum SensorType {
    SENSOR_ICM20948,
    SENSOR_BMP388,
    SENSOR_MAX_M10S
};

class SensorManager {
public:
    struct TaskParams {
        SensorManager* instance;
        SensorType type;
    };

    SensorManager(InterCoreComm* comm, KalmanFilter* kalman);
    void begin();

    void enableIcm();  void disableIcm();
    void enableBmp();  void disableBmp();
    void enableGps();  void disableGps();

    HealthMonitor& getHealthMonitor() { return _kalman->getHealthMonitor(); }
    KalmanFilter* getKalmanFilter() { return _kalman; }

    static void taskWrapper(void* pvParameters);

    SemaphoreHandle_t permitIcm;
    SemaphoreHandle_t permitBmp;
    SemaphoreHandle_t permitGps;

private:
    void icm20948Task();
    void bmp388Task();
    void maxM10sTask();

    bool _icmActive;
    bool _bmpActive;
    bool _gpsActive;

    InterCoreComm* _comm;
    KalmanFilter* _kalman;
    TaskHandle_t _icmTaskHandle;
    TaskHandle_t _bmpTaskHandle;
    TaskHandle_t _gpsTaskHandle;

    SemaphoreHandle_t _i2cMutex;

    struct BmpCalibrationData {
        uint16_t par_t1;
        uint16_t par_t2;
        int8_t par_t3;
        int16_t par_p1;
        int16_t par_p2;
        int8_t par_p3;
        int8_t par_p4;
        uint16_t par_p5;
        uint16_t par_p6;
        int8_t par_p7;
        int8_t par_p8;
        int16_t par_p9;
        int8_t par_p10;
        int8_t par_p11;
        int64_t t_lin;
    } _bmpCalib;

    bool _icmInitialized = false;
    bool _bmpInitialized = false;

#ifdef ESP_PLATFORM
    esp_err_t i2cWriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t val);
    esp_err_t i2cReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t* readBuf, size_t len);
#endif
    bool initIcm20948();
    bool initBmp388();
    int64_t compensateTemp(uint32_t uncomp_temp);
    uint64_t compensatePress(uint32_t uncomp_press);
};

#endif
