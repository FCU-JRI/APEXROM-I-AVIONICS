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
        double t1, t2, t3;
        double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    } _bmpCalib;

    bool _icmInitialized = false;
    bool _bmpInitialized = false;

#ifdef ESP_PLATFORM
    esp_err_t i2cWriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t val);
    esp_err_t i2cReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t* readBuf, size_t len);
#endif
    bool initIcm20948();
    bool initBmp388();
    double compensateTemp(double uncomp_temp);
    double compensatePress(double uncomp_press, double t_lin);
};

#endif
