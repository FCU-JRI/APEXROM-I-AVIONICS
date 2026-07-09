#ifndef INTER_CORE_COMM_HPP
#define INTER_CORE_COMM_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include <stdio.h>
#include <string.h>

// 1. 定義資料型別
enum SensorDataType : uint8_t {
    DATA_TYPE_PADDING = 0,
    DATA_TYPE_IMU = 1,
    DATA_TYPE_BMP = 2,
    DATA_TYPE_GPS = 3,
    DATA_TYPE_LOG = 4
};

// 2. 定義共用標頭
struct SensorPacketHeader {
    SensorDataType type;
    uint8_t pad[3];    // 明確補齊 3 bytes，確保 Python 解析結果固定為 0x00
    TickType_t timestamp;
};

// 3. 定義不同的資料結構
struct ImuData {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz; // 加入磁力計數據
};

struct BmpData {
    float pressure, temperature;
};

struct GpsData {
    double lat, lon;
    float alt;
};

enum LogEventId : uint8_t {
    EVT_STATE_TRANSITION = 1,
    EVT_SM_CURRENT_STATE = 2,
    EVT_POWERED_FLIGHT_START = 3,
    EVT_SYS_RESET = 4,
    EVT_CALIB_GYRO = 5,
    EVT_DROGUE_DEPLOY = 6,
    EVT_MOTOR_SEP_MAIN = 7,
    EVT_MAIN_NO_SEP = 8,
    EVT_MISSION_TERMINATED = 9,
    EVT_TRIG_FORCE_APOGEE = 10,
    EVT_TRIG_MOTOR_THRUST = 11,
    EVT_TRIG_ALT_GAIN = 12,
    EVT_TRIG_BURNOUT_IMU = 13,
    EVT_TRIG_BURNOUT_TIME = 14,
    EVT_TRIG_APOGEE_SENSORS = 15,
    EVT_TRIG_DESCENT = 16,
    EVT_DECISION_GPS_FAIL_MAIN = 17,
    EVT_TRIG_LAUNCH_G_FORCE = 18,
    EVT_GYRO_CALIB_STARTED = 19,
    EVT_GYRO_CALIB_DONE = 20,
    EVT_HW_ERROR_IMU = 21,
    EVT_HW_ERROR_BMP = 22,
    EVT_BMP_CALIB_COMPLETED = 23,
    EVT_BMP_CALIB_PARAMS = 24,
    EVT_ACCEL_CALIB_DONE = 25,
    EVT_MAG_CALIB_DONE = 26,
    EVT_TEMP_CALIB_DONE = 27
};

struct LogData {
    uint8_t event_id;
    float param1;
    float param2;
    float param3;
};

// 4. Kalman 資料型別與標頭
enum KalmanDataType : uint8_t {
    KALMAN_TYPE_QUATERNION = 10,
    KALMAN_TYPE_GPS = 11,
    KALMAN_TYPE_ALTITUDE = 12
};

struct KalmanPacketHeader {
    KalmanDataType type;
    uint8_t pad[3];    // 明確補齊 3 bytes，確保 Python 解析結果固定為 0x00
    TickType_t timestamp;
};

struct KalmanQuaternionData { float q[4]; };
struct KalmanGpsData { double lat, lon; float velN, velE; };
struct KalmanAltitudeData { float alt, vz, az; };

class InterCoreComm {
public:
    InterCoreComm();
    bool begin();

    bool sendRawImu(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    bool sendRawBmp(float pressure, float temp);
    bool sendRawGps(double lat, double lon, float alt);
    bool sendLogEvent(uint8_t event_id, float p1 = 0, float p2 = 0, float p3 = 0);

    bool sendKalmanQuaternion(const float q[4]);
    bool sendKalmanGps(double lat, double lon, float velN, float velE);
    bool sendKalmanAltitude(float alt, float vz, float az);

    void* receiveRawData(size_t* size, TickType_t waitTicks);
    void returnRawData(void* item);

    void* receiveKalmanData(size_t* size, TickType_t waitTicks);
    void returnKalmanData(void* item);

private:
    RingbufHandle_t _rawRingBuf;
    RingbufHandle_t _kalmanRingBuf;
};

#endif
