#include "InterCoreComm.hpp"
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"

InterCoreComm::InterCoreComm() : _rawRingBuf(NULL), _kalmanRingBuf(NULL) {}

bool InterCoreComm::begin() {
    _rawRingBuf = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
    _kalmanRingBuf = xRingbufferCreate(2048, RINGBUF_TYPE_NOSPLIT);
    return (_rawRingBuf != NULL && _kalmanRingBuf != NULL);
}

bool InterCoreComm::sendRawImu(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    struct { SensorPacketHeader h; ImuData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = DATA_TYPE_IMU;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.d = {ax, ay, az, gx, gy, gz, mx, my, mz};
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendRawBmp(float pressure, float temp) {
    struct { SensorPacketHeader h; BmpData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = DATA_TYPE_BMP;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.d = {pressure, temp};
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendRawGps(double lat, double lon, float alt) {
    struct { SensorPacketHeader h; GpsData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = DATA_TYPE_GPS;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.d = {lat, lon, alt};
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendLogEvent(uint8_t event_id, float p1, float p2, float p3) {
    ESP_LOGI("InterCoreComm", "Sending LOG event: %d (%.2f, %.2f, %.2f)", event_id, p1, p2, p3);
    struct { SensorPacketHeader h; LogData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = DATA_TYPE_LOG;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.d.event_id = event_id;
    pkt.d.param1 = p1;
    pkt.d.param2 = p2;
    pkt.d.param3 = p3;
    bool success = xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
    if (!success) {
        ESP_LOGE("InterCoreComm", "Failed to send LOG to _rawRingBuf! (full?)");
    }
    return success;
}

bool InterCoreComm::sendKalmanQuaternion(const float q[4]) {
    struct { KalmanPacketHeader h; KalmanQuaternionData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = KALMAN_TYPE_QUATERNION;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    memcpy(pkt.d.q, q, sizeof(float)*4);
    return xRingbufferSend(_kalmanRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendKalmanGps(double lat, double lon, float velN, float velE) {
    struct { KalmanPacketHeader h; KalmanGpsData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = KALMAN_TYPE_GPS;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.d = {lat, lon, velN, velE};
    return xRingbufferSend(_kalmanRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendKalmanAltitude(float alt, float vz, float az) {
    struct { KalmanPacketHeader h; KalmanAltitudeData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = KALMAN_TYPE_ALTITUDE;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.d = {alt, vz, az};
    return xRingbufferSend(_kalmanRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

void* InterCoreComm::receiveRawData(size_t* size, TickType_t waitTicks) {
    return xRingbufferReceive(_rawRingBuf, size, waitTicks);
}

void InterCoreComm::returnRawData(void* item) {
    vRingbufferReturnItem(_rawRingBuf, item);
}

void* InterCoreComm::receiveKalmanData(size_t* size, TickType_t waitTicks) {
    return xRingbufferReceive(_kalmanRingBuf, size, waitTicks);
}

void InterCoreComm::returnKalmanData(void* item) {
    vRingbufferReturnItem(_kalmanRingBuf, item);
}
