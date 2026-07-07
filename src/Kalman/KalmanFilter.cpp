#include "KalmanFilter.hpp"
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

// 預設雜訊參數
static const float Q_gyro = 0.001f;
static const float Q_bias = 0.0001f;

KalmanFilter::KalmanFilter(InterCoreComm* comm) : _comm(comm) {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
    _bias[0] = 0.0f; _bias[1] = 0.0f; _bias[2] = 0.0f;
    _calibratingGyro = false;
    _calibSamples = 0;
    _calibSum[0] = _calibSum[1] = _calibSum[2] = 0.0f;
    memset(_P, 0, sizeof(_P));
    for (int i = 0; i < 7; i++) _P[i][i] = 0.1f;
    _lastMicros = esp_timer_get_time();
    _lat = 0; _lon = 0; _velN = 0; _velE = 0;
    _x_alt[0] = 0; _x_alt[1] = 0; _x_alt[2] = 0;
    memset(_P_alt, 0, sizeof(_P_alt));
    _P_alt[0][0] = 10.0f; _P_alt[1][1] = 10.0f; _P_alt[2][2] = 0.01f;
    _az_world = 0;
    loadFromNVM();
}

void KalmanFilter::saveToNVM() {
    nvs_handle_t handle;
    if (nvs_open("kf_storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_blob(handle, "q", _q, sizeof(_q));
        nvs_set_blob(handle, "bias", _bias, sizeof(_bias));
        nvs_set_blob(handle, "x_alt", _x_alt, sizeof(_x_alt));
        nvs_set_blob(handle, "lat_lon", &_lat, sizeof(double) * 2);
        nvs_commit(handle); nvs_close(handle);
    }
}

void KalmanFilter::loadFromNVM() {
    nvs_handle_t handle;
    if (nvs_open("kf_storage", NVS_READONLY, &handle) == ESP_OK) {
        size_t sz;
        sz = sizeof(_q); nvs_get_blob(handle, "q", _q, &sz);
        sz = sizeof(_bias); nvs_get_blob(handle, "bias", _bias, &sz);
        sz = sizeof(_x_alt); nvs_get_blob(handle, "x_alt", _x_alt, &sz);
        sz = sizeof(double)*2; nvs_get_blob(handle, "lat_lon", &_lat, &sz);
        nvs_close(handle);
    }
}

void KalmanFilter::startGyroCalibration() {
    _calibratingGyro = true;
    _calibSamples = 0;
    _calibSum[0] = 0.0f; _calibSum[1] = 0.0f; _calibSum[2] = 0.0f;
    if (_comm) _comm->sendLogEvent(EVT_GYRO_CALIB_STARTED);
}

void KalmanFilter::updateImu(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    // 若正在校準陀螺儀，收集資料並跳過濾波更新
    if (_calibratingGyro) {
        _calibSum[0] += gx;
        _calibSum[1] += gy;
        _calibSum[2] += gz;
        _calibSamples++;
        if (_calibSamples >= 200) {  // 收集約 200 筆樣本
            _bias[0] = _calibSum[0] / _calibSamples;
            _bias[1] = _calibSum[1] / _calibSamples;
            _bias[2] = _calibSum[2] / _calibSamples;
            _calibratingGyro = false;
            saveToNVM();
            if (_comm) {
                _comm->sendLogEvent(EVT_GYRO_CALIB_DONE, _bias[0], _bias[1], _bias[2]);
            }
        }
        return; 
    }

    // 扣除陀螺儀零偏
    gx -= _bias[0];
    gy -= _bias[1];
    gz -= _bias[2];

    _health.updateImuHealth(ax, ay, az, gx, gy, gz);
    if (!_health.isImuHealthy()) return;

    // 增加一個計數器來控制傳輸頻率 (1/5)
    static uint8_t sendCounter = 0;
    bool shouldSend = (sendCounter == 0);
    sendCounter = (sendCounter + 1) % 5;

    // 數據健康，依頻率傳送原始數據
    if (_comm && shouldSend) {
        _comm->sendRawImu(ax, ay, az, gx, gy, gz, mx, my, mz);
    }

    uint32_t now = esp_timer_get_time();
    float dt = (now - _lastMicros) / 1000000.0f;
    _lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 0.01f;

    // 使用 Madgwick 演算法更新四元數姿態
    MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

    float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];
    _az_world = 2.0f*(qx*qz - qw*qy)*ax + 2.0f*(qy*qz + qw*qx)*ay + (qw*qw - qx*qx - qy*qy + qz*qz)*az - 9.80665f;
    predictAltitude(dt);

    // 依頻率傳送濾波後的四元數數據
    if (_comm && shouldSend) {
        _comm->sendKalmanQuaternion(_q);
    }

    static uint32_t saveCounter = 0;
    if (++saveCounter >= 500) { saveToNVM(); saveCounter = 0; }
}

void KalmanFilter::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {
    float q1 = _q[0], q2 = _q[1], q3 = _q[2], q4 = _q[3];
    float norm;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;
    float beta = 0.1f; // Madgwick 增益參數

    // 輔助變數
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _4q3 = 4.0f * q3;
    float _4q4 = 4.0f * q4;
    float _8q2 = 8.0f * q2;
    float _8q3 = 8.0f * q3;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    float q4q4 = q4 * q4;

    // 正規化加速度
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    // 判斷是否使用磁力計 (9-DOF or 6-DOF)
    float magMag = sqrtf(mx*mx + my*my + mz*mz);
    if (magMag > 0.01f) {
        norm = 1.0f / magMag;
        mx *= norm; my *= norm; mz *= norm;

        float _2q1mx = 2.0f * q1 * mx;
        float _2q1my = 2.0f * q1 * my;
        float _2q1mz = 2.0f * q1 * mz;
        float _2q2mx = 2.0f * q2 * mx;
        float hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        float hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 + my * q2q2 + my * q3q3 + 2.0f * q3 * mz * q4 - my * q4q4;
        float _2bx = sqrtf(hx * hx + hy * hy);
        float _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + 2.0f * q3 * my * q4 - mz * q3q3 + mz * q4q4;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q4 = q3 * q4;

        s1 = -_2q3 * (2.0f * q2q4 - 2.0f * q1q3 - ax) + _2q2 * (2.0f * q1q2 + 2.0f * q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - 2.0f * q1q3 - ax) + _2q1 * (2.0f * q1q2 + 2.0f * q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - 2.0f * q1q3 - ax) + _2q4 * (2.0f * q1q2 + 2.0f * q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - 2.0f * q1q3 - ax) + _2q3 * (2.0f * q1q2 + 2.0f * q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    } else {
        // 6-DOF
        s1 = -_4q3 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) + _4q2 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay);
        s2 = _4q1 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay) + _4q4 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) - _8q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az);
        s3 = _4q1 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) - _4q4 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay) - _8q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az);
        s4 = _4q2 * (2.0f * q2 * q4 - 2.0f * q1 * q3 - ax) + _4q3 * (2.0f * q1 * q2 + 2.0f * q3 * q4 - ay);
    }

    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    if (norm > 0.0f) {
        norm = 1.0f / norm;
        s1 *= norm; s2 *= norm; s3 *= norm; s4 *= norm;
    }

    // 計算四元數變化率
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // 積分得到新四元數
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;

    // 正規化
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    _q[0] = q1 * norm;
    _q[1] = q2 * norm;
    _q[2] = q3 * norm;
    _q[3] = q4 * norm;
}

void KalmanFilter::predictAltitude(float dt) {
    float accel = _az_world - _x_alt[2];
    _x_alt[0] += _x_alt[1] * dt + 0.5f * accel * dt * dt;
    _x_alt[1] += accel * dt;
    _P_alt[0][0] += _P_alt[1][1] * dt * dt + 0.001f; 
    _P_alt[1][1] += _P_alt[2][2] * dt * dt + 0.01f;
    _P_alt[2][2] += 0.0001f;
}

void KalmanFilter::updateBmp(float pressure, float temperature) {
    float baroAlt = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));
    _health.updateBmpHealth(baroAlt, _x_alt[0]);
    if (!_health.isBmpHealthy()) return;

    // 數據健康，先傳送原始數據
    if (_comm) {
        _comm->sendRawBmp(pressure/100.0f, temperature);
    }

    float y = baroAlt - _x_alt[0];
    float S = _P_alt[0][0] + 2.0f;
    float K[3] = {_P_alt[0][0]/S, _P_alt[1][0]/S, _P_alt[2][0]/S};
    _x_alt[0] += K[0] * y; _x_alt[1] += K[1] * y; _x_alt[2] += K[2] * y;
    _P_alt[0][0] -= K[0] * _P_alt[0][0]; _P_alt[1][1] -= K[1] * _P_alt[0][1];

    if (_comm) {
        _comm->sendKalmanAltitude(_x_alt[0], _x_alt[1], _az_world);
    }
}

void KalmanFilter::updateGps(double lat, double lon, float alt) {
    _health.updateGpsHealth(alt, _x_alt[0]);
    if (!_health.isGpsHealthy()) return;

    // 數據健康，先傳送原始數據
    if (_comm) {
        _comm->sendRawGps(lat, lon, alt);
    }

    _lat = lat; _lon = lon;
    float y = alt - _x_alt[0];
    float S = _P_alt[0][0] + 10.0f;
    float K[3] = {_P_alt[0][0]/S, _P_alt[1][0]/S, _P_alt[2][0]/S};
    _x_alt[0] += K[0] * y; _x_alt[1] += K[1] * y; _x_alt[2] += K[2] * y;

    if (_comm) {
        _comm->sendKalmanGps(_lat, _lon, _velN, _velE);
    }
}

void KalmanFilter::getQuaternion(float q[4]) { memcpy(q, _q, sizeof(float) * 4); }
void KalmanFilter::getGps(double& lat, double& lon, float& vN, float& vE) { lat=_lat; lon=_lon; vN=_velN; vE=_velE; }
void KalmanFilter::getAltitude(float& alt, float& vz, float& az) { alt=_x_alt[0]; vz=_x_alt[1]; az=_az_world; }
void KalmanFilter::normalizeQuaternion() {
    float mag = sqrtf(_q[0]*_q[0] + _q[1]*_q[1] + _q[2]*_q[2] + _q[3]*_q[3]);
    _q[0] /= mag; _q[1] /= mag; _q[2] /= mag; _q[3] /= mag;
}
