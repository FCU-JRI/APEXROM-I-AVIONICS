#include "StateMachine.hpp"

#include <stdio.h>
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"

StateMachine::StateMachine(SensorManager* sensors, RecoveryManager* recovery, InterCoreComm* comm) : _state(STBY_IDLE), _lastPushedState(STBY_IDLE), _sensors(sensors), _recovery(recovery), _comm(comm), _taskHandle(NULL), 
      _startTime(0.0f), _motorSeparated(false) {
    permitKalmanFilter = xSemaphoreCreateBinary();
    stateQueue = xQueueCreate(10, sizeof(StateEvent));
    _lastLat = 0; _lastLon = 0;

    nvs_handle_t handle;
    if (nvs_open("sm_storage", NVS_READONLY, &handle) == ESP_OK) {
        uint8_t s;
        if (nvs_get_u8(handle, "state", &s) == ESP_OK) {
            _state = (STATENUM)s;
            _lastPushedState = (STATENUM)s;
        }
        uint8_t m;
        if (nvs_get_u8(handle, "motor", &m) == ESP_OK) _motorSeparated = (m != 0);
        size_t sz = sizeof(double) * 2;
        nvs_get_blob(handle, "pos", &_lastLat, &sz);
        nvs_close(handle);
    }
}

void StateMachine::begin() {
    xTaskCreatePinnedToCore(taskWrapper, "SM_Task", 4096, this, 10, &_taskHandle, 1);
}

void StateMachine::taskWrapper(void* pvParameters) {
    StateMachine* instance = (StateMachine*)pvParameters;
    instance->stateMachineTask();
}

void StateMachine::stateMachineTask() {
    StateEvent event;
    const TickType_t timeoutTicks = pdMS_TO_TICKS(60000);
    TickType_t lastLogTick = 0;

    while (1) {
        TickType_t now = xTaskGetTickCount();
        if (now - lastLogTick > pdMS_TO_TICKS(1000)) {
            _comm->sendLogEvent(EVT_SM_CURRENT_STATE, (float)_state, (_startTime > 0) ? (esp_timer_get_time() / 1000000.0f - _startTime) : 0);
            lastLogTick = now;
        }

        if (xQueueReceive(stateQueue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (switchValid(event.state)) {
                transitionTo(event.state);
            } else {
                TickType_t now = xTaskGetTickCount();
                if ((now - event.timestamp) <= timeoutTicks) {
                    xQueueSend(stateQueue, &event, 0);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }
    }
}

void StateMachine::pushState(STATENUM newState) {
    // 防止重複推播相同狀態導致 Queue 溢位
    if (stateQueue != NULL && newState != _lastPushedState) {
        StateEvent event = {newState, xTaskGetTickCount()};
        if (xQueueSend(stateQueue, &event, 0) == pdTRUE) {
            _lastPushedState = newState;
        }
    }
}

bool StateMachine::switchValid(STATENUM newState) {
    // 允許強制切換回 STBY_IDLE 進行系統重置
    if (newState == STBY_IDLE) return true;

    if (_state == FLIGHT_P12_TERMINATE) return false;
    if (newState == FLIGHT_P12_TERMINATE) return true;
    
    // 飛行狀態差時檢查 (時間保護)
    if (_startTime > 0.0f) {
        float currentTime = esp_timer_get_time() / 1000000.0f;
        float diff = currentTime - _startTime;
        
        // 額外的安全性檢查：確保不會過早進入回收狀態
        if (newState == FLIGHT_P8_9_APOGEE && diff < 10.91f) return false;
        if (newState == FLIGHT_P11_MAIN_CHUTE_DEPLOY && diff < 227.17f) return false;
    }

    if (_state >= FLIGHT_P5_IGNITION && _state < FLIGHT_P12_TERMINATE) {
        if (_state == FLIGHT_P10_DESCENT) {
            return (newState == FLIGHT_P11_MAIN_CHUTE_DEPLOY || newState == FLIGHT_P11_SKIP_MAIN_CHUTE);
        }
        if (_state == FLIGHT_P11_MAIN_CHUTE_DEPLOY || _state == FLIGHT_P11_SKIP_MAIN_CHUTE) {
            return (newState == FLIGHT_P12_TERMINATE);
        }
        // 確保不會跳過狀態或往回跳
        return (newState == (STATENUM)((int)_state + 1));
    }
    return true;
}

void StateMachine::transitionTo(STATENUM newState) {
    _comm->sendLogEvent(EVT_STATE_TRANSITION, (float)_state, (float)newState); 

    if (newState == FLIGHT_P6_POWERED) {
        _startTime = esp_timer_get_time() / 1000000.0f;
        _comm->sendLogEvent(EVT_POWERED_FLIGHT_START);
    }

    _state = newState;
    _lastPushedState = newState; // 同步狀態
    
    switch (newState) {
        case STBY_IDLE:
            _startTime = 0.0f;
            _motorSeparated = false;
            _comm->sendLogEvent(EVT_SYS_RESET);
            break;
        case CAL_GYRO:
            _comm->sendLogEvent(EVT_CALIB_GYRO);
            if (_sensors && _sensors->getKalmanFilter()) {
                _sensors->getKalmanFilter()->startGyroCalibration();
            }
            break;
        case FLIGHT_P8_9_APOGEE:
            _recovery->deployDrogue();
            _comm->sendLogEvent(EVT_DROGUE_DEPLOY);
            break;
        case FLIGHT_P11_MAIN_CHUTE_DEPLOY:
            _motorSeparated = true;
            _recovery->deployMain();
            _comm->sendLogEvent(EVT_MOTOR_SEP_MAIN);
            break;
        case FLIGHT_P11_SKIP_MAIN_CHUTE:
            _motorSeparated = false;
            _comm->sendLogEvent(EVT_MAIN_NO_SEP);
            break;
        case FLIGHT_P12_TERMINATE:
            _comm->sendLogEvent(EVT_MISSION_TERMINATED);
            _recovery->powerOffSystem();
            break;
        default:
            break;
    }

    if (newState >= FLIGHT_P5_IGNITION && newState <= FLIGHT_P12_TERMINATE) {
        _sensors->enableIcm();
        _sensors->enableBmp();
        _sensors->enableGps();
        xSemaphoreGive(permitKalmanFilter);
    }
    saveStateToNVM(newState);
}

void StateMachine::saveStateToNVM(STATENUM state) {
    nvs_handle_t handle;
    if (nvs_open("sm_storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u8(handle, "state", (uint8_t)state);
        nvs_set_u8(handle, "motor", _motorSeparated ? 1 : 0);
        nvs_set_blob(handle, "pos", &_lastLat, sizeof(double) * 2);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void StateMachine::processKalmanForTrigger(float alt, float vz, float az) {
    if (!_sensors) return;
    HealthMonitor& health = _sensors->getHealthMonitor();

    // 計算自點火起的飛行時間
    float flightTime = (_startTime > 0) ? (esp_timer_get_time() / 1000000.0f - _startTime) : 0;

    // --- 關鍵修正：加入與 _startTime 差時大於 35 秒的強制頂點觸發 ---
    if (_startTime > 0 && _state < FLIGHT_P8_9_APOGEE && flightTime > TIMEOUT_APOGEE) {
        _comm->sendLogEvent(EVT_TRIG_FORCE_APOGEE);
        pushState(FLIGHT_P8_9_APOGEE);
        return; 
    }

    switch (_state) {
        case FLIGHT_P5_IGNITION:
            if (health.isImuHealthy()) {
                if (az > 10.0f) {
                    _comm->sendLogEvent(EVT_TRIG_MOTOR_THRUST);
                    pushState(FLIGHT_P6_POWERED);
                }
            } else if (health.isBmpHealthy() && alt > 10.0f) {
                _comm->sendLogEvent(EVT_TRIG_ALT_GAIN);
                pushState(FLIGHT_P6_POWERED);
            }
            break;

        case FLIGHT_P6_POWERED:
            if (health.isImuHealthy()) {
                if (az < 2.0f) {
                    _comm->sendLogEvent(EVT_TRIG_BURNOUT_IMU);
                    pushState(FLIGHT_P7_INERTIAL);
                }
            } else if (flightTime > 5.0f) { 
                _comm->sendLogEvent(EVT_TRIG_BURNOUT_TIME);
                pushState(FLIGHT_P7_INERTIAL);
            }
            break;

        case FLIGHT_P7_INERTIAL:
            if (vz < -0.2f && (health.isBmpHealthy() || health.isImuHealthy())) {
                _comm->sendLogEvent(EVT_TRIG_APOGEE_SENSORS);
                pushState(FLIGHT_P8_9_APOGEE);
            }
            // 這裡的定時觸發已被上方全局檢查涵蓋，但保留作為雙重保險
            break;

        case FLIGHT_P8_9_APOGEE:
            // 進入頂點後，若 1 秒後數據仍顯示下降或時間超過安全閾值，進入下降段
            if (az < 2.0f || flightTime > TIMEOUT_APOGEE + 1.0f) {
                _comm->sendLogEvent(EVT_TRIG_DESCENT);
                pushState(FLIGHT_P10_DESCENT);
            }
            break;

        case FLIGHT_P10_DESCENT:
            if (alt < 200.0f || flightTime > TIMEOUT_MAIN_CHUTE) {
                if (!health.isGpsHealthy()) {
                    _comm->sendLogEvent(EVT_DECISION_GPS_FAIL_MAIN);
                    pushState(FLIGHT_P11_MAIN_CHUTE_DEPLOY);
                } else {
                    float dist = calculateDistanceToTarget();
                    if (dist < GPS_THRESHOLD) pushState(FLIGHT_P11_MAIN_CHUTE_DEPLOY);
                    else pushState(FLIGHT_P11_SKIP_MAIN_CHUTE);
                }
            }
            break;

        default:
            break;
    }
}


void StateMachine::processImuForTrigger(float ax, float ay, float az, float gx, float gy, float gz) {
    if (_state == STBY_IDLE && az > 19.6f) {
        _comm->sendLogEvent(EVT_TRIG_LAUNCH_G_FORCE);
        pushState(STBY_BIT);
        pushState(FLIGHT_P5_IGNITION);
    }
}

void StateMachine::updateGpsPosition(double lat, double lon) {
    _lastLat = lat; _lastLon = lon;
}

float StateMachine::calculateDistanceToTarget() {
    double dLat = (_lastLat - TARGET_LAT) * 111000.0;
    double dLon = (_lastLon - TARGET_LON) * 111000.0 * cos(TARGET_LAT * M_PI / 180.0);
    return sqrt(dLat * dLat + dLon * dLon);
}

STATENUM StateMachine::getCurrentState() { return _state; }

