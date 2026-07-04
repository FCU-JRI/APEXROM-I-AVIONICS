#ifndef DATA_MANAGER_HPP
#define DATA_MANAGER_HPP

#include "Communication/InterCoreComm.hpp"
#include "StorageComm/StorageCommManager.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

class StateMachine; // 前向宣告

class DataManager {
public:
    DataManager(InterCoreComm& comm, StateMachine& sm, StorageCommManager& scm);
    
    bool begin();

    void getBuffer256(uint8_t* out, size_t* len);

private:
    InterCoreComm& _comm;
    StateMachine& _sm;
    StorageCommManager& _scm;
    
    TaskHandle_t _rawTaskHandle;
    TaskHandle_t _kalmanTaskHandle;

    uint8_t _buffer256[256];
    size_t  _len256;
    SemaphoreHandle_t _lock256;

    static void rawDataTask(void* pvParameters);
    static void kalmanDataTask(void* pvParameters);

    void processRawData();
    void processKalmanData();

    void updateBuffers(void* data, size_t size);
};

#endif
