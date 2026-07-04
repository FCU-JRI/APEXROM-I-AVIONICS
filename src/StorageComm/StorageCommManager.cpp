#include "StorageCommManager.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include <stdio.h>

#include "../StateMachine/StateMachine.hpp"

static const char* TAG = "StorageComm";

StorageCommManager::StorageCommManager() : _smQueue(NULL), _rxTaskHandle(NULL) {
    _radioMutex = xSemaphoreCreateMutex();
}

void StorageCommManager::begin(QueueHandle_t smQueue) {
    _smQueue = smQueue;
    
    // 建立無線電接收任務 (Core 1)
    xTaskCreatePinnedToCore(radioRxTask, "RadioRxTask", 4096, this, 6, &_rxTaskHandle, 1);
    ESP_LOGI(TAG, "StorageCommManager Initialized with RX Task");
}

void StorageCommManager::radioRxTask(void* pvParameters) {
    StorageCommManager* instance = static_cast<StorageCommManager*>(pvParameters);
    instance->processRxLogic();
}

void StorageCommManager::processRxLogic() {
    // 緩衝讀取，直到收到換行符再解析
    char buf[16];
    int  idx = 0;

    while (true) {
        int c = getchar();

        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                ESP_LOGI(TAG, "Simulated Serial RX: \"%s\"", buf);
                dispatchCommand(buf);
                idx = 0;
            }
        } else if (idx < (int)(sizeof(buf) - 1)) {
            buf[idx++] = (char)c;
        }
    }
}

// ── 指令解析 ────────────────────────────────────────────────────────────────
// 支援格式：
//   "T" 或 "t"   → FLIGHT_P12_TERMINATE（backward-compat）
//   "<數字>"      → 直接切換至對應 STATENUM（0–17）
// ─────────────────────────────────────────────────────────────────────────────
void StorageCommManager::dispatchCommand(const char* cmd) {
    if (_smQueue == NULL) return;

    // 'T' / 't' → Force Terminate (backward-compat)
    if ((cmd[0] == 'T' || cmd[0] == 't') && cmd[1] == '\0') {
        ESP_LOGW(TAG, "Command: Force Terminate Mission!");
        StateMachine::StateEvent evt = {FLIGHT_P12_TERMINATE, xTaskGetTickCount()};
        xQueueSend(_smQueue, &evt, 0);
        return;
    }

    // 數字字串 → 解析為 STATENUM
    bool isNum = (cmd[0] != '\0');
    for (int i = 0; cmd[i] != '\0'; i++) {
        if (cmd[i] < '0' || cmd[i] > '9') { isNum = false; break; }
    }

    if (isNum) {
        int stateNum = atoi(cmd);
        if (stateNum >= 0 && stateNum <= 17) {
            ESP_LOGW(TAG, "Command: Transition to State %d", stateNum);
            StateMachine::StateEvent evt = {(STATENUM)stateNum, xTaskGetTickCount()};
            xQueueSend(_smQueue, &evt, 0);
        } else {
            ESP_LOGE(TAG, "Command: Invalid state number %d (valid: 0-17)", stateNum);
        }
    } else {
        ESP_LOGE(TAG, "Command: Unknown command \"%s\"", cmd);
    }
}

void StorageCommManager::sendViaRadio(const uint8_t* data, size_t len) {
    // --- 原 RF 發送邏輯註解 ---
    /*
    // 傳輸時獲取鎖，確保接收任務在此期間等待
    if (xSemaphoreTake(_radioMutex, portMAX_DELAY) == pdTRUE) {
        ESP_LOGD(TAG, "Radio TX Start: Sending %d bytes...", len);
        
        // 模擬硬體傳輸延遲
        // hardware_send(data, len);
        
        ESP_LOGD(TAG, "Radio TX Finished.");
        xSemaphoreGive(_radioMutex);
    }
    */

    // --- 使用 Serial (stdout) 模擬發送資料 ---
    ESP_LOGI(TAG, "Simulated Serial TX Start: Sending %d bytes...", (int)len);
    printf("Simulated Radio TX Data: ");
    for(size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

void StorageCommManager::saveToStorage(const uint8_t* data, size_t len) {
    ESP_LOGD(TAG, "Storage: Saving %d bytes...", len);
}
