#include "RecoveryManager.hpp"

static const char* TAG = "Recovery";

RecoveryManager::RecoveryManager() {
    gpio_reset_pin(PIN_27);
    gpio_set_direction(PIN_27, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_27, 0);

    gpio_reset_pin(PIN_14);
    gpio_set_direction(PIN_14, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_14, 0);

    gpio_reset_pin(PIN_5);
    gpio_set_direction(PIN_5, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_5, 0);
}
void RecoveryManager::deployDrogue()   { applyLogic(1, 1, 1); }
void RecoveryManager::deployMain() { applyLogic(0, 1, 0); }
void RecoveryManager::powerOffSystem() { applyLogic(0, 1, 1); }

void RecoveryManager::applyLogic(int io27, int io14, int io5) {
    ESP_LOGI(TAG, "Logic Apply -> IO27:%d, IO14:%d, IO5:%d", io27, io14, io5);
    gpio_set_level(PIN_27, io27);
    gpio_set_level(PIN_14, io14);
    gpio_set_level(PIN_5, io5);
}
