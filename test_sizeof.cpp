#include <iostream>
#include <cstdint>

enum SensorDataType : uint8_t {
    DATA_TYPE_PADDING = 0,
    DATA_TYPE_IMU = 1,
    DATA_TYPE_BMP = 2,
    DATA_TYPE_GPS = 3,
    DATA_TYPE_LOG = 4
};

struct SensorPacketHeader {
    SensorDataType type;
    uint8_t pad[3];
    uint32_t timestamp;
};

struct LogData {
    uint8_t event_id;
    float param1;
    float param2;
    float param3;
};

struct pkt {
    SensorPacketHeader h;
    LogData d;
};

int main() {
    std::cout << "SensorPacketHeader: " << sizeof(SensorPacketHeader) << std::endl;
    std::cout << "LogData: " << sizeof(LogData) << std::endl;
    std::cout << "pkt: " << sizeof(pkt) << std::endl;
    return 0;
}
