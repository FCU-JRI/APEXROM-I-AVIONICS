import os
import re

# 1. Update InterCoreComm.hpp
hpp_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/Communication/InterCoreComm.hpp'
with open(hpp_path, 'r') as f:
    hpp = f.read()

hpp = re.sub(
    r'struct LogData \{[\s\S]*?\};',
    '''enum LogEventId : uint8_t {
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
    EVT_HW_ERROR_BMP = 22
};

struct LogData {
    uint8_t event_id;
    float param1;
    float param2;
    float param3;
};''', hpp)

hpp = hpp.replace('bool sendLog(const char* msg);', 'bool sendLogEvent(uint8_t event_id, float p1 = 0, float p2 = 0, float p3 = 0);')

with open(hpp_path, 'w') as f:
    f.write(hpp)

# 2. Update InterCoreComm.cpp
cpp_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/Communication/InterCoreComm.cpp'
with open(cpp_path, 'r') as f:
    cpp = f.read()

cpp = re.sub(
    r'bool InterCoreComm::sendLog\(const char\* msg\) \{[\s\S]*?return xRingbufferSend\(_rawRingBuf, &pkt, sizeof\(pkt\), 0\) == pdTRUE;\n\}',
    '''bool InterCoreComm::sendLogEvent(uint8_t event_id, float p1, float p2, float p3) {
    struct { SensorPacketHeader h; LogData d; } pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.h.type = DATA_TYPE_LOG;
    pkt.h.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.d.event_id = event_id;
    pkt.d.param1 = p1;
    pkt.d.param2 = p2;
    pkt.d.param3 = p3;
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}''', cpp)

with open(cpp_path, 'w') as f:
    f.write(cpp)

# 3. Update DataManager.cpp (Remove ESP_LOGI for string)
dm_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/DataManager/DataManager.cpp'
with open(dm_path, 'r') as f:
    dm = f.read()
dm = dm.replace('ESP_LOGI(TAG, "Log Record: %s", data->message);', 'ESP_LOGI(TAG, "Log Record ID: %d, P1: %.2f", data->event_id, data->param1);')
with open(dm_path, 'w') as f:
    f.write(dm)

# 4. Update KalmanFilter.cpp
kf_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/Kalman/KalmanFilter.cpp'
with open(kf_path, 'r') as f:
    kf = f.read()
kf = kf.replace('_comm->sendLog("ACTION: Gyro Calibration Started");', '_comm->sendLogEvent(EVT_GYRO_CALIB_STARTED);')
kf = re.sub(r'char msg\[64\];\n\s*snprintf\(msg, sizeof\(msg\), "ACTION: Gyro Calib Done \(\%\.3f, \%\.3f, \%\.3f\)", _bias\[0\], _bias\[1\], _bias\[2\]\);\n\s*_comm->sendLog\(msg\);', '_comm->sendLogEvent(EVT_GYRO_CALIB_DONE, _bias[0], _bias[1], _bias[2]);', kf)
with open(kf_path, 'w') as f:
    f.write(kf)

# 5. Update Sensors.cpp
sns_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/Sensors/Sensors.cpp'
with open(sns_path, 'r') as f:
    sns = f.read()
sns = sns.replace('char log_msg[64];', '')
sns = sns.replace('snprintf(log_msg, sizeof(log_msg), "HW ERROR: ICM20948 (IMU) Init Failed");', '')
sns = sns.replace('_comm->sendLog(log_msg);', '_comm->sendLogEvent(EVT_HW_ERROR_IMU);', 1)

sns = sns.replace('snprintf(log_msg, sizeof(log_msg), "HW ERROR: BMP388 (Barometer) Init Failed");', '')
sns = sns.replace('_comm->sendLog(log_msg);', '_comm->sendLogEvent(EVT_HW_ERROR_BMP);', 1)
with open(sns_path, 'w') as f:
    f.write(sns)

# 6. Update StateMachine.cpp
sm_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/StateMachine/StateMachine.cpp'
with open(sm_path, 'r') as f:
    sm = f.read()

sm = re.sub(r'char logBuf\[64\];\n\s*snprintf\(logBuf, sizeof\(logBuf\), "\[SM\] Current State: \%d, Time: \%\.2f", \(int\)_state, \(_startTime > 0\) \? \(esp_timer_get_time\(\) / 1000000\.0f - _startTime\) : 0\);\n\s*_comm->sendLog\(logBuf\);', 
'_comm->sendLogEvent(EVT_SM_CURRENT_STATE, (float)_state, (_startTime > 0) ? (esp_timer_get_time() / 1000000.0f - _startTime) : 0);', sm)

sm = re.sub(r'char logBuf\[64\];\n\s*snprintf\(logBuf, sizeof\(logBuf\), "State: \%d -> \%d", \(int\)oldState, \(int\)newState\);\n\s*_comm->sendLog\(logBuf\);',
'_comm->sendLogEvent(EVT_STATE_TRANSITION, (float)oldState, (float)newState);', sm)

sm = sm.replace('_comm->sendLog("EVENT: Powered flight start time recorded");', '_comm->sendLogEvent(EVT_POWERED_FLIGHT_START);')
sm = sm.replace('_comm->sendLog("ACTION: System Reset to STBY_IDLE");', '_comm->sendLogEvent(EVT_SYS_RESET);')
sm = sm.replace('_comm->sendLog("ACTION: Calibrating Gyro...");', '_comm->sendLogEvent(EVT_CALIB_GYRO);')
sm = sm.replace('_comm->sendLog("ACTION: Drogue Deployed");', '_comm->sendLogEvent(EVT_DROGUE_DEPLOY);')
sm = sm.replace('_comm->sendLog("ACTION: Motor Sep & Main Chute");', '_comm->sendLogEvent(EVT_MOTOR_SEP_MAIN);')
sm = sm.replace('_comm->sendLog("ACTION: Main Chute (No Sep)");', '_comm->sendLogEvent(EVT_MAIN_NO_SEP);')
sm = sm.replace('_comm->sendLog("EVENT: Mission Terminated");', '_comm->sendLogEvent(EVT_MISSION_TERMINATED);')

sm = sm.replace('_comm->sendLog("TRIGGER: Global Time Safety - Forcing Apogee (35s)");', '_comm->sendLogEvent(EVT_TRIG_FORCE_APOGEE);')
sm = sm.replace('_comm->sendLog("TRIGGER: Motor Thrust detected (IMU)");', '_comm->sendLogEvent(EVT_TRIG_MOTOR_THRUST);')
sm = sm.replace('_comm->sendLog("TRIGGER: Altitude gain detected (BMP Backup)");', '_comm->sendLogEvent(EVT_TRIG_ALT_GAIN);')
sm = sm.replace('_comm->sendLog("TRIGGER: Burnout detected (IMU)");', '_comm->sendLogEvent(EVT_TRIG_BURNOUT_IMU);')
sm = sm.replace('_comm->sendLog("TRIGGER: Burnout assumed (Time Backup)");', '_comm->sendLogEvent(EVT_TRIG_BURNOUT_TIME);')
sm = sm.replace('_comm->sendLog("TRIGGER: Apogee detected (Sensors)");', '_comm->sendLogEvent(EVT_TRIG_APOGEE_SENSORS);')
sm = sm.replace('_comm->sendLog("TRIGGER: Descent detected");', '_comm->sendLogEvent(EVT_TRIG_DESCENT);')
sm = sm.replace('_comm->sendLog("DECISION: GPS Failed, forcing Main Chute (Safety First)");', '_comm->sendLogEvent(EVT_DECISION_GPS_FAIL_MAIN);')
sm = sm.replace('_comm->sendLog("TRIGGER: Launch G-force detected");', '_comm->sendLogEvent(EVT_TRIG_LAUNCH_G_FORCE);')

with open(sm_path, 'w') as f:
    f.write(sm)

print("Done refactoring C++ code.")
