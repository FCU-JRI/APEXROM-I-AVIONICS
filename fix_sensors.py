import os
import re

# 1. Update StateMachine.cpp
sm_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/StateMachine/StateMachine.cpp'
with open(sm_path, 'r') as f:
    sm = f.read()

sm = re.sub(
    r'char logBuf\[64\];\n\s*snprintf\(logBuf, sizeof\(logBuf\), "State: \%d -> \%d", \(int\)_state, \(int\)newState\);\n\s*_comm->sendLog\(logBuf\);',
    '_comm->sendLogEvent(EVT_STATE_TRANSITION, (float)_state, (float)newState);', sm)

with open(sm_path, 'w') as f:
    f.write(sm)


# 2. Update Sensors.cpp
sns_path = '/Users/hekote/Documents/PlatformIO/Projects/P2026/src/Sensors/Sensors.cpp'
with open(sns_path, 'r') as f:
    sns = f.read()

# Fix ICM20948 Init Failed
sns = re.sub(
    r'if \(_comm\) \{\n\s*char log_msg\[64\];\n\s*snprintf\(log_msg, sizeof\(log_msg\), "HW ERROR: ICM20948 \(IMU\) Init Failed"\);\n\s*_comm->sendLog\(log_msg\);\n\s*\}',
    'if (_comm) _comm->sendLogEvent(EVT_HW_ERROR_IMU);', sns
)

# Fix BMP388 Init Failed
sns = re.sub(
    r'if \(_comm\) \{\n\s*char log_msg\[64\];\n\s*snprintf\(log_msg, sizeof\(log_msg\), "HW ERROR: BMP388 \(Barometer\) Init Failed"\);\n\s*_comm->sendLog\(log_msg\);\n\s*\}',
    'if (_comm) _comm->sendLogEvent(EVT_HW_ERROR_BMP);', sns
)

# Fix BMP388 Calibration Completed
sns = re.sub(
    r'if \(_comm\) \{\n\s*char log_msg\[64\];\n\s*snprintf\(log_msg, sizeof\(log_msg\), "BMP388 Calibration Completed."\);\n\s*_comm->sendLog\(log_msg\);\n\s*snprintf\(log_msg, sizeof\(log_msg\), "Calib T1=\%\.2f, P1=\%\.2f", _bmpCalib\.t1, _bmpCalib\.p1\);\n\s*_comm->sendLog\(log_msg\);\n\s*\}',
    'if (_comm) {\n        _comm->sendLogEvent(EVT_BMP_CALIB_COMPLETED);\n        _comm->sendLogEvent(EVT_BMP_CALIB_PARAMS, _bmpCalib.t1, _bmpCalib.p1);\n    }', sns
)

# Remove the fallback mechanism (Simulated sensors)
sns = re.sub(
    r'if \(!_imuReady\) \{[\s\S]*?vTaskDelay\(pdMS_TO_TICKS\(100\)\);\n\s*\}',
    'if (!_imuReady) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }', sns
)

sns = re.sub(
    r'if \(!_bmpReady\) \{[\s\S]*?vTaskDelay\(pdMS_TO_TICKS\(100\)\);\n\s*\}',
    'if (!_bmpReady) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }', sns
)

with open(sns_path, 'w') as f:
    f.write(sns)

print("Sensors and StateMachine fixed.")
