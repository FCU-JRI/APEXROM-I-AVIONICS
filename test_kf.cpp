#include <iostream>
#include <cmath>
#include <vector>

class KalmanFilter {
public:
    float _x_alt[3] = {0, 0, 0}; // alt, vz, bias
    float _q[4] = {1, 0, 0, 0};
    float _az_world = 0;

    void updateImu(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
        // Madgwick omitted for simplicity, assume identity quaternion
        float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];
        _az_world = 2.0f*(qx*qz - qw*qy)*ax + 2.0f*(qy*qz + qw*qx)*ay + (qw*qw - qx*qx - qy*qy + qz*qz)*az - 9.80665f;
        predictAltitude(dt);
    }

    void predictAltitude(float dt) {
        float accel = _az_world - _x_alt[2];
        _x_alt[0] += _x_alt[1] * dt + 0.5f * accel * dt * dt;
        _x_alt[1] += accel * dt;
    }

    void updateBmp(float pressure, float temperature) {
        float baroAlt = 44330.0f * (1.0f - std::pow(pressure / 101325.0f, 0.1903f));
        float y = baroAlt - _x_alt[0];
        
        float alpha = 0.2f;    // 高度修正增益 (K0)
        float beta_gain = 0.05f; // 速度修正增益 (K1)
        float gamma = 0.001f;  // 加速度偏差修正增益 (K2)
        
        _x_alt[0] += alpha * y; 
        _x_alt[1] += beta_gain * y; 
        _x_alt[2] -= gamma * y; // 注意 bias 的符號
    }
};

int main() {
    KalmanFilter kf;
    float sim_alt = 0;
    float sim_vz = 0;
    float sim_az = 9.80665f;
    
    float dt = 0.01f;
    for(int i=0; i<5000; i++) {
        float t = i * dt;
        if (t > 1.0f && t < 5.0f) {
            sim_az = 9.80665f + 40.0f; // 4G thrust
        } else if (t >= 5.0f && t < 15.0f) {
            sim_az = 0.0f; // Free fall
        } else if (t >= 15.0f) {
            sim_az = 9.80665f; // Parachute (terminal velocity)
            if (sim_vz < -10.0f) {
                 sim_az = 9.80665f + 10.0f; // Force it to terminal velocity
            }
        }
        
        sim_vz += (sim_az - 9.80665f) * dt;
        sim_alt += sim_vz * dt;
        if (sim_alt < 0) { sim_alt = 0; sim_vz = 0; }
        
        float pressure = 101325.0f * std::pow(1.0f - sim_alt / 44330.0f, 5.255f);
        
        // IMU runs at 100Hz
        kf.updateImu(0, 0, sim_az, 0, 0, 0, dt);
        
        // BMP runs at 25Hz (every 4th step)
        if (i % 4 == 0) {
            kf.updateBmp(pressure, 25.0f);
        }
        
        if (i % 100 == 0) {
            std::cout << "t=" << t << " | TRUE alt:" << sim_alt << " vz:" << sim_vz 
                      << " || KF alt:" << kf._x_alt[0] << " vz:" << kf._x_alt[1] << " bias:" << kf._x_alt[2] << std::endl;
        }
    }
    return 0;
}
