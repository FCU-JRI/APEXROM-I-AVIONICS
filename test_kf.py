import math

class KalmanFilter:
    def __init__(self):
        self.x_alt = [0.0, 0.0, 0.0]
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.az_world = 0.0

    def updateImu(self, ax, ay, az, gx, gy, gz, dt):
        qw, qx, qy, qz = self.q
        self.az_world = 2.0*(qx*qz - qw*qy)*ax + 2.0*(qy*qz + qw*qx)*ay + (qw*qw - qx*qx - qy*qy + qz*qz)*az - 9.80665
        self.predictAltitude(dt)

    def predictAltitude(self, dt):
        accel = self.az_world - self.x_alt[2]
        self.x_alt[0] += self.x_alt[1] * dt + 0.5 * accel * dt * dt
        self.x_alt[1] += accel * dt

    def updateBmp(self, pressure, temperature):
        baroAlt = 44330.0 * (1.0 - math.pow(pressure / 101325.0, 0.1903))
        y = baroAlt - self.x_alt[0]
        
        alpha = 0.2
        beta_gain = 0.05
        gamma = 0.001
        
        self.x_alt[0] += alpha * y
        self.x_alt[1] += beta_gain * y
        self.x_alt[2] -= gamma * y

kf = KalmanFilter()
sim_alt = 0.0
sim_vz = 0.0
sim_az = 9.80665
dt = 0.01

for i in range(5000):
    t = i * dt
    if 1.0 < t < 5.0:
        sim_az = 9.80665 + 40.0
    elif 5.0 <= t < 15.0:
        sim_az = 0.0
    elif t >= 15.0:
        sim_az = 9.80665
        if sim_vz < -10.0:
            sim_az = 9.80665 + 10.0
            
    sim_vz += (sim_az - 9.80665) * dt
    sim_alt += sim_vz * dt
    if sim_alt < 0:
        sim_alt = 0
        sim_vz = 0
        
    pressure = 101325.0 * math.pow(1.0 - sim_alt / 44330.0, 5.255)
    
    kf.updateImu(0, 0, sim_az, 0, 0, 0, dt)
    
    if i % 4 == 0:
        kf.updateBmp(pressure, 25.0)
        
    if i % 100 == 0:
        print(f"t={t:.2f} | TRUE alt:{sim_alt:.2f} vz:{sim_vz:.2f} || KF alt:{kf.x_alt[0]:.2f} vz:{kf.x_alt[1]:.2f} bias:{kf.x_alt[2]:.2f}")
