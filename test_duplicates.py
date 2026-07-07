import sys
sys.path.append("/Users/hekote/Documents/PlatformIO/Projects/P2026")
import monitor
import struct

def make_imu_packet(ts, ax=1.0):
    return struct.pack('<B3xI 9f', 1, ts, ax, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)

buf1 = make_imu_packet(1000, 1.1) + bytes(256 - 44)

print("Testing buf1:")
monitor.parse_rf_buffer(buf1)
