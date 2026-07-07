import sys
sys.path.append("/Users/hekote/Documents/PlatformIO/Projects/P2026")
import monitor
import struct

def make_log_packet(ts, event_id, p1, p2, p3):
    return struct.pack('<B3xI B3xfff', 4, ts, event_id, p1, p2, p3)

buf1 = make_log_packet(1000, 1, 0, 1, 0) + bytes(256 - 24)

print("Testing LOG buf1:")
monitor.parse_rf_buffer(buf1)
