import struct
from monitor import parse_rf_buffer
import monitor

# Create a mock packet for EVT_SYS_RESET (4)
# Header: type=4, ts=1000
header = struct.pack('<B3xI', 4, 1000)
# Data: event_id=4, p1,p2,p3=0
data = struct.pack('<B3xfff', 4, 0.0, 0.0, 0.0)

packet = header + data
print("Packet length:", len(packet))

monitor.rf_buffer.extend(packet)
monitor.batch_data = []

def mock_broadcast(batch):
    print("BROADCAST CALLED WITH:", batch)

monitor.ws_loop = None # bypass async
monitor.broadcast_ws = mock_broadcast

monitor.parse_rf_buffer()

