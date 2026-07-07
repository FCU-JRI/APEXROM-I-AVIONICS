import sys
sys.path.append("/Users/hekote/Documents/PlatformIO/Projects/P2026")
from monitor import build_event_string

for i in range(1, 25):
    try:
        s = build_event_string(i, 0.0, 0.0, 0.0)
        print(f"Event {i}: {s}")
    except Exception as e:
        print(f"Event {i} CRASHED: {e}")

