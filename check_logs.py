import sys
sys.path.append("/Users/hekote/Documents/PlatformIO/Projects/P2026")
import monitor

def test():
    print("Testing build_event_string with EVT_SM_CURRENT_STATE (2)...")
    msg = monitor.build_event_string(2, 0.0, 0.0, 0.0)
    print("Result:", msg)
    if "Current State" not in msg:
        print("ERROR: build_event_string failed!")
        
test()
