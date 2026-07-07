import serial
import time
try:
    ser = serial.Serial('/dev/cu.usbserial-110', 115200, timeout=1)
    print("Listening...")
    start = time.time()
    while time.time() - start < 3:
        line = ser.readline()
        if b"InterCoreComm" in line or b"LOG" in line:
            print(line.decode('utf-8', errors='ignore').strip())
except Exception as e:
    print(e)
