import serial
import time

try:
    ser = serial.Serial('/dev/cu.usbserial-110', 115200, timeout=1)
    print("Listening to serial port...")
    start_time = time.time()
    
    while time.time() - start_time < 10:
        line = ser.readline()
        if b"Simulated Radio TX Data:" in line:
            hex_str = line.split(b"Simulated Radio TX Data:")[1].strip().decode('ascii', errors='ignore')
            hex_list = hex_str.split()
            print("\n--- HEX DUMP ---")
            print(" ".join(hex_list[:32]) + " ... (length: " + str(len(hex_list)) + ")")
            
            # 尋找 Type 4 (LOG) 或 Type 3 (LOG old)
            found = False
            for i in range(len(hex_list)):
                if hex_list[i] == '04' or hex_list[i] == '03':
                    print(f"Found Type {hex_list[i]} at index {i}!")
                    if i + 24 <= len(hex_list):
                        print("Payload: " + " ".join(hex_list[i:i+24]))
                    found = True
            
            if not found:
                print("No LOG packet (04 or 03) found in this chunk.")
except Exception as e:
    print(f"Error: {e}")
