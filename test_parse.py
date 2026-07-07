import serial
import struct
import time

try:
    ser = serial.Serial('/dev/cu.usbserial-110', 115200, timeout=1)
    print("Listening for LOG packets...")
    start = time.time()
    rf_buffer = bytearray()
    
    while time.time() - start < 10:
        line = ser.readline()
        try:
            line_str = line.decode('utf-8', errors='ignore')
            if "Simulated Radio TX Data:" in line_str:
                hex_str = line_str.split("Simulated Radio TX Data:")[1].strip()
                hex_list = hex_str.split()
                byte_data = bytes([int(x, 16) for x in hex_list])
                rf_buffer.extend(byte_data)
                
                while len(rf_buffer) >= 256:
                    chunk = rf_buffer[:256]
                    rf_buffer = rf_buffer[256:]
                    
                    offset = 0
                    while offset < 256:
                        if len(chunk) - offset < 8:
                            break
                        pkt_type, pad1, pad2, pad3, ts = struct.unpack_from('<BBBBT', chunk, offset) if struct.calcsize('T') == 4 else struct.unpack_from('<BBBBI', chunk, offset)
                        
                        if pkt_type == 4:
                            if len(chunk) - offset < 24: break
                            event_id, p1, p2, p3 = struct.unpack_from('<B3xfff', chunk, offset + 8)
                            print(f"FOUND LOG! Type 4, Ts={ts}, Evt={event_id}, P1={p1}")
                            offset += 24
                        elif pkt_type == 0:
                            offset += 1
                        elif pkt_type == 1: offset += 44
                        elif pkt_type == 2: offset += 16
                        elif pkt_type == 3: offset += 32
                        elif pkt_type == 10: offset += 24
                        elif pkt_type == 12: offset += 20
                        elif pkt_type == 13: offset += 20
                        else:
                            offset += 1
        except Exception as e:
            pass
except Exception as e:
    print(e)
