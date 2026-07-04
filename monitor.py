import sys
import subprocess

# stdout 強制不緩衝，確保 daemon thread 的輸出即時可見
sys.stdout.reconfigure(line_buffering=True)


import serial
import serial.tools.list_ports
import threading
import struct
import json
import asyncio
import websockets
import queue
import time
import math

serial_cmd_queue = queue.Queue()
BAUD_RATE = 115200

def choose_serial_port():
    # 若有帶入命令列參數，優先使用
    if len(sys.argv) > 1:
        return sys.argv[1]
        
    ports = serial.tools.list_ports.comports()
    if not ports:
        port = input("未偵測到任何 Serial Port，請手動輸入路徑 (例如 /dev/cu.usbserial-10): ").strip()
        return port if port else '/dev/cu.usbserial-10'
    
    print("\n可用的 Serial Ports:")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
        
    choice = input(f"\n請選擇 Serial Port [0-{len(ports)-1}]，或直接手動輸入路徑 (直接 Enter 預設選 [0]): ").strip()
    if choice == "":
        return ports[0].device
    
    if choice.isdigit() and 0 <= int(choice) < len(ports):
        return ports[int(choice)].device
        
    return choice

# WebSocket 廣播機制
ws_clients = set()
ws_loop    = None
_ser_global = None  # 由 main() 設定，供 ws_handler 轉發指令至 Serial

# STATENUM 合法範圍（對應 StateMachine.hpp enum）
_VALID_STATE_IDS = set(range(18))  # 0–17

async def ws_handler(websocket):
    """雙向 WebSocket handler：
    - 下行（FC→GND）：由 broadcast_ws 主動推送感測資料
    - 上行（GND→FC）：接收地面端指令並轉發至 Serial
    """
    ws_clients.add(websocket)
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
                if msg.get("type") == "cmd" and msg.get("action") == "setState":
                    state_id = int(msg["stateId"])
                    if state_id not in _VALID_STATE_IDS:
                        print(f"[WS→Serial] ❌ 非法狀態碼：{state_id}", flush=True)
                        continue
                    if _ser_global and _ser_global.is_open:
                        serial_cmd_queue.put(f"{state_id}\n".encode('utf-8'))
                        print(f"[WS→Serial] 📡 切換指令發送：State {state_id}", flush=True)
                    else:
                        print("[WS→Serial] ⚠️ Serial port 未開啟，指令丟棄", flush=True)
            except Exception as e:
                print(f"[WS CMD] ❌ 解析或處理錯誤：{e}", flush=True)
    finally:
        ws_clients.discard(websocket)

def clean_nans(obj):
    if isinstance(obj, dict):
        return {k: clean_nans(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [clean_nans(v) for v in obj]
    elif isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return None
        return obj
    return obj

async def broadcast_ws(batch_data):
    if ws_clients:
        clean_batch = clean_nans(batch_data)
        msg = json.dumps({"batch": clean_batch, "status": "0-0", "pkt": len(clean_batch)})
        await asyncio.gather(*[client.send(msg) for client in ws_clients])

async def _ws_main():
    global ws_loop
    ws_loop = asyncio.get_running_loop()
    async with websockets.serve(ws_handler, "localhost", 8765):
        await asyncio.Future()  # 永久等待，直到 thread 結束

def ws_server_thread():
    asyncio.run(_ws_main())

def parse_rf_buffer(byte_data):
    """
    解析 ESP32 傳來的混合資料 Buffer (256 bytes 以內)。
    資料結構會依照 InterCoreComm.hpp 的定義。
    """
    hex_dump = " ".join([f"{b:02X}" for b in byte_data])
    print(f"  [RAW HEX] {hex_dump}")
    
    batch_data = []
    offset = 0
    parsed_count = 0
    while offset < len(byte_data):
        # 至少要能讀取 8 bytes 的標頭
        if len(byte_data) - offset < 8:
            # print(f"  [Debug] Buffer 剩餘 {len(byte_data) - offset} bytes，不足以讀取標頭，結束解析。")
            break 
        
        # 讀取標頭：type (1 byte), padding (3 bytes), timestamp (4 bytes uint32)
        pkt_type, timestamp = struct.unpack_from('<B3xI', byte_data, offset)
        
        if pkt_type == 0: # DATA_TYPE_IMU
            if len(byte_data) - offset < 44: 
                print(f"  [Debug] IMU 封包長度不足 (需要 44, 剩餘 {len(byte_data) - offset})")
                break
            ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack_from('<9f', byte_data, offset + 8)
            print(f"  🔹 [IMU] {timestamp}ms: Acc=({ax:.2f}, {ay:.2f}, {az:.2f}) Gyro=({gx:.2f}, {gy:.2f}, {gz:.2f}) Mag=({mx:.2f}, {my:.2f}, {mz:.2f})")
            batch_data.append({"type": "IMU", "ts": timestamp, "data": {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz, "mx": mx, "my": my, "mz": mz}})
            offset += 44
            parsed_count += 1
            
        elif pkt_type == 1: # DATA_TYPE_BMP
            if len(byte_data) - offset < 16: break
            pressure, temp = struct.unpack_from('<2f', byte_data, offset + 8)
            print(f"  🔹 [BMP] {timestamp}ms: Press={pressure:.2f}hPa, Temp={temp:.2f}C")
            batch_data.append({"type": "BMP", "ts": timestamp, "data": {"pressure": pressure, "temp": temp}})
            offset += 16
            parsed_count += 1
            
        elif pkt_type == 2: # DATA_TYPE_GPS
            # GPS 結構因 double 對齊 8 bytes，加上 padding 後大小為 32 bytes
            if len(byte_data) - offset < 32: break
            lat, lon, alt = struct.unpack_from('<ddf', byte_data, offset + 8)
            print(f"  🔹 [GPS] {timestamp}ms: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m")
            batch_data.append({"type": "GPS", "ts": timestamp, "data": {"lat": lat, "lon": lon, "alt": alt}})
            offset += 32
            parsed_count += 1
            
        elif pkt_type == 3: # DATA_TYPE_LOG
            if len(byte_data) - offset < 72: break
            msg = struct.unpack_from('<64s', byte_data, offset + 8)[0].decode('utf-8', errors='ignore').strip('\x00')
            print(f"  📝 [LOG] {timestamp}ms: {msg}")
            batch_data.append({"type": "LOG", "ts": timestamp, "data": {"msg": msg}})
            offset += 72
            parsed_count += 1
            
        elif pkt_type == 10: # KALMAN_TYPE_QUATERNION
            if len(byte_data) - offset < 24: 
                print(f"  [Debug] QUAT 封包長度不足 (需要 24, 剩餘 {len(byte_data) - offset})")
                break
            qw, qx, qy, qz = struct.unpack_from('<4f', byte_data, offset + 8)
            print(f"  🚀 [QUAT] {timestamp}ms: w={qw:.3f}, x={qx:.3f}, y={qy:.3f}, z={qz:.3f}")
            batch_data.append({"type": "KALMAN_QUATERNION", "ts": timestamp, "data": {"q": [qw, qx, qy, qz]}})
            offset += 24
            parsed_count += 1
            
        elif pkt_type == 11: # KALMAN_TYPE_GPS
            if len(byte_data) - offset < 32: break
            lat, lon, velN, velE = struct.unpack_from('<ddff', byte_data, offset + 8)
            print(f"  🚀 [KF_GPS] {timestamp}ms: Lat={lat:.6f}, Lon={lon:.6f}, vN={velN:.2f}, vE={velE:.2f}")
            batch_data.append({"type": "KALMAN_GPS", "ts": timestamp, "data": {"lat": lat, "lon": lon, "vN": velN, "vE": velE}})
            offset += 32
            parsed_count += 1
            
        elif pkt_type == 12: # KALMAN_TYPE_ALTITUDE
            if len(byte_data) - offset < 20: break
            alt, vz, az = struct.unpack_from('<3f', byte_data, offset + 8)
            print(f"  🚀 [KF_ALT] {timestamp}ms: Alt={alt:.2f}m, Vz={vz:.2f}m/s, Az={az:.2f}m/s²")
            batch_data.append({"type": "KALMAN_ALTITUDE", "ts": timestamp, "data": {"alt": alt, "vz": vz, "az": az}})
            offset += 20
            parsed_count += 1
            
        elif pkt_type == 0x00: # Padding from empty buffer
            offset += 1
            
        else:
            print(f"  ❌ [UNKNOWN] Type ID: {pkt_type} at offset {offset}")
            break # 格式未知的錯誤，停止解析這段 Buffer
    
    if parsed_count > 0:
        print(f"  ✅ 解析完成：共 {parsed_count} 筆資料封包\n")
        if batch_data and ws_loop:
            asyncio.run_coroutine_threadsafe(broadcast_ws(batch_data), ws_loop)


def read_from_port(ser):
    while True:
        try:
            # 優先處理待發送指令 (保證單一執行緒存取 Serial)
            while not serial_cmd_queue.empty():
                cmd = serial_cmd_queue.get_nowait()
                ser.write(cmd)

            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if "Simulated Radio TX Data:" in line:
                    print(f"\n[📦 收到 RF 緩衝區封包]")
                    try:
                        hex_str = line.split("Simulated Radio TX Data:")[1].strip()
                        hex_list = hex_str.split()
                        byte_data = bytes([int(x, 16) for x in hex_list])
                        
                        # 呼叫解析函式
                        parse_rf_buffer(byte_data)
                    except Exception as e:
                        print(f"  -> [解析錯誤] {e}")
                else:
                    print(line)
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Serial error: {e}")
            break

def write_to_port(ser):
    from prompt_toolkit import prompt
    from prompt_toolkit.patch_stdout import patch_stdout

    print("=======================================")
    print("輸入 'T' 發送強制終止指令")
    print("輸入 'exit' 離開程式")
    print("=======================================\n")
    
    with patch_stdout():
        while True:
            try:
                user_input = prompt("指令 > ")
                if user_input.lower() == 'exit':
                    print("Exiting...")
                    ser.close()
                    sys.exit(0)
                
                if user_input.upper() == 'T':
                    print(">> [傳送] Force Terminate Command ('T')")
                    serial_cmd_queue.put(b'T\n')
                elif user_input.strip() != "":
                    serial_cmd_queue.put((user_input + '\n').encode('utf-8'))
            except KeyboardInterrupt:
                print("\nExiting...")
                ser.close()
                sys.exit(0)
            except EOFError:
                time.sleep(1)
                continue
            except Exception as e:
                print(f"Input error: {e}")
                break

def main():
    global _ser_global
    serial_port = choose_serial_port()
    try:
        ser = serial.Serial(serial_port, BAUD_RATE, timeout=1)
        import time
        ser.setDTR(False)
        ser.setRTS(True)
        time.sleep(0.1)
        ser.setRTS(False)
        time.sleep(0.1)
        _ser_global = ser  # 讓 ws_handler 可存取 Serial port
        print(f"成功連接 {serial_port} @ {BAUD_RATE} baud.")
        
        # 啟動 WebSocket 伺服器
        threading.Thread(target=ws_server_thread, daemon=True).start()
        print("WebSocket Server started at ws://localhost:8765")
        
        read_thread = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
        read_thread.start()
        
        write_to_port(ser)
    except serial.SerialException as e:
        print(f"無法打開 Serial Port {serial_port}: {e}")
        print("請確認開發板已連接，或輸入正確的 Serial Port 路徑。")

if __name__ == "__main__":
    main()
