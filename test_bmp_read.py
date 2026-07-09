import struct
def parse_raw(data):
    # data is [press_xlsb, press_lsb, press_msb, temp_xlsb, temp_lsb, temp_msb]
    raw_press = data[0] | (data[1] << 8) | (data[2] << 16)
    raw_temp = data[3] | (data[4] << 8) | (data[5] << 16)
    return raw_press, raw_temp

print(parse_raw([0x00, 0x12, 0x34, 0x56, 0x78, 0x9A]))
