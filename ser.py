import serial

PACKET_SIZE_LEN = 1
PACKET_DATA_LEN = 32
PACKET_CRC_LEN = 1
PACKET_TOT_LEN = PACKET_SIZE_LEN + PACKET_DATA_LEN + PACKET_CRC_LEN

PACKET_ACK_DATA0 = 0x15
PACKET_RETX_DATA0 = 0x19

def crc8_h2f(data):
    crc = 0xFF
    poly = 0x2F

    for byte in data:
        crc ^= byte  # XOR with data byte
        for _ in range(8):  # process each bit
            if crc & 0x80:  # MSB is 1
                crc = ((crc << 1)& 0xff) ^ poly  # shift left and XOR poly
            else:
                crc = ((crc << 1) & 0xff)  # just shift left

    return crc ^ 0xff




ack: list = [0x01, PACKET_ACK_DATA0] + [0xff]*(PACKET_TOT_LEN - 3)
ack.append(crc8_h2f(ack))

retx: list = [0x01, PACKET_RETX_DATA0] + [0xff]*(PACKET_TOT_LEN - 3)
retx.append(crc8_h2f(retx))

print(ack)

while True:

    with serial.Serial('/dev/ttyACM0',115200) as ser:
        print("Waiting for bytes...")
        length = ser.read(PACKET_SIZE_LEN)

        packet = ser.read(PACKET_TOT_LEN - PACKET_SIZE_LEN)

        lst_byte = list(length) + list(packet)

        crc = crc8_h2f(lst_byte[:-1])

        if crc != lst_byte[-1]:
            print(f"CRC mismatch - Computed {crc:#02X} - got {lst_byte[-1]:#02X}")
            ser.write(bytes(retx))
            
        else:
            print(f"CRC is computed correctly: Got {lst_byte[-1]:#02X}, Computed {crc:#02X}")
            ser.write(bytes(ack))

