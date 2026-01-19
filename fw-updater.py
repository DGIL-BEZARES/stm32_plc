from serial import Serial

PACKET_SIZE_LEN = 1
PACKET_DATA_LEN = 32
PACKET_CRC_LEN = 1
PACKET_TOT_LEN = PACKET_SIZE_LEN + PACKET_DATA_LEN + PACKET_CRC_LEN

PACKET_ACK_DATA0 = 0x15
PACKET_RETX_DATA0 = 0x19

BL_PACKET_SYNC_OBS_DATA0 = 0X20
BL_PACKET_FW_UP_REQ_DATA0 = 0X31
BL_PACKET_FW_UP_RES_DATA0 = 0X37
BL_PACKET_DEV_ID_REQ_DATA0 = 0X3C
BL_PACKET_DEV_ID_RES_DATA0 = 0X3F
BL_PACKET_FW_LEN_REQ_DATA0 = 0X42
BL_PACKET_FW_LEN_RES_DATA0 = 0X45
BL_PACKET_RDY_DATA_DATA0 = 0X48
BL_PACKET_FW_UP_SCS_DATA0 = 0X54
BL_PACKET_NACK_DATA0 = 0X58

DEVICE_ID = 0X42

SYNC_SEQ0 = 0XC4
SYNC_SEQ1 = 0X55
SYNC_SEQ2 = 0X7E
SYNC_SEQ3 = 0X10

SYNC_SEQ = [SYNC_SEQ0, SYNC_SEQ1, SYNC_SEQ2, SYNC_SEQ3]

DEFAULT_TIMEOUT = 5000

serialPort = '/dev/ttyACM0'
baudRate = 115200

def crc8_h2f(data: list[int]):
    crc = 0xFF
    poly = 0x2F

    for byte in data:
        crc ^= byte  # XOR with data byte
        for _ in range(8):  # process each bit
            if crc & 0x80:  # MSB is 1
                crc = ((crc << 1) & 0xFF) ^ poly  # shift left and XOR poly
            else:
                crc = (crc << 1) & 0xFF  # just shift left

    return crc ^ 0xFF

def main():
    pass