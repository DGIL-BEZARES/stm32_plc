import serial
import time

PACKET_SIZE_LEN = 1
PACKET_DATA_LEN = 32
PACKET_CRC_LEN = 1
PACKET_TOT_LEN = PACKET_SIZE_LEN + PACKET_DATA_LEN + PACKET_CRC_LEN

PACKET_ACK_DATA0 = 0x15
PACKET_RETX_DATA0 = 0x19

class Packet():
    def __init__(self, length: int, data: list[int], crc: int | None) -> None:
        self.length = length
        self.data = data
        if crc is not None:
            self.crc = crc
        else:
            self.compute_crc()

    def compute_crc(self):
        self.crc = crc8_h2f([self.length] + self.data)

    def get_buffer(self):
        return ([self.length] + self.data + [self.crc])
    
class Cmd_Packet(Packet):
    def __init__(self, byte: int) -> None:
        length = 0x01
        data = [byte] + [0xff]*(PACKET_TOT_LEN - 3)
        crc = crc8_h2f([length] + data)
        super().__init__(length, data, crc)


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


#ack: list = [0x01, PACKET_ACK_DATA0] + [0xFF] * (PACKET_TOT_LEN - 3)
#ack.append(crc8_h2f(ack))
#
#retx: list = [0x01, PACKET_RETX_DATA0] + [0xFF] * (PACKET_TOT_LEN - 3)
#retx.append(crc8_h2f(retx))
#
#dummy: list = [0x01, 0xAB] + [0xFF] * (PACKET_TOT_LEN - 3)
#dummy.append(crc8_h2f(dummy) + 1)

while True:
    with serial.Serial("/dev/ttyACM0", 115200) as ser:
        # print("Waiting for bytes...")
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        # packet = ser.read(PACKET_TOT_LEN)

        # lst_byte = list(packet)

        # crc = crc8_h2f(lst_byte[:-1])

        # print(lst_byte[0])

        # if crc != lst_byte[-1]:
        #     print(f"CRC mismatch - Computed {crc:#02X} - got {lst_byte[-1]:#02X}")
        #     ser.write(bytes(retx))
        #
        # else:
        #     print(f"CRC is computed correctly: Got {lst_byte[-1]:#02X}, Computed {crc:#02X}")
        #     ser.write(bytes(ack))

        print(f"Writing bytes: {dummy}")
        ser.write(bytes(dummy))
        time.sleep(0.5)
