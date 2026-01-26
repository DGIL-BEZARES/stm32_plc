import serial
import time
import struct
from pathlib import Path
from binascii import crc32

# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────
PORT = "/dev/ttyACM0"        # COMx on Windows
BAUDRATE = 115200
TIMEOUT = 0.1               # serial read timeout
SYNC_TIMEOUT = 5.0          # seconds

SYNC_SEQ = bytes([0xC4, 0x55, 0x7E, 0x10])
PACKET_DATA_LEN = 32

PACKET_ACK_DATA0 = 0x15
PACKET_RETX_DATA0 = 0x19


BOOTLOADER_SIZE = 0x4000
VECTOR_TABLE_SIZE = 0X150
FWINFO_SIZE = (10 * 4)

FWINFO_SENTINEL = 0xDEADBEEF
FWINFO_VALIDATE_FROM = (VECTOR_TABLE_SIZE + FWINFO_SIZE)

FWINFO_DEVICE_ID_OFFSET = VECTOR_TABLE_SIZE + (1 * 4)
FWINFO_VERSION_OFFSET = VECTOR_TABLE_SIZE + (2 * 4)
FWINFO_LENGTH_OFFSET = VECTOR_TABLE_SIZE + (3 * 4)
FWINFO_CRC32_OFFSET = VECTOR_TABLE_SIZE + (4 * 4)


BL_PACKET_SYNC_OBS_DATA0   = 0x20
BL_PACKET_FW_UP_REQ_DATA0  = 0x31
BL_PACKET_FW_UP_RES_DATA0  = 0X37
BL_PACKET_DEV_ID_REQ_DATA0 = 0x3C
BL_PACKET_DEV_ID_RES_DATA0 = 0x3F
BL_PACKET_FW_LEN_REQ_DATA0 = 0x42
BL_PACKET_FW_LEN_RES_DATA0 = 0x45
BL_PACKET_RDY_DATA_DATA0   = 0x48
BL_PACKET_FW_UP_SCS_DATA0  = 0x54
BL_PACKET_NACK_DATA0       = 0x58

class Packet():
    def __init__(self, length: int, data: list[int], crc: int | None = None) -> None:
        self.length = length
        self.data = data
        
        bytes_to_pad = PACKET_DATA_LEN - len(self.data)
        padding = [0xff] * bytes_to_pad
        self.data += padding

        if crc is not None:
            self.crc = crc
        else:
            self.crc = self.compute_crc()

    def __eq__(self, other) -> bool:
        if not isinstance(other, Packet):
            return NotImplemented
        
        return self.length == other.length and self.crc == other.crc and self.data[0] == other.data[0]
    
    def __str__(self) -> str:
        data_hex = " ".join(f"{b:02x}" for b in self.data)
        return (
            f"Packet(length={self.length:02x}, "
            f"data={data_hex}, "
            f"crc=0x{self.crc:02x})"
        )
    
    def __repr__(self) -> str:
        return (
            f"Packet(length={self.length}, "
            f"data={self.data}, "
            f"crc={self.crc})"
        )
    
    def compute_crc(self) -> int:
        crc = 0xFF
        poly = 0x2F
        all_data = [self.length] + self.data

        for byte in all_data:
            crc ^= byte  # XOR with data byte
            for _ in range(8):  # process each bit
                if crc & 0x80:  # MSB is 1
                    crc = ((crc << 1) & 0xFF) ^ poly  # shift left and XOR poly
                else:
                    crc = (crc << 1) & 0xFF  # just shift left
        return crc ^ 0xFF
    
    def to_buffer(self) -> bytes:
        return bytes( [self.length] + self.data + [self.crc] )
    
    def is_retx(self):
        return self == Packet(1, [PACKET_RETX_DATA0])
    
    def is_ack(self):
        return self == Packet(1, [PACKET_ACK_DATA0])

last_transmitted: Packet
retx = Packet(1, [PACKET_RETX_DATA0])
ack = Packet(1, [PACKET_ACK_DATA0])

# ─────────────────────────────────────────────
# COMMS HELPERS
# ─────────────────────────────────────────────


def read_packet(ser: serial.Serial):
    raw = list(ser.read(1 + PACKET_DATA_LEN + 1))

    if len(raw) != 34:

        return None
    
    crc = raw[-1]

    pkt = Packet(raw[0],raw[1:-1])

    if pkt.crc != crc:
        print(f"CRC mismatch {hex(crc)} {hex(pkt.crc)}")
        print("Requesting retransmit...")

        write_packet(ser, retx)
        return None
    else:
        write_packet(ser, ack)
    
    return pkt

def write_packet(ser: serial.Serial, packet: Packet):
    global last_transmitted
    ser.write(packet.to_buffer())
    last_transmitted = packet



def expect_single_byte(ser: serial.Serial, expected: int, desc=""):
    while True:
        pkt = read_packet(ser)
        if pkt is None:
            continue

        if pkt.is_ack():
            continue
        
        if pkt.is_retx():
            print("Received an RETX. Retransmitting...")
            write_packet(ser, last_transmitted)
        
        if pkt.length == 1 and pkt.data[0] == expected:
            return

        if pkt.length == 1 and pkt.data[0] == BL_PACKET_NACK_DATA0:
            raise RuntimeError("Received NACK")


def insert_into_list(src: list[int], data: int, idx: int) -> list[int]:
    data_byte_list = list(struct.pack("<I",data))
    return src[:idx] + data_byte_list + src[(idx + len(data_byte_list)):]

def stm32_crc32(data: list[int]) -> int:
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7
    
    for word in data:

        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF

    return crc 

# ─────────────────────────────────────────────
# SYNC
# ─────────────────────────────────────────────

def perform_sync(ser: serial.Serial):
    print("→ Synchronizing with bootloader...")
    start = time.monotonic()
    while True:
        ser.write(SYNC_SEQ)
        time.sleep(0.5)

        pkt = read_packet(ser)
        if pkt and pkt.length == 1 and pkt.data[0] == BL_PACKET_SYNC_OBS_DATA0:
            return

        if time.monotonic() - start > SYNC_TIMEOUT:
            raise TimeoutError("Sync timeout")


# ─────────────────────────────────────────────
# MAIN UPLOAD LOGIC
# ─────────────────────────────────────────────
def upload_firmware(port, firmware_path):
    binary = Path(firmware_path).read_bytes()
    firmware = list(binary[BOOTLOADER_SIZE:])
    fw_len = len(firmware)
    print(f"Read file {firmware_path} [{fw_len} bytes]")

    print("Injecting into firmware information section")

    firmware = insert_into_list(firmware, fw_len, FWINFO_LENGTH_OFFSET)
    firmware = insert_into_list(firmware, 0x00000001, FWINFO_VERSION_OFFSET)


    # Convert the list of bytes to little endian 32bit bytes in order for the CRC32 algorithm to work
    fw_le = []

    for i in range(0x178,len(firmware),4):
        fw_le.append(int.from_bytes(firmware[i:i+4],byteorder='little'))

    crc32_val = stm32_crc32(fw_le)
    print(f"Computed CRC32: {hex(crc32_val)}")

    firmware = insert_into_list(firmware, crc32_val, FWINFO_CRC32_OFFSET)

    Path("app/firmware_out.bin").write_bytes(bytes(firmware))
    print(f"Output binary has {len(Path("app/firmware_out.bin").read_bytes())} bytes")


    time.sleep(1)

    with serial.Serial(port, BAUDRATE, timeout=TIMEOUT) as ser:

        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        time.sleep(1)

        # ── SYNC ───────────────────────────────
        perform_sync(ser)

        # ── FW UPDATE REQUEST ──────────────────
        print("→ Firmware update request")
        write_packet(ser,Packet(1, [BL_PACKET_FW_UP_REQ_DATA0]))

        # ── FW UPDATE RESPONSE ──────────────────
        expect_single_byte(ser, BL_PACKET_FW_UP_RES_DATA0, "Waiting for FW_UP_RES")

        # ── DEVICE ID EXCHANGE ─────────────────
        print("← Device ID request")
        expect_single_byte(ser, BL_PACKET_DEV_ID_REQ_DATA0, "waiting for DEV_ID_REQ")

        DEVICE_ID = firmware.copy().pop(FWINFO_DEVICE_ID_OFFSET)

        print("→ Sending Device ID")
        write_packet(ser, Packet(2,[BL_PACKET_DEV_ID_RES_DATA0, DEVICE_ID]))

        # ── FW LENGTH ──────────────────────────
        print("← Firmware length request")
        expect_single_byte(ser, BL_PACKET_FW_LEN_REQ_DATA0, "waiting for FW_LEN_REQ")

        print(f"→ Sending firmware length ({fw_len} bytes)")
        write_packet(ser,Packet(5, [BL_PACKET_FW_LEN_RES_DATA0] + list(struct.pack("<I", fw_len))) )

        print("Waiting for flash erasure")
        time.sleep(3)


        # ── ERASE COMPLETE / READY ─────────────
        print("← Waiting for READY")
        expect_single_byte(ser, BL_PACKET_RDY_DATA_DATA0, "waiting for READY")

        # ── SEND FIRMWARE DATA ─────────────────
        print("→ Sending firmware data")
        bytes_written = 0

        while bytes_written < fw_len:

            # expect_single_byte(ser, BL_PACKET_RDY_DATA_DATA0, "waiting for READY")

            dataBytes = firmware[bytes_written:bytes_written + PACKET_DATA_LEN]
            dataLength = len(dataBytes)
            dataPacket = Packet(dataLength - 1, dataBytes)

            write_packet(ser, dataPacket)
            bytes_written += dataLength

            if bytes_written < fw_len:
                expect_single_byte(
                    ser,
                    BL_PACKET_RDY_DATA_DATA0,
                    "waiting for READY (next chunk)"
                )

            print(f"{dataLength:>2} bytes [{bytes_written:>{len(str(len(firmware)))}}/{len(firmware)}] written")

        # ── DONE ───────────────────────────────
        print("← Waiting for success")
        expect_single_byte(ser, BL_PACKET_FW_UP_SCS_DATA0, "waiting for SUCCESS")

        print("✓ Firmware upload complete")


# ─────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python fw-updater.py firmware.bin")
        sys.exit(1)

    upload_firmware(PORT, sys.argv[1])
