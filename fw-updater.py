import serial
import time
import struct
from pathlib import Path

# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────
PORT = "/dev/ttyACM0"        # COMx on Windows
BAUDRATE = 115200
TIMEOUT = 0.1               # serial read timeout
SYNC_TIMEOUT = 5.0          # seconds
DEVICE_ID = 0x42

SYNC_SEQ = bytes([0xC4, 0x55, 0x7E, 0x10])
PACKET_DATA_LEN = 32

PACKET_ACK_DATA0 = 0x15
PACKET_RETX_DATA0 = 0x19

BOOTLOADER_SIZE = 0x4000


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
def compute_crc(data):
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


def read_packet(ser: serial.Serial):
    raw = list(ser.read(1 + PACKET_DATA_LEN + 1))

    if len(raw) != 34:

        return None
    
    crc = raw[-1]

    if compute_crc(raw[:-1]) != crc:
        print(f"CRC mismatch {Packet(raw[0], raw[1:-1], raw[-1])}")
        print("Requesting retransmit...")

        write_packet(ser, retx)
        return None
    else:
        write_packet(ser, ack)
    
    return Packet(raw[0], raw[1:-1], raw[-1])

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


# ─────────────────────────────────────────────
# SYNC
# ─────────────────────────────────────────────

#def send_sync_sequence(ser):
#    for b in (0xC4, 0x55, 0x7E, 0x10):
#        ser.write(bytes([b]))
#        ser.flush()          # force immediate TX
#        time.sleep(0.002)    # 2 ms gap (safe)

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
    firmware = list(Path(firmware_path).read_bytes()[BOOTLOADER_SIZE:])
    fw_len = len(firmware)
    print(f"Read file {firmware_path} [{fw_len} bytes]")

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

            print(f"{dataLength} bytes [{bytes_written}/{len(firmware)}] have been written")

        # ── DONE ───────────────────────────────
        print("← Waiting for success")
        expect_single_byte(ser, BL_PACKET_FW_UP_SCS_DATA0, "waiting for SUCCESS")

        print("✓ Firmware upload complete")


# ─────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python uploader.py firmware.bin")
        sys.exit(1)

    upload_firmware(PORT, sys.argv[1])
