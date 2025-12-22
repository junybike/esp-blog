import serial
import struct
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

BIN_SOF = 0xAA
CMD_BRIGHT = 0x05

def crc(pkt):
    return pkt[0] ^ pkt[1] ^ pkt[2] ^ pkt[3]

def send(cmd, value=0):
    pkt = bytearray(5)
    pkt[0] = BIN_SOF
    pkt[1] = cmd
    pkt[2] = value & 0xFF
    pkt[3] = (value >> 8) & 0xFF
    pkt[4] = crc(pkt)

    print("Sending: ", pkt.hex())
    ser.write(pkt)

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    time.sleep(2)

    # Send

    # send(0x01)    # on
    # send(0x02)    # off
    send(0x03)    # slow
    # send(0x04)    # fast
    # send(0x05, 3000) # brightness

    # Receive

    resp = ser.read(5)

    if len(resp) == 5 and resp[0] == BIN_SOF:
        sof, cmd, val_l, val_h, crc = resp
        value = val_l | (val_h << 8)
        print("status: ", value)

    
