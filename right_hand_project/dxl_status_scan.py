import glob
import sys
from dynamixel_sdk import *

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 4000000
PROTOCOL = 2.0

def auto_detect_port():
    if len(sys.argv) > 1 and sys.argv[1].startswith("/dev/"):
        return sys.argv[1]
    pkt = PacketHandler(PROTOCOL)
    for p in sorted(glob.glob("/dev/ttyUSB*")):
        try:
            ph = PortHandler(p)
            if ph.openPort():
                if ph.setBaudRate(BAUD):
                    _, res, _ = pkt.ping(ph, 0)
                    ph.closePort()
                    if res == COMM_SUCCESS:
                        return p
                else:
                    ph.closePort()
        except Exception:
            pass
    return DEFAULT_PORT

PORT = auto_detect_port()
print(f"Connecting to port: {PORT}")

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HW_ERROR = 70
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_TEMP = 146

port = PortHandler(PORT)
pkt = PacketHandler(PROTOCOL)

if not port.openPort():
    print(f"port open failed: {PORT}")
    raise SystemExit

if not port.setBaudRate(BAUD):
    print(f"baudrate set failed: {BAUD}")
    raise SystemExit


print("ID | op_mode | torque | hw_error | current | position (deg) | voltage | temp")
print("-" * 80)

for dxl_id in range(16):
    op_mode, r0, e0 = pkt.read1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE)
    torque, r1, e1 = pkt.read1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE)
    hw, r2, e2 = pkt.read1ByteTxRx(port, dxl_id, ADDR_HW_ERROR)
    cur, r3, e3 = pkt.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_CURRENT)
    pos, r4, e4 = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    volt, r5, e5 = pkt.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_VOLTAGE)
    temp, r6, e6 = pkt.read1ByteTxRx(port, dxl_id, ADDR_PRESENT_TEMP)

    if r1 != COMM_SUCCESS:
        print(f"ID {dxl_id:2d}: NO RESPONSE")
        continue

    if cur > 32767:
        cur -= 65536

    deg = (pos % 4096) / 4096.0 * 360.0
    print(f"{dxl_id:2d} |    {op_mode:3d} | {torque:6d} | {hw:8d} | {cur:7d} | {pos:5d} ({deg:5.1f}°) | {volt/10:6.1f}V | {temp:3d}C")

port.closePort()
