from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 4000000
PROTOCOL = 2.0
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

portHandler = PortHandler(PORT)
packetHandler = PacketHandler(PROTOCOL)

if not portHandler.openPort():
    print("port open failed")
    raise SystemExit

if not portHandler.setBaudRate(BAUD):
    print("baudrate set failed")
    raise SystemExit

print("Reading present position ID 0~15...")
ok = []

for dxl_id in range(16):
    pos, result, error = packetHandler.read4ByteTxRx(
        portHandler, dxl_id, ADDR_PRESENT_POSITION
    )
    if result == COMM_SUCCESS and error == 0:
        print(f"ID {dxl_id}: OK, position={pos}")
        ok.append(dxl_id)
    else:
        print(f"ID {dxl_id}: FAIL / {packetHandler.getTxRxResult(result)} / error={error}")

portHandler.closePort()
print("READ OK:", ok)
