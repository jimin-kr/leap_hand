from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 4000000
PROTOCOL = 2.0

portHandler = PortHandler(PORT)
packetHandler = PacketHandler(PROTOCOL)

if not portHandler.openPort():
    print("port open failed")
    raise SystemExit

if not portHandler.setBaudRate(BAUD):
    print("baudrate set failed")
    raise SystemExit

print("Scanning ID 0~15...")
found = []

for dxl_id in range(16):
    model_number, result, error = packetHandler.ping(portHandler, dxl_id)
    if result == COMM_SUCCESS and error == 0:
        print(f"ID {dxl_id}: OK, model={model_number}")
        found.append(dxl_id)
    else:
        print(f"ID {dxl_id}: NO RESPONSE / {packetHandler.getTxRxResult(result)} / error={error}")

portHandler.closePort()
print("FOUND:", found)
