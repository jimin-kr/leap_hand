#!/usr/bin/env python3
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = "/dev/ttyUSB0"
BAUD = 4000000
PROTOCOL = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_HW_ERROR = 70
ADDR_PRESENT_VOLTAGE = 144

def main():
    p = PortHandler(PORT)
    k = PacketHandler(PROTOCOL)

    if not p.openPort():
        print(f"[ERROR] 포트 열기 실패: {PORT}")
        return

    if not p.setBaudRate(BAUD):
        print(f"[ERROR] baudrate 설정 실패: {BAUD}")
        p.closePort()
        return

    print("=== EMERGENCY ALL TORQUE OFF & REBOOT IF HW ERROR ===")
    
    # 1. 모든 모터에 무조건 Torque OFF 전송
    for dxl_id in range(16):
        k.write1ByteTxRx(p, dxl_id, ADDR_TORQUE_ENABLE, 0)

    time.sleep(0.1)

    # 2. 상태 점검 및 하드웨어 에러 발생 모터 자동 리부트
    for dxl_id in range(16):
        hw, r_hw, _ = k.read1ByteTxRx(p, dxl_id, ADDR_HW_ERROR)
        torque, r_tq, _ = k.read1ByteTxRx(p, dxl_id, ADDR_TORQUE_ENABLE)
        volt, r_v, _ = k.read2ByteTxRx(p, dxl_id, ADDR_PRESENT_VOLTAGE)

        if r_tq == COMM_SUCCESS:
            v_str = f"{volt/10:.1f}V" if r_v == COMM_SUCCESS else "N/A"
            if hw != 0:
                print(f"ID {dxl_id:2d}: torque={torque}, volt={v_str}, HW ERROR=0x{hw:02X} -> Rebooting...")
                k.reboot(p, dxl_id)
                time.sleep(0.3)
                k.write1ByteTxRx(p, dxl_id, ADDR_TORQUE_ENABLE, 0)
            else:
                print(f"ID {dxl_id:2d}: torque={torque}, volt={v_str}, HW ERROR=0x00 [OK]")
        else:
            print(f"ID {dxl_id:2d}: NO RESPONSE")

    p.closePort()
    print("=== ALL MOTORS ARE NOW SAFE (TORQUE OFF) ===")

if __name__ == "__main__":
    main()
