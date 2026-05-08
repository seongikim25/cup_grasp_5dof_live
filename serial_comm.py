import time
import serial
from config import CFG

class SerialComm:
    def __init__(self):
        self.ser = None
        if CFG.RUN_MODE:
            self.ser = serial.Serial(CFG.SERIAL_PORT, CFG.SERIAL_BAUD, timeout=1)
            time.sleep(2)
            print(f"[Serial] connected: {CFG.SERIAL_PORT}")
        else:
            print("[Serial] DEV mode: no hardware command will be sent")

    def send_positions(self, positions):
        if len(positions) != 5:
            raise ValueError("5DOF command requires exactly 5 encoder values")

        cmd = "0," + ",".join(str(int(p)) for p in positions) + "*"

        if self.ser is None:
            print(f"[TX simulated] {cmd}")
            return

        self.ser.write(cmd.encode("utf-8"))
        print(f"[TX] {cmd}")

    def torque(self, on: bool):
        cmd = f"1,{1 if on else 0}*"
        if self.ser is None:
            print(f"[TX simulated] {cmd}")
            return
        self.ser.write(cmd.encode("utf-8"))

    def close(self):
        if self.ser is not None:
            self.ser.close()
