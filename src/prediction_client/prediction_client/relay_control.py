import serial
import time

class RelaySerialDriver():
    def __init__(self, port):
        self.windows = False
        # (38400Kbit/s, 8,N,1)
        baud = 9600
        self.ser = None

        self.ser = serial.Serial(port, baud, timeout=1)
        print("port open success: ", port)

        self.open_hex = "010500000000CDCA"
        self.close_hex = "01050000FF008C3A"

    def open(self):
        send_data = bytes.fromhex(self.open_hex)
        self.ser.write(send_data)

    def close(self):
        send_data = bytes.fromhex(self.close_hex)
        self.ser.write(send_data)

if __name__ == '__main__':
    srt = RelaySerialDriver("/dev/ttyUSB0")
    while 1:

        input_cmd = int(input("Type 1 for vacuum off, 0 for vacuum on: "))
        if input_cmd == 1:
            srt.open()
            time.sleep(1.0)
        elif input_cmd == 0:
            srt.close()
            time.sleep(1.0)
        else:
            print("wrong input, do retry")