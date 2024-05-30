import serial
import threading
import re

class SerialLineBuffer:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.buffer = b''
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.reading_thread)


    def start(self):
        self.thread.start()


    def readline_newest(self):
        with self.lock:
            tmp = self.buffer
            # self.buffer = []
            return tmp
        

    def reading_thread(self):
        while True:
            line = self.ser.readline()
            with self.lock:
                self.buffer = line


    def close(self):
        self.ser.close()


def bytes_to_doubles(raw_bytes: bytes) -> list:
    try:
        data_str = raw_bytes.decode()
        data = re.findall(r'-?\d+\.\d+', data_str)
        data = list([float(num) for num in data])
        return data
    except:
        return []
