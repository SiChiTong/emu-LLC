import serial
import time
from colorama import init
from termcolor import cprint 
import sys
import binascii
init(strip=not sys.stdout.isatty())

class Emuart:
    def __init__(self, port, baudrate = 1000000, device = "Emulator"):
        try:
            self.device = device
            self.ser = serial.Serial(port = port, baudrate = baudrate)
            a = ("{} is connected via {}".format(self.device, self.ser.portstr))
            cprint (' ','white','on_green',end = ' ')
            cprint (a,'green')
            self.status = 1
        except:
            cprint (' ','white','on_red',end = ' ')
            a = ("{} is not connected".format(self.device))
            cprint (a,'red')
            self.status = 0

    def __str__(self):
        return self.device

    def flush(self):
        self.ser.reset_input_buffer()

    def flush_out(self):
        self.ser.reset_output_buffer()

    def rts(self, state):
        self.ser.rts = state
        
    def cts(self, state):
        self.ser.cts = state

    def close(self):
        self.ser.close()

    def read(self):
        header = self.ser.read(4)
        if header[0] == 0x7E and header[1] == 0x77:
            command = header[2]
            dataLen = header[3]
            data = self.ser.read(dataLen)
            expCrc32 = binascii.crc32(bytes(data+[command]))
            crc32 = list(self.ser.read(4))
            crc32 = (crc32[0]<<24)|(crc32[1]<<16)|(crc32[2]<<8)|crc32[3]
        return int(command), list(data)

    def write(self, command, data):    
        header = [0x7E, 0x77, command, len(data)]
        crc32 = binascii.crc32(bytes(data+[command]))
        crcSep = [(crc32>>24)&0xFF, (crc32>>16)&0xFF, (crc32>>8)&0xFF, crc32&0xFF]
        data = header+data+crcSep
        self.ser.write(bytes(data))





if __name__ == "__main__":
    et = Emuart('COM7')
    et.write(0x94,[0x01])


