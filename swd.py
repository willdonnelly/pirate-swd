import serial
import time
import sys

class BusPirate:
    def __init__ (self, f = "/dev/bus_pirate"):
        self.port = serial.Serial(port = f, baudrate = 115200, timeout = 0.01)
        self.reset()

    def reset (self):
        self.port.write(bytearray([0x0F]))
        while self.port.read(5) != "BBIO1":
            self.port.write(bytearray([0x00]))
            time.sleep(0.01)
        self.port.write(bytearray([0x05]))
        if self.port.read(4) != "RAW1":
            print("error initializing bus pirate")
            sys.exit(1)
        self.port.write(bytearray([0x63,0x88]))
        self.port.read(9999)

    def send (self, data):
        for byte in data:
            self.port.write(bytearray([0x10,byte]))
            self.port.read(2)

    def bits (self, count):
        self.port.write(bytearray([0x07] * count))
        return [self.port.read(1) for x in range(count)]

    def bytes (self, count):
        self.port.write(bytearray([0x06] * count))
        return [self.port.read(1) for x in range(count)]

def bitCount(int_type):
    count = 0
    while(int_type):
        int_type &= int_type - 1
        count += 1
    return(count)

def calcOpcode (ap=False, register=0x00, read=True):
    opcode = 0x00
    opcode = opcode | (0x20 if read else 0x00)
    opcode = opcode | (0x40 if ap else 0x00)
    opcode = opcode | ((register & 0x03) << 3)
    opcode = opcode | ((bitCount(opcode) & 1) << 2)
    opcode = opcode | 0x81
    return opcode

class SWDP:
    def __init__ (self, f = "/dev/bus_pirate"):
        self.pirate = BusPirate(f)
        self.pirate.send([0xFF] * 8)
        self.pirate.send([0x79,0xE7])
        self.resync()

    def resync (self):
        self.pirate.send([0xFF] * 8)
        self.pirate.send([0x00] * 8)

    def read (self, ap = False, register = 0x00):
        self.pirate.send([calcOpcode(ap, register, True)])
        ack = self.pirate.bits(3)
        if ack[0:3] != ['\x01', '\x00', '\x00']:
            print("error in SWD stream")
            print(ack[0:3])
            sys.exit(1)
        data = self.pirate.bytes(4)
        extra = self.pirate.bits(3)
        if sum([bitCount(ord(x)) for x in data[0:3]]) % 2 != ord(extra[0]):
            print("parity error")
        return data

    def write (self, data, ap = False, register = 0x00):
        self.pirate.send([calcOpcode(ap, register, False)])
        ack = self.pirate.bits(5)
        if ack[0:3] != ['\x01', '\x00', '\x00']:
            print("error in SWD stream")
            print(ack[0:3])
            sys.exit(1)
        self.pirate.send(data[0:4])
        if sum([bitCount(x) for x in data[0:4]]) % 2:
            self.pirate.port.write(bytearray([0x0D, 0x09]))
            self.pirate.port.read(2)
        else:
            self.pirate.port.write(bytearray([0x0C, 0x09]))
            self.pirate.port.read(2)

def main():
    swdp = SWDP("/dev/ttyUSB0")
    swdp.resync()
    print hexEncode( swdp.read(ap = False, register = 0b00) )
    for i in range(5):
        swdp.write([0x3F, 0x00, 0x00, 0x00], ap = False, register = 0b01)
        swdp.write([0x3F, 0x00, 0x00, 0x00], ap = False, register = 0b10)
        print hexEncode( swdp.read(ap = True, register = 0b00) )

def hexEncode (bs):
    return ''.join(["%02X " % b for b in bytearray(bs)]).strip()

if __name__ == "__main__":
    main()
