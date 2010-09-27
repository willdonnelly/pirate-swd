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
        return bytearray([self.port.read(1) for x in range(count)])

# put the STM32 chip into SWD mode by sending the correct bitstream
def swdEntry(pirate):
    pirate.send([0xFF] * 8)
    pirate.send([0x79,0xE7])
    swdReset(pirate)

# send a bit pattern which fixes any SWD framing issues
def swdReset(pirate):
    pirate.send([0xFF] * 8)
    pirate.send([0x00] * 8)

def calcOpcode(ap=False, register=0x00, read=True):
    def bitCount(int_type):
        count = 0
        while(int_type):
            int_type &= int_type - 1
            count += 1
        return(count)
    opcode = 0x00
    opcode = opcode | (0x20 if read else 0x00)
    opcode = opcode | (0x40 if ap else 0x00)
    opcode = opcode | ((register & 0x03) << 3)
    opcode = opcode | ((bitCount(opcode) & 1) << 2)
    opcode = opcode | 0x81
    return opcode

def swdRead(pirate, ap=False, register=0x00):
    pirate.send([calcOpcode(ap, register, True)])
    ack = pirate.bits(3)
    data = pirate.bytes(4)
    extra = pirate.bits(3)
    return data

def main():
    pirate = BusPirate("/dev/ttyUSB0")
    swdEntry(pirate)
    swdReset(pirate)
    print(hexEncode(swdRead(pirate, register=0x00)))

def hexEncode (ba):
    return ''.join(["%02X " % b for b in ba]).strip()

if __name__ == "__main__":
    main()
