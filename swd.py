import serial
import time
import sys

class BusPirate:
    def __init__ (self, f = "/dev/bus_pirate"):
        self.port = serial.Serial(port = f, baudrate = 115200, timeout = 0.01)

    def write (self, data):
        self.port.write(data)
    def read (self, count):
        return self.port.read(count)
    def flush (self):
        self.port.flush()

# reset the bus pirate into raw binary mode suitable for SWD
def bpReset(pirate):
    pirate.write("\x0F")
    while pirate.read(5) != "BBIO1":
        pirate.write("\x00")
        time.sleep(0.01)

    # set it to raw binary mode and verify
    pirate.write("\x05")
    if pirate.read(4) != "RAW1":
        print("error initializing bus pirate to raw binary mode")
        sys.exit(1)

    # set the speed and various output mode flags
    pirate.write("\x63\x88")

# put the STM32 chip into SWD mode by sending the correct bitstream
def swdEntry(pirate):
    pirate.write("\x17\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF")
    pirate.write("\x11\x79\xE7")
    swdReset(pirate)

# send a bit pattern which fixes any SWD framing issues
def swdReset(pirate):
    pirate.write("\x17\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF")
    pirate.write("\x17\x00\x00\x00\x00\x00\x00\x00\x00")
    pirate.flush()

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
    opcode = calcOpcode(ap, register, True)
    # discard any buffered data
    pirate.read(9999)
    # write the opcode bit
    pirate.write(chr(0x10)+chr(opcode))
    pirate.read(2)
    # read the ACK
    pirate.write("\x07\x07\x07")
    ack = pirate.read(3)
    # read the data
    pirate.write("\x06\x06\x06\x06")
    data = pirate.read(4)
    # other bits
    pirate.write("\x07\x07\x07")
    pirate.read(3)
    return data

def main():
    pirate = BusPirate("/dev/ttyUSB0")
    bpReset(pirate)
    swdEntry(pirate)
    swdReset(pirate)
    print(swdRead(pirate).encode("hex"))
    print(swdRead(pirate, register=0x00).encode("hex"))

if __name__ == "__main__":
    main()
