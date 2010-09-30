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
        return [ord(self.port.read(1)) for x in range(count)]

    def bytes (self, count):
        self.port.write(bytearray([0x06] * count))
        return [ord(self.port.read(1)) for x in range(count)]

def bitCount(int_type):
    count = 0
    while(int_type):
        int_type &= int_type - 1
        count += 1
    return(count)

def reverseBits (x):
    a = ((x & 0xAA) >> 1) | ((x & 0x55) << 1)
    b = ((a & 0xCC) >> 2) | ((a & 0x33) << 2)
    c = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4)
    return c

def calcOpcode (ap=False, register=0x00, read=True):
    opcode = 0x00
    opcode = opcode | (0x20 if read else 0x00)
    opcode = opcode | (0x40 if ap else 0x00)
    opcode = opcode | ((register & 0x01) << 4) | ((register & 0x02) << 2)
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

    def read (self, ap = False, register = 0b00):
        # transmit the opcode
        self.pirate.send([calcOpcode(ap, register, True)])
        # check the response
        ack = self.pirate.bits(3)
        if ack[0:3] != [1,0,0]:
            print("error in SWD stream")
            print(ack[0:3])
            sys.exit(1)
        # read the next 4 bytes
        data = [reverseBits(b) for b in self.pirate.bytes(4)]
        data.reverse()
        extra = self.pirate.bits(3)
        # check the parity
        if sum([bitCount(x) for x in data[0:4]]) % 2 != extra[0]:
            print("parity error")
        # required miniminum idle clocking is 8 bits
        self.pirate.send(bytearray([0x00]))
        # finally return the data
        return data

    def write (self, data, ap = False, register = 0b00):
        # transmit the opcode
        self.pirate.send([calcOpcode(ap, register, False)])
        # check the response
        ack = self.pirate.bits(5)
        if ack[0:3] != [1,0,0]:
            print("error in SWD stream")
            print(ack[0:3])
            sys.exit(1)
        # output the data
        payload = [reverseBits(x) for x in data[0:4]]
        payload.reverse()
        self.pirate.send(payload)
        # output the parity bit
        if sum([bitCount(x) for x in payload]) % 2:
            self.pirate.port.write(bytearray([0x0D, 0x09]))
            self.pirate.port.read(2)
        else:
            self.pirate.port.write(bytearray([0x0C, 0x09]))
            self.pirate.port.read(2)
        # required minimum idle clocking is 8 bits
        self.pirate.send(bytearray([0x00]))

#class SWAP:
#    def __init__ (self, swdp, apsel = 0x00):
#        self.swdp = swdp

def main():
    swdp = SWDP("/dev/ttyUSB0")
    swdp.resync()

    # read from the DP.IDCODE register
    print hexEncode( swdp.read(ap = False, register = 0b00) )

    rID     = 0
    rMulti  = 1
    rSelect = 2

    # select Wire Control Register and set properly
    swdp.write([0x00,0x00,0x00,0x01], ap = False, register = rSelect)
    swdp.write([0x00,0x00,0x00,0x40], ap = False, register = rMulti)

    # select Status Register and power shit up
    swdp.write([0x00,0x00,0x00,0x00], ap = False, register = rSelect)

    # CSYSPOWERUPREQ | CDBGPWRUPREQ
    swdp.write([0x54,0x00,0x00,0x00], ap = False, register = rMulti)
    if swdp.read(ap = False, register = rMulti)[0] != 0xF4:
        print "error powering up system"
        sys.exit(1)

    # try to write to AP.TAP
    swdp.write([0x00, 0x00, 0x00, 0x00], ap = False, register = rSelect)
    swdp.write([0x12, 0x23, 0x34, 0x45], ap = True,  register = 1)
    print hexEncode( swdp.read(ap = True,  register = 1) )
    print hexEncode( swdp.read(ap = False, register = 3) )

    # try to read AP zero IDR
    swdp.write([0x00, 0x00, 0x00, 0xF0], ap = False, register = 2)
    print hexEncode( swdp.read(ap = True,  register = 3) )
    print hexEncode( swdp.read(ap = True,  register = 3) )
    print hexEncode( swdp.read(ap = False, register = 3) )

def hexEncode (bs):
    return ''.join(["%02X " % b for b in bytearray(bs)]).strip()

if __name__ == "__main__":
    main()
