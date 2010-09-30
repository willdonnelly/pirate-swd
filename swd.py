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

    def readBits (self, count):
        self.port.write(bytearray([0x07] * count))
        return [ord(self.port.read(1)) for x in range(count)]

    def readBytes (self, count):
        self.port.write(bytearray([0x06] * count))
        return [ord(self.port.read(1)) for x in range(count)]

    def sendBytes (self, data):
        for byte in data:
            self.port.write(bytearray([0x10,byte]))
            self.port.read(2)

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

class SWD:
    def __init__ (self, port):
        self.port = port
        self.port.sendBytes([0xFF] * 8)
        self.port.sendBytes([0x79, 0xE7])
        self.resync()

    def resync (self):
        self.port.sendBytes([0xFF] * 8)
        self.port.sendBytes([0x00] * 8)

    def read (self, ap, register):
        # transmit the opcode
        self.port.sendBytes([calcOpcode(ap, register, True)])
        # check the response
        ack = self.port.readBits(3)
        if ack[0:3] != [1,0,0]:
            print("error in SWD stream")
            print(ack[0:3])
            sys.exit(1)
        # read the next 4 bytes
        data = [reverseBits(b) for b in self.port.readBytes(4)]
        data.reverse()
        extra = self.port.readBits(3)
        # check the parity
        if sum([bitCount(x) for x in data[0:4]]) % 2 != extra[0]:
            print("parity error")
        # required miniminum idle clocking is 8 bits
        self.port.sendBytes(bytearray([0x00]))
        # finally return the data
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]

    def write (self, ap, register, data):
        # transmit the opcode
        self.port.sendBytes([calcOpcode(ap, register, False)])
        # check the response
        ack = self.port.readBits(5)
        if ack[0:3] != [1,0,0]:
            print("error in SWD stream")
            print(ack[0:3])
            sys.exit(1)
        # output the data
        payload = [0x00, 0x00, 0x00, 0x00]
        payload[0] = reverseBits((data >>  0) & 0xFF)
        payload[1] = reverseBits((data >>  8) & 0xFF)
        payload[2] = reverseBits((data >> 16) & 0xFF)
        payload[3] = reverseBits((data >> 24) & 0xFF)
        self.port.sendBytes(payload)
        # output the parity bit and idle clocking
        if sum([bitCount(x) for x in payload]) % 2:
            self.port.sendBytes([0x80, 0x00])
        else:
            self.port.sendBytes([0x00, 0x00])

class DebugPort:
    def __init__ (self, swd):
        self.swd = swd
        # read the IDCODE
        if self.idcode() != 0x1BA01477:
            print "warning: unexpected idcode"
        # power shit up
        self.swd.write(False, 1, 0x54000000)
        if (self.status() >> 24) != 0xF4:
            print "error powering up system"
            sys.exit(1)
        # get the SELECT register to a known state
        self.select(0,0)
        self.curAP = 0
        self.curBank = 0

    def idcode (self):
        return self.swd.read(False, 0)

    def abort (self, orunerr, wdataerr, stickyerr, stickycmp, dap):
        value = 0x00000000
        value = value | (0x10 if orunerr else 0x00)
        value = value | (0x08 if wdataerr else 0x00)
        value = value | (0x04 if stickyerr else 0x00)
        value = value | (0x02 if stickycmp else 0x00)
        value = value | (0x01 if dap else 0x00)
        self.swd.write(False, 0, value)

    def status (self):
        return self.swd.read(False, 1)

    def control (self, trnCount = 0, trnMode = 0, maskLane = 0, orunDetect = 0):
        value = 0x54000000
        value = value | ((trnCount & 0xFFF) << 12)
        value = value | ((maskLane & 0x00F) << 8)
        value = value | ((trnMode  & 0x003) << 2)
        value = value | (0x1 if orunDetect else 0x0)
        self.swd.write(False, 1, value)

    def select (self, apsel, apbank):
        value = 0x00000000
        value = value | ((apsel  & 0xFF) << 24)
        value = value | ((apbank & 0x0F) <<  4)
        self.swd.write(False, 2, value)

    def readRB (self):
        return self.swd.read(False, 3)

    def readAP (self, apsel, address):
        adrBank = (address >> 4) & 0xF
        adrReg  = (address >> 2) & 0x3
        if apsel != self.curAP or adrBank != self.curBank:
            self.select(apsel, adrBank)
            self.curAP = apsel
            self.curBank = adrBank
        return self.swd.read(True, adrReg)

    def writeAP (self, apsel, address, data):
        adrBank = (address >> 4) & 0xF
        adrReg  = (address >> 2) & 0x3
        if apsel != self.curAP or adrBank != self.curBank:
            self.select(apsel, adrBank)
            self.curAP = apsel
            self.curBank = adrBank
        self.swd.write(True, adrReg, data)

def main():
    busPirate = BusPirate("/dev/ttyUSB0")
    debugPort = DebugPort(SWD(busPirate))

    print hex( debugPort.idcode() )

    debugPort.readAP(0, 0xFC)
    print hex( debugPort.readRB() )

    debugPort.readAP(0, 0x00)
    csw = debugPort.readRB()
    debugPort.writeAP(0, 0x00, csw | 0x12)
    debugPort.readAP(0, 0x00)
    print hex( csw                )
    print hex( debugPort.readRB() )

    debugPort.writeAP(0, 0x04, 0xE0042000)
    debugPort.readAP(0,0x0C)
    print hex(debugPort.readRB())

    debugPort.writeAP(0, 0x04, 0x08000000)
    for off in range(1024):
        debugPort.readAP(0,0x04)
        adr = debugPort.readRB()
        debugPort.readAP(0,0x0C)
        val = debugPort.readRB()
        print "%08X: %08X" % (adr, val)

#    debugPort.readAP(0, 0x00)
#    debugPort.writeAP(0, 0x04, 0x12345678)
#    print hex( debugPort.readAP(0, 0x04) )
#    print hex( debugPort.readAP(0, 0x08) )
#    print hex( debugPort.readAP(0, 0x04) )
#    print hex( debugPort.readRB() )

#    rID     = 0
#    rMulti  = 1
#    rSelect = 2
#
#    # try to write to AP.TAP
#    swdp.write([0x00, 0x00, 0x00, 0x00], ap = False, register = rSelect)
#    swdp.write([0x12, 0x23, 0x34, 0x45], ap = True,  register = 1)
#    print hexEncode( swdp.read(ap = True,  register = 1) )
#    print hexEncode( swdp.read(ap = False, register = 3) )
#
#    # try to read AP zero IDR
#    swdp.write([0x00, 0x00, 0x00, 0xF0], ap = False, register = 2)
#    print hexEncode( swdp.read(ap = True,  register = 3) )
#    print hexEncode( swdp.read(ap = True,  register = 3) )
#    print hexEncode( swdp.read(ap = False, register = 3) )

def hexEncode (bs):
    return ''.join(["%02X " % b for b in bytearray(bs)]).strip()

if __name__ == "__main__":
    main()
