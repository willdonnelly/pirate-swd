import serial
import time
import sys
import array

class SWDProtocolError(Exception):
    "The target responded with an invalid ACK"
    pass
class SWDFaultError(Exception):
    "The target responded with a 'fault' ACK"
    pass
class SWDWaitError(Exception):
    "The target responded with a 'wait' ACK"
    pass

class BusPirate:
    def __init__ (self, f = "/dev/bus_pirate"):
        self.port = serial.Serial(port = f, baudrate = 115200, timeout = 0.01)
        self.reset()

    def reset (self):
        self.port.write(bytearray([0x0F]))
        while self.port.read(5) != "BBIO1":
            self.port.read(9999)
            self.port.write(bytearray([0x00]))
            time.sleep(0.01)
        self.port.write(bytearray([0x05]))
        if self.port.read(4) != "RAW1":
            print("error initializing bus pirate")
            sys.exit(1)
        self.port.write(bytearray([0x63,0x88]))
        self.expected = 9999
        self.clear()

    # this is the fastest port-clearing scheme I could devise
    def clear (self, more = 0):
        vals = self.port.read(self.expected + more)
        self.expected = 0
        return vals[-more:]

    def readBits (self, count):
        self.port.write(bytearray([0x07] * count))
        return [ord(x) for x in self.clear(count)]

    def skipBits (self, count):
        self.port.write(bytearray([0x07] * count))
        self.expected = self.expected + count

    def readBytes (self, count):
        self.port.write(bytearray([0x06] * count))
        return [ord(x) for x in self.clear(count)]

    def sendBytes (self, data):
        self.port.write(bytearray([0x10 + ((len(data) - 1) & 0x0F)] + data))
        self.expected = self.expected + 1 + len(data)

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
        self.port.sendBytes([0x00])
        # finally return the data
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]

    def write (self, ap, register, data, ignore = False):
        # transmit the opcode
        self.port.sendBytes([calcOpcode(ap, register, False)])
        # check the response
        if ignore:
            self.port.skipBits(5)
        else:
            ack = self.port.readBits(5)
            if ack[0:3] != [1,0,0]:
                print("error in SWD stream")
                print(ack[0:3])
                sys.exit(1)
        # mangle the data properly
        payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        payload[0] = reverseBits((data >>  0) & 0xFF)
        payload[1] = reverseBits((data >>  8) & 0xFF)
        payload[2] = reverseBits((data >> 16) & 0xFF)
        payload[3] = reverseBits((data >> 24) & 0xFF)
        # set the parity bit
        if sum([bitCount(x) for x in payload[0:4]]) % 2:
            payload[4] = 0x80
        # output data, parity bit, and idle clocking
        self.port.sendBytes(payload)

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

    def writeAP (self, apsel, address, data, ignore = False):
        adrBank = (address >> 4) & 0xF
        adrReg  = (address >> 2) & 0x3
        if apsel != self.curAP or adrBank != self.curBank:
            self.select(apsel, adrBank)
            self.curAP = apsel
            self.curBank = adrBank
        self.swd.write(True, adrReg, data, ignore)

class AHB_AP:
    def __init__ (self, dp, apsel):
        self.dp = dp
        self.apsel = apsel
        self.csw(1,2) # 32-bit auto-incrementing addressing

    def csw (self, addrInc, size):
        print "  Setting CSW"
        self.dp.readAP(self.apsel, 0x00)
        csw = self.dp.readRB() & 0xFFFFFF00
        self.dp.writeAP(self.apsel, 0x00, csw + (addrInc << 4) + size)

    def idcode (self):
        self.dp.readAP(self.apsel, 0xFC)
        return self.dp.readRB()

    def readWord (self, adr):
        self.dp.writeAP(self.apsel, 0x04, adr)
        self.dp.readAP(self.apsel, 0x0C)
        return self.dp.readRB()

    def writeWord (self, adr, data):
        self.dp.writeAP(self.apsel, 0x04, adr)
        self.dp.writeAP(self.apsel, 0x0C, data)
        return self.dp.readRB()

    def readBlock (self, adr, count):
        self.dp.writeAP(self.apsel, 0x04, adr)
        vals = [self.dp.readAP(self.apsel, 0x0C) for off in range(count)]
        vals.append(self.dp.readRB())
        return vals[1:]

    def writeBlock (self, adr, data):
        self.dp.writeAP(self.apsel, 0x04, adr)
        for val in data:
            self.dp.writeAP(self.apsel, 0x0C, val)

    def writeHalfs (self, adr, data):
        self.csw(2, 1) # 16-bit packed-incrementing addressing
        print "  Writing TAR"
        self.dp.writeAP(self.apsel, 0x04, adr)
        print "  Writing to DRW"
        for val in data:
            time.sleep(0.001)
            self.dp.writeAP(self.apsel, 0x0C, val, ignore = True)
        self.csw(1, 2) # 32-bit auto-incrementing addressing

class STM32:
    def __init__ (self, debugPort):
        self.ahb = AHB_AP(debugPort, 0)

    def halt (self):
        # halt the processor core
        self.ahb.writeWord(0xE000EDF0, 0xA05F0003)

    def unhalt (self):
        # unhalt the processor core
        self.ahb.writeWord(0xE000EDF0, 0xA05F0000)

    def sysReset (self):
        # restart the processor and peripherals
        self.ahb.writeWord(0xE000ED0C, 0x05FA0004)

    def flashUnlock (self):
        # unlock main flash
        self.ahb.writeWord(0x40022004, 0x45670123)
        self.ahb.writeWord(0x40022004, 0xCDEF89AB)

    def flashErase (self):
        # start the mass erase
        self.ahb.writeWord(0x40022010, 0x00000204)
        self.ahb.writeWord(0x40022010, 0x00000244)
        # check the BSY flag
        while (self.ahb.readWord(0x4002200C) & 1) == 1:
            print "waiting for erase completion..."
            time.sleep(0.01)
        self.ahb.writeWord(0x40022010, 0x00000200)

    def flashProgram (self):
        self.ahb.writeWord(0x40022010, 0x00000201)

    def flashProgramEnd (self):
        self.ahb.writeWord(0x40022010, 0x00000200)

def loadFile(path):
    arr = array.array('L')
    try:
        arr.fromfile(open(sys.argv[1], 'rb'), 1024*1024)
    except EOFError:
        pass
    return arr.tolist()

def main():
    busPirate = BusPirate("/dev/ttyUSB0")
    debugPort = DebugPort(SWD(busPirate))
    stm32     = STM32(debugPort)

    print "DP.IDCODE: %08X" % debugPort.idcode()
    print "AP.IDCODE: %08X" % stm32.ahb.idcode()
    print ""
    print "Loading File: '%s'" % sys.argv[1]
    vals = loadFile(sys.argv[1])

    print "Halting Processor"
    stm32.halt()
    print "Erasing Flash"
    stm32.flashUnlock()
    stm32.flashErase()
    print "Programming Flash"
    stm32.flashProgram()
    stm32.ahb.writeHalfs(0x08000000, vals)
    stm32.flashProgramEnd()
    print "Resetting"
    stm32.sysReset()

if __name__ == "__main__":
    main()
