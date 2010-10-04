import serial

from SWDErrors import *

class PirateSWD:
    def __init__ (self, f = "/dev/bus_pirate"):
        self.port = serial.Serial(port = f, baudrate = 115200, timeout = 0.01)
        self.resetBP()
        self.sendBytes([0xFF] * 8)
        self.sendBytes([0x79, 0xE7])
        self.resyncSWD()

    def resetBP (self):
        self.expected = 9999
        self.clear()
        self.port.write(bytearray([0x0F]))
        while self.port.read(5) != "BBIO1":
            self.clear(9999)
            self.port.write(bytearray([0x00]))
        self.port.write(bytearray([0x05]))
        if self.port.read(4) != "RAW1":
            raise SWDInitError("error initializing bus pirate")
        self.port.write(bytearray([0x63,0x88]))
        self.clear(9999)

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

    def resyncSWD (self):
        self.sendBytes([0xFF] * 8)
        self.sendBytes([0x00] * 8)

    def readSWD (self, ap, register):
        # transmit the opcode
        self.sendBytes([calcOpcode(ap, register, True)])
        # check the response
        ack = self.readBits(3)
        if ack[0:3] != [1,0,0]:
            if   ack[0:3] == [0,1,0]:
                raise SWDWaitError(ack[0:3])
            elif ack[0:3] == [0,0,1]:
                raise SWDFaultError(ack[0:3])
            else:
                raise SWDProtocolError(ack[0:3])
        # read the next 4 bytes
        data = [reverseBits(b) for b in self.readBytes(4)]
        data.reverse()
        # read the parity bit and turnaround period
        extra = self.readBits(3)
        # check the parity
        if sum([bitCount(x) for x in data[0:4]]) % 2 != extra[0]:
            raise SWDParityError()
        # idle clocking to allow transactions to complete
        self.sendBytes([0x00])
        # return the data
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]

    def writeSWD (self, ap, register, data, ignoreACK = False):
        # transmit the opcode
        self.sendBytes([calcOpcode(ap, register, False)])
        # check the response if required
        if ignoreACK:
            self.skipBits(5)
        else:
            ack = self.readBits(5)
            if ack[0:3] != [1,0,0]:
                if   ack[0:3] == [0,1,0]:
                    raise SWDWaitError(ack[0:3])
                elif ack[0:3] == [0,0,1]:
                    raise SWDFaultError(ack[0:3])
                else:
                    raise SWDProtocolError(ack[0:3])
        # mangle the data endianness
        payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        payload[0] = reverseBits((data >>  0) & 0xFF)
        payload[1] = reverseBits((data >>  8) & 0xFF)
        payload[2] = reverseBits((data >> 16) & 0xFF)
        payload[3] = reverseBits((data >> 24) & 0xFF)
        # add the parity bit
        if sum([bitCount(x) for x in payload[0:4]]) % 2:
            payload[4] = 0x80
        # output the data, idle clocking is on the end of the payload
        self.sendBytes(payload)

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

def calcOpcode (ap, register, read):
    opcode = 0x00
    opcode = opcode | (0x20 if read else 0x00)
    opcode = opcode | (0x40 if ap else 0x00)
    opcode = opcode | ((register & 0x01) << 4) | ((register & 0x02) << 2)
    opcode = opcode | ((bitCount(opcode) & 1) << 2)
    opcode = opcode | 0x81
    return opcode
