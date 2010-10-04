import time
import sys
import array

from PirateSWD import *
from SWDCommon import *
from STM32 import *

def loadFile(path):
    arr = array.array('L')
    try:
        arr.fromfile(open(sys.argv[1], 'rb'), 1024*1024)
    except EOFError:
        pass
    return arr.tolist()

def main():
    busPirate = PirateSWD("/dev/ttyUSB0")
    debugPort = DebugPort(busPirate)
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
