from SWDCommon import *

class STM32:
    def __init__ (self, debugPort):
        self.ahb = MEM_AP(debugPort, 0)

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
