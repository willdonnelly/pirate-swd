class SWDInitError(Exception):
    "There was an error initializing SWD communications"
    pass
class SWDProtocolError(Exception):
    "The target responded with an invalid ACK"
    pass
class SWDFaultError(Exception):
    "The target responded with a 'fault' ACK"
    pass
class SWDWaitError(Exception):
    "The target responded with a 'wait' ACK"
    pass
class SWDParityError(Exception):
    "The target sent data with incorrect parity"
    pass
