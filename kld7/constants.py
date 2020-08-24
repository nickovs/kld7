"""Various constants and data types used by the K-LD7 driver"""

from enum import IntEnum, IntFlag

DEFAULT_BAUD_RATE = 115200

SUPPORTED_RATES = [115200, 460800, 921600, 2000000, 3000000]

_RPS_FORMAT = "<19s8B2b4Bb4BH2B"

_PDAT_FORMAT = "<HhhH" # Also used for TDAT
_RADC_FORMAT = "<1536H"
_RFFT_FORMAT = "<512H"
_DDAT_FORMAT = "<6B"

class Response(IntEnum):
    """K-LD7 response codes"""
    OK = 0 #: Command was successful
    UnknownCommand = 1 #: Command was not known
    InvalidParameter = 2 #: Invalid command parameters
    InvalidRPSTVersion = 3 #: Bad version infromation when setting radar parameter structure
    UARTError = 4 #: Framing, sync or parity error
    SensorBusy = 5 #: Sensor is already fetching frame data

    MAX_RESPONSE = 6
    UnknownError = -1 #: Response code not known

class FrameCode(IntFlag):
    """Flags for the types of data frames from K-LD7 sensor"""
    RADC = 1 << 0 #: Raw ADC samples
    RFFT = 1 << 1 #: Raw FFT output
    PDAT = 1 << 2 #: Data for possible tracking targets
    TDAT = 1 << 3 #: Tracked (most significant) target
    DDAT = 1 << 4 #: Detection status flags
    DONE = 1 << 5 #: Frame completion marker (with frame count)
