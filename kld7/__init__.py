"""Driver for the K-LD7 radar unit"""

from .constants import DEFAULT_BAUD_RATE, SUPPORTED_RATES, FrameCode

from .device import KLD7, RadarParamProxy, Target, Detection

__version__ = "0.1.1"
