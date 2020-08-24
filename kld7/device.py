"""Code device driver for K-LD7"""

import time
import struct
from collections import namedtuple

import serial

from .constants import (
    DEFAULT_BAUD_RATE, SUPPORTED_RATES,
    _RPS_FORMAT,
    _PDAT_FORMAT, _RADC_FORMAT, _RFFT_FORMAT, _DDAT_FORMAT,
    Response, FrameCode
)

Target = namedtuple("Target",
                    ['distance', 'speed', 'angle', 'magnitude'])
Detection = namedtuple("Detection",
                       ['detection', 'micro_detection', 'angle', 'direction', 'range', 'speed'])

def _count_bits(value):
    count = 0
    while value:
        count += value & 1
        value >>= 1
    return count

def _make_target(fields):
    return Target(fields[0]/100, fields[1]/100, fields[2]/100, fields[3])

def _decode_frame(code, payload):
    if code == 'RDAC':
        values = struct.unpack(_RADC_FORMAT, payload)
        payload = [[values[0:256], values[256:512]],
                   [values[512:768], values[768:1024]],
                   [values[1024:1280], values[1280:1536]]]
    elif code == 'RFFT':
        values = struct.unpack(_RFFT_FORMAT, payload)
        payload = [values[0:256], values[256:512]]
    elif code == 'PDAT':
        if payload:
            payload = list(_make_target(v) for v in struct.iter_unpack(_PDAT_FORMAT, payload))
        else:
            payload = []
    elif code == 'TDAT':
        if payload:
            payload = _make_target(struct.unpack(_PDAT_FORMAT, payload))
    elif code == 'DDAT':
        payload = Detection(*struct.unpack(_DDAT_FORMAT, payload))

    return (code, payload)

_param_struct_fields = {}

class _RadarParamDescriptor:
    """Radar parameter structure, represented as a mapping"""
    def __init__(self, index, description):
        self._cmd = ""
        self._index = index
        self.__doc__ = description + " (property)"

    def __set_name__(self, owner, name):
        self._cmd = name
        _param_struct_fields[self._index] = name

    def __set__(self, obj, value):
        obj._parent._set_param(self._cmd, value)

    def __get__(self, obj, owner):
        return obj._parent._get_param(self._cmd)

class RadarParamProxy:
    """Proxy class for accessing radar sensor parameters

    This class should not be instantiated directly. An instance is returned
    as the :any:`params` attribute of a :any:`KLD7` object"""

    # pylint: disable=too-few-public-methods

    def __init__(self, parent):
        self._parent = parent

    def __repr__(self):
        return "<Parameter proxy for {}>".format(self._parent)

    # Radar parameters
    RBFR = _RadarParamDescriptor(1, "Base frequency")
    RSPI = _RadarParamDescriptor(2, "Maximum speed")
    RRAI = _RadarParamDescriptor(3, "Maximum range")
    THOF = _RadarParamDescriptor(4, "Threshold offset")
    TRFT = _RadarParamDescriptor(5, "Tracking filter type")
    VISU = _RadarParamDescriptor(6, "Vibration suppression")
    MIRA = _RadarParamDescriptor(7, "Min detection distance")
    MARA = _RadarParamDescriptor(8, "Max detection distance")
    MIAN = _RadarParamDescriptor(9, "Min detection angle")
    MAAN = _RadarParamDescriptor(10, "Max detection angle")
    MISP = _RadarParamDescriptor(11, "Min detection speed")
    MASP = _RadarParamDescriptor(12, "Max detection speed")
    DEDI = _RadarParamDescriptor(13, "Detection direction")
    RATH = _RadarParamDescriptor(14, "Range threshold")
    ANTH = _RadarParamDescriptor(15, "Angle threshold")
    SPTH = _RadarParamDescriptor(16, "Speed threshold")
    DIG1 = _RadarParamDescriptor(17, "Digital output 1")
    DIG2 = _RadarParamDescriptor(18, "Digital output 2")
    DIG3 = _RadarParamDescriptor(19, "Digital output 3")
    HOLD = _RadarParamDescriptor(20, "Hold time")
    MIDE = _RadarParamDescriptor(21, "Micro detection retriger")
    MIDS = _RadarParamDescriptor(22, "Micro detection sensitivity")

class KLD7:
    """High-level driver for the K-LD7 radar unit"""
    # pylint: disable=invalid-name
    def __init__(self, port, baudrate=DEFAULT_BAUD_RATE):
        # Aleays start with thhe baud rate set to 115200
        self._port = serial.Serial(port=port,
                                   baudrate=DEFAULT_BAUD_RATE,
                                   parity=serial.PARITY_EVEN,
                                   stopbits=1)
        self._port.timeout = 0.2

        if baudrate not in SUPPORTED_RATES:
            raise Exception("Unsupported baud rate")
        response = self._send_command('INIT', SUPPORTED_RATES.index(baudrate))
        if response != Response.OK:
            raise Exception("Failed to initialise device")

        if baudrate != DEFAULT_BAUD_RATE:
            self._port.baudrate = baudrate

        self._param_dict = {}
        self._param_proxy = RadarParamProxy(self)

        self._fetch_radar_params()

        self._stop_flag = False

    def __repr__(self):
        return "{}('{}', baudrate={})".format(
            self.__class__.__name__,
            self._port.port, self._port.baudrate
        )

    def close(self):
        # pylint: disable=bare-except
        """Close the connection to the sensor"""
        self._stop_flag = True

        try:
            self._send_command('GBYE')
        except:
            pass

        try:
            self._port.close()
        except:
            pass

        self._port = None

    def __del__(self):
        self.close()

    def __enter__(self):
        """Start using the sensor object in a context handler"""
        return self

    def __exit__ (self, exc_type, exc_value, traceback):
        """Finish using the sensor object in a context handler"""
        self.close()

    def _drain_serial(self):
        old_timeout = self._port.timeout
        self._port.timeout = 0
        while self._port.read(256):
            pass
        self._port.timeout = old_timeout

    @property
    def timeout(self):
        """Command timeout in seconds"""
        return self._port.timeout

    @timeout.setter
    def timeout(self, value):
        self._port.timeout = value

    def _send_command(self, cmd, data=None):
        self._drain_serial()
        if self._port is None:
            raise Exception("serial port has been closed")
        if data is None:
            data = b''
        if isinstance(data, int):
            data = struct.pack("<I", data)
        if isinstance(cmd, str):
            cmd = cmd.upper().encode("ASCII")
        length = len(data)

        packet = struct.pack("<4sI", cmd, length) + data
        self._port.write(packet)

        return self._get_response()

    def _read_packet(self):
        if self._port is None:
            raise Exception("serial port has been closed")

        header = self._port.read(8)
        if len(header) == 0:
            raise Exception("Timeout waiting for reply")
        if len(header) != 8:
            raise Exception("Wrong length reply")
        reply, length = struct.unpack("<4sI", header)
        reply = reply.decode("ASCII")
        if length != 0:
            payload = self._port.read(length)
            if len(payload) != length:
                print("Failed to read all of reply")
        else:
            payload = None
        return (reply, payload)

    def _get_response(self):
        reply, payload = self._read_packet()
        if reply != 'RESP':
            raise Exception("Packet was not a response")
        if len(payload) != 1:
            raise Exception("Response packet with incorrect payload length")
        code = payload[0]
        return Response(code if code < Response.MAX_RESPONSE else -1)

    def _get_param(self, cmd):
        return self._param_dict[cmd]

    def _set_param(self, cmd, value):
        value = int(value)
        response = self._send_command(cmd, value)
        if response != Response.OK:
            raise Exception("Failed to set parameter")
        self._param_dict[cmd] = value

    def _fetch_radar_params(self):
        """Read the radar parameter structure"""
        resp = self._send_command('GRPS')
        if resp != Response.OK:
            raise Exception("GRPS command failed: {}".format(resp))
        code, payload = self._read_packet()
        if code != 'RPST':
            raise Exception("GRPS data has wrong packet type")

        values = struct.unpack(_RPS_FORMAT, payload)
        for index, cmd in _param_struct_fields.items():
            self._param_dict[cmd] = values[index]

    def stream_frames(self, frame_codes, max_count=-1, min_frame_interval=0):
        """Yield a stream of data frames of various types

        :param frame_codes: flags for which frame types to be returned
        :type frame_codes: kld7.FrameCode
        :param max_count: maximum number of frame sets to return (or -1 for no limit)
        :type max_count: int
        :param min_frame_interval: minimum time between frames
        :type min_frame_interval: float

        Note that if the larger frame times are selected, in particular the raw ADC
        frames, it is easy for the serial port buffer to overflow.
        """
        code_count = _count_bits(frame_codes)
        self._stop_flag = False
        try:
            count = 0
            while not self._stop_flag and (max_count == -1 or count < max_count):
                last_frame_time = time.time()
                self._send_command('GNFD', frame_codes)
                for _ in range(code_count):
                    code, payload = self._read_packet()
                    yield _decode_frame(code, payload)
                    if code == 'DONE':
                        break
                count += 1
                now = time.time()
                if (min_frame_interval is not None and
                        now < last_frame_time + min_frame_interval):
                    time.sleep(last_frame_time + min_frame_interval - now)
        finally:
            self._stop_flag = True

    def _read_single_stream(self, frame_code, max_count=-1, min_frame_interval=0):
        for _, payload in self.stream_frames(frame_code, max_count, min_frame_interval):
            yield payload

    def _read_single_frame(self, frame_code):
        self._send_command('GNFD', frame_code)
        _, payload = _decode_frame(*self._read_packet())
        return payload

    def read_RADC(self):
        """Fetch a single raw ADC frame"""
        return self._read_single_frame(FrameCode.RADC)

    def stream_RADC(self, max_count=-1, min_frame_interval=0):
        """Yield a stream of raw ADC frames

        Note that the raw ADC frames are large and unless you are using a very fast serial
        port or you slow the frame interval it is likely that the serial port buffer will
        overrun, resulting in reception errors.
        """
        yield from self._read_single_stream(FrameCode.RADC, max_count, min_frame_interval)

    def read_RFFT(self):
        """Fetchh a single raw FFT frame"""
        return self._read_single_frame(FrameCode.RFFT)

    def stream_RFFT(self, max_count=-1, min_frame_interval=0):
        """Yield a stream of raw FFT frames"""
        yield from self._read_single_stream(FrameCode.RFFT, max_count, min_frame_interval)

    def read_PDAT(self):
        """Fetch a single set of detected targets"""
        return self._read_single_frame(FrameCode.PDAT)

    def stream_PDAT(self, max_count=-1, min_frame_interval=0):
        """Yield a stream of detected targets"""
        yield from self._read_single_stream(FrameCode.PDAT, max_count, min_frame_interval)

    def read_TDAT(self):
        """Fetch tracked target data"""
        return self._read_single_frame(FrameCode.TDAT)

    def stream_TDAT(self, max_count=-1, min_frame_interval=0):
        """Yield a stream of tracked target data"""
        yield from self._read_single_stream(FrameCode.TDAT, max_count, min_frame_interval)

    def read_DDAT(self):
        """Fetch detection data info"""
        return self._read_single_frame(FrameCode.DDAT)

    def stream_DDAT(self, max_count=-1, min_frame_interval=0):
        """Yield a stream of detection data info"""
        yield from self._read_single_stream(FrameCode.DDAT, max_count, min_frame_interval)

    @property
    def params(self):
        """A proxy for the radar parameters structure.

        See :any:`RadarParamProxy` for details of the proxy attributes"""
        return self._param_proxy
