import time
import struct
from sliplib import SlipStream
from serial import Serial
from serial.tools import list_ports

PIN_MODE_HI_Z               = 0
PIN_MODE_INPUT              = 1
PIN_MODE_INPUT_PULLUP       = 2
PIN_MODE_INPUT_PULLDOWN     = 3
PIN_MODE_OUTPUT             = 4

FORWARD                     = True
"""Motor forwards direction"""

REVERSE                     = False
"""Motor reverse direction"""

BUILTIN_POTENTIOMETER       = 24
"""Pin number of builtin potentiometer"""

BUILTIN_BUTTON              = 25
"""Pin number of builtin button"""

OP_RESET                    = 0x00

OP_DELAY_MS                 = 0x10
OP_DELAY_US                 = 0x11

OP_SET_PIN_MODE             = 0x20
OP_DIGITAL_READ             = 0x21
OP_DIGITAL_WRITE            = 0x22
OP_DIGITAL_WAIT             = 0x23

OP_SCANNER_ENABLE           = 0x30
OP_SCANNER_READ             = 0x31
OP_SCANNER_DISABLE          = 0x32

OP_PWM_SET_DUTY             = 0x42

OP_ANALOG_READ              = 0x52

OP_MOTOR_SET                = 0x60

OP_LED_SET                  = 0x71
OP_LED_TOGGLE               = 0x72

OP_SPI_ENABLE               = 0x80
OP_SPI_CONFIGURE            = 0x81
OP_SPI_WRITE                = 0x82
OP_SPI_READ                 = 0x83
OP_SPI_DISABLE              = 0x8F

class BreadboardNotFound(Exception):
    pass

class OpenException(Exception):
    pass

class InvalidResponse(Exception):
    pass

class CommandFailed(Exception):
    def __init__(self, cmd_type, status, body):
        self.cmd_type = cmd_type
        self.status = status
        self.body = body

class OperationFailed(Exception):
    def __init__(self, status, body):
        self.status = status
        self.body = body

def parseTLV(blob, callback):
    while len(blob) > 0:
        if len(blob) < 2:
            raise Exception("invalid TLV format")
        chunk_type = blob[0]
        chunk_len = blob[1]
        callback(chunk_type, blob[2:2 + chunk_len])
        blob = blob[(2 + chunk_len):]

def findBreadboard():
    """Attempt to find a breadboard connected to any available serial port.

    :returns: breadboard instance
    :rtype: Breadboard
    :raises BreadboardNotFound: if no breadboard can be found connected to any of the system's serial ports
    """
    for p in list_ports.comports():
        if p.vid != 4292 or p.pid != 60016:
            continue
        try:
            return Breadboard(p.device)
        except Exception as e:
            pass
    raise BreadboardNotFound()

def waitForBreadboard():
    """Wait for a breadboard to be connected.

    :returns: breadboard instance
    :rtype: Breadboard
    """
    while True:
        try:
            return findBreadboard()
        except BreadboardNotFound:
            time.sleep(1)
        
class Breadboard():
    """Breadboard class

    Blah blah blah
    """
    
    DEVICE_ID_RESPONSE  = bytes([0x01, 0x77, 0x3E, 0xE2, 0xF0, 0x6A, 0x19, 0x44])

    CMD_DEVICE						= 0x00
    CMD_SIMPLE_IO					= 0x01
    CMD_START_MORSE_CODE_DECODER	= 0xE0

    FLAG_CONTINUE       = 0x01

    def __init__(self, port_name):
        """Connect to a breadboard on the given serial port"""
        port = Serial(port = port_name, baudrate = 115200, timeout = 0.2)
        self.port = port
        self.transport = SlipStream(port, 1)
        try:
            id = self.__command([0x00, 0x00])
            if id != self.DEVICE_ID_RESPONSE:
                raise OpenException("received invalid response from device ID request")
        except Exception as e:
            port.close()
            raise e
        
        port.timeout = None
        
        self.__cmd_buffer = bytearray(1024)
        self.__readDeviceInfo()
    
    def exec(self, ops, continue_on_error = False):
        """Execute the given operations and return the results.

        :param ops: list of operations to execute
        :param continue_on_error: Set to `True` if breadboard should continue processing successive operations if an operation fails.
        :returns: list of operation results, one per input operation
        """
        self.__cmd_buffer[0] = self.CMD_SIMPLE_IO
        
        flags = 0
        if continue_on_error:
            flags |= self.FLAG_CONTINUE
        
        self.__cmd_buffer[1] = flags

        wp = 2
        for op in ops:
            enclen = op.encodeInto(self.__cmd_buffer, wp)
            wp += enclen

        result = self.__command(self.__cmd_buffer[0:wp])
        
        results = []
        rp = 0
        while rp < len(result):
            if len(result) < 3:
                raise InvalidResponse("expected operation result to be at least 3 bytes")
            status = result[rp]
            payload_len = (result[rp+1] << 8) | result[rp+2]
            if len(result) < (3 + payload_len):
                raise InvalidResponse("reported operation result length overruns actual length")
            rp += 3
            results.append(OpResult(status, result[rp:rp+payload_len]))
            rp += payload_len

        return results
    
    def mustExec(self, ops, continue_on_error = False):
        """Execute the given operations and return the results,
        raising an exception if any of the return results indicate
        failure.

        :param ops: list of operations to execute
        :param continue_on_error: Set to `True` if breadboard should continue processing successive operations if an operation fails.
        :raises OperationFailed: if the return status of any operation indicates failure
        :returns: list of operation results, one per input operation
        """
        results = self.exec(ops, continue_on_error)
        for r in results:
            r.raiseIfError()
        return results

    #
    #

    def reset(self):
        """Reset the breadboard, restoring all pins to their power-on states"""
        self.mustExec([ opReset() ])

    def delayMillis(self, millis):
        """Instruct the breadboard to delay for the given number of milliseconds
        
        :param millis: milliseconds to wait
        :type millis: int
        """
        self.mustExec([ opDelayMillis(millis) ])
    
    def pinMode(self, pin, mode):
        """Set a pin's mode
        
        :param pin: pin number
        :param mode: pin mode, one of the PIN_MODE_* constants
        :type pin: int
        :type mode: int
        """
        self.mustExec([ opPinMode(pin, mode) ])
        
    def digitalRead(self, pin):
        """Read pin level. If the pin is not currently configured as an input
        the breadboard will first attempt to change its mode.

        :param pin: pin number
        :type pin: int
        :returns: pin level
        :rtype: bool
        """
        res = self.mustExec([ opDigitalRead(pin) ])
        return res[0].body[0] > 0

    def digitalWrite(self, pin, level):
        """Write pin level. If the pin is not currently configured as an output
        the breadboard will first attempt to change its mode.

        :param pin: pin number
        :param level: level (True=high, False=low)
        :type pin: int
        :type level: bool
        """
        self.mustExec([ opDigitalWrite(pin, level) ])

    # def digitalWait(self, pin, level, settle_us = 0, timeout_us = 0):
    # 	self.mustExec([ Op(OP_DIGITAL_WAIT, struct.pack(">BBLL", pin, level, settle_us, timeout_us)) ])

    def createButton(self, pin, polarity = False, debounce_time_ms = 30):
        """Create a button
        
        :param pin: pin number
        :param polarity: logic level that is considered "pressed", defaults to `False`
        :param debounce_time_ms: Number of milliseconds pin level must remain active level before it is considered to have been pressed.
        :type pin: int
        :type polarity: bool
        :type debounce_time_ms: int
        :returns: button instance
        :rtype: Button
        """
        return Button(self, pin, polarity, debounce_time)

    def scanEnable(self, pin, polarity = False, debounce_time_ms = 0):
        """Enable scanning on the specified pin. While scanning is enabled on a pin
        the breadboard will continuously monitor its state, keeping track of the
        number of state changes that occur between each call to `getScanState()`.

        Scanning allows buttons (or other inputs) to be monitored without having
        to continuously poll the pin state, and without loss of information.

        :param pin: pin number
        :param polarity: pin level that is considered "active", defaults to `False` (low)
        :param debounce_time_ms: number of milliseconds pin must be stay at active level before it is considered to have changed to the active state, defaults to 0
        :type pin: int
        :type polarity: bool
        :type debounce_time_ms: int
        """
        self.mustExec([ opScanEnable(pin, polarity, debounce_time_ms) ])

    def scanDisable(self, pin):
        """Disable scanning on the specified pin.

        :param pin: pin number
        :type pin: int
        """
        self.mustExec([ opScanDisable(pin) ])
    
    def getScanState(self, pin):
        """Query the scan state for the specified pin

        :param pin: pin number
        :type pin: int
        :returns: scan state; tuple of `(bool, int)` representing the whether the input is currently active, plus the number of state transitions that have occurred since the previous call to `getScanState()`
        :rtype: (bool, int)
        """
        res = self.mustExec([ opGetScanState(pin) ])
        b = res[0].body
        active = b[0] > 0
        changes = (b[1] << 24) | (b[2] << 16) | (b[3] << 8) | (b[4])
        return (active, changes)
    
    # TODO: pwmInit
    # TODO: pwmConfigure
    # TODO: pwmDisable
    
    def pwmWrite(self, pin, duty):
        """Set PWM duty. If pin is not currently configured as a PWM, the
        breadboard will first attempt to enable the PWM function.

        :param pin: pin number
        :param duty: PWM duty cycle, 0-255
        :type pin: int
        :type duty: int
        """
        self.mustExec([ opPwmWrite(pin, duty) ])

    # TODO: analogInit
    # TODO: analogConfigure
    # TODO: analogDisable

    def analogRead(self, pin):
        """Read ADC level. If pin is not currently configured as an ADC, the
        the breadboard will first attempt to enable the ADC function.

        :param pin: pin number
        :type pin: int
        :returns: ADC level (0-1023)
        :rtype: int
        """
        res = self.mustExec([ opAnalogRead(pin) ])
        b = res[0].body
        return b[0] << 8 | b[1]

    def motorOn(self, motor, direction, speed):
        """Turn on the specified motor.

        :param motor: motor number (0-3)
        :param direction: motor direction (`True` = forwards, `False` = reverse)
        :param speed: motor speed (0-255)
        :type motor: int
        :type direction: bool
        :type speed: int
        """
        self.mustExec([ opMotorOn(motor, direction, speed) ])
    
    def motorOff(self, motor):
        """Turn off the specified motor. Equivalent to calling `motorOn()` with a speed of zero.

        :param motor: motor number (0-3)
        :type motor: int
        """
        return self.motorOn(motor, FORWARD, 0)
    
    def ledSet(self, led, state):
        """Set an onboard LED's state.

        :param led: LED number (0-3)
        :param state: desired LED state (`True` = on, `False` = off)
        :type led: int
        :type state: bool
        """
        self.mustExec([ opLedSet(led, state) ])
    
    def ledToggle(self, led):
        """Toggle an onboard LED's state.
        
        :param led: LED number (0-3)
        :type led: int
        """
        self.mustExec([ opLedToggle(led) ])
    
    def createSPIDevice(self, bus, cs_pin, mode, order, baud_rate):
        """Create a SPIDevice on the specified bus.
        
        :param bus: bus number (0-2)
        :param cs_pim: chip-select pin
        :param mode: SPI mode
        :param order: byte order
        :baud_rate: baud rate
        :type bus: int
        :type cs_pin: int
        :type mode: int
        :type order: int
        :type baud_rate: int
        :returns: SPI device
        :rtype: SPIDevice
        """
        return SPIDevice(self, bus, cs_pin, mode, order, baud_rate)

    def spiEnable(self, bus):
        """Enable SPI on the specified bus.

        :param bus: bus number (0-2)
        :type bus: int
        """
        self.mustExec([ opSpiEnable(bus) ])

    def spiConfigure(self, bus, mode, order, baud_rate):
        """Configure SPI on the specified bus.

        :param bus: bus number (0-2)
        :param mode: SPI mode
        :param order: byte order
        :param baud_rate: baud rate
        :type bus: int
        :type mode: int
        :type order: int
        :type baud_rate: int
        """
        self.mustExec([ opSpiConfigure(bus, mode, order, baud_rate) ])

    def spiWrite(self, bus, data):
        """Write SPI data.

        :param bus: bus number (0-2)
        :param data: data to write to target SPI device
        :type bus: int
        :type data: bytes
        """
        data = self.__coerceToBytes(data)
        self.mustExec([ opSpiWrite(bus, data) ])

    def spiRead(self, bus, count):
        """Read data from a SPI device.

        :param bus: bus number (0-2)
        :param count: number of bytes to read
        :type bus: int
        :type count: int
        :return: data read from SPI device
        :rtype: bytes
        """
        res = self.mustExec([ opSpiRead(bus, count) ])
        return res[0].body

    def spiDisable(self, bus):
        """Disable SPI on the given bus.
        
        :param bus: bus number (0-2)
        :type bus: int
        """
        self.mustExec([ opSpiDisable(bus) ])

    def decodeMorse(self, pin=None, min_press_time=20, dash_press_threshold=250, inter_character_time=300, inter_word_time=700):
        """Activate morse code decoding on the specified pin.
        
        Once activated, morse code decoding can only be deactivated by power-cycling
        the breadboard.

        :param pin: pin to use as the input signal; if unspecified, the internal button is used.
        :param min_press_time: minimum time button must be low to be considered valid (in milliseconds)
        :param dash_press_threshold: press time required to be interpreted as a dash (in milliseconds)
        :param inter_character_time: release time required before next press is considered a new letter (in milliseconds)
        :param inter_word_time: release time required before emitted a word separator (ASCII space) (in milliseconds)
        :type pin: int
        :type min_press_time: int
        :type dash_press_threshold: int
        :type inter_character_time: int
        :type inter_word_time: int
        :returns: a generator that yields each character as it is read from the breadboard
        :rtype: generator
        """
        if pin is None:
            pin = BUILTIN_BUTTON
        self.__command(struct.pack(">BBLLLL", self.CMD_START_MORSE_CODE_DECODER, pin, min_press_time, dash_press_threshold, inter_character_time, inter_word_time))
        while True:
            yield self.port.read().decode('utf-8')

    #
    #

    def __readDeviceInfo(self):
        def chunk(type, data):
            if type == 1:
                self.manufacturer = data.decode('utf-8')
            elif type == 2:
                self.product = data.decode('utf-8')
            elif type == 3:
                self.firmware_version = [data[0], data[1], data[2]]
        
        info_response = self.__command([self.CMD_DEVICE, 0x01])
        parseTLV(info_response, chunk)

    def __command(self, cmd):
        self.__send(cmd)
        response = self.__recv()
        if len(response) < 2:
            raise InvalidResponse("expected response to be at least 2 bytes")
        elif response[0] != (cmd[0] | 0x80):
            raise InvalidResponse("response type does not match expected")
        if response[1] != 0:
            raise CommandFailed(cmd[0], response[1], response[2:])
        return response[2:]
        
    def __send(self, msg):
        self.transport.send_msg(msg)

    def __recv(self):
        return self.transport.recv_msg()
    
    def __coerceToBytes(val):
        if type(val) is str:
            return bytes(val, 'utf8')
        return val

class Op:
    def __init__(self, opcode, payload):
        self.opcode = opcode
        self.payload = payload
    
    def encodeInto(self, buffer, offset):
        fmt = f">BH{len(self.payload)}s"
        struct.pack_into(fmt, buffer, offset, self.opcode, len(self.payload), self.payload)
        return struct.calcsize(fmt)

class OpResult:
    def __init__(self, status, body):
        self.status = status
        self.body = body
    
    def raiseIfError(self):
        if self.status != 0:
            raise OperationFailed(self.status, self.body)

class SPIDevice:
    """A SPIDevice represents a device on the SPI bus plus all of
    the config options required to communicate with it.
    
    SPIDevice exposes high-level read and write operations that
    take care of configuring the device before each operation,
    as well as driving the chip select pin.
    """
    def __init__(self, breadboard, bus, cs_pin, mode, order, baud_rate):
        self.bb = breadboard
        self.bus = bus
        self.cs = cs_pin
        self.mode = mode
        self.order = order
        self.baud_rate = baud_rate
    
    def read(self, length):
        """Read data from the SPIDevice.

        :param length: number of bytes to read
        :type length: int
        :returns: device response
        :rtype: bytes
        """
        res = self.bb.mustExec([
            self.__cs(False),
            self.__configure(),
            self.bb.__opSpiRead(self.bus, length),
            self.__cs(True)
        ])
        return res[3].body

    def write(self, data):
        """Write data to the SPIDevice

        :param data: data to write
        :type data: bytes
        """
        self.bb.mustExec([
            self.__cs(False),
            self.__configure(),
            opSpiWrite(self.bus, data),
            self.__cs(True)
        ])

    def writeRead(self, data, delay, response_length):
        """Write data to SPIDevice then read response after a configurable delay.

        :param data: data to write
        :param delay: delay between write and read (in milliseconds)
        :param response_length: number of response bytes to read
        :type data: bytes
        :type delay: int
        :type response_length: int
        :returns: device response
        :rtype: bytes
        """
        res = self.bb.mustExec([
            self.__cs(False),
            self.__configure(),
            opSpiWrite(self.bus, data),
            opDelayMillis(delay),
            opSpiRead(self.bus, response_length),
            self.__cs(True)
        ])
        return res[5].body

    def __configure(self):
        return opSpiConfigure(self.bus, self.mode, self.order, self.baud_rate)
    
    def __cs(self, level):
        return opDigitalWrite(self.cs, level)

class Button:
    """A button
    """
    def __init__(self, breadboard, pin, polarity, debounce_time):
        self.bb = breadboard
        self.pin = pin
        self.bb.scanEnable(pin, polarity, debounce_time)
    
    def read(self):
        """Read the button state, including whether it is currently pressed, and the
        number of state transitions that have occurred since the last call to `read()`.

        :returns: button state; tuple of `(bool, int)` representing the whether the button is currently active, plus the number of state transitions that have occurred since the previous call to `read()`
        :rtype: (bool, int)
        """
        return self.bb.getScanState(self.pin)

#
# Operations

def opReset():
    return Op(OP_RESET, bytearray(0))

def opDelayMillis(ms):
    return Op(OP_DELAY_MS, struct.pack(">L", ms))

def opPinMode(pin, mode):
    return Op(OP_SET_PIN_MODE, struct.pack("BB", pin, mode))

def opDigitalRead(pin):
    return Op(OP_DIGITAL_READ, struct.pack("B", pin))

def opDigitalWrite(pin, level):
    return Op(OP_DIGITAL_WRITE, struct.pack("BB", pin, level))

def opScanEnable(pin, polarity, debounce_time):
    return Op(OP_SCANNER_ENABLE, struct.pack(">BBL", pin, polarity, debounce_time))

def opScanDisable(pin):
    return Op(OP_SCANNER_DISABLE, struct.pack("B", pin))

def opGetScanState(pin):
    return Op(OP_SCANNER_READ, struct.pack("B", pin))

def opPwmWrite(pin, duty):
    return Op(OP_PWM_SET_DUTY, struct.pack(">BB", pin, duty))

def opAnalogRead(pin):
    return Op(OP_ANALOG_READ, struct.pack("B", pin))

def opMotorOn(motor, direction, speed):
    return Op(OP_MOTOR_SET, struct.pack("BBB", motor, direction, speed))

def opLedSet(led, state):
    return Op(OP_LED_SET, struct.pack("BB", led, state))

def opLedToggle(led):
    return Op(OP_LED_TOGGLE, struct.pack("B", led))

def opSpiEnable(bus):
    return Op(OP_SPI_ENABLE, struct.pack("B", bus))

def opSpiConfigure(bus, mode, order, baud_rate):
    return Op(OP_SPI_CONFIGURE, struct.pack(">BBBL", bus, mode, order, baud_rate))

def opSpiWrite(bus, data):
    return Op(OP_SPI_WRITE, struct.pack(">BH", bus, len(data)) + data)

def opSpiRead(bus, length):
    return Op(OP_SPI_READ, struct.pack(">BH", bus, length))

def opSpiDisable(bus):
    return Op(OP_SPI_DISABLE, struct.pack("B", bus))