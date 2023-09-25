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

REVERSE                     = 0
FORWARD                     = 1

BUITLIN_POTENTIOMETER       = 24
BUILTIN_BUTTON              = 25

OP_RESET			        = 0x00

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
    
    Returns
    -------
    breadboard
        Breadboard instance

    Raises
    ------
    BreadboardNotFound
        If no breadboard can be found
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

    Returns
    -------
    breadboard
        Breadboard instance
    """
    while True:
        try:
            return findBreadboard()
        except BreadboardNotFound:
            time.sleep(1)
        
class Breadboard():
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
        
        Parameters
        ----------
        ops
            List of operations
        continue_on_error: bool
            Set to True if the breadboard should continue processing
            successive operations when an operation fails.
        
        Returns
        -------
        results
            List of results, one per operation
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

        Parameters
        ----------
        ops
            List of operations
        continue_on_error: bool
            Set to True if the breadboard should continue processing
            successive operations when an operation fails.
        
        Returns
        -------
        results
            List of results, one per operation
        
        Raises
        ------
        OperationFailed
            If any of the operations failed
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

    def delayMillis(self, ms):
        """Instruct the breadboard to delay for the given number of milliseconds"""
        self.mustExec([ opDelayMillis(ms) ])
    
    def pinMode(self, pin, mode):
        """Set a pin's mode
        
        Parameters
        ----------
        pin: int
            Pin number
        mode: int
            Pin mode, one of the PIN_MODE constants
        """
        self.mustExec([ opPinMode(pin, mode) ])
        
    def digitalRead(self, pin):
        """Read pin level. If the pin is not currently configured as an input
        the breadboard will first attempt to change its mode.
        
        Parameters
        ----------
        pin: int
            Pin number

        Returns
        -------
        level: bool
            Pin level
        """
        res = self.mustExec([ opDigitalRead(pin) ])
        return res[0].body[0] > 0

    def digitalWrite(self, pin, level):
        """Write pin level. If the pin is not currently configured as an output
        the breadboard will first attempt to change its mode.
        
        Parameters
        ----------
        pin: int
            Pin number
        level: bool
            Level
        """
        self.mustExec([ opDigitalWrite(pin, level) ])

    # def digitalWait(self, pin, level, settle_us = 0, timeout_us = 0):
    # 	self.mustExec([ Op(OP_DIGITAL_WAIT, struct.pack(">BBLL", pin, level, settle_us, timeout_us)) ])

    def createButton(self, pin, polarity = False, debounce_time = 0):
        """Create a button
        
        Parameters
        ----------
        pin: int
            Pin number
        polarity: bool
            Pin level that is considered "pressed"
        debounce_time: int
            Number of milliseconds pin level must stay at the pressed level
            before it is considered to have been pressed.
        
        Returns
        -------
        button
            Button instance
        """
        return Button(self, pin, polarity, debounce_time)

    def scanEnable(self, pin, polarity = False, debounce_time = 0):
        """Enable scanning on the specified pin. While scanning is enabled on a pin
        the breadboard will continuously monitor its state, keeping track of the
        number of state changes that occur between each call to getScanState().

        Scanning allows buttons (or other inputs) to be monitored without having
        to continuously poll the pin state, and without loss of information.
        
        Parameters
        ----------
        pin: int
            Pin number
        polarity: bool
            Pin level that is considered "active"
        debounce_time: int
            Number of milliseconds pin must be stay at active level before it is
            considered to have changed to the active state.
        """
        self.mustExec([ opScanEnable(pin, polarity, debounce_time) ])

    def scanDisable(self, pin):
        """Disable scanning on the specified pin.
        
        Parameters
        ----------
        pin: int
            Pin number
        """
        self.mustExec([ opScanDisable(pin) ])
    
    def getScanState(self, pin):
        """Query the scan state for the specified pin
        
        Parameters
        ----------
        pin: int
            Pin number

        Returns
        -------
        state
            Tuple of (bool, int), representing the current active state of the
            scanner, plus the number of state transitions that have occurred
            since the last call to getScanState()
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
        device will first attempt to enable it.
        
        Parameters
        ----------
        pin: int
            pin number
        duty: int
            duty (0-255)
        """
        self.mustExec([ opPwmWrite(pin, duty) ])

    # TODO: analogInit
    # TODO: analogConfigure
    # TODO: analogDisable

    def analogRead(self, pin):
        """Read ADC level. If pin is not currently configured as an ADC, the
        the device will first attempt to enable it.
        
        Parameters
        ----------
        pin: int
            pin number
        
        Returns
        -------
        adc_level
            ADC level (0-1023)
        """
        res = self.mustExec([ opAnalogRead(pin) ])
        b = res[0].body
        return b[0] << 8 | b[1]

    def motorOn(self, motor, direction, speed):
        """Turn on the specified motor.
        
        Parameters
        ----------
        motor: int
            motor number
        direction: bool
            motor direction (forwards: true, backwards: false)
        speed: int
            motor speed (0-255)
        """
        self.mustExec([ opMotorOn(motor, direction, speed) ])
    
    def motorOff(self, motor):
        """Turn off the specified motor. Equivalent to calling motorOn with a speed of zero.
        
        Parameters
        ----------
        motor: int
            motor number
        """
        return self.motorOn(motor, FORWARD, 0)
    
    def ledSet(self, led, state):
        """Set an onboard LED's state.
        
        Parameters
        ----------
        led: int
            LED number (0-3)
        state: bool
            on (True) or off (False)
        """
        self.mustExec([ opLedSet(led, state) ])
    
    def ledToggle(self, led):
        """Toggle an onboard LED's state.
        
        Parameters
        ----------
        led: int
            LED number (0-3)
        """
        self.mustExec([ opLedToggle(led) ])
    
    def createSPIDevice(self, bus, cs_pin, mode, order, baud_rate):
        """Create a SPIDevice on the specified bus.
        
        Parameters
        ----------
        bus: int
            Bus number (0-2)
        cs_pin: int
            Chip-select pin
        mode: int
            SPI mode
        order: int
            Byte order
        baud_rate: int
            Baud rate
        
        Returns
        -------
        spi_device
            SPIDevice instance
        """
        return SPIDevice(self, bus, cs_pin, mode, order, baud_rate)

    def spiEnable(self, bus):
        """Enable SPI on the specified bus.
        
        Parameters
        ----------
        bus: int
            Bus number (0-2)
        """
        self.mustExec([ opSpiEnable(bus) ])

    def spiConfigure(self, bus, mode, order, baud_rate):
        """Configure SPI on the specified bus.
        
        Parameters
        ----------
        bus: int
            Bus number (0-2)
        mode: int
            SPI mode
        order: int
            Byte order
        baud_rate: int
            Baud rate
        """
        self.mustExec([ opSpiConfigure(bus, mode, order, baud_rate) ])

    def spiWrite(self, bus, data):
        """Write SPI data.
        
        Parameters
        ----------
        bus: int
            Bus number (0-2)
        data: bytes
            Data to write
        """
        data = self.__coerceToBytes(data)
        self.mustExec([ opSpiWrite(bus, data) ])

    def spiRead(self, bus, length):
        """Read SPI data.
        
        Parameters
        ----------
        bus: int
            Bus number (0-2)
        length:
            Number of bytes to read
        
        Returns
        -------
        data:
            Data read from bus
        """
        res = self.mustExec([ opSpiRead(bus, length) ])
        return res[0].body

    def spiDisable(self, bus):
        """Disable SPI on the given bus.
        
        Parameters
        ----------
        bus: int	
            Bus number (0-2)
        """
        self.mustExec([ opSpiDisable(bus) ])

    def decodeMorse(self, pin=None, min_press_time=20, dash_press_threshold=250, inter_character_time=300, inter_word_time=700):
        """Activate morse code decoding on the specified pin.
        
        Once activated, morse code decoding can only be deactivated by power-cycling
        the breadboard.

        Parameters
        ----------
        pin: int
            Pin to use as the input signal; if unspecified, the internal button is used.
        min_press_time: int
            Minimum time button must be low to be considered valid (in milliseconds)
        dash_press_threshold: int
            Press time required to be interpreted as a dash (in milliseconds)
        inter_character_time: int
            Release time required before next press is considered a new letter (in milliseconds)
        inter_word_time: int
            Release time required before emitted a word separator (ASCII space) (in milliseconds)
        
        Returns
        -------
        generator
            A generator that yields each character as it is read
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
        
        Parameters
        ----------
        length: int
            Number of bytes to read

        Returns
        -------
        data: bytes
            Response from device
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
        
        Parameters
        ----------
        data: bytes
            Data to write
        """
        self.bb.mustExec([
            self.__cs(False),
            self.__configure(),
            opSpiWrite(self.bus, data),
            self.__cs(True)
        ])

    def writeRead(self, data, delay, response_length):
        """Write data to SPIDevice then read response after a configurable delay.
        
        Parameters
        ----------
        data: bytes
            Data to write
        delay: int
            Delay between write and read (in milliseconds)
        response_length: int
            Number of response bytes to read
        
        Returns
        -------
        data: bytes
            Response from device
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
    def __init__(self, breadboard, pin, polarity, debounce_time):
        self.bb = breadboard
        self.pin = pin
        self.bb.scanEnable(self, pin, polarity, debounce_time)
    
    def query(self):
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
    return Op(OP_SCANNER_ENABLE, struct.pack("B", pin))

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