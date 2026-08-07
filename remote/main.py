"""LMS-ESP32 program for oRIon."""

import logging
from time import sleep_ms, ticks_add, ticks_diff, ticks_ms

import lpf2
from machine import I2C, PWM, Pin
from neopixel import NeoPixel
from pupremote import PUPRemoteSensor

MUX_ADDRESS = 0x70
selected_channel = None
logging.basicConfig(format="%(chrono)s [%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def select_mux(i2c: I2C, channel: int) -> None:
    """Point the mux at `channel`, unless it is already there."""
    global selected_channel
    if selected_channel != channel:
        try:
            i2c.writeto_mem(MUX_ADDRESS, 0, bytes([1 << channel]))
            selected_channel = channel
        except OSError:
            logger.exception("Mux not found")


class TOFSenseF2Mini:
    """Driver for the Nooploop TOFSense-F2 Mini in I2C mode.

    The 7-bit slave address is 0x08 + module id. Change the id for each sensor.
    """

    BASE_ADDRESS = 0x08
    REG_DISTANCE = 0x24

    def __init__(self, i2c: I2C, module_id: int = 0) -> None:
        """Init."""
        self.i2c = i2c
        self.address = self.BASE_ADDRESS + module_id

    def read(self) -> dict[str, int | bool]:
        """Read a full measurement.

        Returns a dict with:
            distance_mm      distance in millimetres (may be negative/garbage
                             when status != 0)
            status           0 = normal, non-zero = unreliable / out of range
            signal_strength  raw signal strength
            valid            True when status == 0
        """
        # distance(3) + status(1) + signal strength(2)
        try:
            data = self.i2c.readfrom_mem(self.address, self.REG_DISTANCE, 6)
        except OSError:
            return {"distance_mm": 0, "status": 1, "signal_strength": 0, "valid": False}

        raw = data[0] | (data[1] << 8) | (data[2] << 16)
        if raw & 0x800000:  # sign-extend the 24-bit value
            raw -= 0x1000000

        status = data[3]
        signal_strength = data[4] | (data[5] << 8)

        return {
            "distance_mm": raw,
            "status": status,
            "signal_strength": signal_strength,
            "valid": status == 0,
        }

    def get_distance(self) -> int | None:
        """Return distance in millimetres, or None if the reading is invalid."""
        m = self.read()
        return m["distance_mm"] if m["valid"] else None


class TCS3472:
    """Driver for the AMS TCS3472 (TCS34725/34727) RGB colour sensor in I2C mode."""

    # Fixed adddress
    ADDRESS = 0x29

    # Command register: bit 7 = command, bit 5 = auto-increment
    CMD = 0x80
    CMD_AUTO = 0xA0

    # Register addresses (without the command bit)
    REG_ENABLE = 0x00
    REG_ATIME = 0x01
    REG_CONTROL = 0x0F
    REG_ID = 0x12
    REG_STATUS = 0x13
    REG_CDATAL = 0x14  # start of Clear/Red/Green/Blue data (8 bytes)

    # ENABLE register bits
    ENABLE_PON = 0x01  # power on
    ENABLE_AEN = 0x02  # ADC (RGBC) enable

    # STATUS register bits
    STATUS_AVALID = 0x01  # RGBC data valid

    # Supported gains; the tuple index is the CONTROL register value.
    GAINS = (1, 4, 16, 60)

    def __init__(self, i2c: I2C, mux_channel: int, integration_ms: float, gain: int) -> None:
        """Init."""
        self.i2c = i2c
        self.mux_channel = mux_channel
        cycles = min(256, max(1, round(integration_ms / 2.4)))
        self._integration_ms = cycles * 2.4
        self._atime = 256 - cycles
        self._control = self.GAINS.index(gain) if gain in self.GAINS else 1
        self.configure()

    def configure(self) -> None:
        """Write the integration time, gain and enable the sensor.

        Safe to call at any time: register writes to an absent sensor are
        silently ignored, so this doubles as a reconnect routine after the
        sensor has been unplugged (which resets all registers to defaults).
        """
        self._write(self.REG_ATIME, self._atime)
        self._write(self.REG_CONTROL, self._control)
        self.enable()

    def _write(self, reg: int, value: int) -> None:
        """Write a single byte to a register."""
        select_mux(self.i2c, self.mux_channel)
        try:
            self.i2c.writeto_mem(self.ADDRESS, self.CMD | reg, bytes([value]))
        except OSError:
            return

    def _read(self, reg: int, count: int = 1) -> bytes | None:
        """Read one or more bytes from a register."""
        select_mux(self.i2c, self.mux_channel)
        try:
            return self.i2c.readfrom_mem(self.ADDRESS, self.CMD_AUTO | reg, count)
        except OSError:
            return None

    def _get_color(self) -> dict[str, int | bool] | None:
        """Read the raw 16-bit Clear, Red, Green and Blue channels.

        Returns a dict with:
            clear, red, green, blue   raw 16-bit counts (0..65535)
            valid                     True when the reading is complete
        """
        data = self._read(self.REG_STATUS, 9)
        if data is None:
            return None
        clear = data[1] | (data[2] << 8)
        red = data[3] | (data[4] << 8)
        green = data[5] | (data[6] << 8)
        blue = data[7] | (data[8] << 8)
        return {
            "clear": clear,
            "red": red,
            "green": green,
            "blue": blue,
            "valid": bool(data[0] & self.STATUS_AVALID),
        }

    @property
    def device_id(self) -> int | None:
        """Read the device ID (0x44 = TCS34725, 0x4D = TCS34727)."""
        reg_id = self._read(self.REG_ID)
        if reg_id is None:
            return None
        return reg_id[0]

    def enable(self) -> None:
        """Power on the sensor and enable the RGBC ADC."""
        self._write(self.REG_ENABLE, self.ENABLE_PON)
        sleep_ms(3)  # 2.4 ms warm-up required before enabling the ADC
        self._write(self.REG_ENABLE, self.ENABLE_PON | self.ENABLE_AEN)

    def disable(self) -> None:
        """Put the sensor into low-power sleep."""
        self._write(self.REG_ENABLE, 0x00)

    def rgb(self) -> tuple[int, int, int] | None:
        """Return (r, g, b) scaled to 0..255 relative to the clear channel.

        Returns None when the reading is invalid or the clear channel is 0.
        """
        m = self._get_color()
        if m is None:
            return None
        clear = m["clear"]
        if not m["valid"] or clear == 0:
            return None
        return (
            min(255, m["red"] * 255 // clear),
            min(255, m["green"] * 255 // clear),
            min(255, m["blue"] * 255 // clear),
        )

    def brightness(self) -> int | None:
        """Return the brightness of the sensor's RGB reading from 0..255."""
        rgb = self.rgb()
        if rgb is None:
            return None
        return (rgb[0] + rgb[1] + rgb[2]) // 3

    def clear_channel(self) -> int | None:
        """Return the value of the clear channel (overall brightness) scaled to 0..255.

        Raw 16-bit value (out of 65535) should be under 1024 for our 2.4 ms integration time.
        """
        m = self._get_color()
        if m is None:
            return None
        clear = m["clear"]
        if not m["valid"] or clear == 0:
            return None
        return min(255, clear // 4)


class Servo:
    """Driver for a servo motor using PWM."""

    def __init__(self, pwm: PWM) -> None:
        """Init."""
        self.pwm = pwm

    def set_angle(self, angle: int) -> None:
        """Move the servo motor using PWM.

        Map angle (0-180) to duty cycle
        Typical range: 0.5ms (0°) to 2.5ms (180°) pulse width
        """
        min_duty = 500_000  # 0.5 ms
        max_duty = 2_500_000  # 2.5 ms
        duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
        self.pwm.duty_ns(duty)


def patched_connect(self: lpf2.LPF2) -> None:
    """Connect to the hub, using a fixed fast/slow UART probe.

    Upstream `lpf2.LPF2.connect` counts loop iterations as milliseconds, so
    interpreter overhead and GC pauses make the high-pulse measurement under-read.
    The hub pulses at 16 ms, right on the `n < 16` boundary, so the probe randomly
    picks the fast UART branch and then runs the handshake at 115200 against a hub
    still listening at 2400. Measure elapsed time with ticks_ms and leave margin.
    """
    assert len(self.modes) > 0, "No modes (commands) defined"  # noqa: S101
    fast_uart_hub = False
    self.init_pins()
    self.wrt_tx_pin(1, 5)  # Say hello!
    self.wrt_tx_pin(0, 0)
    for attempt in range(25):  # Wait for AOK
        start = ticks_ms()
        n = 0
        while self.rx_pin.value() == 1:
            sleep_ms(1)
            n = ticks_diff(ticks_ms(), start)
            if n > 20:
                break
        if self.debug:
            logger.debug("probe %s: falling after ms high: %s", attempt, n)
        if attempt > 10 and (n > 21 or n < 13):
            fast_uart_hub = True
            break
        while self.rx_pin.value() == 0:
            sleep_ms(1)  # Wait until rise again

    if fast_uart_hub:
        self.fast_uart()
        sleep_ms(5)
        self.write(b"\x04")
    else:
        self.slow_uart()
        self.write(b"\x00")
    self.write(self.setType(self.sensor_id))
    self.write(self.defineModes())  # tell how many modes
    self.write(self.defineBaud(115200))
    self.write(self.defineVers("0.1", lpf2.__version__))
    num = len(self.modes) - 1
    for mode in reversed(self.modes):
        sleep_ms(20)
        self.setupMode(mode, num)
        num -= 1

    self.write(b"\x04")  # ACK
    end = ticks_add(ticks_ms(), 2500)
    while ticks_diff(end, ticks_ms()) > 0:  # Wait for ack
        if self.readchar() == 0x04:  # BYTE_ACK
            self.connected = True
            break

    if self.connected:
        self.last_nack = ticks_ms()
        logger.info("Connected to hub with sensor id %s", self.sensor_id)
        if not fast_uart_hub:
            self.fast_uart()
    else:
        logger.error("Failed to connect to hub")


def beacn(*_: None) -> None:
    """Flash the NeoPixel like a beacon and strobe light."""
    global beacon_state, beacon_start_time
    pixels[0] = (255, 255, 255)
    pixels.write()
    beacon_state = 0
    beacon_start_time = ticks_ms()


def cserv(servo_num: int, angle: int) -> None:
    """PUP command to control the servo."""
    logger.info("Servo %s to %s deg", servo_num, angle)
    servos[servo_num].set_angle(angle)


# Override the built-in connect function with our patched version
lpf2.LPF2.connect = patched_connect

# Onboard NeoPixel
pixels = NeoPixel(Pin(25), 1)
pixels.fill((0, 0, 0))
pixels.write()

# TOFSense-F2 Mini
# tof_i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
# tofsense = TOFSenseF2Mini(tof_i2c, 2)

# TCS3472
tcs_i2c = I2C(0, scl=Pin(4), sda=Pin(5), freq=400000)
tcs_line_sensors = []
for i in range(2):
    tcs = TCS3472(tcs_i2c, mux_channel=i, integration_ms=2.4, gain=16)
    tcs_line_sensors.append(tcs)

# Servo
servos = []
servo_pins = (22,)
for pin in servo_pins:
    servo_pwm = PWM(Pin(pin), freq=50)
    servo = Servo(servo_pwm)
    servos.append(servo)

beacon_state = 67
beacon_start_time = 0
flash_pattern = [
    (0, (255, 255, 255)),
    (50, (0, 0, 0)),
    (100, (255, 255, 255)),
    (150, (0, 0, 0)),
    (500, (255, 0, 0)),
    (600, (0, 0, 0)),
]
prev_connected = False

# Connection to hub
remote = PUPRemoteSensor(power=True, max_packet_size=16)
# remote.lpup.debug = True
remote.add_command("beacn", "", "B")
remote.add_command("cserv", "", "Bh")
remote.add_channel("brigh", "BB")

frame_count = 0
while True:
    connected = remote.process()
    # Show black light when not connected to hub
    if prev_connected and not connected:
        pixels[0] = (0, 0, 0)
        pixels.write()
        prev_connected = False
        continue

    # Show blue light when connected to hub (when hub code is not running)
    if not prev_connected and connected:
        pixels[0] = (0, 0, 255)
        pixels.write()
        prev_connected = True

    # Show strobe and beacon colour
    if beacon_state <= 4:
        state_switch_time = flash_pattern[beacon_state + 1][0]
        if ticks_diff(ticks_ms(), beacon_start_time) > state_switch_time:
            beacon_state += 1
            pixels[0] = flash_pattern[beacon_state][1]
            pixels.write()
    if ticks_diff(ticks_ms(), beacon_start_time) > 1000:
        # Restart the sequence
        pixels[0] = flash_pattern[0][1]
        pixels.write()
        beacon_state = 0
        beacon_start_time += 1000  # Prevents gradual drifting
        logger.debug("FPS: %s", frame_count)
        frame_count = 0

    # distance = tofsense.get_distance()
    # logger.debug(f"Distance: {distance} mm")

    brightnesses = [-1 for _ in tcs_line_sensors]
    for i, s in enumerate(tcs_line_sensors):
        reading = s.clear_channel()
        if brightnesses[i] is not None:
            brightnesses[i] = reading
            logger.debug("sens %s: %s", i, brightnesses[i])
        elif s.device_id is not None:
            # Exists but not sending data, because it was just reconnected. Reconfigure.
            logger.info("sens %s: reconnected, reconfiguring", i)
            s.configure()
        else:
            logger.error("sens %s: gone", i)

    # Send -1 if reading is None
    left_brightness = brightnesses[1]
    right_brightness = brightnesses[0]
    logger.debug("Sending brightnesses %s, %s", left_brightness, right_brightness)
    remote.update_channel("brigh", left_brightness, right_brightness)

    frame_count += 1
