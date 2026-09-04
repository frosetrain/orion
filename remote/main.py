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

    def __init__(self, i2c: I2C, mux_channel: int, module_id: int = 0) -> None:
        """Init."""
        self.i2c = i2c
        self.mux_channel = mux_channel
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
        select_mux(self.i2c, self.mux_channel)
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
            min(255, m["red"]),  # // clear
            min(255, m["green"]),
            min(255, m["blue"]),  # * 255
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


# How long the hub's TX must stay high before we believe it is listening (ms).
# While identifying the device the hub pulls this line low for ~2 ms every ~16 ms,
# and while it still holds a stale connection it sends a keep-alive every 100 ms.
HUB_QUIET_MS = 150
# Per-call budget for watching the line before giving the main loop back (ms).
HUB_WATCH_MS = 170
# Time to wait after a failed handshake before trying again (ms).
CONNECT_RETRY_MS = 1000
# Slack added to the computed handshake transmission time when waiting for ACK (ms).
ACK_MARGIN_MS = 300
SLOW_BAUD = 2400


def handshake_bytes(self: lpf2.LPF2) -> bytes:
    """Build the whole LPF2 sync message (type, modes, baud, version, mode infos, ACK)."""
    msg = bytearray(self.setType(self.sensor_id))
    msg += self.defineModes()
    msg += self.defineBaud(115200)
    msg += self.defineVers("0.1", lpf2.__version__)
    num = len(self.modes) - 1
    for mode in reversed(self.modes):
        self.load_payload(b"\x00" * mode[8], num)  # Store empty payload for this mode
        plus_8 = 0x00
        n = num
        if n > 7:
            n -= 8
            plus_8 = lpf2.MSG_INFO_PLUS8
        msg += self.str_info(mode[0], n, lpf2.NAME | plus_8)
        msg += self.buildRange(mode[2], n, lpf2.RAW | plus_8)
        msg += self.buildRange(mode[3], n, lpf2.PCT | plus_8)
        msg += self.buildRange(mode[4], n, lpf2.SI | plus_8)
        msg += self.str_info(mode[5], n, lpf2.SYM | plus_8)
        msg += self.buildFunctMap(mode[6], n, lpf2.FUNCTION_MAP | plus_8)
        msg += self.buildFormat(mode[1], n, lpf2.FMT | plus_8)
        num -= 1
    msg += b"\x04"  # ACK: end of info
    return bytes(msg)


def pybricks_connect(self: lpf2.LPF2) -> None:
    """Connect to a Pybricks hub, replacing `lpf2.LPF2.connect`.

    Pybricks identifies a UART device by driving ID2 (our TX) high through a
    resistor and finding it held LOW, while ID1 (our RX) reads as open when the
    hub releases it. It needs 20 identical detections in a row, one per ~16 ms
    cycle, and only then opens its UART, sends a SPEED probe at 115200 and waits
    at 2400 baud for the TYPE message. Upstream `connect` holds TX low for a
    fixed 25 probe cycles and then sends TYPE whether or not the hub has got
    that far, so from a cold boot the first attempt usually loses the race.

    So instead: hold TX low, watch RX until the hub stops pulsing it (it idles
    high once its UART is open), and only then raise TX and send the handshake
    in one go. The ACK timeout covers the full transmission time at 2400 baud;
    with five modes that is ~1.5 s, longer than any fixed short timeout.

    Watching the line is bounded to HUB_WATCH_MS per call so the main loop keeps
    running while the hub is still identifying us (or absent). The hub program's
    PUPDevice() waits while the device is syncing and only errors when there is
    no device at all, so keeping the hub in "device present" state matters.
    """
    assert len(self.modes) > 0, "No modes (commands) defined"  # noqa: S101
    now = ticks_ms()
    if ticks_diff(now, getattr(self, "next_attempt", 0)) < 0:
        return  # Backing off after a failed handshake

    if not getattr(self, "waiting_for_hub", False):
        # First call since the last (re)connect: take the pins back from any
        # previous UART and present ourselves as a UART device.
        uart = getattr(self, "uart", None)
        if uart is not None:
            try:  # noqa: SIM105 (no contextlib on MicroPython)
                uart.deinit()
            except (AttributeError, OSError):
                pass
        # ID1 must read high (open) when the hub releases it, low would look like a motor
        self.rx_pin = Pin(self.RX_PIN_N, Pin.IN, Pin.PULL_UP)
        # ID2 held low against the hub's weak high is what marks a UART device
        self.tx_pin = Pin(self.TX_PIN_N, Pin.OUT, value=0)
        self.waiting_for_hub = True

    # Wait for the hub's TX to go quiet-high: detection finished, UART listening.
    # Busy-poll rather than sleep: the hub's low pulses are only ~2 ms wide.
    high_since = ticks_ms()
    end = ticks_add(high_since, HUB_WATCH_MS)
    while True:
        now = ticks_ms()
        if self.rx_pin.value() == 0:
            high_since = now
        elif ticks_diff(now, high_since) >= HUB_QUIET_MS:
            break
        if ticks_diff(now, end) >= 0:
            return  # Still being identified (or no hub). Try again next process().

    # Hub is listening at 2400 baud for TYPE. Send everything in one stream.
    self.waiting_for_hub = False
    msg = handshake_bytes(self)
    tx_ms = len(msg) * 10 * 1000 // SLOW_BAUD
    self.slow_uart()  # TX now idles high
    sleep_ms(5)  # Give the hub's UART a clean idle period after the long low
    deadline = ticks_add(ticks_ms(), tx_ms + ACK_MARGIN_MS)
    self.write(msg)
    self.flush()  # Drop anything received while the hub was at 115200

    while ticks_diff(deadline, ticks_ms()) > 0:  # Wait for ack
        if self.readchar() == lpf2.BYTE_ACK:
            self.connected = True
            break

    if self.connected:
        self.last_nack = ticks_ms()
        self.fast_uart()  # Hub switches to 115200 10 ms after its ACK
        logger.info("Connected to hub with sensor id %s", self.sensor_id)
    else:
        self.next_attempt = ticks_add(ticks_ms(), CONNECT_RETRY_MS)
        logger.error("Failed to connect to hub, retrying in %s ms", CONNECT_RETRY_MS)


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


def read_scanning_rgb() -> tuple[int, ...]:
    """Read the RGB values from the scanning TCS sensors."""
    rgbs = [(-1, -1, -1) for _ in scanning_tcs]
    for i, s in enumerate(scanning_tcs):
        rgb = s.rgb()
        if rgb is not None:
            rgbs[i] = rgb
            logger.debug("sens %s: %s", i + 4, rgbs[i])  # Addresses start from 4
        elif s.device_id is not None:
            # Exists but not sending data, because it was just reconnected. Reconfigure.
            logger.info("sens %s: reconnected, reconfiguring", i + 4)
            s.configure()
        else:
            logger.error("sens %s: gone", i + 4)
        # print("rgb", i, rgbs[i][0], rgbs[i][1], rgbs[i][2])
    return sum(rgbs, ())  # Flatten rgbs into a 9-tuple


def read_distances() -> tuple[int, ...]:
    """Read the distances in mm from the TOFSense sensors (-1 if invalid)."""
    distances = [-1 for _ in tofsenses]
    for i, t in enumerate(tofsenses):
        d = t.get_distance()
        if d is not None:
            distances[i] = d
            logger.debug("tof %s: %s mm", i, d)
        else:
            logger.error("tof %s: invalid reading", i)
    return tuple(distances)


# Override the built-in connect function with the Pybricks-aware version
lpf2.LPF2.connect = pybricks_connect

# Onboard NeoPixel
pixels = NeoPixel(Pin(25), 1)
pixels.fill((0, 0, 0))
pixels.write()

# I2C bus behind the TCA9548A mux
i2c = I2C(0, scl=Pin(4), sda=Pin(5), freq=400000)

# TOFSense-F2 Mini. Each sits on its own mux channel, so they can share an address.
tofsenses = [TOFSenseF2Mini(i2c, mux_channel=channel, module_id=5) for channel in (2, 3)]

# TCS3472
tcs_i2c = i2c
linetrack_tcs = []
for i in range(2):
    tcs = TCS3472(tcs_i2c, mux_channel=i, integration_ms=2.4, gain=16)
    linetrack_tcs.append(tcs)
scanning_tcs = []
for i in (4, 5, 6):
    tcs = TCS3472(tcs_i2c, mux_channel=i, integration_ms=2.4, gain=60)
    scanning_tcs.append(tcs)

# Servo
servos = []
servo_pins = (22,)
for pin in servo_pins:
    servo_pwm = PWM(Pin(pin), freq=50)
    servo = Servo(servo_pwm)
    servos.append(servo)

remote_enable = True
remote_disable_jumper = Pin(15, Pin.IN, Pin.PULL_DOWN)
if remote_disable_jumper.value() == 1:
    # Jumper between pin 15 and 3V3, disable connecting to SPIKE Prime
    logger.warning("REMOTE TO SPIKE PRIME DISABLED")
    remote_enable = False

if remote_enable:
    # Connection to hub
    remote = PUPRemoteSensor(power=True, max_packet_size=16)
    # remote.lpup.debug = True
    remote.add_command("beacn", "", "B")
    remote.add_command("cserv", "", "Bh")
    remote.add_channel("brigh", "BB")
    remote.add_channel("rgb", "9B")
    remote.add_channel("dist", "hh")

flash_pattern = [
    (0, (255, 255, 255)),
    (50, (0, 0, 0)),
    (100, (255, 255, 255)),
    (150, (0, 0, 0)),
    (500, (255, 0, 0)),
    (600, (0, 0, 0)),
]
# Start the strobe straight away so the LED shows the loop is alive from boot
beacon_state = 0
beacon_start_time = ticks_ms()
pixels[0] = flash_pattern[0][1]
pixels.write()
prev_connected = False

frame_count = 0
while True:
    connected = remote.process() if remote_enable else True

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
        beacon_start_time = ticks_add(beacon_start_time, 1000)  # Prevents gradual drifting
        if ticks_diff(ticks_ms(), beacon_start_time) > 1000:
            # Still over a second behind: the loop was blocked (e.g. inside
            # connect()). Resync instead of replaying every missed second.
            beacon_start_time = ticks_ms()
        logger.info("FPS: %s", frame_count)
        frame_count = 0

    # Read TCS sensors
    brightnesses = [-1 for _ in linetrack_tcs]
    for i, s in enumerate(linetrack_tcs):
        b = s.clear_channel()
        if b is not None:
            brightnesses[i] = b
        elif s.device_id is not None:
            # Exists but not sending data, because it was just reconnected. Reconfigure.
            logger.info("sens %s: reconnected, reconfiguring", i)
            s.configure()
        else:
            logger.error("sens %s: gone", i)

    # Send line sensor brightnesses (-1 if reading is None)
    left_brightness = brightnesses[0]
    right_brightness = brightnesses[1]
    logger.debug("Sending brightnesses %s, %s", left_brightness, right_brightness)
    if remote_enable:
        remote.update_channel("brigh", left_brightness, right_brightness)
        # Only read the scanning sensors while the hub has the rgb mode
        # selected (during voting); skipping them keeps line-tracking FPS up.
        if remote.lpup.current_mode == remote.modes["rgb"]:
            remote.update_channel("rgb", *read_scanning_rgb())
        # Likewise, only read the TOF sensors while the hub has the dist mode selected.
        elif remote.lpup.current_mode == remote.modes["dist"]:
            remote.update_channel("dist", *read_distances())

    frame_count += 1
