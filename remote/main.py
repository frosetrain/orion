"""LMS-ESP32 program for oRIon."""

import logging
from time import sleep_ms, ticks_diff, ticks_ms

from machine import I2C, Pin
from neopixel import NeoPixel
from pupremote import PUPRemoteSensor

MUX_ADDRESS = 0x70
selected_channel = None
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def select_mux(i2c: I2C, channel: int) -> None:
    """Point the mux at `channel`, unless it is already there."""
    global selected_channel
    if selected_channel != channel:
        try:
            i2c.writeto_mem(MUX_ADDRESS, 0, bytes([1 << channel]))
        except OSError:
            logger.exception("Mux not found")
        selected_channel = channel


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


def beacn(*_: list) -> int:
    """Flash the NeoPixel red (like a beacon light)."""
    global beacon_state, beacon_start_time
    pixels[0] = (50, 50, 50)
    pixels.write()
    beacon_state = 0
    beacon_start_time = ticks_ms()
    return 67


# Connection to hub
remote = PUPRemoteSensor(power=True, max_packet_size=16)
remote.add_command("beacn", "b", "b")
remote.add_channel("brigh", "BB")

# Onboard NeoPixel
pixels = NeoPixel(Pin(25), 1)
pixels.fill((0, 0, 0))
pixels.write()

# TOFSense-F2 Mini
# tof_i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
# tofsense = TOFSenseF2Mini(tof_i2c, 2)

# TCS3472
tcs_i2c = I2C(0, scl=Pin(4), sda=Pin(5), freq=400000)
tcs_sensors = []
for i in range(2):
    tcs = TCS3472(tcs_i2c, mux_channel=i, integration_ms=2.4, gain=16)
    tcs_sensors.append(tcs)

beacon_state = 67
beacon_start_time = 0
flash_pattern = [
    (0, (50, 50, 50)),
    (50, (0, 0, 0)),
    (100, (50, 50, 50)),
    (150, (0, 0, 0)),
    (500, (100, 0, 0)),
    (600, (0, 0, 0)),
]
prev_connected = False

while True:
    connected = remote.process()

    # Show blue light when connected to hub (when hub code is not running)
    if not prev_connected and connected:
        pixels[0] = (0, 0, 100)
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
        beacn()  # Restart the sequence

    # distance = tofsense.get_distance()
    # print(f"Distance: {distance} mm")

    brightnesses = [None for _ in range(2)]
    for i, s in enumerate(tcs_sensors):
        brightnesses[i] = s.clear_channel()
        if brightnesses[i] is not None:
            # logger.debug(f"sens {i}: rgb {rgb[0]} {rgb[1]} {rgb[2]}")
            logger.info("sens %s: %s", i, brightnesses[i])
        elif s.device_id is not None:
            # Exists but not sending data, because it was just reconnected. Reconfigure.
            logger.info("sens %s: reconnected, reconfiguring", i)
            s.configure()
        else:
            logger.error("sens %s: gone", i)

    # Send -1 if reading is None
    left_brightness = brightnesses[1] or -1
    right_brightness = brightnesses[0] or -1
    print("Sending brightnesses", left_brightness, right_brightness)
    remote.update_channel("brigh", left_brightness, right_brightness)
