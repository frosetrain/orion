from time import ticks_diff, ticks_ms

from machine import Pin
from neopixel import NeoPixel
from pupremote import PUPRemoteSensor


def beacn(*argv):
    """Flash the NeoPixel red (like a beacon light)."""
    global beacon_state, beacon_start_time
    pixels[0] = (50, 50, 50)
    pixels.write()
    beacon_state = 0
    beacon_start_time = ticks_ms()
    return 0


pixels = NeoPixel(Pin(25), 1)
pixels.fill((0, 0, 0))
pixels.write()

remote = PUPRemoteSensor(power=True, max_packet_size=16)
remote.add_command("beacn", "b", "b")
remote.add_channel("clock", "i")

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
count = 0

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

    remote.update_channel("clock", count)
    count += 1
