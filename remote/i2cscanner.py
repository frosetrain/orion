from machine import I2C, Pin

i2c = I2C(0, scl=Pin(4), sda=Pin(5))

for channel in range(8):
    i2c.writeto(0x70, bytes([1 << channel]))
    devices = i2c.scan()
    print(f"Devices on channel {channel}: {devices}")
