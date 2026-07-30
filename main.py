"""Pybricks code for SPIKE Prime brain."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

BLACK_THRESHOLD = 40
WHITE_THRESHOLD = 45
LIGHT_SENSOR_Y = 60

# Initialise hardware
hub = PrimeHub()
left_motor = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
# right_motor.control.limits(1700, 10000, 800)
# print(right_motor.control.limits())
db = DriveBase(left_motor, right_motor, 62.4, 167)
db.use_gyro(True)
db.settings(straight_speed=750, straight_acceleration=1000, turn_rate=320, turn_acceleration=800)
print("db settings:", db.settings())

# Initialise connection to remote
remote = PUPRemoteHub(Port.B, max_packet_size=16)
remote.add_command("beacn", "b", "b")
remote.add_channel("brigh", "BB")

beacon_timer = StopWatch()
beacon_flashes = 0


def linetrack(
    junction_type: str,
    start: int,
    stretch: int,
    theoretical: int,
    *,
    move_forward: bool = True,
    linetrack: bool = True,
) -> None:
    """Line track.

    Args:
        junction_type (str): Left, both, or right.
        start (str): How far to drive before going quickly.
        stretch (int): How far to drive before slowing down and enabling detection.
        theoretical (int): The theoretical distance to line track.
        move_forward (bool): Whether to go forward to get axle on the line. Defaults to True.
        linetrack (bool): Whether to linetrack. Defaults to True.

    """
    slow = True
    db.reset()

    while True:
        stretch = max(0, theoretical - LIGHT_SENSOR_Y - 100)
        drive_limit = theoretical - LIGHT_SENSOR_Y + 50
        if slow and db.distance() > start:
            slow = False
        if not slow and db.distance() > stretch:
            slow = True

        # Receive brightness data
        brightness_rx = remote.call("brigh")
        left_brightness = brightness_rx[0]
        right_brightness = brightness_rx[1]
        print(left_brightness, right_brightness)

        # Detect junctions
        if junction_type == "left":
            hit = left_brightness < BLACK_THRESHOLD and right_brightness > WHITE_THRESHOLD
        elif junction_type == "both":
            hit = left_brightness < BLACK_THRESHOLD and right_brightness < BLACK_THRESHOLD
        elif junction_type == "right":
            hit = left_brightness > WHITE_THRESHOLD and right_brightness < BLACK_THRESHOLD
        else:
            hit = False
        if (hit and db.distance() > stretch) or db.distance() > drive_limit:
            if move_forward:
                db.straight(LIGHT_SENSOR_Y, then=Stop.BRAKE)
            else:
                db.brake()
            break

        # Follow the line and drive
        difference = left_brightness - right_brightness if linetrack else 0

        if db.distance() < start:
            db.drive(200, difference * 1.5)
        elif db.distance() > stretch:
            db.drive(200, difference * 0.8)
        else:
            db.drive(500, difference * 0.8)


# remote.call("beacn", 67)

while True:
    db.turn(60)
    db.turn(-120)
    db.curve(130, 120)
    db.curve(130, -120)
    db.turn(60)
    db.straight(-450)

# Side quest 3
# db.curve(99, 90)
# linetrack("left", 50, 0, 219)
# db.turn(-90)
# while True:
#     linetrack("right", 50, 0, 423)
#     db.turn(90)
#     linetrack("both", 50, 0, 407)
#     db.straight(534)
#     linetrack("both", 200, 0, 407)
#     db.turn(-90)
#     linetrack("right", 50, 0, 249)
#     db.turn(90)
#     linetrack("both", 50, 0, 295)
#     db.turn(180)
#     linetrack("left", 200, 0, 295)
#     db.turn(-90)
#     linetrack("right", 50, 0, 672)
#     db.turn(90)
#     linetrack("both", 50, 0, 674)
#     db.turn(-90)
#     linetrack("both", 50, 0, 97)
#     db.turn(180)
#     linetrack("both", 50, 0, 97)
#     db.turn(-90)
#     linetrack("right", 50, 0, 674)
#     db.turn(90)
