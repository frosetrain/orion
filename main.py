from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

BLACK_THRESHOLD = 67
WHITE_THRESHOLD = 0
LIGHT_SENSOR_X = 60

# Initialise hardware
hub = PrimeHub()
left_motor = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
# right_motor.control.limits(1700, 10000, 800)
# print(right_motor.control.limits())
db = DriveBase(left_motor, right_motor, 62.4, 164)
db.settings(straight_speed=750, straight_acceleration=2000, turn_rate=320, turn_acceleration=800)
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
    *,
    move_forward: bool = True,
    linetrack: bool = True,
) -> None:
    """Line track.

    Args:
        junction_type (str): Left, both, or right.
        start (str): How far to drive before going quickly.
        stretch (int): How far to drive before slowing down and enabling detection.
        move_forward (bool): Whether to go forward to get axle on the line. Defaults to True.
        linetrack (bool): Whether to linetrack. Defaults to True.

    """
    slow = True
    db.reset()
    while True:
        if slow and db.distance() > start:
            slow = False
        if not slow and db.distance() > stretch:
            slow = True
        brightness_rx = remote.call("brigh")
        left_brightness = brightness_rx[0]
        right_brightness = brightness_rx[1]
        print(left_brightness, right_brightness)
        if junction_type == "left":
            hit = left_brightness < BLACK_THRESHOLD and right_brightness > WHITE_THRESHOLD
        elif junction_type == "both":
            hit = left_brightness < BLACK_THRESHOLD and right_brightness < BLACK_THRESHOLD
        elif junction_type == "right":
            hit = left_brightness > WHITE_THRESHOLD and right_brightness < BLACK_THRESHOLD
        else:
            hit = False
        if hit and db.distance() >= stretch:
            if move_forward:
                db.straight(LIGHT_SENSOR_X, then=Stop.BRAKE)
            else:
                db.brake()
            break
        difference = left_brightness - right_brightness if linetrack else 0
        if slow:
            db.drive(300, difference * 1)
        else:
            db.drive(750, difference * 0.5)


# remote.call("beacn", 67)


while True:
    # Some line tracking thing...
    linetrack("both", 50, 500)
    db.turn(-90)
    linetrack("both", 0, 0)
    db.turn(180)
    linetrack("both", 0, 0)
    db.turn(-90)
    linetrack("both", 50, 500)
    db.turn(-90)
    linetrack("right", 50, 400)
    db.turn(90)
    linetrack("both", 0, 0)
    db.turn(180)
    linetrack("left", 0, 0)
    db.turn(-90)
    linetrack("right", 50, 400)
    db.turn(90)
    linetrack("right", 50, 1000)
    db.turn(90)
    linetrack("right", 50, 200)
    db.turn(90)
    linetrack("both", 50, 200)
    db.turn(180)
    linetrack("both", 50, 200)
    db.turn(-90)
    linetrack("both", 50, 200)
    db.turn(-90)
