from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, ForceSensor, Motor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

BLACK_THRESHOLD = 25
WHITE_THRESHOLD = 50
LIGHT_SENSOR_X = 64

# Initialise hardware
hub = PrimeHub()
left_color_sensor = ColorSensor(Port.A)
right_color_sensor = ColorSensor(Port.E)
left_motor = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
db = DriveBase(left_motor, right_motor, 62.4, 164)
db.settings(straight_speed=480, straight_acceleration=1200, turn_rate=320, turn_acceleration=800)
print(db.settings())

# Initialise connection to remote
remote = PUPRemoteHub(Port.C, max_packet_size=16)
remote.add_command("beacn", "b", "b")
remote.add_channel("clock", "i")

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
        junction_type (str): Left, both, right or green.
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
        left_reflection = left_color_sensor.reflection()
        right_reflection = right_color_sensor.reflection()
        # print(left_reflection, right_reflection)
        if junction_type == "left":
            hit = left_reflection < BLACK_THRESHOLD and right_reflection > WHITE_THRESHOLD
        elif junction_type == "both":
            hit = left_reflection < BLACK_THRESHOLD and right_reflection < BLACK_THRESHOLD
        elif junction_type == "right":
            hit = left_reflection > WHITE_THRESHOLD and right_reflection < BLACK_THRESHOLD
        else:
            hit = False
        if hit and db.distance() >= stretch:
            if move_forward:
                db.straight(LIGHT_SENSOR_X + 2, then=Stop.BRAKE)
            else:
                db.brake()
            break
        difference = left_reflection - right_reflection if linetrack else 0
        if slow:
            db.drive(200, difference * 0.6)
        else:
            db.drive(480, difference * 0.4)


while True:
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
