"""Pybricks code for SPIKE Prime brain."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

BLACK_THRESHOLD = 40
WHITE_THRESHOLD = 45
LIGHT_SENSOR_Y = 58
MAX_SPEED = 750
ACCEL_DISTANCE = 300

# Initialise hardware
hub = PrimeHub()
left_motor = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
# right_motor.control.limits(1700, 10000, 800)
# print(right_motor.control.limits())
db = DriveBase(left_motor, right_motor, 62.4, 167)
db.use_gyro(True)
db.settings(
    straight_speed=MAX_SPEED, straight_acceleration=1500, turn_rate=320, turn_acceleration=800
)
print("db settings:", db.settings())

# Initialise connection to remote
# PUPDevice() and info() only read Pybricks' cached sync state, so they succeed against a
# remote that is still booting or re-handshaking. Only a real round trip proves the link,
# so keep the first call() inside the retry.
while True:
    try:
        remote = PUPRemoteHub(Port.B, max_packet_size=16)
        remote.add_command("beacn", "b", "b")
        remote.add_channel("brigh", "BB")
        remote.call("beacn", 67, wait_ms=2)
        hub.speaker.beep(523)
        break
    except OSError:
        wait(100)

beacon_timer = StopWatch()
beacon_flashes = 0


def linetrack(
    junction_type: str,
    theoretical: int,
    *,
    move_forward: bool = True,
    linetrack: bool = True,
) -> None:
    """Line track.

    Args:
        junction_type (str): Left, both, or right.
        theoretical (int): The theoretical distance to line track.
        move_forward (bool): Whether to go forward to get axle on the line. Defaults to True.
        linetrack (bool): Whether to linetrack. Defaults to True.

    """
    db.reset()
    left_brightness = -1
    right_brightness = -1

    while True:
        # Calculate some things
        detection_start = max(0, theoretical - LIGHT_SENSOR_Y - 50)
        detection_end = theoretical - LIGHT_SENSOR_Y + 50

        # Receive brightness data, keeping the last readings if the link glitches
        try:
            brightness_rx = remote.call("brigh")
            left_brightness = brightness_rx[0]
            right_brightness = brightness_rx[1]
        except OSError:
            print("remote call failed")
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

        # If either light sensor is broken
        if left_brightness == -1 or right_brightness == -1:
            difference = 0  # Disable line tracking, just go straight
            # Ignore light sensor readings for junction detection, instead use distance
            hit = db.distance() >= theoretical
        else:
            difference = left_brightness - right_brightness if linetrack else 0

        # Check for the robot reaching a junction
        # Or when robot has driven past detection_end,
        # significantly after the point where it should have stopped
        if (hit and db.distance() > detection_start) or db.distance() > detection_end:
            if move_forward:
                db.straight(LIGHT_SENSOR_Y, then=Stop.BRAKE)
            else:
                db.brake()
            break

        # Ramp up speed at the start, then ramp down speed before the end
        start_speed_cap = max(0, db.distance() / ACCEL_DISTANCE * 500 + 250)
        end_speed_cap = max(0, (detection_start - db.distance()) / ACCEL_DISTANCE * 500 + 250)
        speed = max(250, min(MAX_SPEED, start_speed_cap, end_speed_cap))
        # Follow the line
        kp = max(0.7, 1.4 - db.distance() / ACCEL_DISTANCE * 0.7)
        db.drive(speed, difference * kp)


# linetrack("both", 1349)
# stop

# Side quest 3
db.curve(99, 90)
linetrack("left", 221)
db.turn(-90)
while True:
    linetrack("left", 664)
    db.turn(-90)
    linetrack("both", 198)
    db.turn(180)
    linetrack("right", 198)
    db.turn(90)
    linetrack("both", 664)
    db.turn(-90)
    linetrack("both", 674)
    db.turn(-90)
    linetrack("both", 181)
    db.turn(180)
    linetrack("both", 181)
    db.turn(-90)
    linetrack("both", 674)
    db.turn(-90)
    linetrack("both", 424)
    db.turn(-90)
    linetrack("both", 407)
    db.turn(180)
    linetrack("both", 407)
    db.turn(90)
    linetrack("right", 424)
    db.turn(90)
    linetrack("right", 1349)
    db.turn(90)
