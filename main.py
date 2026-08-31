"""Pybricks code for SPIKE Prime brain."""

from pybricks.hubs import PrimeHub
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

LEFT_WHITE = 88
RIGHT_WHITE = 100
BLACK_THRESHOLD = 30
WHITE_THRESHOLD = 40
LIGHT_SENSOR_Y = 64
MAX_SPEED = 500
ACCEL_DISTANCE = 200
DECEL_DISTANCE = 200

# Initialise hardware
hub = PrimeHub()
left_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
arm_motor = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
claw_motor = Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)
cage_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
db = DriveBase(left_motor, right_motor, 62.4, 167)
db.use_gyro(True)
db.settings(
    straight_speed=MAX_SPEED, straight_acceleration=1000, turn_rate=300, turn_acceleration=600
)
print("db settings:", db.settings())

# Initialise connection to remote.
print("Waiting for remote on Port.C")
while True:
    try:
        PUPDevice(Port.C)
    except OSError:
        wait(100)
        continue
    try:
        remote = PUPRemoteHub(Port.C, max_packet_size=16)
        remote.add_command("beacn", "", "B")
        remote.add_command("cserv", "", "Bh")
        remote.add_channel("brigh", "BB")
        remote.add_channel("rgb", "9B")
        remote.call("beacn", 67, wait_ms=2)
        hub.speaker.beep(523)
        break
    except OSError:
        wait(500)

fps_timer = StopWatch()

arm_motor.control.limits(2000, 5000, 263)
claw_motor.control.limits(2000, 2000, 320)


def arm_up() -> None:
    """Move arm up (currently assuming it starts from grabbing height)."""
    arm_motor.run_angle(1000, 180)
    arm_motor.run_until_stalled(1000, duty_limit=50)  # Remove this line to disable aligning


def arm_down() -> None:
    """Move arm down (currently to the correct height for grabbing)."""
    # arm_motor.run_angle(500, -280)  # Go down without aligning (aligning is done when going up)

    arm_motor.run_angle(1000, -130, then=Stop.NONE)  # Quickly go down
    arm_motor.run_until_stalled(-300, duty_limit=20)  # Slow down just before hitting ground
    # This results in a gentle bonk on the ground
    # dispensing might need to go up 60 deg
    arm_motor.run_angle(1000, 67)
    # wait(1000)


def claw_open() -> None:
    """Open the claw."""
    claw_motor.run_angle(1000, 90)


def claw_close() -> None:
    """Close the claw."""
    claw_motor.run_until_stalled(-1000, then=Stop.HOLD, duty_limit=100)


def cage_up() -> None:
    """Raise the cage and close the channels."""
    cage_motor.run_until_stalled(800, duty_limit=75)


def cage_down() -> None:
    """Lower the cage and open the channels."""
    cage_motor.run_until_stalled(-800, duty_limit=50)


def starting_to_middle() -> None:
    db.curve(158, 90)
    linetrack("both", 824)


def linetrack(
    junction_type: str,
    theoretical: int,
    sensor: str = "both",
    *,
    move_forward: bool = True,
    linetrack: bool = True,
) -> None:
    """Line track.

    Args:
        junction_type (str): Left, both, right or distance.
        theoretical (int): The theoretical distance to line track.
        sensor (str): Line track using left, right or both sensors.
        move_forward (bool): Whether to go forward to get axle on the line. Defaults to True.
        linetrack (bool): Whether to linetrack. Defaults to True.

    """
    db.reset()
    left_brightness = -1
    right_brightness = -1
    fps_timer.reset()
    frame_count = 0
    integral = 0
    p_difference = 0

    while True:
        # Calculate some things
        detection_start = max(0, theoretical - LIGHT_SENSOR_Y - 50)
        detection_end = theoretical * 1.05 - LIGHT_SENSOR_Y + 20

        # Receive brightness data, keeping the last readings if the link glitches
        try:
            brightness_rx = remote.call("brigh")
            left_brightness = brightness_rx[0]
            right_brightness = brightness_rx[1]
        except OSError:
            print("remote call failed")
        # print(left_brightness, right_brightness)

        # Detect junctions
        if junction_type == "left":
            hit = left_brightness < BLACK_THRESHOLD and right_brightness > WHITE_THRESHOLD
        elif junction_type == "both":
            hit = left_brightness < BLACK_THRESHOLD and right_brightness < BLACK_THRESHOLD
        elif junction_type == "right":
            hit = left_brightness > WHITE_THRESHOLD and right_brightness < BLACK_THRESHOLD
        elif junction_type == "distance":
            hit = db.distance() >= theoretical - LIGHT_SENSOR_Y
        else:
            hit = False

        # If either light sensor is broken
        if left_brightness == -1 or right_brightness == -1:
            difference = 0  # Disable line tracking, just go straight
            # Ignore light sensor readings for junction detection, instead use distance
            hit = db.distance() >= theoretical
        else:
            if not linetrack:
                difference = 0
            elif sensor == "left":
                difference = 50 - left_brightness
            elif sensor == "right":
                difference = right_brightness - 50
            else:
                difference = left_brightness - right_brightness

        # Check for the robot reaching a junction,
        # then move forward onto the line if enabled
        if hit and db.distance() > detection_start:
            if move_forward:
                db.straight(LIGHT_SENSOR_Y, then=Stop.BRAKE)
            else:
                db.brake()
            break
        # Or when robot has driven past detection_end,
        # significantly after the point where it should have stopped,
        # move forward until the theoretical stopping point
        if db.distance() > detection_end:
            hub.speaker.beep(440)
            if move_forward:
                db.straight(max(0, theoretical + LIGHT_SENSOR_Y - db.distance()))
            else:
                db.brake()
            break

        # Ramp up speed at the start, then ramp down speed before the end
        start_speed_cap = max(0, db.distance() / ACCEL_DISTANCE * 300 + 200)
        end_speed_cap = max(0, (detection_start - db.distance()) / DECEL_DISTANCE * 300 + 200)
        speed = max(200, min(MAX_SPEED, start_speed_cap, end_speed_cap))
        # speed = 500
        # Add 0.5 at start, ramp down
        # start_kp_boost = (1 - db.distance() / ACCEL_DISTANCE) * 0.5
        # start_kp_boost = max(0, start_kp_boost)
        # # Subtract 0.5 at end
        # end_kp_buck = (1 - (detection_start - db.distance()) / DECEL_DISTANCE) * 0.5
        # end_kp_buck = max(0, end_kp_buck)
        # kp = 0.7 + start_kp_boost - end_kp_buck
        # kp = max(0, kp)
        kp = speed * 0.002
        # difference = min(difference, 100)
        # difference = max(difference, -100)
        # kp = 0.3  # 0.8
        # integral += difference * 0.01
        # integral = max(-25, min(25, integral))
        # kp *= 1.4
        d = difference - p_difference
        # Follow the line
        # db.drive(speed, difference * kp + integral + d * 0.3)

        # dc version
        base = speed / 5  # 100% DC is ~500mm/s speed
        correction = abs(difference) * 0.4
        if difference < 0:
            left_motor.dc(base - correction)
            right_motor.dc(base)
        else:
            left_motor.dc(base)
            right_motor.dc(base - correction)

        frame_count += 1
        if fps_timer.time() > 1000:
            print("FPS:", frame_count)
            fps_timer.reset()
            frame_count = 0

        # wait(8)


def one_sensor_linetrack(speed: int) -> None:
    """Track the right side of the line using the left sensor."""
    db.reset()
    left_brightness = 50

    while True:
        if db.distance() > 1000:
            db.stop()
            break

        try:
            brightness_rx = remote.call("brigh")
            left_brightness = brightness_rx[0]
            right_brightness = brightness_rx[1]
        except OSError:
            print("remote call failed")

        difference = 50 - left_brightness
        base = speed / 5  # 100% DC is ~500mm/s speed
        correction = abs(difference) * 0.8
        if difference < 0:
            left_motor.dc(base - correction)
            right_motor.dc(base)
        else:
            left_motor.dc(base)
            right_motor.dc(base - correction)


def voting() -> list[int]:
    """Scan the mosaic (from the side, 2 rows only). Starts from line sensors on the junction.

    Returns:
        List of mosaic colours in each slot.
        From the perspective of the robot, list indices are
        1 3 5
        0 2 4

    """
    db.reset()
    # Reading the rgb channel switches the ESP32 to the rgb mode, which makes
    # it start reading the scanning sensors; do it now (result discarded) so
    # fresh data is already streaming when the first window starts.
    try:
        remote.call("rgb")
    except OSError:
        print("remote call failed")
    average_colors: list[tuple[int, int, int]] = [(-1, -1, -1)] * 6
    first_start = 32
    # (Start, end) of each slot
    slot_distances = ((first_start, first_start + 32), (first_start + 50, first_start + 50 + 32))
    # TODO: tune

    for row in range(2):
        print("Start row", row)

        # Drive to the start of the slot (accounting for window)
        while db.distance() < slot_distances[row][0] + 10:
            db.drive(100, 0)
        print("    Start of window")
        # db.straight(0)
        # wait(1000)

        # Collect ambient values until the end of the window
        reading_count = 0
        tallies = [[0, 0, 0] for _ in range(3)]
        while db.distance() < slot_distances[row][1] - 10:
            try:
                rgb = remote.call("rgb")
            except OSError:
                print("remote call failed")
                db.drive(100, 0)
                continue
            for sensor in range(3):
                tallies[sensor][0] += rgb[sensor * 3]
                tallies[sensor][1] += rgb[sensor * 3 + 1]
                tallies[sensor][2] += rgb[sensor * 3 + 2]
            reading_count += 1
            db.drive(100, 0)

        reading_count = max(reading_count, 1)
        print(reading_count, "readings")
        for sensor in range(3):
            avg_rgb = (
                tallies[sensor][0] // reading_count,
                tallies[sensor][1] // reading_count,
                tallies[sensor][2] // reading_count,
            )
            average_colors[sensor * 2 + row] = avg_rgb
        print("    End of window")
        # db.straight(0)
        # wait(1000)

    db.straight(0)
    out = []
    for slot in range(6):
        out.append(67)
    print(average_colors)
    return out


def collect(side: str, num: int) -> None:
    """Collect blocks, starting with the robot on the junction to the colour zone.

    Args:
        side (str): The side of the junction.
        num (int): 1 -> collect first one only
                   2 -> collect both first and second (or second only)
                   3 -> collect both second and third (or third only)

    """
    if side == "left":
        turn_direction = -1
    elif side == "right":
        turn_direction = 1
    else:
        raise ValueError

    db.straight(44 * turn_direction)
    db.turn(-90)
    arm_down()
    db.straight(-8 + 64 * num)  # 120 for both
    claw_close()
    db.straight(-22 - 64 * num)
    claw_open()
    arm_up()
    claw_close()
    db.straight(-100)
    db.turn(-9 * turn_direction)
    cage_down()
    db.straight(180)
    cage_up()
    db.turn(9 * turn_direction)
    db.straight(-45)
    db.turn(90)
    db.straight(-12 * turn_direction)
    claw_open()


def linetrack_test() -> None:
    """Line tracking test thing."""
    starting_to_middle()
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


def lightsensor_test() -> None:
    """hi."""
    while True:
        try:
            brightness_rx = remote.call("brigh")
            left_brightness = brightness_rx[0]
            right_brightness = brightness_rx[1]
            print(left_brightness, right_brightness)
        except OSError:
            print("remote call failed")


def deposittest() -> None:
    # deposit
    arm_down()
    claw_close()
    arm_up()
    linetrack("both", 407, move_forward=False)
    arm_down()
    claw_open()
    arm_up()
    cage_down()
    db.straight(85)
    db.straight(-80)
    claw_motor.run_angle(1000, 30)  # open a bit more
    arm_down()
    claw_close()
    claw_motor.dc(-100)  # constant force
    wait(500)
    arm_up()
    db.straight(100)
    arm_down()
    arm_motor.run_angle(1000, 40)  # drop from a bit higher
    claw_open()
    arm_up()


def cagesmashtesst() -> None:
    starting_to_middle()
    db.turn(90)
    cage_motor.run_angle(500, -120)
    db.straight(-260)
    while True:
        cage_down()
        # db.straight(-15)
        # db.straight(30)
        # db.straight(-15)
        # db.turn(10)
        # db.turn(-20)
        # db.turn(10)
        cage_up()
        db.straight(-3)


def collecttest() -> None:
    starting_to_middle()
    db.turn(180)
    linetrack("right", 674)
    db.turn(90)
    linetrack("left", 344)
    collect("left", 2)
    collect("right", 1)


def main() -> None:
    # starting_to_middle()
    # db.turn(90)
    # arm_down()
    # arm_motor.run_angle(1000, 40)
    # claw_close()
    # arm_up()
    # db.turn(-90)
    # linetrack("both", 675)
    # db.turn(-90)
    # linetrack("distance", 202)
    # db.turn(-90)
    # arm_down()
    # claw_open()
    # arm_up()
    # db.turn(90)
    # linetrack("both", 202)
    # db.turn(-90)
    # linetrack("both", 408, move_forward=False)
    # claw_open()  # second time
    # arm_down()
    # voting()
    # arm_up()
    # db.straight(-458)  # TODO: not magic
    # db.turn(-90)
    linetrack("right", 404)
    db.straight(120)
    db.turn(90)
    cage_motor.run_angle(1000, -90)
    db.straight(300)
    cage_up()
    db.turn(30)
    db.straight(120)
    db.turn(-30)
    db.straight(1150)
    cage_down()
    db.straight(-100)
    cage_motor.run_angle(1000, 45)
    db.turn(7)
    db.straight(-320)
    cage_down()
    db.turn(-7)
    db.straight(280)
    cage_up()
    db.turn(110)
    db.straight(140)
    db.turn(-20)


if __name__ == "__main__":
    # main()
    # collecttest()
    # linetrack("both", 408, move_forward=False)
    one_sensor_linetrack(500)
    # voting()
    # linetrack_test()
    # lightsensor_test()
    # deposittest()
    # cagesmashtesst()
