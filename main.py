"""Pybricks code for SPIKE Prime brain."""

from pybricks.hubs import PrimeHub
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

WHEEL_DIAMETER = 62.4
AXLE_TRACK = 167
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
db = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
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
        # remote.add_channel("dist", "hh")
        remote.call("beacn", 67, wait_ms=2)
        hub.speaker.beep(523)
        break
    except OSError:
        wait(500)

fps_timer = StopWatch()

arm_motor.control.limits(2000, 5000, 263)
claw_motor.control.limits(2000, 2000, 320)

# Since we grab tiles twice, we have to keep track of which ones are left over from the first grab
available_stock = [list(range(6)) for i in range(4)]  # Oh no, global state!


def rgb_to_hsv(rgb: tuple[int, int, int]):
    # Normalize RGB values to the range [0, 1]
    r, g, b = (x / 255 for x in rgb)

    max_val = max(r, g, b)
    min_val = min(r, g, b)
    diff = max_val - min_val

    # Value (Brightness)
    v = max_val

    # Saturation
    s = 0.0 if max_val == 0 else diff / max_val

    # Hue
    if diff == 0:
        h = 0.0
    elif max_val == r:
        h = (60 * ((g - b) / diff) + 360) % 360
    elif max_val == g:
        h = (60 * ((b - r) / diff) + 120) % 360
    elif max_val == b:
        h = (60 * ((r - g) / diff) + 240) % 360

    # Return H in [0, 360], S and V in [0, 1] (or scale as needed)
    return h, s, v


def turn(angle) -> None:
    db.turn(angle)


def arm_up() -> None:
    """Move arm up (currently assuming it starts from grabbing height)."""
    arm_motor.run_angle(1000, 180)
    arm_motor.run_until_stalled(1000, duty_limit=50)  # Remove this line to disable aligning


def arm_down(*, absolute=False) -> None:
    """Move arm down (currently to the correct height for grabbing)."""
    if absolute:
        arm_motor.run_target(1000, 0)
    else:
        arm_motor.run_angle(1000, -130, then=Stop.NONE)  # Quickly go down
        arm_motor.run_until_stalled(-300, duty_limit=20)  # Slow down just before hitting ground
        arm_motor.run_angle(1000, 67)


def claw_open(stage: int = 0) -> None:
    """Open the claw."""
    stages = [80, 120, 240]
    claw_motor.run_target(1000, stages[stage])


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
    """Leave the starting area and line track to the middle junction."""
    db.curve(158, 90)
    linetrack("both", 824)


def linetrack(
    junction_type: str,
    theoretical: int,
    *,
    sensor: str = "both",
    move_forward: bool = True,
    follow_line: bool = True,
) -> None:
    """Line track.

    Args:
        junction_type: Left, both, right, blue or distance.
        theoretical: The theoretical distance to line track.
        sensor: Line track using left, right or both sensors. Defaults to both.
        move_forward: Whether to go forward to get axle on the line. Defaults to True.
        follow_line: Whether to linetrack. Defaults to True.

    """
    db.reset()
    left_brightness = -1
    right_brightness = -1
    fps_timer.reset()
    frame_count = 0

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
        elif junction_type == "blue":  # Looser threshold for detecting blue box
            hit = left_brightness < 50 and right_brightness < 50
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
            if not follow_line:
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

        # DC version
        base = speed / 5  # 100% DC is ~500mm/s speed
        kp = 0.6 if sensor == "both" else 0.8
        correction = abs(difference) * kp
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


def voting() -> list[int]:
    """Scan the mosaic (from the side, 2 rows only). Starts from line sensors on the junction.

    Returns:
        List of mosaic colors in each slot.
        From the perspective of the robot, list indices are
            1 3 5
            0 2 4
        Enumerated colors are
            0: Yellow
            1: Green
            2: Blue
            3: White

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
        # print("Start row", row)

        # Drive to the start of the slot (accounting for window)
        while db.distance() < slot_distances[row][0] + 10:
            db.drive(100, 0)
        # print("    Start of window")
        # db.straight(0)
        # wait(1000)

        # Collect RGB values until the end of the window
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
        # print("    End of window")
        # db.straight(0)
        # wait(1000)

    db.straight(0)
    out = []
    for slot in range(6):
        h, s, v = rgb_to_hsv(average_colors[slot])
        if v > 0.67:
            # White or yellow
            if s > 0.35:
                out.append(0)  # Yellow
            else:
                out.append(3)  # White
        else:
            # Green or blue
            if h > 150:
                out.append(2)  # Blue
            else:
                out.append(1)  # Green
    print("average_colors", average_colors)
    print("detected", out)
    return out


def grab_seq_algo(mosaic_colors: tuple[int, ...]) -> list[tuple[int, int]]:
    """Greedy algorithm to calculate the order of grabs.

    Args:
        mosaic_colors: See the return type of `voting`.

    Returns:
        A list of grabs. Each grab is a tuple of (color, channel).

    """
    grabs = []
    mosaic_tile_ids = [0, 2, 4]  # For each channel, put this in before 1, 3 or 5
    current_color = 3  # Assume we start at white zone
    # If both middle tiles are the same color, grab them together in one claw grab at the end
    middle_pair = mosaic_colors[2] == mosaic_colors[3]
    for i in range(6):  # 6 grabs in total
        best_dist = 6767  # Closest distance
        best_color = -1
        best_channel = -1
        best_mosaic_tid = -1
        # For each color's tile grab zone
        for channel, mosaic_tid in enumerate(mosaic_tile_ids):
            if mosaic_tid == -1:
                # Channel is done already
                continue
            if mosaic_tid == 3 and i < 5:
                # Don't grab tid 3 yet! It must be last.
                continue
            if middle_pair and mosaic_tid == 2 and i < 4:
                # Hold tid 2 back so it is grabbed right before tid 3 and merged with it
                continue
            color = mosaic_colors[mosaic_tid]
            dist = abs(color - current_color)
            if dist < best_dist or (dist == best_dist and color > current_color):
                # Prefer going North
                best_dist = dist
                best_color = color
                best_channel = channel
                best_mosaic_tid = mosaic_tid
        # We now have the best one to grab and where it should go
        print(best_color, best_channel)
        grabs.append((best_color, best_channel))
        if best_mosaic_tid % 2 == 0:
            # If we grab the first one, the one behind can be grabbed in future
            mosaic_tile_ids[best_channel] += 1
        else:
            mosaic_tile_ids[best_channel] = -1
        current_color = best_color
    return grabs


def grab_mvt_algo(seq: list[tuple[int, int]]) -> list[list[int]]:
    """Calculate the actual grab movements, i.e. calls to `collect`.

    Args:
        seq: See the return type of `grab_seq_algo`.

    Returns:
        A list of actual grab movements. Each is a list of [color, side, channel, num].
        Num is how deep to go (see the args of `grab_one`).

    """
    # For each of the 4 colors of grabbable tiles, all 6 are available at first
    global available_stock
    grabs = []
    for tile in seq:
        t_color, t_channel = tile
        if grabs:  # Avoid index out of range
            p_color, p_side, p_channel, p_num = grabs[-1]
            if p_color == t_color and p_channel == t_channel and p_num < 3:
                # 1. New grab color == previous color
                # 2. Both in same channel
                # 3. Previous grab is not the back-most one (3rd row)
                print("-> grab two at the same time")
                grabs[-1][3] += 1  # Now this grab grabs both -> go one layer deeper than prev
                grab_tid = p_num * 2 + p_side  # The tile directly behind the previous one
                available_stock[p_color].remove(grab_tid)
                continue  # No need to add a new grab
        grab_tid = min(available_stock[t_color])  # Smallest tid in this color's zone
        grabs.append([t_color, grab_tid % 2, t_channel, grab_tid // 2 + 1])
        available_stock[t_color].remove(grab_tid)
    print(grabs)
    return grabs


def grab_one(channel: int, num: int) -> None:
    """Collect one grab, starting on the line and facing the tiles.

    Args:
        channel: The channel to collect into: 0 (left), 1 (middle), 2 (right) or 3 (claw).
        num: 1 -> collect first one only
             2 -> collect both first and second (or second only)
             3 -> collect both second and third (or third only)

    """
    print(f"Grabbing into {channel}, num {num}")
    # TODO: middle channel back is channel, front is claw
    channel_polarity = channel - 1  # -1, 0 or 1

    arm_down(absolute=True)
    claw_open()
    db.straight(64 * num)  # 120 for both
    claw_close()
    if channel == 3:
        # Claw
        db.straight(8 - 64 * num)
    elif channel == 1:
        # Middle channel
        db.straight(-20 - 64 * num)
        claw_open(2)
        arm_up()
        cage_down()
        db.straight(80)
        cage_up()
        arm_down(absolute=True)
        db.straight(-94)
    else:
        # Either of the side channels
        db.straight(-20 - 64 * num)
        claw_open(2)
        db.straight(-50)
        arm_up()
        turn(-12 * channel_polarity)
        cage_down()
        db.straight(160)
        cage_up()
        arm_down(absolute=True)
        db.straight(-100)
        turn(12 * channel_polarity)


def grab(grabs: list[list[int]]) -> None:
    """Grab the correct tiles for one side of the mosaic (6 tiles).

    The robot starts on the junction to white zone.
    """
    current_color = 3
    current_side = 0
    # CLAW RESET PROC.
    claw_close()
    claw_motor.reset_angle(0)
    claw_open()
    # ARM RESET PROC.
    arm_down()
    arm_motor.reset_angle(0)

    for i, g in enumerate(grabs):
        g_color, g_side, g_channel, g_num = g
        print(f"Move to color {g_color}, side {g_side}")
        if g_color == current_color:
            if g_side < current_side:
                turn(90)
                db.straight(88)
                turn(-90)
            elif g_side > current_side:
                turn(-90)
                db.straight(88)
                turn(90)
            # else do nothing - we are already in the correct place
        else:
            # move!
            if g_color > current_color:
                print("linetrack riughtwarrd")
                turn(90)
                linetrack(
                    "left",
                    (g_color - current_color) * 160 + (44 if current_side == 1 else -44),
                    move_forward=False,
                )
                # wait(1000)
                db.straight(LIGHT_SENSOR_Y + (44 if g_side == 0 else -44))
                turn(-90)
            elif g_color < current_color:
                print("linetrack lefthwuduaw")
                turn(-90)
                linetrack(
                    "right",
                    (current_color - g_color) * 160 + (44 if current_side == 0 else -44),
                    move_forward=False,
                )
                # wait(1000)
                db.straight(LIGHT_SENSOR_Y + (44 if g_side == 1 else -44))
                turn(90)

        if g_channel == 1 and i == len(grabs) - 1:
            # Last grab is always with the claw
            grab_one(3, g_num)
        else:
            grab_one(g_channel, g_num)
        current_color = g_color
        current_side = g_side
        # wait(2000)


def linetrack_test() -> None:
    starting_to_middle()
    turn(-90)
    while True:
        linetrack("left", 664)
        turn(-90)
        linetrack("both", 198)
        turn(180)
        linetrack("right", 198)
        turn(90)
        linetrack("both", 664)
        turn(-90)
        linetrack("both", 674)
        turn(-90)
        linetrack("both", 181)
        turn(180)
        linetrack("both", 181)
        turn(-90)
        linetrack("both", 674)
        turn(-90)
        linetrack("both", 424)
        turn(-90)
        linetrack("both", 407)
        turn(180)
        linetrack("both", 407)
        turn(90)
        linetrack("right", 424)
        turn(90)
        linetrack("right", 1349)
        turn(90)


def lightsensor_test() -> None:
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
    claw_open(1)
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
    claw_open(1)
    arm_up()


def cagesmashtesst() -> None:
    starting_to_middle()
    turn(90)
    cage_motor.run_angle(500, -120)
    db.straight(-260)
    while True:
        cage_down()
        # db.straight(-15)
        # db.straight(30)
        # db.straight(-15)
        # turn(10)
        # turn(-20)
        # turn(10)
        cage_up()
        db.straight(-3)


def collecttest() -> None:
    starting_to_middle()
    turn(180)
    linetrack("right", 674)
    turn(90)
    linetrack("left", 344)
    grab_one("left", "left", 2)
    grab_one("left", "right", 1)
    grab_one("right", "right", 1)


def cheese() -> None:
    # # CLAW RESET PROC.
    claw_close()
    claw_motor.reset_angle(0)
    claw_open(2)
    # ARM RESET PROC.
    arm_down()
    arm_motor.reset_angle(0)
    arm_motor.run_target(1000, 40)
    db.straight(200)
    db.straight(-10)
    claw_close()
    claw_open(2)
    db.straight(20)
    claw_close()
    db.straight(-60)
    turn(90)
    db.straight(100)
    linetrack("both", 900)
    arm_up()


def main() -> None:
    """Execute the main mission."""
    # starting_to_middle()
    # turn(90)
    # claw_open(1)
    # arm_down()
    # arm_motor.run_angle(1000, 40)
    # claw_close()
    # arm_up()
    # turn(-90)
    # linetrack("both", 675)
    # turn(-90)
    # linetrack("distance", 202)
    # turn(-90)
    # arm_down()
    # claw_open()
    # arm_up()
    # turn(90)
    # linetrack("both", 202)
    # turn(-90)
    # linetrack("both", 408, move_forward=False)
    # claw_open(2)  # flat
    # arm_down()
    # mosaic_colors = voting()
    # arm_up()
    # db.straight(-458)  # TODO: not magic
    # turn(-90)
    # linetrack("right", 404)
    # db.straight(130)
    # turn(90)
    # cage_motor.run_angle(1000, -90)
    # db.straight(300)
    # cage_up()
    # turn(28)
    # db.straight(260)
    # turn(-28)
    # linetrack("blue", 1543 - 300 - 225, move_forward=False)
    # turn(-20)
    # db.straight(100)
    # turn(20)
    # cage_down()
    # db.straight(-100)
    # cage_motor.run_angle(1000, 45)
    # turn(11)
    # db.straight(-320)
    # cage_down()
    # turn(-11)
    # db.straight(280)
    # cage_up()
    # turn(120)
    # db.straight(150)
    # turn(-30)
    mosaic_colors = (1, 3, 0, 2, 3, 1)
    linetrack("left", 650)  # Mandatory track until the end for alignment
    db.straight(44)
    turn(-90)
    seq = grab_seq_algo(mosaic_colors)
    mvt = grab_mvt_algo(seq)
    grab(mvt)


if __name__ == "__main__":
    main()
    # cheese()
    # collecttest()
    # # CLAW RESET PROC.
    # claw_close()
    # claw_motor.reset_angle(0)
    # claw_open()
    # # ARM RESET PROC.
    # arm_down()
    # arm_motor.reset_angle(0)
    # grab_one(2, 1)
    # linetrack("both", 408, move_forward=False)
    # linetrack("distance", 800, sensor="left")
    # voting()
    # linetrack_test()
    # lightsensor_test()
    # deposittest()
    # cagesmashtesst()
