"""Pybricks code for SPIKE Prime brain."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

from pupremote_hub import PUPRemoteHub

WHEEL_DIAMETER = 62.4
AXLE_TRACK = 167
LEFT_WHITE = 88
RIGHT_WHITE = 100
BLACK_THRESHOLD = 20
WHITE_THRESHOLD = 29
BLUE_THRESHOLD = 44
LIGHT_SENSOR_Y = 64
MAX_SPEED = 500
ACCEL_DISTANCE = 200
DECEL_DISTANCE = 150

# Mechanism positions in degrees, relative to the zero set by the reset functions.
ARM_GRAB = 40  # claw at ground level  (67)
ARM_UP = 250  # carry height, channels unblocked   (220)
ARM_HOVER_CHEDDAR = 40
ARM_HOVER_BLUE = 40
ARM_DROP = 67  # release height in deposit()       (90)
ARM_PUSH = -13  # low enough to push the released piece into place

CLAW_CLOSED = 0
CLAW_OPEN = {1: 70, 2: 120, 3: 240, 4: 167}  # keyed by blocks abreast (167)
CLAW_PUSH = 105  # partly closed onto a released piece to push it away
CLAW_CHANNEL_PULL = 150

CAGE_UP = 0  # raised, channel doors closed
CAGE_BOWL = -80
CAGE_TROWEL = -110
CAGE_SMASH = -120

ARM_STACK = 100
CLAW_TWO = 160

# Initialise hardware
hub = PrimeHub()
left_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
arm_motor = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
claw_motor = Motor(Port.E, positive_direction=Direction.CLOCKWISE)  # CLOCKWISE FOR 1CLAW
cage_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
db = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

db.use_gyro(True)
db.settings(
    straight_speed=MAX_SPEED, straight_acceleration=1000, turn_rate=300, turn_acceleration=600
)
arm_motor.control.limits(1000, 5000, 263)
claw_motor.control.limits(1000, 2000, 320)

print("db settings:", db.settings())
print("battery voltage:", hub.battery.voltage())

# Initialise connection to remote.
print("Waiting for remote on Port.C")
while True:
    try:
        # Raises OSError only when no device is present at all; while the ESP32
        # is still syncing, Pybricks waits inside PUPDevice() for it to finish.
        remote = PUPRemoteHub(Port.C, max_packet_size=16)
        remote.add_command("beacn", "", "B")
        remote.add_command("cserv", "", "Bh")
        remote.add_channel("brigh", "BB")
        remote.add_channel("rgb", "9B")
        remote.add_channel("dist", "hh")
        remote.call("beacn", 67)
        # hub.speaker.beep(523)
        break
    except OSError:
        wait(100)
        continue

fps_timer = StopWatch()

# Since we grab tiles twice, we have to keep track of which ones are left over from the first grab
available_stock = [list(range(6)) for i in range(4)]  # Oh no, global state!
intended_heading = 0


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
    # global intended_heading
    # imu_heading = hub.imu.heading()
    # print(angle, imu_heading, intended_heading)
    # db.turn(angle - imu_heading + intended_heading)
    # intended_heading += angle
    db.turn(angle)


def arm_to(angle: int, speed: int = 1000, wait: bool = True) -> None:
    arm_motor.run_target(speed, angle, wait=wait)


def arm_up(*, speed: int = 1000, wait: bool = True) -> None:
    arm_to(ARM_UP, speed=speed, wait=wait)


def arm_down(*, wait: bool = True) -> None:
    arm_to(ARM_GRAB, 1000, wait=wait)


def arm_wait() -> None:
    """Block until a non-blocking arm move has finished."""
    while not arm_motor.control.done():
        wait(10)


def reset_arm() -> None:
    """Find the floor by stalling, then zero at grab height. Start with the arm up."""
    arm_motor.run_angle(1000, -130, then=Stop.NONE)
    arm_motor.run_until_stalled(-300, duty_limit=25)
    arm_motor.reset_angle(-67)
    arm_down()


def claw_to(angle: int, speed: int = 1000) -> None:
    claw_motor.run_target(speed, angle)


def claw_open(abreast: int = 1, speed: int = 1000) -> None:
    claw_to(CLAW_OPEN[abreast], speed=speed)


def claw_close() -> None:
    """Close until stalled so the claw grips whatever is inside."""
    claw_motor.run_until_stalled(-1000, then=Stop.HOLD, duty_limit=100)


def reset_claw() -> None:
    claw_close()
    claw_motor.reset_angle(CLAW_CLOSED)
    claw_open(3)


def cage_to(angle: int, speed: int = 1000, wait: bool = True) -> None:
    cage_motor.run_target(speed, angle, wait=wait)


def cage_up(*, hold: bool = False) -> None:
    """Raise until stalled. Without hold it reached the end stop, so re-zero there.

    With hold it may have stalled against an object (e.g. the bowl), so the angle is kept.
    """
    cage_motor.run_until_stalled(1000, duty_limit=75, then=Stop.HOLD if hold else Stop.COAST)
    if not hold:
        cage_motor.reset_angle(CAGE_UP)


def cage_down(*, hold: bool = False) -> None:
    """Lower the cage and open the channel doors."""
    cage_motor.run_until_stalled(-1000, duty_limit=50, then=Stop.HOLD if hold else Stop.COAST)


def reset_mechanisms() -> None:
    """Calibrate every mechanism. Start with the arm up so it can find the floor."""
    cage_up()
    reset_claw()
    reset_arm()


def wall_align(duration: int, new_heading: int) -> None:
    db.drive(-400, 0)
    wait(duration)
    db.stop()
    wait(500)
    db.reset(angle=new_heading)
    wait(500)


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
    db.reset(distance=0, angle=hub.imu.heading())
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
            hit = left_brightness < BLUE_THRESHOLD and right_brightness < BLUE_THRESHOLD
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
        speed = min(speed, 377)

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
    db.reset(distance=0, angle=hub.imu.heading())
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
    channel_polarity = channel - 1  # -1, 0 or 1

    arm_down()  # Also finishes the non-blocking lower from the previous grab
    claw_open()
    db.straight(64 * num)  # 120 for both
    claw_close()
    if channel == 3:
        # Claw
        db.straight(8 - 64 * num)
    elif channel == 1:
        # Middle channel
        db.straight(-20 - 64 * num)
        claw_open(3)
        arm_up()  # Must finish before the cage moves: a low arm blocks the channels
        cage_down()
        db.straight(80)
        cage_up()
        arm_down()  # Lower while reversing; the next grab_one waits for it
        db.straight(-74)  # -94
    else:
        # Either of the side channels
        db.straight(-20 - 64 * num)  # -20
        claw_open(3)
        db.straight(-50)
        arm_up()
        turn(-12 * channel_polarity)
        arm_wait()  # Arm must be clear of the channels before the cage moves
        cage_down()
        db.straight(140)
        cage_up()
        arm_down()  # Lower while reversing and turning back
        db.straight(-80)  # 100
        turn(12 * channel_polarity)


def go_to_zone(
    current_color: int, current_side: int, g_color: int, g_side: int, backward: bool = False
) -> None:
    north = g_color > current_color
    turn_direction = 1 if north else -1  # 1 = right then left, -1 = left then right
    north_weird = 1 if north else 0  # the side that is southern -> more line track, less stra
    if backward:
        north = not north  # only used for junction type

    print("linetrack north", north)
    turn(90 * turn_direction)
    linetrack(
        "left" if north else "right",
        abs(g_color - current_color) * 160 + (36 if current_side == north_weird else -36),
        move_forward=False,
    )
    db.straight(LIGHT_SENSOR_Y + (-36 if g_side == north_weird else 36))
    turn(-90 * turn_direction)


def grab(grabs: list[list[int]]) -> None:
    """Grab the correct tiles for one side of the mosaic (6 tiles).

    The robot starts on the junction to white zone.
    """
    current_color = 3
    current_side = 0
    reset_mechanisms()

    for i, g in enumerate(grabs):
        g_color, g_side, g_channel, g_num = g
        print(f"Move to color {g_color}, side {g_side}")
        if g_color == current_color:
            if g_side < current_side:
                turn(90)
                db.straight(72)
                turn(-90)
            elif g_side > current_side:
                turn(-90)
                db.straight(72)
                turn(90)
            # else do nothing - we are already in the correct place
        else:
            go_to_zone(current_color, current_side, g_color, g_side)

        if g_channel == 1 and i == len(grabs) - 1:
            # Last grab is always with the claw
            grab_one(3, g_num)
        else:
            grab_one(g_channel, g_num)
        current_color = g_color
        current_side = g_side
        # wait(2000)
    arm_wait()  # Don't return with the arm still moving

    # Move to deposit area (at "color 1.5")
    turn(-90)
    linetrack("both", abs(current_color + 1) * 160 + (36 if current_side == 0 else -36))
    turn(180)


def deposit_from_channels() -> None:
    # arm_down()
    claw_open(2)
    arm_up()
    cage_down()
    db.straight(85)
    db.straight(-90)
    claw_open(3)
    arm_to(ARM_HOVER_CHEDDAR)
    # db.settings(straight_speed=200)
    # db.straight(200)
    # db.settings(straight_speed=MAX_SPEED)
    # db.straight(-10)
    claw_close()
    claw_open(3)
    arm_to(ARM_HOVER_CHEDDAR)
    db.straight(20)
    claw_close()
    wait(500)
    arm_up()
    deposit()


def deposit() -> None:
    db.straight(105)  # 105?
    reset_arm()
    arm_to(ARM_DROP)
    claw_open(4)  # Special dropping size
    db.straight(-50)
    arm_to(ARM_PUSH)
    db.settings(straight_speed=200)
    db.straight(50)
    db.settings(straight_speed=MAX_SPEED)
    claw_to(CLAW_PUSH)
    claw_open(3)
    arm_up()
    db.straight(-105)
    turn(180)
    linetrack("both", 408 - LIGHT_SENSOR_Y)
    turn(180)
    # db.straight(-105 - 408 + LIGHT_SENSOR_Y)  # back on light sensors on line


def cage_smash() -> None:
    """Use the cage to get mosaic tiles into the correct positions. Starts at left junction."""
    cage_to(CAGE_SMASH)
    db.straight(-310)
    db.straight(20)
    cage_up()
    cage_down()
    db.straight(-15)
    db.straight(15)
    turn(10)
    turn(-20)
    turn(10)
    cage_up()
    db.straight(-3)
    linetrack("both", 290)


def trowels() -> None:
    """Start with the robot on the right side cross-junction, facing south."""
    linetrack("right", 404)
    db.straight(130)
    turn(90)
    cage_to(CAGE_TROWEL)
    db.straight(300)
    cage_up()
    db.straight(700)
    db.straight(-80)
    turn(30)
    db.straight(167)  # 230
    turn(-30)
    # linetrack("blue", 1543 - 1065, move_forward=False, follow_line=False)  # 300+700-80+230cos30
    db.straight(1543 - 1065)
    turn(90)
    wall_align(1000, 0)
    db.straight(110)
    turn(-90)
    # db.straight(120)
    cage_to(CAGE_BOWL)
    db.straight(-80)
    cage_up()
    turn(90)


def robots_meet_culture() -> None:
    linetrack("both", 407, move_forward=False)
    db.straight(-62 + LIGHT_SENSOR_Y)
    turn(-45)
    db.straight(330)
    turn(45)
    db.straight(103)
    turn(-180)
    db.straight(-103)
    turn(45)
    db.straight(-330)
    turn(-45)


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


def grab_test():
    seq = grab_seq_algo((0, 2, 1, 2, 3, 0))
    mvt = grab_mvt_algo(seq)
    grab(mvt)
    linetrack("right", 424)
    turn(90)
    linetrack("both", 408, move_forward=False)
    deposit_from_channels()


def deposit_test():
    reset_mechanisms()
    arm_to(ARM_HOVER_CHEDDAR)
    claw_close()
    arm_up()
    linetrack("both", 408, move_forward=False)
    deposit()


def cheese() -> None:
    """Acquire, transport and deliver the Cheese."""
    # THROW 2 GREEN
    claw_close()
    # cage_down()
    cage_motor.run_until_stalled(-1000, duty_limit=100)

    # BOTTOM BARRIER
    cage_up()
    # db.curve(158, 90)
    db.straight(158 - 16)
    turn(90)
    db.straight(158)
    linetrack("right", 824 - 288)
    turn(30)
    db.straight(350)
    db.turn(-120)
    cage_down()
    db.straight(130 + 175)
    db.straight(-130)
    turn(90)
    cage_up()

    # RIGHT BARRIERS
    db.straight(675)  # line track?
    # start from that junction
    db.straight(200)
    turn(-90)
    db.straight(424)
    turn(90)
    db.settings(straight_speed=350)
    db.straight(-580 - 30)
    db.settings(straight_speed=MAX_SPEED)
    # db.straight
    linetrack("both", 580 + 30 - 200)
    db.straight(190)
    turn(90)
    db.straight(-325)
    turn(75)
    cage_to(CAGE_TROWEL)
    db.settings(straight_speed=350)
    db.straight(700 - 80)
    db.settings(straight_speed=MAX_SPEED)
    db.straight(-300)
    cage_up()
    db.straight(-400)
    turn(-75)

    # ROBOTS MEET CULTURE
    cage_up()
    wall_align(1500, 180)
    db.straight(100)
    turn(90)
    db.straight(821 - 180 + 50 + 20)
    arm_motor.run_until_stalled(-1000, duty_limit=25)  # DOWN BEFORE CALIB
    arm_motor.run_angle(1000, 100)
    claw_motor.run_angle(1000, 200)  # OPEN BEFORE CALIB
    arm_motor.run_until_stalled(1000, duty_limit=67)  # UP BEFORE CALIB
    db.straight(-821 + 180 - 20)
    turn(-90)

    # RHOMBI (from that junction)
    db.straight(597)
    turn(90)
    cage_up()
    wall_align(2000, 270)
    db.straight(100)
    turn(-90)
    db.straight(-520)
    db.straight(26)
    # turn(15)
    cage_down()
    # turn(-15)
    db.straight(580 + 70 - 40 - 300 + 50)
    db.curve(300, 90)
    db.straight(580 - 300 - 50)  # - 20
    turn(-90)
    db.straight(-60)
    cage_up()
    db.straight(60)
    turn(-90)
    db.straight(222 - 50)

    # TROWELS
    reset_mechanisms()
    arm_up()
    claw_open(3)
    turn(-90)
    cage_up()
    wall_align(1500, 0)
    db.straight(100)
    turn(-90)
    cage_to(CAGE_TROWEL)
    db.straight(1533)
    db.straight(-183)

    # LEFT BARRIER
    turn(90)
    cage_up()
    wall_align(1000, 0)
    claw_open(2)
    db.straight(736 - 180 - 44 + 30)
    arm_up()
    turn(45)
    arm_up()
    turn(-45)
    arm_to(130)
    claw_close()
    db.straight(180)
    turn(90)
    db.straight(240)
    claw_open(2)
    db.straight(-100)
    turn(180)
    linetrack("both", 240 - 100)
    turn(-90)
    linetrack("both", 463)

    # STACK WHITES
    turn(180)
    linetrack("left", 664)
    turn(-90)
    arm_to(20)
    claw_to(CLAW_TWO)
    db.settings(straight_speed=200)
    db.straight(64 * 2 + 20)  # 120 for both
    db.settings(straight_speed=MAX_SPEED)
    claw_close()
    claw_to(CLAW_TWO)
    db.straight(-32)
    claw_close()
    arm_up()
    db.straight(32 + 16)
    arm_to(ARM_STACK)
    claw_open(2)

    END

    # arm_up()  # IF STARTING BEFORE TROWELS
    claw_open(3)
    arm_to(ARM_HOVER_CHEDDAR)
    db.settings(straight_speed=200)
    db.straight(200)
    db.settings(straight_speed=MAX_SPEED)
    db.straight(-10)
    claw_close()
    claw_open(3)
    arm_to(ARM_HOVER_CHEDDAR)
    db.straight(30)
    claw_close()
    claw_motor.dc(-100)
    arm_up(speed=300)
    db.straight(-70)
    turn(90)

    # COLLECT BOWL
    db.straight(100)
    linetrack("both", 900, move_forward=False)
    db.straight(-191 + 60 + LIGHT_SENSOR_Y)
    cage_to(CAGE_BOWL, wait=False)  # NOTE: Swapped this and the following turn
    turn(55)
    db.straight(111)  # 111
    cage_up(hold=True)
    turn(-110)
    db.straight(119)  # 111
    turn(55)
    linetrack("both", 642, move_forward=False)
    db.straight(-112)

    # DEPOSIT BOWL
    turn(90)
    db.straight(-300)
    cage_to(CAGE_BOWL)
    db.straight(-124)
    # cage_up()
    turn(90)

    # DEPOSIT CHEDDAR
    linetrack("both", 408 - 104 - LIGHT_SENSOR_Y, move_forward=False)
    deposit()

    # COLLECT SHREDDED MOULD
    db.straight(-250)
    cage_down()
    turn(10)
    turn(-20)
    turn(10)
    cage_down(hold=True)
    linetrack("both", 250)
    turn(180 + 30)
    turn(-30)

    # DEPOSIT SHREDDED MOULD
    db.straight(-300)
    cage_up()
    db.straight(-100)
    linetrack("both", 400)
    turn(90)

    # TROWELS
    trowels()

    # COLLECT BLUE CHEESE
    db.straight(140)
    turn(90)
    wall_align(1500, 90)
    db.straight(136)  # TODO: this is  weird
    turn(-90)
    reset_mechanisms()
    arm_to(ARM_HOVER_BLUE)
    db.settings(straight_speed=200)
    db.straight(150)
    db.settings(straight_speed=MAX_SPEED)
    db.straight(-10)
    claw_close()
    claw_open(3)
    arm_to(ARM_HOVER_BLUE)
    db.straight(20)
    claw_close()
    arm_up()
    turn(90)

    # DEPOSIT BLUE CHEESE
    db.straight(298)
    turn(-90)
    linetrack("right", 230)
    turn(90)
    linetrack("both", 408, move_forward=False)
    deposit()
    turn(90)

    # CAGE SMASH
    linetrack("both", 424)
    turn(-90)
    linetrack("both", 675)
    turn(90)
    cage_smash()

    # COLLECT SHREDDED CHEDDAR
    turn(-90)
    linetrack("both", 675)
    turn(-90)
    linetrack("right", 674)
    turn(-90)
    db.straight(-250)
    cage_down()
    turn(10)
    turn(-20)
    turn(10)
    cage_down(hold=True)
    linetrack("left", 250, move_forward=False)
    db.straight(-125 + LIGHT_SENSOR_Y)

    # DEPOSIT SHREDDED CHEDDAR
    turn(90)
    db.straight(-691)
    db.curve(-AXLE_TRACK / 2, -90)
    db.straight(-690)
    turn(90 + 30)
    turn(-30)
    cage_up()
    db.straight(-235)


if __name__ == "__main__":
    db.reset()
    cheese()
    # lightsensor_test()
    # grab_test()
    # deposit_test()
