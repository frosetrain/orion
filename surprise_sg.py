# SURPRISE
ARM_BALL = 250
ARM_BALLDROP = 130  # (130)

CAGE_TWO = -102


def trowels() -> None:
    """Start with the robot on the right side cross-junction, facing south."""
    linetrack("right", 404 + 250)
    ...


def cheese() -> None:
    """Acquire, transport and deliver the Cheese."""
    # GRAB CHEDDAR
    reset_mechanisms()
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
    cage_to(CAGE_BOWL)  # TODO: try wait=False
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
    turn(-90)

    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE
    # SURPRISE

    # COLLECT BALL 1
    linetrack("right", 250)
    db.straight(44)
    turn(-90)
    arm_up()
    db.straight(357 - 138)
    claw_open(2)
    arm_to(ARM_BALL)
    claw_close()
    arm_up()
    db.straight(138 - 357)
    turn(-90)

    # TROWELS
    trowels()

    # COLLECT BLUE AND GREEN THINGS
    db.straight(140)
    turn(90)
    wall_align(1500, 90)
    # db.straight(490)
    linetrack("blue", 466, follow_line=False)  # should be 466?
    turn(-90)
    # reset_mechanisms()
    # linetrack("right", 300)
    # Added
    linetrack("left", 300 + 160, move_forward=False)
    db.straight(-80 + LIGHT_SENSOR_Y)
    # End added
    turn(-90)
    cage_to(CAGE_TWO)  # TODO: big enough for both sides
    db.straight(200)
    cage_up()
    db.straight(-100)
    cage_to(CAGE_TWO)  # yes.
    db.straight(-130)
    cage_up()

    # DEPOSIT BALL 1
    turn(-90)
    db.straight(48)
    turn(90)
    db.straight(80)  # TODO : ???
    arm_to(ARM_BALLDROP)
    claw_open(2)
    arm_up()
    db.straight(-50)
    cage_up()
    turn(-90)
    linetrack("both", 424 - 48, move_forward=False)

    # COLLECT BALL 2
    db.straight(-129 + LIGHT_SENSOR_Y)
    turn(-90)
    arm_up()
    db.straight(357 - 138)
    claw_open(2)
    arm_to(ARM_BALL)
    claw_close()
    arm_up()
    db.straight(138 - 357)
    turn(-90)
    linetrack("right", 294)

    # DEPOSIT BALL 2
    db.straight(48)
    turn(-90)
    db.straight(47)
    arm_to(ARM_BALLDROP)
    claw_open(2)
    arm_up()
    db.straight(-47)

    # SMASH WHITE
    turn(90)
    claw_open(3)
    linetrack("left", 320 - 48)
    turn(-90)
    db.straight(250)
    db.straight(-250)

    # COLLECT SHREDDED CHEDDAR
    turn(-90)
    linetrack("both", 664)
    turn(-90)
    linetrack("both", 1349)
    turn(-90)
    linetrack("right", 674)

    turn(-90)
    db.straight(-250)
    cage_down()
    turn(10)
    turn(-20)
    turn(10)
    cage_down(hold=True)
    db.straight(125)

    # DEPOSIT SHREDDED CHEDDAR
    turn(90)
    db.straight(-691)
    db.curve(-AXLE_TRACK / 2, -90)
    db.straight(-690)
    turn(90 + 30)
    turn(-30)
    cage_up()
    db.straight(-235)
