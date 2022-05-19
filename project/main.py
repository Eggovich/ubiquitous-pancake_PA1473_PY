import sys
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port,  Direction, Stop, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import time
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor


# Group 17 - Egon Grans, Noel Freij,
# Simon Gottschalk, Adam Karlsson, Olof Samuelsson

# GLOBALA VARIABLES
ev3 = EV3Brick()
# Motor
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
Crane_motor = Motor(Port.A,
                    positive_direction=Direction.CLOCKWISE,
                    gears=[12, 36])
Front_button = TouchSensor(Port.S1)
us = UltrasonicSensor(Port.S4)
# Color sensor
Color_sensor = ColorSensor(Port.S3)
# Mid sector color
CIRCLE_COLOR = Color.BROWN
DRIVE_SPEED = -60
PROPORTIONAL_GAIN = 1.3
ANGLE_CONSTANT = 2
# Drive base. #wheel_diameter= 47, axle_track= 128
robot = DriveBase(left_motor, right_motor, wheel_diameter=47, axle_track=128)

# ---------------------------The logic under-----------------------


def line_follow(robot, dest, DRIVE_SPEED,
                PROPORTIONAL_GAIN, CIRCLE_COLOR, pallet, fc):
    threshold = threshold_calculator(color_detection())
    approved_colors = [fc, dest, CIRCLE_COLOR, Color.WHITE, Color.BLACK]
    cc = 0
    detect = False
    dest_zone = 0
    circle_check = False
    dest_check = False
    while detect is False and dest_zone < 5:
        # Detection-process.
        detecting_other_robot(robot)
        detecting_obstacles()
        # Checks if the color black has been seen multiple times in a row,
        # indicating the the color in fact is black
        dest_zone += detecting_zones(color_detection())
        if dest_zone < 1:
            dest_zone = 0

        # Checking for specific color on our route
        # to turn 90 degrees if requriement are met
        if color_detection() == dest and dest_check is False and circle_check is True:
            if dest == Color.BLUE:
                if rgb_check() is True:
                    threshold = threshold_calculator(color_detection())
                    robot.turn(90)
                    dest_check = True
                    cc = 0
                else:
                    robot.turn(15*ANGLE_CONSTANT)
            else:
                threshold = threshold_calculator(color_detection())
                robot.turn(90)
                dest_check = True
                cc = 0
        elif color_detection() == CIRCLE_COLOR and circle_check is False:
            threshold = threshold_calculator(color_detection())
            robot.turn(90)
            circle_check = True
            cc = 0
        # Correction turn when the robot sees a non approved color
        if color_detection() not in approved_colors:
            cc += 1
            if cc > 6:
                robot.turn(-30*ANGLE_CONSTANT)
                cc = 0
        else:
            cc = 0

        # This code would work if the pallet was firmly pressing the
        # button at all times (But is turned off since thats not the case)
        # if pallet is True:
        #    if Front_button.pressed() is False:
        #        detect = True

        # Calculate the deviation from the threshold.
        deviation = Color_sensor.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation

        if circle_check is True and len(approved_colors) == 5:
            approved_colors.pop(0)

        # print(Color_sensor.reflection())
        # Calculate the turn rate.
        if dest_zone < 5 and detect is False:
            # Set the drive base speed and turn rate.
            robot.drive(DRIVE_SPEED, turn_rate)
    robot.stop()
    if dest_zone < 5:
        dest_zone = False
    else:
        dest_zone = True
    return detect, dest_zone, fc, dest


# Checks if color is that is seen is blue or purple
def rgb_check():
    result = False
    if Color_sensor.rgb()[1] > 13 and Color_sensor.rgb()[1] < 30 and Color_sensor.color() == Color.BLUE:
        result = True
    return result


def threshold_calculator(color):
    WHITE_REF = 62
    threshold = 50
    if str(color) == "Color.BLUE":  # Blå
        threshold = (WHITE_REF + 12)/2
    elif str(color) == "Color.GREEN":  # Grön
        threshold = (WHITE_REF + 12)/2
    elif str(color) == "Color.YELLOW":  # Gul
        threshold = (WHITE_REF + 41)/2
    elif str(color) == "Color.RED":  # Röd
        threshold = (WHITE_REF + 65)/2
    elif str(color) == "Color.BLACK":  # Svart
        threshold = (WHITE_REF + 0)/2
    elif str(color) == "Color.BROWN":  # Brun
        threshold = (WHITE_REF + 20)/2
    return threshold


def pick_up_pallet(robot):
    while Front_button.pressed() is not True:
        robot.straight(-10)
    if Front_button.pressed() is True:
        Crane_motor.run_time(100, 5000, then=Stop.HOLD, wait=True)
        robot.turn(ANGLE_CONSTANT * -180)  # 180 grader
    pallet = True
    return pallet


# Counting the amount of consecutive "black" returns
def detecting_zones(color):
    if color == Color.BLACK:
        dest_zone = 1
    else:
        dest_zone = -1
    return dest_zone


def pick_up_fail_detection(Crane_motor):
    print("Pick up fail detection noticed")
    Crane_motor.run_until_stalled(-50, then=Stop.HOLD, duty_limit=None)


# Function determining which way it should go
def destination(home, fc):
    if home is True:
        dest = Color.BLUE  # Start destination-color
    else:
        dest = fc  # Return color
    return dest


# Detecing obstacals infront of the robot and
# stop while the obstacle is there
def detecting_obstacles():
    dist = us.distance()
    while dist < 500 and dist > 326:
        robot.stop()
        time.sleep(4)
        print("Något som inte är en robot står i vägen")


# Detecting robot and holding postion if so
def detecting_other_robot(robot):
    dist = us.distance()
    while us.presence() is True and dist < 600 and dist > 326:
        robot.stop()
        time.sleep(4)
        print('Detected other robot')


# function used inside line_follow to determine the line it should follow.
def color_detection():
    color = Color_sensor.color()
    return color


# Color calibration at startup process
def calibration():
    reflect = []
    for _ in range(100):
        print(Color_sensor.reflection())
        reflect.append(Color_sensor.reflection())
    return min(reflect), Color_sensor.color()

# ---------------------------- Main function ----------------------------------


def main():
    pallet = False
    fc = Color.BLUE
    detect, dest_zone, fc, dest = line_follow(robot, destination(True, None), DRIVE_SPEED, PROPORTIONAL_GAIN, CIRCLE_COLOR, pallet, fc)
    if dest_zone is True:
        pallet = pick_up_pallet(robot)
        detect, dest_zone, fc, dest = line_follow(robot, destination(False, fc), DRIVE_SPEED, PROPORTIONAL_GAIN, CIRCLE_COLOR, pallet, dest)
    else:
        print("Something went wrong")

# Startup calibration process
# print(calibration())
# print(Color_sensor.rgb()[1])


# ---------------------------- Ignore this ------------------------------------
if __name__ == '__main__':
    sys.exit(main())
