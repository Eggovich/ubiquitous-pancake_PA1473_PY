#!/usr/bin/env pybricks-micropython

#BIBLIOTEK
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port,  Direction, stop, color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# GLOBALA VARIABLES
ev3 = EV3Brick()
# Motor (Korrigera portarna då dessa stämmer inte)
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
Crane_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears = [12, 36])
Front_button = Port.S1
us = UltrasonicSensor(Port.S4)
# Color sensor.
Color_sensor = ColorSensor(Port.S3)
# All the colors
possible_colors = [color.RED, color.GREEN, color.BLUE, color.BROWN, color.YELLOW, color.BLACK, color.WHITE]
# Drive base. #wheel_diameter= 47, axle_track= 128
robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# ---------------------------The logic under-----------------------

def line_follow (robot): # måste ändra till våra värden!!!!!!!!
    # Calculate the color
    threshold = threshold_calculator(color_detection())
    # Calculate the light threshold. Choose values based on your measurements.
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 70
    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.
    # For example, if the light value deviates from the threshold by 10, the robot
    # steers at 10*1.2 = 12 degrees per second.
    PROPORTIONAL_GAIN = 2

    # Start following the line endlessly.
    detect = False
    while detect is False:
        #Updates the current color-correction
        threshold = threshold_calculator(color_detection())
        # Calculate the deviation from the threshold.
        deviation = Color_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        # Detection-process.
        detect = detecting_obstacles()


def threshold_calculator(color):
    threshold = 50#Värden ska ändras
    if color == 2:
        threshold = 50
    elif color == 3:
        threshold = 50
    elif color == 4:
        threshold = 50
    elif color == 5:
        threshold = 50
    elif color == 6:
        threshold = 50
    return threshold


def pick_up_pallet(robot): 
    while Front_button.pressed() is not True:
        robot.straight(10)
    if Front_button.pressed() is True:
        Crane_motor.run_untill_stalled(50, then = stop.HOLD, duty_limit = 60)
        robot.straight(-80)
        robot.turn(180)
    if Front_button.pressed() is False:
        pick_up_fail_detection(Crane_motor)
    else:
        try_pickup_again(robot)

      
def try_pickup_again(robot):
    robot.straight(-100)
    robot.turn(270)
    robot.straight(5) #how big of a sidement adjustment
    robot.turn(90)
    pick_up_pallet(robot)


def pick_up_fail_detection(Crane_motor):
    print("Pick up fail detection noticed")
    Crane_motor.run_untill_stalled(-50, then = stop, duty_limit = None)
    return False


# Function determining which way it should go
def direction():
    pass


# Detecing obsticals infront of the robot
def detecting_obstacles():
    if us.distance_centimeters() < 20:
        detect = True
    else:
        detect = False
    return detect
    

# function used inside line_follow to determine the line it should follow. 
# Must have a variabel that sends the correct line to follow depending on where we want the robot to go.
def color_detection():
    color = Color_sensor.color()
    print(color)
    return color
        
            
def road_choice(choice, current_color):
    pass


# ---------------------------- Main funktion ----------------------------------

def main():
    line_follow(robot)
    #Line_up_function()
    pick_up_pallet(robot)
        



# ---------------------------- Ignore this ------------------------------------
if __name__ == '__main__':
    sys.exit(main())