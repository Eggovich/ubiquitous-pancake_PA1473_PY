#!/usr/bin/env pybricks-micropython

#BIBLIOTEK
import sys


from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port,  Direction, stop, color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# GLOBALA VARIABLES

# Motor (Korrigera portarna då dessa stämmer inte)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

left_lift = Motor(Port.D)
right_lift = Motor(Port.A)

# Color sensor.
line_sensor = ColorSensor(Port.S3)

# All the colors
possible_colors = [color.RED, color.GREEN, color.BLUE, color.BROWN, color.YELLOW]

# Drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# The logic under
def line_follow (robot, line_sensor, left_motor, right_motor): # måste ändra till våra värden!!!!!!!!
    # Initialize the motors.

# Calculate the light threshold. Choose values based on your measurements.
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100

    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.

    # For example, if the light value deviates from the threshold by 10, the robot
    # steers at 10*1.2 = 12 degrees per second.
    PROPORTIONAL_GAIN = 1.2

    # Start following the line endlessly.
    while True:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)

def pick_up_fail_detection():
    pass

def pick_up_(left_lift, right_lift):
    pass

# Function determining which way it should go
def direction():
    

# function used inside line_follow to determine the line it should follow. 
# Must have a variabel that sends the correct line to follow depending on where we want the robot to go.
def color_line_follow():
    route_choice = 
    for color in possible_colors:
        if color == color.YELLOW:
            pass
        elif color == color.RED:
            pass
        elif color == color.BLUE:
            pass
        elif color == color.BROWN:
            pass
        elif color == color.GREEN:
            pass
    pass



# over

def main():
    
    return 0


if __name__ == '__main__':
    sys.exit(main())