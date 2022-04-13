#!/usr/bin/env pybricks-micropython

#BIBLIOTEK
import sys


from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port,  Direction, stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# GLOBALA VARIABLES

# Motor
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Color sensor.
line_sensor = ColorSensor(Port.S3)

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


# over
def main():
    
    return 0


if __name__ == '__main__':
    sys.exit(main())