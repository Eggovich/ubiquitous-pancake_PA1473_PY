#!/usr/bin/env pybricks-micropython

#BIBLIOTEK
import sys


from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port,  Direction, stop, color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# GLOBALA VARIABLES

# Motor (Korrigera portarna då dessa stämmer inte)
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)

Crane_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears = [12, 36])

Front_button=Port.S1

Ultrasonic_sensor = UltrasonicSensor(Port.S4)

# Color sensor.
Color_sensor = ColorSensor(Port.S3)

# All the colors
possible_colors = [color.RED, color.GREEN, color.BLUE, color.BROWN, color.YELLOW, color.BLACK, color.WHITE]

# Drive base. #wheel_diameter= 47, axle_track= 128
robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# ---------------------------The logic under-----------------------

def line_follow (robot): # måste ändra till våra värden!!!!!!!!
    # Initialize the motors.
    
    # Calculate the color
    
    color = color_detection()
    # Calculate the light threshold. Choose values based on your measurements.
    WHITE = 85 #not our value
    threshold = (color + WHITE) / 2

   

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
        deviation = Color_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)

def pick_up_pallet(Crane_motor, Front_button):
    try: 
        if Front_button.pressed() == True:
         Crane_motor.run_untill_stalled(50, then = stop.HOLD, duty_limit = 60)
        else:
            pick_up_fail_detection()
    except:
        print("Another problem")
        
    

def pick_up_fail_detection(Crane_motor, Front_button):
    try:
        if Front_button.pressed() == False:
            print("Pick up fail detection noticed")
            Crane_motor.run_untill_stalled(-50, then = stop, duty_limit = None)
            return False
        else:
            pass
    except:
        print("something went wrong")

# Function determining which way it should go
def direction():
    pass

# function used inside line_follow to determine the line it should follow. 
# Must have a variabel that sends the correct line to follow depending on where we want the robot to go.
def color_detection():
    color = color()
    return color
        
            


def road_choice():
    pass


# ---------------------------- Main funktion ----------------------------------

def main():
    line_follow(robot)


# ---------------------------- Ignore this ------------------------------------
if __name__ == '__main__':
    sys.exit(main())