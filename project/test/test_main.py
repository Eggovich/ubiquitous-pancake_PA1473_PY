#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import sys

# GLOBALA VARIABLES
ev3 = EV3Brick()
# Motor (Korrigera portarna då dessa stämmer inte)
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
Crane_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears = [12, 36])
Front_button = TouchSensor(Port.S1)
us = UltrasonicSensor(Port.S4)
# Color sensor.
Color_sensor = ColorSensor(Port.S3)
# All the colors
possible_colors = [Color.RED, Color.GREEN, Color.BLUE, Color.BROWN, Color.YELLOW, Color.BLACK, Color.WHITE]
# Drive base. #wheel_diameter= 47, axle_track= 128
robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# ---------------------------The logic under-----------------------

def line_follow (robot): # måste ändra till våra värden!!!!!!!!
    # Calculate the color
    
    threshold = threshold_calculator(color_detection())
    # Calculate the light threshold. Choose values based on your measurements.
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = -70
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
    
def line_follow_2():
    threshold = threshold_calculator(color_detection())
    DRIVE_SPEED = -70
    PROPORTIONAL_GAIN = 2

    path = True
    # Start following the line endlessly.
    while path == True:
        #Updates the current color-correction
        threshold = threshold_calculator(color_detection())
        # Calculate the deviation from the threshold.
        deviation = Color_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        

        while not color_detection() is Color.BROWN:
            robot.straight(5)
            robot.turn(turn_rate)
        
        while not color_detection() is Color.RED:
            robot.straight(5)
            robot.turn(turn_rate)
        
        while not color_detection() is Color.RED:
            robot.straight(5)
            robot.turn(turn_rate)
        



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
        robot.straight(-100)
    if Front_button.pressed() is True:
        Crane_motor.run_time(30, 3000, then = Stop.HOLD, wait=True)
        robot.straight(250)
        robot.turn(360) #ish 90 degrees
        line_follow(robot)
    if Front_button.pressed() is False:
        pick_up_fail_detection(Crane_motor)
        
    
      
def try_pickup_again(robot):
    robot.straight(200)
    robot.straight(-200)
    print('try pickup again')

    

def pick_up_fail_detection(Crane_motor):
    print("Pick up fail detection noticed")
    Crane_motor.run_time(-30, 3000, then = Stop.HOLD, wait=True)
    try_pickup_again(robot)
    return False


# Function determining which way it should go
def robot_route():
    color = color_detection()
    while color == color.GREEN and (color == color.WHITE or color == color.YELLOW):
        line_follow(robot)
        if color == color.BROWN:
            break
    while color == color.BROWN and (color == color.WHITE or color == color.YELLOW):
        line_follow(robot)
        if color == color.RED:
            break
    while color == color.RED and (color == color.WHITE or color == color.YELLOW):
        line_follow(robot)
    else:
        robot.turn(200)


# Detecing obsticals infront of the robot
def detecting_obstacles():
    if us.distance() < 500: #shoiuld change to us.presence()??
        detect = True
        print(us.distance())
    else:
        print(us.distance())
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
    #line_follow(robot)
    #Line_up_function()
    #pick_up_pallet(robot)
    robot_route()
    



# ---------------------------- Ignore this ------------------------------------
if __name__ == '__main__':
    sys.exit(main())
