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
# Mid sector color
CIRCLE_COLOR = 4
DRIVE_SPEED = 70
PROPORTIONAL_GAIN = 1.2
# Drive base. #wheel_diameter= 47, axle_track= 128
robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# ---------------------------The logic under-----------------------

def line_follow(robot, dest, DRIVE_SPEED, PROPORTIONAL_GAIN, CIRCLE_COLOR):
    threshold = threshold_calculator(color_detection())
    detect = False
    dest_zone = False
    circle_check = False
    dest_check = False
    while detect is False and dest_zone is False:
        # Detection-process.
        dest_zone = detecting_zones(color_detection())
        detect = detecting_obstacles()
        # Checking for specific color on our route
        if color_detection() == dest and dest_check is False:
            threshold = threshold_calculator(color_detection())
            robot.turn(-90)
            dest_check = True
            
        elif color_detection() == CIRCLE_COLOR and circle_check is False:
            threshold = threshold_calculator(color_detection())
            robot.turn(-90)
            circle_check = True
        # Calculate the deviation from the threshold.
        deviation = Color_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation
        if dest_zone is False and detect is False:
        # Set the drive base speed and turn rate.
            robot.drive(DRIVE_SPEED, turn_rate)
    robot.stop()
    return detect, dest_zone


def threshold_calculator(color):#Värden ska ändras
    WHITE_REF = 100
    if color == 2: #Blå
        threshold = (WHITE_REF + 0)/2
    elif color == 3: #Grön
        threshold = (WHITE_REF + 7)/2
    elif color == 4: #Gul
        threshold = (WHITE_REF + 41)/2
    elif color == 5: #Röd
        threshold = (WHITE_REF + 38)/2
    elif color == 1: #Svart
        threshold = (WHITE_REF + 0)/2
    elif color == 7: #Brun
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


def detecting_zones(color):
    if color == 1:
        dest_zone = True
    else:
        dest_zone = False
    return dest_zone

      
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
def destination():
    return int(input("Välj destinations-färg (1:Svart ,2:Blått, 3:Grönt, 4:Gult, 5:Rött, 7:Brunt)"))

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
    detect, dest_zone = line_follow(robot, destination(), DRIVE_SPEED, PROPORTIONAL_GAIN, CIRCLE_COLOR)
    #Line_up_function()
    #pick_up_pallet(robot)
        

detect, dest_zone = line_follow(robot, destination(), DRIVE_SPEED, PROPORTIONAL_GAIN, CIRCLE_COLOR)
print(detect, dest_zone)

# ---------------------------- Ignore this ------------------------------------
if __name__ == '__main__':
    sys.exit(main())