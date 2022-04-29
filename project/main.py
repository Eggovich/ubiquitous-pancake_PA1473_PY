#!/usr/bin/env pybricks-micropython

#BIBLIOTEK
from pickle import NONE
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port,  Direction, Stop, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

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
# Mid sector color
POSSIBLE_COLORS = [Color.GREEN,Color.RED,Color.WHITE,Color.BLACK,Color.BLUE,Color.YELLOW,Color.PURPLE,Color.BROWN,]
CIRCLE_COLOR = Color.BROWN
DRIVE_SPEED = -80
PROPORTIONAL_GAIN = 1.2
ANGLE_CONSTANT = 3.2
# Drive base. #wheel_diameter= 47, axle_track= 128
robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# ---------------------------The logic under-----------------------

def line_follow(robot, dest, DRIVE_SPEED, PROPORTIONAL_GAIN, CIRCLE_COLOR):
    threshold = threshold_calculator(color_detection())
    fc = color_detection()
    approved_colors = [fc, dest, CIRCLE_COLOR, Color.WHITE]
    cc = 0
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
            cc = 0
            
        elif color_detection() == CIRCLE_COLOR and circle_check is False:
            threshold = threshold_calculator(color_detection())
            robot.turn(-90)
            circle_check = True
            cc = 0
        
        # Calculate the deviation from the threshold.
        deviation = Color_sensor.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        if color_detection() not in approved_colors:
            cc += 1
            if cc > 5:
                robot.turn(30)
                cc = 0
        if circle_check is True:
            fc = None
        
        #print(Color_sensor.reflection())
        # Calculate the turn rate.
        if dest_zone is False and detect is False:
        # Set the drive base speed and turn rate.
            robot.drive(DRIVE_SPEED, turn_rate)
    robot.stop()
    return detect, dest_zone


def threshold_calculator(color):#Värden ska ändras
    WHITE_REF = 94
    threshold = 50
    if str(color) == "Color.BLUE": #Blå
        threshold = (WHITE_REF + 16)/2
    elif str(color) == "Color.GREEN": #Grön
        threshold = (WHITE_REF + 13)/2
    elif str(color) == "Color.YELLOW": #Gul
        threshold = (WHITE_REF + 41)/2
    elif str(color) == "Color.RED": #Röd
        threshold = (WHITE_REF + 82.5)/2
    elif str(color) == "Color.BLACK": #Svart
        threshold = (WHITE_REF + 0)/2
    elif str(color) == "Color.BROWN": #Brun
        threshold = (WHITE_REF + 20)/2
    print(threshold)
    return threshold


def pick_up_pallet(robot): 
    while Front_button.pressed() is not True:
        robot.straight(-10)
    if Front_button.pressed() is True:
        Crane_motor.run_time(-100, 5000, then = Stop.HOLD, wait=True)
        robot.turn(ANGLE_CONSTANT * 180 ) #90 grader

    if Front_button.pressed() is False:
        pick_up_fail_detection(Crane_motor)
    else:
        try_pickup_again(robot)

 
def detecting_zones(color):
    if color == Color.BLACK:
        dest_zone = True
    else:
        dest_zone = False
    return dest_zone

      
def try_pickup_again(robot):
    print('JO')
    robot.straight(-100)
    robot.turn(270)
    robot.straight(5) #how big of a sidement adjustment
    robot.turn(90)
    pick_up_pallet(robot)


def pick_up_fail_detection(Crane_motor):
    print("Pick up fail detection noticed")
    Crane_motor.run_until_stalled(-50, then = Stop.HOLD, duty_limit = None)
    

# Function determining which way it should go
def destination():
    return Color.RED

# Detecing obsticals infront of the robot
def detecting_obstacles():
    if us.distance() < 20:
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
    #robot.straight(90)
    
main()
#Crane_motor.run_until_stalled(50, then=Stop.HOLD, duty_limit = None)
      

#detect, dest_zone = line_follow(robot, destination(), DRIVE_SPEED, PROPORTIONAL_GAIN, CIRCLE_COLOR)
#print(detect, dest_zone)


# ---------------------------- Ignore this ------------------------------------
if __name__ == '__main__':
    sys.exit(main())