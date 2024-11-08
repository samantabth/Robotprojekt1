#!/usr/bin/env pybricks-micropython

# Pybricks imports
from math import sin
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# Robot definitions
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Sensor definitions
color_sensor = ColorSensor(Port.S3)

"""
Important! The code may not function correctly if the first line from the template is not included.
"""
#I stort upptäcktes att roboten presterar som bäst i innerkurva, därav är sökmönstret vid 'tappad linje' anpassad för
# ytterkurva.

#Konstanter som definierar hastighet, hur responsiv roboten är samt vad som anses "vitt" och "svart".
BASE_SPEED = 150  
WHITE_THRESHOLD = 80  
BLACK_THRESHOLD = 20  
KP = 2.0  

# Huvudfunktionen som följer linjen, högre värde av vitt medför snabbare korrigering mot linje.
def follow_line():
    while True:
       
        reflection = color_sensor.reflection()

        error = reflection - ((WHITE_THRESHOLD + BLACK_THRESHOLD) / 2)

        turn_rate = KP * error

        left_motor_speed = BASE_SPEED - turn_rate
        right_motor_speed = BASE_SPEED + turn_rate

        left_motor.run(left_motor_speed)
        right_motor.run(right_motor_speed)

        
        if reflection > WHITE_THRESHOLD + 10:
            search_for_line()

        wait(10)

# Funktion för att hitta linjen vid 'tapp', nyttjar små repetetiva rörelser med en kontroll av värde efter varje.
def search_for_line():
    ev3.speaker.beep()  
    
    left_motor.run_angle(100, 20)   
    right_motor.run_angle(-100, -20) 
    wait(200)
    
    for i  in range(40):
        if(i <= 15 ):
            
            if color_sensor.reflection() < WHITE_THRESHOLD:
                return  

            right_motor.run_angle(150, 35)  
            left_motor.run_angle(100, -35)
            wait(200)

            if color_sensor.reflection() < WHITE_THRESHOLD:
                return  
            ev3.speaker.beep()
        
        else:
             
            if color_sensor.reflection() < WHITE_THRESHOLD:
                return  

            left_motor.run_angle(150, 35)  
            right_motor.run_angle(100, -35)
            
            wait(200)
            
            if color_sensor.reflection() < WHITE_THRESHOLD:
                        return  
            ev3.speaker.beep()


follow_line()