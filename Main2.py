#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initiera komponenter
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
line_sensor = ColorSensor(Port.S3)
parking_sensor = ColorSensor(Port.S2)
distance_sensor = UltrasonicSensor(Port.S4) 
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Definiera konstanta värden 
WHITE_THRESHOLD = 50  
BLACK_THRESHOLD = 20  
KP = 2.0
CRUISE_SPEED = 100  #farthållare, vanliga hastigheten
MIN_DISTANCE = 300  #innan 30cm (300mmm) görs inget. inom 30cm så sänkts hastigheter
STOP_DISTANCE = 100

def check_distance(distance_sensor):
   """Anton S"""
   spot_distance = distance_sensor.distance()
   print("Distance: ", spot_distance)
   return spot_distance

"""
def cruise_control(robot, distance): #funkar till för att stoppa roboten när den närmare sig ett objekt
   Anton S

   if distance < STOP_DISTANCE:
       robot.stop()
   elif distance < MIN_DISTANCE:
       slow_cruise = int(CRUISE_SPEED * (distance / MIN_DISTANCE))
       left_motor.run(slow_cruise)
       right_motor.run(slow_cruise)
   else:
       robot.drive(20, 0)"""


def follow_line():
   while True:
       reflection = line_sensor.reflection()
       distance = distance_sensor.distance()#avståndet till ett objekt
       # Justera hastighet med cruise control
       #cruise_control(robot, distance)
       find_parking(distance)
       error = reflection - ((WHITE_THRESHOLD + BLACK_THRESHOLD) / 2)
       turn_rate = KP * error
       left_motor_speed = CRUISE_SPEED - turn_rate
       right_motor_speed = CRUISE_SPEED + turn_rate
       left_motor.run(left_motor_speed)
       right_motor.run(right_motor_speed)
       if reflection > WHITE_THRESHOLD:
            search_for_line()

def search_for_line():
   
    
    left_motor.run_angle(75, 20)
    right_motor.run_angle(-75, -20) 
    wait(200)

    for i  in range(40):
        if(i <= 15 ):
            if line_sensor.reflection() < WHITE_THRESHOLD:
               return  
            right_motor.run_angle(100, 20)  
            left_motor.run_angle(75, -20)
            wait(200)
            if line_sensor.reflection() < WHITE_THRESHOLD:
               return  
            ev3.speaker.beep()
        
        else:
            if line_sensor.reflection() < WHITE_THRESHOLD:
               return
            left_motor.run_angle(100, 20)
            right_motor.run_angle(75, -20)
            wait(200)
            if line_sensor.reflection() < WHITE_THRESHOLD:
               return
            ev3.speaker.beep()
    

def find_parking(distance): #returnerar true när det är dags för att parkera
   ev3.speaker.beep()
   parking_reflection = parking_sensor.reflection()
   if parking_reflection < BLACK_THRESHOLD:
       robot.stop()
       ev3.speaker.say('Possible parking')
       robot.turn(-60) #Ändrat utefter banan /Anton S
       
       print("stopped")
       robot.stop()
       if check_distance(distance_sensor) > 350:
           print("distance",distance)
           print("Parkeringslinje hittad")
           park()
       else: 
           robot.turn(60)
           print("distance",distance)   
           robot.stop()
          



def reverse():
   """Anton S"""
   ev3.speaker.say('reversing')
   robot.straight(250)
   
   robot.turn(-90)
   robot.straight(50)
   
   ev3.speaker.say('continue')
   print("reverse is done")
   
   


def park():
   """Anton S"""
   ev3.speaker.say('parking')
   robot.turn(60)
   robot.straight(175)

   robot.turn(100)
   
   robot.straight(-260)
   
   robot.stop()
   ev3.speaker.say('parked')
   wait(5000)
   reverse()
   print("after reverse it will jump here ")

   


# Kör programmet

follow_line()