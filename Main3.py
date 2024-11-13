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
WHITE_THRESHOLD = 90
BLACK_THRESHOLD = 20  
KP = 2.0
CRUISE_SPEED = 70  #farthållare, vanliga hastigheten
MIN_DISTANCE = 300  #innan 30cm (300mm) görs inget. inom 30cm så sänks hastigheten
STOP_DISTANCE = 100

# Flagga för att hantera parkeringsläge



def check_distance():
   """Anton S"""
   spot_distance = distance_sensor.distance()
   print("Distance: ", spot_distance)
   return spot_distance

def cruise_control(distance):
    """
    Anpassar hastigheten på roboten beroende på avståndet till objektet framför.
    Om avståndet är under `MIN_DISTANCE` minskar hastigheten.
    Om avståndet är under `STOP_DISTANCE` stoppar roboten helt.
    """
    if distance < STOP_DISTANCE:
        robot.stop()
        ev3.speaker.say('Too close, stopping')
        print("Stopping, too close to object")
        return False  # Anger att roboten är för nära och bör stanna
    elif distance < MIN_DISTANCE:
        reduced_speed = CRUISE_SPEED * (distance / MIN_DISTANCE)  # Sänker hastigheten proportionellt
        robot.drive(reduced_speed, 0)  # Kör framåt med reducerad hastighet
        #print("Reducing speed to:", reduced_speed)
        return True  # Fortsätt köra med minskad hastighet
    else:
        robot.drive(CRUISE_SPEED, 0)  # Kör på normalt CRUISE_SPEED när avståndet är säkert
        
        return True  # Fortsätt köra normalt

def follow_line():
    
    while True:
        reflection = line_sensor.reflection()
        distance = distance_sensor.distance()
        
        # Anropa cruise_control för att anpassa hastigheten beroende på avståndet till objektet framför
        cruise_control(distance)
        # Avbryter om roboten måste stanna helt
        
        # Kolla om det är dags att parkera
        
        find_parking(distance)
        
        # Linjeföljning
        error = reflection - ((WHITE_THRESHOLD + BLACK_THRESHOLD) / 2)
        turn_rate = KP * error
        robot.drive(CRUISE_SPEED, turn_rate)  # Hanterar kurvtagning och linjeföljning
        # Om linjen tappas, starta sökprocessen
        if reflection > WHITE_THRESHOLD:
            print("reflection in if" , reflection)
            print("Before stop & search for line")
            robot.stop()
            print("After stop - search for line")
            search_for_line()

def search_for_line():
    print("enters search for line")
    print("reflection in search for line", line_sensor.reflection())
    left_motor.run_angle(-50, -10)
    right_motor.run_angle(50, 10) 
    wait(100)
    print("After wait")
    for i  in range(40):
        if (i <= 15 ):
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

def find_parking(distance):
   parking_reflection = parking_sensor.reflection()
   if parking_reflection < BLACK_THRESHOLD:
       robot.stop()
       ev3.speaker.say('Possible parking')
       robot.turn(-60)
       print("Stopped for parking check")
       robot.stop()
       if check_distance() > 350:
           print("Parkeringslinje hittad")
           park()
             # Aktivera parkeringsläge
       else: 
           robot.turn(60)
           robot.drive(10,0)
           robot.stop()

def reverse():
   """Anton S"""
   ev3.speaker.say('Exiting parking')
   robot.straight(250)
   robot.turn(-90)
   robot.straight(50)
   ev3.speaker.say('Continue')
   print("Reverse done")

def park():
   """Anton S"""
   ev3.speaker.say('Parking')
   robot.turn(60)
   robot.straight(175)
   robot.turn(105)
   robot.straight(-260)
   robot.stop()
   ev3.speaker.say('Parked')
   wait(5000)
   reverse()

# Huvudloop för att kontrollera robotens status
while True:
    follow_line()