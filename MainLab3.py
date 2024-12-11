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


ALL_PARKINGS = []
lap_count = 0



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
        
        find_parking()
        
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
    left_motor.run_angle(-50, -10)
    right_motor.run_angle(50, 10) 
    wait(100)
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

def check_parking():
    """Kontrollerar om en parkering är tom eller upptagen."""
    if check_distance() > 350:  # 350 mm = tom parkering
        print("Parkeringslinje hittad - TOM")
        ev3.speaker.say('Parking Empty')
        return "Empty"
    else:
        print("Parkeringslinje hittad - UPPTAGEN")
        ev3.speaker.say('Parking Occupied')
        return "Occupied"


def find_parking():
    global lap_count, ALL_PARKINGS
    
    parking_reflection = parking_sensor.reflection()
    if parking_reflection < BLACK_THRESHOLD and lap_count < 5:
        # Stanna och kontrollera parkering
        robot.stop()
        ev3.speaker.say('Possible parking')
        print("Stopped for parking check")
        robot.turn(-60)  # Svänger för att läsa av parkering

        # Kontrollera om parkeringen är tom eller upptagen
        parking_status = check_parking()
        ALL_PARKINGS.append(parking_status)
        robot.drive(5, 0)
        robot.turn(70)
        robot.drive(20, 0)
        robot.stop()
        
        lap_count += 1
        

    elif lap_count >= 3: #Ändra antal parkeringar efter banan
        print("Alla parkeringar kontrollerade:", ALL_PARKINGS)
        park_on_empty()


def park_on_empty(): #Fungerar inte som den ska, logiskt problem
    """
    Parkerar endast på tomma platser i ordning och kör vidare om parkeringen är upptagen.
    """
    for i, status in enumerate(ALL_PARKINGS):
        print("Kollar parkering", i + 1,"Status:", status)
        if status == "Empty":
            print("Kör till parkering för att parkera.")
              # Navigera till rätt parkering
            park()
            drive_to_parking(i + 1)  # Utför parkeringsmanövrer
        else:
            print("Parkering", (i + 1), "är upptagen. Fortsätter...")
            if i < len(ALL_PARKINGS) - 1:
                drive_to_parking(i + 1)  # Fortsätt till nästa parkering
            
        
def drive_to_parking(target_index):
    """
    Navigerar till en specifik parkeringsplats genom att följa linjen.
    """
    print("Navigerar till parkering" )
    current_index = 0  # Startar vid nuvarande parkering

    while current_index < target_index:
        # Linjeföljning med färgsensor
        reflection = line_sensor.reflection()
        error = reflection - ((WHITE_THRESHOLD + BLACK_THRESHOLD) / 2)
        turn_rate = KP * error
        robot.drive(CRUISE_SPEED, turn_rate)

        # Kontrollera om vi passerar en parkeringslinje
        parking_reflection = parking_sensor.reflection()
        if parking_reflection < BLACK_THRESHOLD:
            wait(500)  # Vänta för att undvika dubbelregistrering
            current_index += 1
            print("Parkering", current_index, "passerad")

    robot.stop()
    print("Framme vid parkering", (target_index + 1))
            
    

        

def reverse():
   """Anton S"""
   ev3.speaker.say('Exiting parking')
   robot.straight(50)
   robot.turn(-90)
   robot.straight(50)
   ev3.speaker.say('Continue')
   print("Reverse done")

def park():
   """Anton S"""
   ev3.speaker.say('Parking')
  
   robot.straight(50)
   robot.turn(90)
   robot.straight(-50)
   robot.stop()
   ev3.speaker.say('Parked')
   wait(5000)
   reverse()

# Huvudloop 
while True:
    follow_line()