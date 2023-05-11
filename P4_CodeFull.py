#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()


rightMotor = Motor(Port.B, Direction.CLOCKWISE,[40,24,24,40])
leftMotor = Motor(Port.C, Direction.CLOCKWISE,[40,24,24,40])
gyro = GyroSensor(Port.S4, Direction.CLOCKWISE)

robot = DriveBase(leftMotor, rightMotor, wheel_diameter=56, axle_track=112)



def ultras():
    ultrasonic_sensor = UltrasonicSensor(Port.S2)
 
    while True:
     
        distance = ultrasonic_sensor.distance()
        print("Distance: " + str(distance) + "mm")
        sleep(0.1)

claws = Motor(Port.A)

def clawup():

    claws.dc(60)
    wait(1500)
    
    
    claws.stop()

def clawdown():

    claws.dc(-30)
    wait(1500)
    

    claws.stop()


def clawdown2():
    claws.dc(-90)
    wait(1000)
    claws.stop()
color_sensor = ColorSensor(Port.S3)

def color():


 
    while True:
     
        color = color_sensor.color()
        ev3.screen.clear()
        print("Color: " + str(color))
        


def gyroDrive(distTravel,velocity):

    distTravel = (17.136*distTravel)-20.00 

    if distTravel > 0:
        while robot.distance() < distTravel:
            accuracy = (0 - (gyro.angle()* 5))
            robot.drive(velocity, accuracy) 
            angle = gyro.angle()
    
        
    else:
        while robot.distance() > 2:
            accuracy = (0 - (gyro.angle()* 5))
            robot.drive(velocity, accuracy) 
            angle = gyro.angle()
        

    gyro.reset_angle(0)
    robot.stop()


def gyroTurn(angle, velocity, rotation):
    gyro.reset_angle(0)
    if rotation == 'counterclockwise':
        while gyro.angle() < (angle):
            robot.drive(0, velocity)
    elif rotation == 'clockwise':
        while gyro.angle() > ((angle * -1)):
            robot.drive(0, -velocity)
    

def scan():
    barcode = [0, 0, 0, 0]
    intensity = [0, 0, 0, 0]


    for i in range(4):
        intensity[i] = color_sensor.reflection()
        if intensity[i] > 65:
            barcode[i] = 0
        else:
            barcode[i] = 1
        wait(100)
        robot.straight(1.27) 
        
   
    if barcode == [0,0,0,1]:
        print("Box type 1")
    elif barcode == [0,1,0,1]:
        print("Box type 2")
    elif barcode == [0,0,1,1]:
        print("Box type 3")
    elif barcode == [1,0,0,1]:
        print("Box type 4")
    else:
        print("No box")


def subtask1():
    g=12
    g=g*2.54+15.24
    gyro.reset_angle(0)
    gyroDrive(118, 200)
    gyroTurn(90, 200, 'counterclockwise')
    
    
    gyro.reset_angle(0)
    gyro.reset_angle(0)
    gyroDrive(g+155,200)
    wait(5000)
    
    
    
    gyroTurn(90, 200, 'clockwise')
    
    gyro.reset_angle(0)
    gyro.reset_angle(0)
    
    


   
def subtask2():
    gyroTurn(175,200,'clockwise')
    gyro.reset_angle(0)
    gyroDrive(45.72, 200)

    gyroTurn(89, 200, 'clockwise')
    g=12
    g=g*2.54+15.24
    
    gyro.reset_angle(0)
    gyro.reset_angle(0)
    gyroDrive(g+45.72,200)
    
    
    
    
    gyroDrive((232.2-g)+(45.72+15.24),200)
    gyroTurn(88, 200, 'clockwise')
    
    gyro.reset_angle(0)
    gyro.reset_angle(0)
    
    gyroDrive((232.2-g)+((15.24+45.72))+45.72,200)

def subtask3():
    gyro.reset_angle(0)
    
    scan()
    

def subtask4():
    
    gyroTurn(90 , 200 ,'counterclockwise')
    gyro.reset_angle(0)
    gyroDrive(20,200)
    wait(500)
    gyroTurn(175,200,'clockwise')
    gyro.reset_angle(0)
    gyroDrive(35,200)
    wait(500)
    clawdown()
    gyroDrive(45,200)
    wait(500)
    clawup()
    

    gyroTurn(180,200,'clockwise')
    gyro.reset_angle(0)
    gyroDrive(65,200)

    gyroTurn(89,200,'counterclockwise')
    gyro.reset_angle(0)
    wait(200)
    gyroDrive(123.42,200)
    wait(200)
    clawdown2()





#subtask1()
#subtask2()
#subtask3()
#subtask4()
#ultras()
#color()
#clawdown()
#scanner()

