''' Run Serial Servo in Motor Mode '''

#include required libraries
from SerialServo import SBSServo
from time import sleep

servo=SBSServo() #create instance 

ID = 3 #Serial servo motor ID, change ID to operate corresponding connected serial servo

servo.writeAngleLimit(ID, angleMin=0, angleMax=0)
sleep(0.5)

print("Clockwise Direction")
servo.motor_mode(ID, speed=1000, direction=0) #clockwise direction
sleep(2)

servo.motor_mode(ID, speed=0, direction=0) #stop rotation
sleep(1)


print("Anti-clockwise Direction")
servo.motor_mode(ID, speed=1000, direction=0) #Anti-clockwise direction
sleep(1)

servo.motor_mode(ID, speed=0, direction=0) #stop rotation
