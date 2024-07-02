''' Servo mode demo example '''

#include required libraries
from SerialServo import SBSServo
from time import sleep

#create instance
servo=SBSServo()
ServoID=servo.readID() # Read ID's of connected motor
print("Total, ID => ", len(ServoID), ServoID)

ID = 3 # select motor with ID 

servo.writeAngleLimit(ID, angleMin=0, angleMax=1000) #set min max
sleep(0.5)

''' move servo Shaft to corresponding position '''
servo.servoWrite(ID, position=100, r_time=0, r_speed=1000) #move to position
sleep(2)
servo.servoWrite(ID, position=0, r_time=0, r_speed=500) #move to position
sleep(1)
servo.servoWrite(ID, position=800, r_time=0, r_speed=1000) #move to position
