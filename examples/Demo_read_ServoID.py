''' Read Serial Servo Motor ID '''

#include required libraries
from SerialServo import SBSServo
from time import sleep


servo=SBSServo() 		# create instance 
ServoID=servo.readID()	# read ID's of connected serial servo motors

 # print number of motor connected and ID list
print("Total => ", len(ServoID))
print("ID List=> ", ServoID)  




