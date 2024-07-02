''' Read/Write Serial Servo Motor ID '''

#include required libraries
from SerialServo import SBSServo
from time import sleep


servo=SBSServo() 		# create instance 
ServoID=servo.readID()	# read ID's of connected serial servo motors

 # print number of motor connected and ID list
print("Total => ", len(ServoID))
print("ID List=> ", ServoID)  


''' To change ID of servo, use below command '''
#ID = 3
#newID = 2
#servo.writeID(oldID, newID) # pass arguments both old and new ID


