''' Read Serial Servo Motor ID '''

#include required libraries
from SerialServo import SBSServo
from time import sleep
from machine import Pin, UART,SPI
import time
import st7789 #library of TFT display controller uses SPI interface
import vga1_16x32 as font
import vga1_8x16 as font1
import vga2_8x8 as font2
import vga1_16x16 as font4
import vga1_bold_16x32 as fontb


servo=SBSServo() 		# create instance 
ServoID=servo.readID()	# read ID's of connected serial servo motors

 # print number of motor connected and ID list
print("Total => ", len(ServoID))
print("ID List=> ", ServoID)  




