''' Pico Serial Servo HAT Demo Testcode '''

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


servo=SBSServo()
ServoID=servo.readID()
print("total , ID", len(ServoID), ServoID)

ID = 2
servo.motor_mode(ID, speed=1000, direction=0) #clockwise direction

sleep(2)

servo.motor_mode(ID, speed=0, direction=0) #clockwise direction

sleep(2)

servo.motor_mode(ID, speed=1000, direction=1) #clockwise direction

sleep(2)

servo.motor_mode(ID, speed=0, direction=0) #clockwise direction


