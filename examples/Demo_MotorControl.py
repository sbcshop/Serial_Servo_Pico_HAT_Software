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

#define and configure TFT display 
spi = SPI(1, baudrate=40000000, sck=Pin(10), mosi=Pin(11))
tft = st7789.ST7789(spi,135,240,reset=Pin(12, Pin.OUT),cs=Pin(9, Pin.OUT),dc=Pin(8, Pin.OUT),
backlight=Pin(13, Pin.OUT),rotation=3)#SPI interface for tft screen

BT1 = Pin(14, Pin.IN) # define onboard programmable button 1
BT2 = Pin(15, Pin.IN) # define onboard programmable button 2

servo=SBSServo()
#ServoID=servo.readID()
#print(len(ServoID))

servoID1 = 2  # provide ID of Servo to control
servoID2 = 3  # provide ID of Servo to control

tft.init()
tft.fill(0)
tft.text(font,"Ready!", 60,40,st7789.YELLOW)
sleep(1)
tft.fill(0)

while 1:
    Btn1_val = BT1.value()
    Btn2_val = BT2.value()
    
    print("BT1 value: ",Btn1_val )
    print("BT2 value: ",Btn2_val)
    
    if Btn1_val == 0:
        print("BT1 pressed")
        tft.fill(0)
        tft.text(fontb,"Clockwise", 40,50,st7789.YELLOW)
        servo.motor_mode(servoID1, speed=1000, direction=0) #clockwise direction
        servo.motor_mode(servoID2, speed=1000, direction=0) #clockwise direction
        sleep(1)
    
    elif Btn2_val == 0:
        print("BT2 pressed")
        tft.fill(0)
        tft.text(fontb,"Anti-Clockwise", 20,50,st7789.YELLOW)
        servo.motor_mode(servoID1, speed=1000, direction=1) #Anti-clockwise direction
        servo.motor_mode(servoID2, speed=1000, direction=1) #Anti-clockwise direction
        sleep(1)
            
    else:
        print("Press Any Button")
        tft.text(fontb,"Press Button", 20,50,st7789.YELLOW)
        #stop motors
        servo.motor_mode(servoID1, speed=0, direction=0) 
        servo.motor_mode(servoID2, speed=0, direction=0) 
        sleep(0.3)
    
    sleep(0.2)

    

