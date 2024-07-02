''' Display Demo Code '''
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

tft.init()
tft.fill(0)
tft.text(font,"PICO", 70,30, st7789.YELLOW)
tft.text(font,"Serial Servo", 30,70, st7789.YELLOW)
sleep(2)	# wait for 2 seconds

tft.fill(0)	# clear display
tft.text(font,"Thanks You!", 35,20, st7789.YELLOW)
tft.text(fontb,"SB COMPONENTS", 20,70,st7789.BLUE)
tft.text(font1,"shop.sb-components.co.uk", 25,100,st7789.WHITE)
sleep(2)
    
tft.fill(0)
tft.text(font,"Bye!", 40,50,st7789.YELLOW)
sleep(0.5)

    



