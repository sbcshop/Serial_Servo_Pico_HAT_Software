''' Onboard Button Demo Code'''

from time import sleep
from machine import Pin

BT1 = Pin(14, Pin.IN) # define onboard programmable button 1
BT2 = Pin(15, Pin.IN) # define onboard programmable button 2

sleep(0.5)

while 1:
    Btn1_val = BT1.value()
    Btn2_val = BT2.value()
    
    print(f"BT1 Value: {Btn1_val}, BT2 Value: {Btn2_val}")
    
    if Btn1_val == 0:
        print("BT1 pressed")
        sleep(0.2)
    
    elif Btn2_val == 0:
        print("BT2 pressed")
        sleep(0.2)
            
    sleep(0.2)

    



